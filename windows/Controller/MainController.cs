﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;
using System.Threading;
using Thrift;
using Thrift.Protocol;
using Thrift.Server;
using Thrift.Transport;

using Communication;
using DataStore;
using VoiceRecog;
using System.Windows.Media;

namespace Controller
{
    public class MainController
    {
        public static readonly int THRIFT_SLEEP = 100;

        public static readonly String OBJECT_LIB_PATH = "../../object_lib.data";
        public static readonly String NAO_IP = "127.0.0.1";
        //public static readonly String NAO_IP = "128.208.4.19";
        //public static readonly String SERVER_IP = "localhost";
        public static readonly String SERVER_IP = "128.208.4.24";

        public enum State { waiting, start, confirmation, learn, getName, getProperties, find };
        private volatile State state;
        private volatile State prevState;
        private volatile State lastChangedState;
        private string prevCommand;

        private MainWindow window;
        private Rpc.Client thriftClient;
        private TTransport thriftTransport;

        private VoiceRecogition recog;

        private NaoController nao;
        private ObjectLibrary lib;
        private NavigationController nav;

        private Thread navThread;
        private Thread updateThread;

        public MainController(MainWindow window)
        {
            this.window = window;
            this.nao = new NaoController(NAO_IP, this);

            this.thriftTransport = new TSocket(SERVER_IP, 9090);
            thriftTransport.Open();
            TProtocol protocol = new TBinaryProtocol(thriftTransport);
            this.thriftClient = new Rpc.Client(protocol);

            this.lib = new ObjectLibrary(this, this.thriftClient, MainController.OBJECT_LIB_PATH);
            this.updateThread = new Thread(this.lib.updatePointClouds);
            this.updateThread.Start();

            this.nav = new NavigationController(this, nao, lib);
            this.navThread = null;

            switchStates(State.waiting);
        }

        public void abort()
        {
            Console.Write("Aborting...");
            this.nav.stop();
            this.nao.isWalking();
            this.navThread.Join();
            this.navThread = null;
            this.sleep(false);
            this.nao.isWalking();
            this.lib.cancelLearning();
            this.switchStates(State.start);
            Console.WriteLine("Done!");
        }

        public void setWalking(bool isWalking)
        {
            window.Dispatcher.BeginInvoke(new Action(
                delegate()
                {
                    window.walking_indicator.Background = new SolidColorBrush(isWalking ? Colors.Red : Colors.Green);
                }));
        }

        public void setLoading(bool isLoading)
        {
            window.Dispatcher.BeginInvoke(new Action(
                delegate()
                {
                    window.blocking_indicator.Background = new SolidColorBrush(isLoading ? Colors.Yellow : Colors.Green);
                }));
        }

        public void block()
        {
            window.Dispatcher.BeginInvoke(new Action(
                delegate()
                {
                    window.blocking_indicator.Background = new SolidColorBrush(Colors.Red);
                }));
        }

        public void switchStates(State state)
        {
            window.Dispatcher.BeginInvoke(new Action(
                delegate()
                {
                    window.currentState.Text = state.ToString();
                    window.availableCommands.Items.Clear();
                    CommandGrammarBuilder builder = new CommandGrammarBuilder(state, lib);
                    foreach (string s in builder.getStrings())
                        window.availableCommands.Items.Add(s);
                    window.prefix.Text = builder.getPrefix();
                }));

            if (state != this.state)
            {
                this.lastChangedState = this.state;
            }
            this.prevState = this.state;
            this.state = state;
            if (this.recog != null)
            {
                this.recog.exit();
            }
            this.recog = new VoiceRecogition(state, this);
            if (state == State.learn)
            {
                // make it a thread
                this.navThread = new Thread(this.nav.learnUnknownObjects);
                this.navThread.Start();
            }
        }

        public void processCommand(string command)
        {
            Console.WriteLine("Current State: " + this.state);
            if (command.Equals("nao abort"))
            {
                throw new NotImplementedException("nao abort");
            }
            else if (command.Equals("nao exit"))
            {
                throw new NotImplementedException("nao exit");
            }
            else
            {
                switch (this.state)
                {
                    case State.waiting:
                        switchStates(State.start);
                        break;
                    case State.start:
                        string prefix = command.Split(' ')[1];
                        if (prefix.Equals("identify"))
                        {
                            this.navThread = new Thread(this.nav.identifyKnownObjects);
                            this.navThread.Start();
                            this.recog.exit();
                        }
                        else if (prefix.Equals("learn"))
                        {
                            this.switchStates(State.learn);
                        }
                        else if (prefix.Equals("find"))
                        {
                            string property = removeNWords(command, 3).Replace(" objects", "").Trim();
                            this.nav.setProperty(property);
                            this.navThread = new Thread(this.nav.findObjects);
                            this.navThread.Start();
                            this.recog.exit();
                        }
                        else if (prefix.Equals("locate"))
                        {
                            string identifier = removeNWords(command, 2);
                            this.nav.setIdentifier(identifier);
                            this.navThread = new Thread(this.nav.locateObject);
                            this.navThread.Start();
                            this.recog.exit();
                        }
                        else
                        {
                            Console.WriteLine("Invalid command: " + command);
                        }
                        break;
                    case State.learn:
                        Console.WriteLine("FATAL ERROR: entered learn state on a command");
                        break;
                    case State.getName:
                        String name = command.Replace("it is called", "").Trim();
                        nao.speak("did you say " + name);
                        this.lib.setLearnedName(name);
                        this.switchStates(State.confirmation);
                        break;
                    case State.getProperties:
                        String prop = command.Replace("it has the property", "").Replace("object", "").Trim();
                        nao.speak("did you say " + prop);
                        this.lib.setLearnedProperty(prop);
                        this.switchStates(State.confirmation);
                        break;
                    case State.find:
                        break;
                    case State.confirmation:
                        if (command.Equals("confirm yes"))
                        {
                            if (this.prevCommand.Equals("nao exit"))
                            {
                                nao.speak("Goodbye");
                                this.exit();
                            }
                            else if (this.lastChangedState == State.getName)
                            {
                                if (this.prevState == State.confirmation)
                                {
                                    this.switchStates(State.getProperties);
                                    nao.speak("What is one of it's properties?");
                                }
                                else
                                {
                                    nao.speak("Ok, does a " + this.lib.curLearningName + " have any properties I should know about?");
                                    this.switchStates(State.confirmation);
                                }
                            }
                            else if (this.lastChangedState == State.getProperties)
                            {
                                if (this.prevState == State.confirmation)
                                {
                                    this.nao.speak("Ok, what is one of it's properties?");
                                    this.switchStates(State.getProperties);
                                }
                                else if (this.lib.addPropertyToLearning())
                                {
                                    nao.speak("Ok, I've saved this property. Does " +
                                        this.lib.curLearningName + " have any more properties?");
                                    this.switchStates(State.confirmation);
                                }
                                else
                                {
                                    nao.speak("I'm sorry, it looks like there was an error");
                                    this.switchStates(State.start);
                                }
                            }
                            else
                            {
                                nao.speak("I'm sorry, it looks like there was an error");
                                this.switchStates(State.start);
                            }
                        }
                        else if (command.Equals("confirm no"))
                        {
                            if (this.prevCommand.Equals("nao exit"))
                            {
                                this.switchStates(State.start);
                            }
                            else if (lastChangedState == State.getName)
                            {
                                if (prevState == State.confirmation)
                                {
                                    this.lib.saveObject();
                                    this.switchStates(State.learn);
                                }
                                else
                                {
                                    this.switchStates(State.getName);
                                    nao.speak("OK, what is it really called?");
                                }
                            }
                            else if (lastChangedState == State.getProperties)
                            {
                                if (prevState == State.getProperties)
                                {
                                    this.switchStates(State.getProperties);
                                    nao.speak("Ok, what is the property really called");
                                }
                                else if (this.lib.saveObject())
                                {
                                    nao.speak("OK, I've saved this object.");
                                    this.switchStates(State.learn);
                                }
                                else
                                {
                                    nao.speak("I'm sorry, it looks like there was an error");
                                    this.switchStates(State.start);
                                }
                            }
                            else
                            {
                                nao.speak("I'm sorry, it looks like there was an error");
                                this.switchStates(State.start);
                            }
                        }
                        break;
                }
            }
            this.prevCommand = command;
            Console.WriteLine("New State: " + this.state);
        }

        public Point locateNao()
        {
            while (this.thriftClient.isWaiting) { this.sleep(true); }
            this.setLoading(true);
            return this.thriftClient.locateNao();
        }

        public void sleep(bool thrift)
        {
            if (thrift)
            {
                Console.Write(".");
                Thread.Sleep(THRIFT_SLEEP);
                Console.Write("!");
            }
            else
            {
                Console.Write("-");
                Thread.Sleep(1000);
                Console.Write("!");
            }
        }

        private string removeNWords(string str, int n)
        {
            string rtn = "";
            string[] words = str.Split(' ');
            for (int i = n; i < words.Length; i++)
            {
                rtn += words[i] + " ";
            }
            return rtn.Trim();
        }

        public void exit()
        {
            this.lib.saveAndExit(MainController.OBJECT_LIB_PATH);
            this.recog.exit();
            if (this.navThread != null)
            {
                this.navThread.Abort();
            }
            this.updateThread.Abort();
            this.nao.exit();
            this.thriftTransport.Close();
            Environment.Exit(0);
        }

        public ObjectLibrary getLibrary()
        {
            return this.lib;
        }
    }
}
