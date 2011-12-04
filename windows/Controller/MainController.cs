using System;
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

namespace Controller
{
    public class MainController
    {
        public static readonly String ACTION_LIB_PATH = "temp";
        public static readonly String NAO_IP = "127.0.0.1";
        //public static readonly String NAO_IP = "128.208.4.225";

        public enum State { waiting, start, confirmation, learn, getName, find };
        private volatile State state;
        private volatile State prevState;

        private MainWindow window;
        private Rpc.Client thriftClient;
        private TTransport thriftTransport;

        private VoiceRecogition recog;
        private VoiceRecogition interrupt;

        private NaoController nao;
        private ObjectLibrary lib;
        private NavigationController nav;

        private Thread navThread;
        private Thread interruptThread;

        public MainController(MainWindow window)
        {
            this.window = window;
            this.nao = new NaoController(NAO_IP);
            this.lib = new ObjectLibrary();

            this.thriftTransport = new TSocket("localhost", 9090);
            thriftTransport.Open();
            TProtocol protocol = new TBinaryProtocol(thriftTransport);
            this.thriftClient = new Rpc.Client(protocol);
            
            this.nav = new NavigationController(this, nao, lib, this.thriftClient);
            this.navThread = null;

            this.interrupt = new VoiceRecogition(CommandGrammarBuilder.buildInterruptGrammar(), this);
            this.interruptThread = new Thread(this.interrupt.start);
            this.interruptThread.Start();
            switchStates(State.waiting);
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
                if (this.navThread != null)
                {
                    Console.Write("Aborting...");
                    this.nav.stop();
                    this.navThread.Join();
                    Console.WriteLine("Done!");
                }
                else
                {
                    this.lib.cancelLearning();
                    this.switchStates(State.start);
                }
            }
            else
            {
                switch (this.state)
                {
                    case State.waiting:
                        switchStates(State.start);
                        this.lib.loadPointClouds(this.thriftClient.getObjects());
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
                            // TODO(namos): have Nao find all ______ objects
                        }
                        else if (prefix.Equals("locate"))
                        {
                            // TODO(namos): have Nao locate _______ object
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
                    case State.find:
                        break;
                    case State.confirmation:
                        if (command.Equals("confirm yes"))
                        {
                            if (this.lib.saveObject(this.thriftClient))
                            {
                                nao.speak("Ok, I've saved this object");
                                this.switchStates(State.learn);
                            }
                            else
                            {
                                nao.speak("I'm sorry, it looks like there was an error");
                                this.switchStates(State.start);
                            }
                        }
                        else if (command.Equals("confirm no"))
                        {
                            this.switchStates(State.getName);
                            nao.speak("OK, what is it really called?");
                        }
                        break;
                }
            }
            Console.WriteLine("New State: " + this.state);
        }

        public void exit()
        {
            this.save();
            this.recog.exit();
            this.nao.exit();
            this.thriftTransport.Close();
            Environment.Exit(0);
        }

        public ObjectLibrary getLibrary()
        {
            return this.lib;
        }

        private void save()
        {
            this.lib.save(ACTION_LIB_PATH);
        }
    }
}
