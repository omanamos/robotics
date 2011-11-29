using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;
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
        //public static readonly String NAO_IP = "127.0.0.1";
        public static readonly String NAO_IP = "128.208.4.225";

        public enum State { waiting, start, confirmation, learn, getName, find };
        private State state;
        private State prevState;

        private MainWindow window;
        private Rpc.Client thriftClient;
        private TTransport thriftTransport;

        private VoiceRecogition recog;
        private VoiceRecogition interrupt;

        private NaoController nao;
        private ObjectLibrary lib;

        public MainController(MainWindow window)
        {
            this.window = window;
            this.nao = new NaoController(NAO_IP);
            this.lib = new ObjectLibrary();

            this.thriftTransport = new TSocket("localhost", 9090);
            thriftTransport.Open();
            TProtocol protocol = new TBinaryProtocol(thriftTransport);
            this.thriftClient = new Rpc.Client(protocol);

            this.interrupt = new VoiceRecogition(CommandGrammarBuilder.buildInterruptGrammar(), this);
            switchStates(State.waiting);
        }

        private void switchStates(State state)
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
                this.learn();
            }
        }

        public void processCommand(string command)
        {
            Console.WriteLine("Current State: " + this.state);
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
                        // TODO(namos): spawn this off in a thread so it can be stopped by interrupt

                        // initial naoposition
                        Point naoLocation = this.thriftClient.locateNao();


                        foreach (KeyValuePair<string, PointCloud> pair in lib.getKnownObjects())
                        {
                            RecogObject obj = lib.getObject(pair.Key);
                            nao.walkToObject(pair.Value.Average, naoLocation);
                            nao.speak("this is a " + pair.Key);
                            nao.speak("it has the following properties");
                            string props = "";
                            foreach (string property in obj.getProperties())
                            {
                                props += property + ", ";
                            }
                            nao.speak(props + "and no more");

                            naoLocation = pair.Value.Average;

                            // waits for nao to finish speaking
                            System.Threading.Thread.Sleep(4000);
                        }
                    }
                    else if (prefix.Equals("learn"))
                    {
                        // TODO(namos): have Nao find all unknown objects and learn them
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
                        if (this.lib.saveObject())
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
            Console.WriteLine("New State: " + this.state);
        }

        private void learn()
        {
            PointCloud obj = this.lib.learnObject();
            if (obj == null)
            {
                nao.speak("I'm done learning");
                this.switchStates(State.start);
            }
            else
            {
                Point naoLocation = this.thriftClient.locateNao();
                nao.walkToObject(obj.Average, naoLocation);
                nao.speak("what is this object called?");
                this.switchStates(State.getName);
            }
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
