using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;

using DataStore;
using VoiceRecog;

namespace Controller
{
    public class MainController
    {
        public static readonly String ACTION_LIB_PATH = "temp";
        public static readonly String NAO_IP = "127.0.0.1";

        public enum State { waiting, start, confirmation, learn, find };
        private State state;
        private State prevState;

        private VoiceRecogition recog;
        private VoiceRecogition interrupt;

        private NaoController nao;
        private ObjectLibrary lib;

        public MainController()
        {
            this.nao = new NaoController(NAO_IP);
            this.lib = new ObjectLibrary();

            this.interrupt = new VoiceRecogition(CommandGrammarBuilder.buildInterruptGrammar(), this);
            switchStates(State.waiting);
        }

        private void switchStates(State state)
        {
            this.prevState = this.state;
            this.state = state;
            if (this.recog != null)
            {
                this.recog.exit();
            }
            this.recog = new VoiceRecogition(state, this);
        }

        public void processCommand(string command)
        {
            Console.WriteLine("Current State: " + this.state);
            switch (this.state)
            {
                case State.waiting:
                    break;
                case State.start:
                    break;
                case State.learn:
                    break;
                case State.find:
                    break;
                case State.confirmation:
                    break;
            }
            Console.WriteLine("New State: " + this.state);
        }

        public void exit()
        {
            this.save();
            this.recog.exit();
            this.nao.exit();
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
