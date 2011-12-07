using System;
using System.Collections.Generic;
using System.Linq;
using System.Speech.Recognition;
using System.Text;

using Controller;
using DataStore;

namespace VoiceRecog
{
    class CommandGrammarBuilder
    {
        private static string[] nouns = loadNouns();
        private static string[] adj = loadAdj();
        private Grammar grammar;
        private string prefix;
        private List<string> strings;

        private static string[] loadNouns()
        {
            string[] rtn = System.IO.File.ReadAllLines("../../VoiceRecog/nouns.txt");
            return rtn;
        }

        private static string[] loadAdj()
        {
            string[] input = System.IO.File.ReadAllLines("../../VoiceRecog/adj.txt");
            HashSet<string> rtn = new HashSet<string>();
            foreach (string s in input)
            {
                if (s.Length != 0)
                {
                    rtn.Add(s);
                }
            }
            return rtn.ToArray();
        }
        
        public CommandGrammarBuilder(MainController.State state, ObjectLibrary lib)
        {
            this.strings = new List<string>();
            switch (state)
            {
                case MainController.State.waiting:
                    this.grammar = buildWaitingGrammar();
                    break;
                case MainController.State.start:
                    this.grammar = buildStartGrammar(lib);
                    break;
                case MainController.State.learn:
                    this.grammar = null;
                    break;
                case MainController.State.find:
                    this.grammar = buildFindGrammar();
                    break;
                case MainController.State.confirmation:
                    this.grammar = buildConfGrammar();
                    break;
                case MainController.State.getName:
                    this.grammar = buildGetNameGrammar();
                    break;
                case MainController.State.getProperties:
                    this.grammar = buildGetPropertiesGrammar();
                    break;
            }
        }

        public Grammar getGrammar()
        {
            return this.grammar;
        }

        public string getPrefix()
        {
            return this.prefix;
        }

        public List<string> getStrings()
        {
            return this.strings;
        }

        public static Grammar buildInterruptGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            builder.Append(new Choices(new string[] { "abort" }));
            return new Grammar(builder);
        }

        private Grammar buildGetNameGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("it is called");
            builder.Append(new Choices(CommandGrammarBuilder.nouns));
            this.prefix = "it is called";
            return new Grammar(builder);
        }

        private Grammar buildGetPropertiesGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("it has the property");
            builder.Append(new Choices(CommandGrammarBuilder.adj));
            this.prefix = "it has the property ";
            return new Grammar(builder);
        }

        private Grammar buildWaitingGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            builder.Append("set up");
            this.prefix = "nao";
            this.strings.Add("set up");
            return new Grammar(builder);
        }

        private Grammar buildStartGrammar(ObjectLibrary lib)
        {
            this.prefix = "nao";
            GrammarBuilder builder = new GrammarBuilder("nao");
            Choices topLevel = new Choices();
            topLevel.Add("identify known objects");
            this.strings.Add("identify known objects");
            topLevel.Add("learn unknown objects");
            this.strings.Add("learn unknown objects");

            string[] properties = lib.getProperties().ToArray();
            if (properties.Length != 0)
            {
                GrammarBuilder propertyGrammar = new GrammarBuilder("find all");
                propertyGrammar.Append(new Choices(properties));
                propertyGrammar.Append("objects");
                topLevel.Add(propertyGrammar);
                this.strings.Add("find all {" + toString(properties) + "} objects");
            }

            string[] identifiers = lib.getIdentifiers().ToArray();
            if (identifiers.Length != 0)
            {
                GrammarBuilder identifierGrammar = new GrammarBuilder("locate");
                identifierGrammar.Append(new Choices(identifiers));
                topLevel.Add(identifierGrammar);
                this.strings.Add("locate {" + toString(identifiers) + "}");
            }
            
            builder.Append(topLevel);
            return new Grammar(builder);
        }

        private string toString(string[] arr)
        {
            string rtn = "";
            foreach (string s in arr)
            {
                rtn += s + ",";
            }
            return rtn.Substring(0, rtn.Length - 1);
        }

        private Grammar buildFindGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            return new Grammar(builder);
        }

        private Grammar buildConfGrammar()
        {
            this.prefix = "confirm";
            this.strings.Add("yes");
            this.strings.Add("no");
            GrammarBuilder builder = new GrammarBuilder("confirm");
            builder.Append(new Choices(new string[] { "yes", "no" }));
            return new Grammar(builder);
        }
    }
}
