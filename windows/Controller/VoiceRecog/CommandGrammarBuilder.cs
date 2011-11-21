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
        private Grammar grammar;

        public CommandGrammarBuilder(MainController.State state, ObjectLibrary lib)
        {
            switch (state)
            {
                case MainController.State.waiting:
                    this.grammar = buildWaitingGrammar();
                    break;
                case MainController.State.start:
                    this.grammar = buildStartGrammar(lib);
                    break;
                case MainController.State.learn:
                    this.grammar = buildLearnGrammar();
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
            }
        }

        public Grammar getGrammar()
        {
            return this.grammar;
        }

        public static Grammar buildInterruptGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            builder.Append(new Choices(new string[] { "abort", "restart" }));
            return new Grammar(builder);
        }

        private static Grammar buildGetNameGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("it is called");
            builder.AppendDictation();
            return new Grammar(builder);
        }

        private static Grammar buildWaitingGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            builder.Append("set up");
            return new Grammar(builder);
        }

        private static Grammar buildStartGrammar(ObjectLibrary lib)
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            Choices topLevel = new Choices();
            topLevel.Add("identify known objects");
            topLevel.Add("learn unknown objects");

            GrammarBuilder propertyGrammar = new GrammarBuilder("find all");
            propertyGrammar.Append(new Choices(lib.getProperties().ToArray()));
            propertyGrammar.Append("objects");
            topLevel.Add(propertyGrammar);

            GrammarBuilder identifierGrammar = new GrammarBuilder("locate");
            identifierGrammar.Append(new Choices(lib.getIdentifiers().ToArray()));
            topLevel.Add(propertyGrammar);

            builder.Append(topLevel);
            return new Grammar(builder);
        }

        private static Grammar buildLearnGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder();
            return new Grammar(builder);
        }

        private static Grammar buildFindGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("nao");
            return new Grammar(builder);
        }

        private static Grammar buildConfGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder("confirm");
            builder.Append(new Choices(new string[] { "yes", "no" }));
            return new Grammar(builder);
        }
    }
}
