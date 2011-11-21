using System;
using System.Collections.Generic;
using System.Linq;
using System.Speech.Recognition;
using System.Text;

namespace Controller.VoiceRecog
{
    class Template
    {
        private List<IElement> elements;

        public Template()
        {
            this.elements = new List<IElement>();
        }

        public void addString(string str)
        {
            this.elements.Add(new StringElement(str));
        }

        public void addList(List<string> list)
        {
            this.elements.Add(new ListElement(list));
        }

        public GrammarBuilder toGrammar()
        {
            GrammarBuilder builder = new GrammarBuilder();
            foreach (IElement element in this.elements)
            {
                builder.Append(element.toGrammar());
            }
            return builder;
        }

        public string toRegex()
        {
            String rtn = "";
            foreach (IElement element in this.elements)
            {
                rtn += element.toRegex() + " ";
            }
            return rtn.Substring(0, rtn.Length - 1);
        }
    }

    interface IElement {
        GrammarBuilder toGrammar();
        string toRegex();
    }

    class ListElement : IElement
    {
        private List<string> list;

        public ListElement(List<string> list)
        {
            this.list = list;
        }

        public GrammarBuilder toGrammar()
        {
            return new Choices(this.list.ToArray());
        }

        public string toRegex()
        {
            string rtn = "(";
            foreach (string s in this.list)
            {
                rtn += s + "|";
            }
            return rtn.Substring(0, rtn.Length - 1) + ")";
        }
    }

    class StringElement : IElement
    {
        private string s;

        public StringElement(string s)
        {
            this.s = s;
        }

        public GrammarBuilder toGrammar()
        {
            return new GrammarBuilder(this.s);
        }

        public string toRegex()
        {
            return this.s;
        }
    }
}
