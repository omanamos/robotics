using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.Serialization;

using Communication;

namespace DataStore
{
    public class RecogObject
    {
        public readonly string identifier;
        private List<string> properties;

        public RecogObject(string identifier)
        {
            this.identifier = identifier;
            this.properties = new List<string>();
        }

        public void addProperty(string property) {
            this.properties.Add(property);
        }

        public void addProperties(List<string> properties)
        {
            this.properties = this.properties.Union(properties).ToList();
        }

        public List<string> getProperties()
        {
            return this.properties;
        }

        public string toString()
        {
            String rtn = this.identifier;
            foreach (string p in this.properties) {
                rtn += " " + p;
            }
            return rtn;
        }

        public static RecogObject fromString(string s)
        {
            string[] parts = s.Split(' ');
            RecogObject rtn = new RecogObject(parts[0]);
            bool flag = false;
            foreach (string p in parts) {
                if (flag)
                {
                    rtn.properties.Add(p);
                }
                else
                {
                    flag = true;
                }
            }
            return rtn;
        }
    }
}
