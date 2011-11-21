using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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

        public List<string> getProperties()
        {
            return this.properties;
        }
    }
}
