using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.Serialization;

using Communication;

namespace DataStore
{
    [Serializable]
    public class RecogObject : ISerializable
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

        public RecogObject(SerializationInfo info, StreamingContext ctxt)
        {
            this.identifier = (string)info.GetValue("Identifier", typeof(string));
            this.properties = (List<string>)info.GetValue("Properties", typeof(List<string>));
        }

        public void GetObjectData(SerializationInfo info, StreamingContext ctxt)
        {
            info.AddValue("Identifier", identifier);
            info.AddValue("Properties", properties);
        }
    }
}
