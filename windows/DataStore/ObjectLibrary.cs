using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LicenseCommon;
using System.Runtime.Serialization;

namespace DataStore
{
    [Serializable]
    public class ObjectLibrary : ISerializable
    {
        private SerializableDictionary<string, RecogObject> objects;
        private SerializableDictionary<string, List<RecogObject>> lookupByProperty;

        public ObjectLibrary()
        {
            this.objects = new SerializableDictionary<string, RecogObject>();
            this.lookupByProperty = new SerializableDictionary<string, List<RecogObject>>();
        }

        public void addObject(RecogObject obj) 
        {
            this.objects[obj.identifier] = obj;
            foreach (string property in obj.getProperties())
            {
                if (!this.lookupByProperty.ContainsKey(property))
                {
                    this.lookupByProperty[property] = new List<RecogObject>();
                }
                this.lookupByProperty[property].Add(obj);
            }
        }

        public Dictionary<string, RecogObject>.KeyCollection getIdentifiers()
        {
            return this.objects.Keys;
        }

        public RecogObject getObject(string identifier)
        {
            return this.objects[identifier];
        }

        public List<RecogObject> getObjects(string property)
        {
            return this.lookupByProperty[property];
        }

        public Dictionary<string, List<RecogObject>>.KeyCollection getProperties()
        {
            return this.lookupByProperty.Keys;
        }

        public bool hasProperty(string property)
        {
            return this.lookupByProperty.ContainsKey(property);
        }

        public void save(string ACTION_LIB_PATH)
        {
            throw new NotImplementedException();
        }


        /*
        private SerializableDictionary<string, RecogObject> objects;
        private SerializableDictionary<string, List<RecogObject>> lookupByProperty;
         * 
         */

        public ObjectLibrary(SerializationInfo info, StreamingContext ctxt)
        {
            this.objects = (SerializableDictionary<string, RecogObject>)info.GetValue("Objects", typeof(SerializableDictionary<string, RecogObject>));
            this.lookupByProperty = (SerializableDictionary<string, List<RecogObject>>)info.GetValue("Property", typeof(SerializableDictionary<string, RecogObject>));
        }

        public void GetObjectData(SerializationInfo info, StreamingContext ctxt)
        {
            info.AddValue("Objects", objects);
            info.AddValue("Property", lookupByProperty);
        }
    }
}
