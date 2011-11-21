using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DataStore
{
    public class ObjectLibrary
    {
        private Dictionary<string, RecogObject> objects;
        private Dictionary<string, List<RecogObject>> lookupByProperty;

        public ObjectLibrary()
        {
            this.objects = new Dictionary<string, RecogObject>();
            this.lookupByProperty = new Dictionary<string, List<RecogObject>>();
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
    }
}
