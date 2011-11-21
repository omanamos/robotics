using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Communication;

namespace DataStore
{
    public class ObjectLibrary
    {
        private Dictionary<string, RecogObject> objects;
        private Dictionary<string, PointCloud> knownPointClouds;
        private HashSet<PointCloud> unknownPointClouds;
        private Dictionary<string, List<RecogObject>> lookupByProperty;

        private PointCloud curLearning;
        private string curLearningName;

        public ObjectLibrary()
        {
            this.objects = new Dictionary<string, RecogObject>();
            this.lookupByProperty = new Dictionary<string, List<RecogObject>>();
            this.knownPointClouds = new Dictionary<string, PointCloud>();
            this.unknownPointClouds = new HashSet<PointCloud>();
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

        public void loadPointClouds(List<PointCloud> clouds)
        {
            this.knownPointClouds.Clear();
            foreach (PointCloud pc in clouds)
            {
                if (pc.Identifier != null)
                {
                    this.knownPointClouds[pc.Identifier] = pc;
                }
                else
                {
                    this.unknownPointClouds.Add(pc);
                }
            }
        }

        public PointCloud learnObject()
        {
            if (this.unknownPointClouds.Count == 0)
            {
                return null;
            }
            else
            {
                this.curLearning = this.unknownPointClouds.FirstOrDefault();
                return this.curLearning;
            }
        }

        public void setLearnedName(string s)
        {
            this.curLearningName = s;
        }

        public bool saveObject()
        {
            if (this.curLearning == null)
            {
                return false;
            }
            else
            {
                this.unknownPointClouds.Remove(this.curLearning);
                this.knownPointClouds[this.curLearningName] = this.curLearning;
                RecogObject obj = new RecogObject(this.curLearningName);
                this.objects[this.curLearningName] = obj;
                this.curLearning = null;
                return true;
            }
        }

        public Dictionary<string, RecogObject>.KeyCollection getIdentifiers()
        {
            return this.objects.Keys;
        }

        public Dictionary<string, PointCloud> getKnownObjects()
        {
            return this.knownPointClouds;
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

        public HashSet<PointCloud> getUnknownObjects()
        {
            return this.unknownPointClouds;
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
