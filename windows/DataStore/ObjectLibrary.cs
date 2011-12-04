using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LicenseCommon;
using System.Runtime.Serialization;

using Communication;

namespace DataStore
{
    [Serializable]
    public class ObjectLibrary : ISerializable
    {
        private SerializableDictionary<string, RecogObject> objects;
        private Dictionary<string, PointCloud> knownPointClouds;
        private Dictionary<string, PointCloud> unknownPointClouds;
        private SerializableDictionary<string, List<RecogObject>> lookupByProperty;

        private PointCloud curLearning;
        private string curLearningName;

        public ObjectLibrary()
        {
            this.objects = new SerializableDictionary<string, RecogObject>();
            this.lookupByProperty = new SerializableDictionary<string, List<RecogObject>>();
            this.knownPointClouds = new Dictionary<string, PointCloud>();
            this.unknownPointClouds = new Dictionary<string, PointCloud>();

            // hard coded to recognize for demo milestone 2
            // from saveobject method:
            // this.knownPointClouds[this.curLearningName] = this.curLearning;
            // RecogObject obj = new RecogObject(this.curLearningName);
            // this.objects[this.curLearningName] = obj;
            this.knownPointClouds.Add("box", new PointCloud());
            RecogObject obj = new RecogObject("box");
            this.objects["box"] = obj;

            this.knownPointClouds.Add("ball", new PointCloud());
            RecogObject obj2 = new RecogObject("ball");
            this.objects["ball"] = obj2;

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

        public void cancelLearning()
        {
            this.curLearning = null;
            this.curLearningName = null;
        }

        public void loadPointClouds(List<PointCloud> clouds)
        {
            this.knownPointClouds.Clear();
            foreach (PointCloud pc in clouds)
            {
                if (pc.Identifier.StartsWith("_"))
                {
                    this.unknownPointClouds[pc.Identifier] = pc;
                }
                else
                {
                    this.knownPointClouds[pc.Identifier] = pc;
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
                this.curLearning = this.unknownPointClouds.FirstOrDefault().Value;
                return this.curLearning;
            }
        }

        public void setLearnedName(string s)
        {
            this.curLearningName = s;
        }

        public bool saveObject(Rpc.Client client)
        {
            if (this.curLearning == null)
            {
                return false;
            }
            else
            {
                // Send updated identifier to through thrift
                if (client.update(this.curLearning.Identifier, this.curLearningName))
                {
                    this.unknownPointClouds.Remove(this.curLearning.Identifier);
                    this.curLearning.Identifier = this.curLearningName;
                    this.knownPointClouds[this.curLearningName] = this.curLearning;
                    RecogObject obj = new RecogObject(this.curLearningName);
                    this.objects[this.curLearningName] = obj;
                    this.curLearning = null;
                    this.curLearningName = null;
                    return true;
                }
                else
                {
                    return false;
                }
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
            Console.WriteLine("identifier: " + identifier);
            return this.objects[identifier];
        }

        public PointCloud getPointCloud(string identifier)
        {
            return this.knownPointClouds[identifier];
        }

        public List<RecogObject> getObjects(string property)
        {
            return this.lookupByProperty[property];
        }

        public Dictionary<string, List<RecogObject>>.KeyCollection getProperties()
        {
            return this.lookupByProperty.Keys;
        }

        public Dictionary<string, PointCloud>.ValueCollection getUnknownObjects()
        {
            return this.unknownPointClouds.Values;
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
