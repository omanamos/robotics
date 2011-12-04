using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

using Communication;
using DataStore;

namespace Controller
{
    class NavigationController
    {
        private MainController main;
        private NaoController nao;
        private ObjectLibrary lib;
        private Rpc.Client thriftClient;
        private volatile string identifier;
        private volatile string property;
        private volatile bool stopped;

        public NavigationController(MainController main, NaoController nao, ObjectLibrary lib, Rpc.Client thriftClient)
        {
            this.stopped = false;
            this.nao = nao;
            this.lib = lib;
            this.thriftClient = thriftClient;
            this.main = main;
        }

        public void setIdentifer(string identifier)
        {
            this.identifier = identifier;
        }

        public void setProperty(string property)
        {
            this.property = property;
        }

        public void identifyKnownObjects()
        {
            this.stopped = false;
            Point naoLocation = this.thriftClient.locateNao();
            foreach (KeyValuePair<string, PointCloud> pair in lib.getKnownObjects())
            {
                RecogObject obj = lib.getObject(pair.Key);
                nao.walkToObject(pair.Value.Average, naoLocation);
                while (!this.stopped && nao.isWalking()) { Thread.Sleep(1000); }
                if (this.stopped) { cleanUpIdentify(); return; }
                nao.speak("this is a " + pair.Key);
                if (this.stopped) { cleanUpIdentify(); return; }
                string props = "";
                foreach (string property in obj.getProperties())
                {
                    props += property + ", ";
                }

                if (props.Length != 0)
                {
                    nao.speak("it has the following properties");
                    if (this.stopped) { cleanUpIdentify(); return; }
                    nao.speak(props + "and no more");
                    if (this.stopped) { cleanUpIdentify(); return; }
                }
                else
                {
                    nao.speak("I don't know anything about this object");
                    if (this.stopped) { cleanUpIdentify(); return; }
                }

                naoLocation = pair.Value.Average;

                // waits for nao to finish speaking
                System.Threading.Thread.Sleep(4000);
            }
            this.main.switchStates(MainController.State.start);
        }

        private void cleanUpIdentify()
        {
            this.main.switchStates(MainController.State.start);
        }

        public void findObjects()
        {
            this.stopped = false;
            Point naoLocation = this.thriftClient.locateNao();
            foreach (RecogObject obj in lib.getObjects(property))
            {
                PointCloud pc = this.lib.getPointCloud(obj.identifier);
                nao.walkToObject(pc.Average, naoLocation);
                while (!this.stopped && nao.isWalking()) { Thread.Sleep(1000); }
                if (this.stopped) { cleanUpFind(); return; }
                nao.speak("this is a " + obj.identifier);
                if (this.stopped) { cleanUpFind(); return; }
                nao.speak("it has the property " + property);
                if (this.stopped) { cleanUpFind(); return; }

                naoLocation = pc.Average;

                // waits for nao to finish speaking
                System.Threading.Thread.Sleep(4000);
            }
            this.main.switchStates(MainController.State.start);
        }

        private void cleanUpFind()
        {
            this.main.switchStates(MainController.State.start);
        }

        public void locateObject()
        {
            this.stopped = false;
            RecogObject obj = this.lib.getObject(this.identifier);
            Point naoLocation = this.thriftClient.locateNao();
            PointCloud pc = this.lib.getPointCloud(obj.identifier);
            nao.walkToObject(pc.Average, naoLocation);
            while (!this.stopped && nao.isWalking()) { Thread.Sleep(1000); }
            if (this.stopped) { cleanUpFind(); return; }
            nao.speak("this is a " + obj.identifier);
            if (this.stopped) { cleanUpFind(); return; }

            naoLocation = pc.Average;

            // waits for nao to finish speaking
            System.Threading.Thread.Sleep(4000);
            this.main.switchStates(MainController.State.start);
        }

        private void cleanUpLocate()
        {
            this.main.switchStates(MainController.State.start);
        }

        public void learnUnknownObjects()
        {
            this.stopped = false;
            this.lib.loadPointClouds(this.thriftClient.getObjects());
            PointCloud obj = this.lib.learnObject();
            if (obj == null)
            {
                nao.speak("I'm done learning");
                if (this.stopped) { cleanUpLearn(); return; }
                this.main.switchStates(MainController.State.start);
            }
            else
            {
                Console.WriteLine("learning: " + obj.Identifier);
                Point naoLocation = this.thriftClient.locateNao();
                nao.walkToObject(obj.Average, naoLocation);
                while (!this.stopped && nao.isWalking()) { Thread.Sleep(1000); }
                if (this.stopped) { cleanUpLearn(); return; }
                nao.speak("what is this object called?");
                if (this.stopped) { cleanUpLearn(); return; }
                this.main.switchStates(MainController.State.getName);
            }
        }

        private void cleanUpLearn()
        {
            this.lib.cancelLearning();
            this.main.switchStates(MainController.State.start);
        }

        public void stop()
        {
            this.stopped = true;
            this.nao.stop();
        }
    }
}
