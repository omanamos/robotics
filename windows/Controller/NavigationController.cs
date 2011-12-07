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
        private volatile string identifier;
        private volatile string property;
        private volatile bool stopped;

        public NavigationController(MainController main, NaoController nao, ObjectLibrary lib)
        {
            this.stopped = false;
            this.nao = nao;
            this.lib = lib;
            this.main = main;
        }

        public void setIdentifier(string identifier)
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
            Point naoLocation = this.main.locateNao();
            foreach (KeyValuePair<string, PointCloud> pair in lib.getKnownObjects())
            {
                RecogObject obj = lib.getObject(pair.Key);
                nao.walkToObject(pair.Value.Average, naoLocation);
                do { Thread.Sleep(1000); } while (!this.stopped && nao.isWalking());
                if (this.stopped) { return; }
                nao.speak("this is a " + pair.Key);
                if (this.stopped) { return; }
                string props = "";
                foreach (string property in obj.getProperties())
                {
                    props += property + ", ";
                }

                if (props.Length != 0)
                {
                    nao.speak("it has the following properties");
                    if (this.stopped) { return; }
                    nao.speak(props + "and no more");
                    if (this.stopped) { return; }
                }
                else
                {
                    nao.speak("I don't know anything about this object");
                    if (this.stopped) { return; }
                }

                naoLocation = this.main.locateNao();

                // waits for nao to finish speaking
                System.Threading.Thread.Sleep(4000);
            }
            this.main.switchStates(MainController.State.start);
        }

        public void findObjects()
        {
            this.stopped = false;
            Point naoLocation = this.main.locateNao();
            foreach (RecogObject obj in lib.getObjects(property))
            {
                PointCloud pc = this.lib.getPointCloud(obj.identifier);
                nao.walkToObject(pc.Average, naoLocation);
                do { Thread.Sleep(1000); } while (!this.stopped && nao.isWalking());
                if (this.stopped) { return; }
                nao.speak("this is a " + obj.identifier);
                if (this.stopped) { return; }
                nao.speak("it has the property " + property);
                if (this.stopped) { return; }

                naoLocation = this.main.locateNao();

                // waits for nao to finish speaking
                System.Threading.Thread.Sleep(4000);
            }
            this.main.switchStates(MainController.State.start);
        }

        public void locateObject()
        {
            this.stopped = false;
            RecogObject obj = this.lib.getObject(this.identifier);
            Point naoLocation = this.main.locateNao();
            PointCloud pc = this.lib.getPointCloud(obj.identifier);
            nao.walkToObject(pc.Average, naoLocation);
            do { Thread.Sleep(1000); } while (!this.stopped && nao.isWalking());
            if (this.stopped) { return; }
            nao.speak("this is a " + obj.identifier);
            if (this.stopped) { return; }

            // waits for nao to finish speaking
            System.Threading.Thread.Sleep(4000);
            this.main.switchStates(MainController.State.start);
        }

        public void learnUnknownObjects()
        {
            this.stopped = false;
            PointCloud obj = this.lib.learnObject();
            if (obj == null)
            {
                nao.speak("I'm done learning");
                if (this.stopped) { return; }
                this.main.switchStates(MainController.State.start);
            }
            else
            {
                Console.WriteLine("learning: " + obj.Identifier);
                Point naoLocation = this.main.locateNao();
                nao.walkToObject(obj.Average, naoLocation);
                do { Thread.Sleep(1000); } while (!this.stopped && nao.isWalking());
                if (this.stopped) { return; }
                nao.speak("what is this object called?");
                if (this.stopped) { return; }
                this.main.switchStates(MainController.State.getName);
            }
        }

        public void stop()
        {
            this.stopped = true;
            this.nao.stop();
        }
    }
}
