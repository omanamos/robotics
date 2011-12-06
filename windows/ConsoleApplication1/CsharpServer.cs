using System;
using System.Collections.Generic;
using Thrift.Server;
using Thrift.Transport;
using Communication;

namespace ConsoleApplication1
{
    public class RpcHanlder : Rpc.Iface
    {

        private Dictionary<string, PointCloud> map;
        private PointCloud pc1;
        private PointCloud pc2;

        private int locateCallCount;

        public void init()
        {
            map = new Dictionary<string, PointCloud>();

            this.locateCallCount = 0;

            pc1 = new PointCloud();
            pc1.Average = new Point();
            pc1.Average.X = 1.0;
            pc1.Average.Y = 0.0;
            pc1.Average.Z = 0.0;
            pc1.Identifier = "_item1";
            map[pc1.Identifier] = pc1;

            pc2 = new PointCloud();
            pc2.Average = new Point();
            pc2.Average.X = 1.0;
            pc2.Average.Y = 0.5;
            pc2.Average.Z = 0.0;
            pc2.Identifier = "_item2";
            map[pc2.Identifier] = pc2;
        }

        public void ping()
        {
            Console.WriteLine("ping()");
        }

        public List<PointCloud> getObjects()
        {
            List<PointCloud> l = new List<PointCloud>();
            /*
            PointCloud pc = new PointCloud();
            pc.Average = new Point();
            pc.Average.X = 0.5;
            pc.Average.Y = -0.5;
            pc.Average.Z = 0.0;
            if (unidentifiedCount >= 2)
            {
                pc.Identifier = "_box";
            }
            else
            {
                pc.Identifier = "box";
            }

            PointCloud pc2 = new PointCloud();
            pc2.Average = new Point();
            pc2.Average.X = 0.5;
            pc2.Average.Y = 0.5;
            pc2.Average.Z = 0.0;
            if (unidentifiedCount >= 1)
            {
                pc2.Identifier = "_ball";
            }
            else
            {
                pc2.Identifier = "ball";
            }*/

            l.Add(pc1);
            l.Add(pc2);

            return l;
        }

        public Point locateNao()
        {
            Point p = new Point();
            if (locateCallCount == 0)
            {
                p.X = 0.0;
                p.Y = 0.0;
                p.Z = 0.0;
            }
            else if (locateCallCount == 1)
            {
                p.X = 1.0;
                p.Y = 0.0;
                p.Z = 0.0;
            }
            else
            {
                p.X = 1.0;
                p.Y = 0.5;
                p.Z = 1.57;
            }
            this.locateCallCount++;
            Console.WriteLine("Returning location of nao: ({0}, {1}, {2})", p.X, p.Y, p.Z);
            return p;
        }

        public bool update(string oldIdentifier, string newIdentifier)
        {
            Console.WriteLine("Updating: {0} to {1}.", oldIdentifier, newIdentifier);
            PointCloud pc = this.map[oldIdentifier];
            pc.Identifier = newIdentifier;
            return true;
        }
    }

    public class CSharpServer
    {
        public static void Start()
        {
            try
            {
                RpcHanlder handler = new RpcHanlder();
                handler.init();
                Rpc.Processor processor = new Rpc.Processor(handler);
                TServerTransport serverTransport = new TServerSocket(9090);
                TServer server = new TSimpleServer(processor, serverTransport);

                // Use this for a multithreaded server
                // server = new TThreadPoolServer(processor, serverTransport);

                Console.WriteLine("Starting the server.....");
                server.Serve();
            }
            catch (Exception x)
            {
                Console.WriteLine(x.StackTrace);
            }
            Console.WriteLine("done.");
        }

        static void Main(String[] args)
        {
            Start();
        }
    }
}
