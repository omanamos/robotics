using System;
using System.Collections.Generic;
using Thrift.Server;
using Thrift.Transport;

namespace Communication
{
    public class RpcHanlder : Rpc.Iface
    {

        public void ping()
        {
            Console.WriteLine("ping()");
        }

        public List<PointCloud> getObjects()
        {
            List<PointCloud> l = new List<PointCloud>();

            PointCloud pc = new PointCloud();
            pc.Average = new Point();
            pc.Average.X = 0.5;
            pc.Average.Y = -0.5;
            pc.Average.Z = 0.0;
            pc.Identifier = "box";

            PointCloud pc2 = new PointCloud();
            pc2.Average = new Point();
            pc2.Average.X = 0.5;
            pc2.Average.Y = 0.5;
            pc2.Average.Z = 0.0;
            pc2.Identifier = "ball";

            l.Add(pc);
            l.Add(pc2);

            return l;
        }

        public Point locateNao()
        {
            Point p = new Point();
            return p;
        }

    }

    public class CSharpServer
    {
        public static void Start()
        {
            try
            {
                RpcHanlder handler = new RpcHanlder();
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
