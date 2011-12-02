﻿using System;
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
            pc.Average.X = 0.5;
            pc.Average.Y = 0.5;
            pc.Average.Z = 0.0;
            pc.Identifier = "box";
            l.Add(pc);
            return l;
        }

        public Point locateNao()
        {
            Point p = new Point();
            return p;
        }

        public bool update(string oldIdentifier, string newIdentifier)
        {
            Console.WriteLine("Updating: {1} to {2}.", oldIdentifier, newIdentifier);
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
                Rpc.Processor processor = new Rpc.Processor(handler);
                TServerTransport serverTransport = new TServerSocket(9090);
                TServer server = new TSimpleServer(processor, serverTransport);

                // Use this for a multithreaded server
                // server = new TThreadPoolServer(processor, serverTransport);

                Console.WriteLine("Starting the server.blah..");
                server.Serve();
            }
            catch (Exception x)
            {
                Console.WriteLine(x.StackTrace);
            }
            Console.WriteLine("done.");
        }

        static void main(String[] args)
        {
            Start();
        }
    }
}
