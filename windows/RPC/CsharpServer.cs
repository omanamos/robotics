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

        public string test()
        {
            Console.WriteLine("test()");
            return "Hello World!";
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

                Console.WriteLine("Starting the server...");
                server.Serve();
            }
            catch (Exception x)
            {
                Console.WriteLine(x.StackTrace);
            }
            Console.WriteLine("done.");
        }
    }
}
