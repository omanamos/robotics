using System;
using Thrift;
using Thrift.Protocol;
using Thrift.Server;
using Thrift.Transport;


namespace Communication
{
    public class CSharpClient
    {
        public static void Start()
        {
            try
            {
                TTransport transport = new TSocket("128.208.7.169", 9090);
                TProtocol protocol = new TBinaryProtocol(transport);
                Rpc.Client client = new Rpc.Client(protocol);

                transport.Open();

                client.ping();
                Console.WriteLine("ping");
                Console.WriteLine(client.getObjects());
                Console.WriteLine("getObjects");

                transport.Close();
            }
            catch (TApplicationException x)
            {
                Console.WriteLine(x.StackTrace);
            }

        }
    }
}
