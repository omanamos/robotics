using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using ProtoBufRemote;

namespace RPC
{
    class Main
    {
        static void Main(string[] args)
        {
            Int32 port = 13000;
            TcpClient tcpClient = new TcpClient("127.0.0.1", port);

            var controller = new RpcController();
            var client = new RpcClient(controller);

            var channel = new NetworkStreamRpcChannel(controller, tcpClient.GetStream());
            channel.Start();

            IRosService service = client.GetProxy<IRosService>();

            int counter = 0;
            while (true)
            {
                Console.WriteLine("Enter number to test:");
                int x = Int32.Parse(Console.ReadLine());

                bool isPrime;
                if (counter++ % 2 == 0)
                {
                    Console.WriteLine(" Asking server if " + x + " is prime...");
                    isPrime = service.TestPrime(x);
                }
                else
                {
                    Console.WriteLine(" Asking server if " + x + " is prime, using an async query...");
                    IAsyncResult asyncResult = service.BeginTestPrime(x, null, null);
                    Console.WriteLine(" Doing some other stuff while the server is calculating...");
                    isPrime = service.EndTestPrime(asyncResult);
                }

                if (isPrime)
                    Console.WriteLine(" Server says: Prime!");
                else
                    Console.WriteLine(" Server says: Not prime!");
            }
        }
    }
}
