using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ServiceModel;
using System.ServiceModel.Channels;
using System.ServiceModel.Description;
using uwb;
namespace Server
{
    class Program
    {
        static void Main(string[] args)
        {
            ServiceHost sh = new ServiceHost(typeof(Service1));
            sh.Open();
            Console.WriteLine("Running");
            Console.ReadLine();
            sh.Close();
        }
    }
}