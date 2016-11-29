using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.ServiceModel.Web;
using System.Text;

namespace uwb
{
    // NOTE: You can use the "Rename" command on the "Refactor" menu to change the class name "Service1" in code, svc and config file together.
    // NOTE: In order to launch WCF Test Client for testing this service, please select Service1.svc or Service1.svc.cs at the Solution Explorer and start debugging.
    public class Service1 : IService1
    {
        public string GetData()
        {
            return string.Format("You entered: {0}","testyey");
        }

        public Position[] GetPositions()
        {
            Position p = new Position();
            p.X = 10;
            p.Y = 15;
            p.DateTime = "10 jan 2016";

            Position p2 = new Position();
            p2.X = 23;
            p2.Y = 14;
            p2.DateTime = "13 jan 2017";
            Position[] positions = new Position[2];
            positions[0] = p;
            positions[1] = p2;
            return positions;
        }
    }
}
