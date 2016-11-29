using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.ServiceModel.Web;
using System.Text;
using System.Diagnostics;
using MySql.Data.MySqlClient;

namespace uwb
{
    // NOTE: You can use the "Rename" command on the "Refactor" menu to change the class name "Service1" in code, svc and config file together.
    // NOTE: In order to launch WCF Test Client for testing this service, please select Service1.svc or Service1.svc.cs at the Solution Explorer and start debugging.
    public class Service1 : IService1
    {
        Database db = new Database();

        public string GetData()
        {
            db.open();
            string data = "";

            string query = "Select * from positions";

            MySqlDataReader dataReader = db.getReader(query);

            //Read the data and store them in the list
            while (dataReader.Read())
            {
                data = dataReader["y"] + "";
                
            }

            //close Data Reader
            dataReader.Close();

            //close Connection
            db.close();
            return data;
        }

        public Position[] GetPositions(string sessionId)
        {
            List<Position> positions = new List<Position>();
            db.open();
            string data = "";

            string query = "Select * from positions Where sessionId="+ sessionId;

            MySqlDataReader dataReader = db.getReader(query);

            //Read the data and store them in the list
            while (dataReader.Read())
            {
                Position p = new Position();
                p.X = (float)dataReader["x"];
                p.Y = (float)dataReader["y"];
                p.DateTime = dataReader["time"].ToString();
                p.SessionId = (int)dataReader["sessionId"];
                positions.Add(p);


            }

            //close Data Reader
            dataReader.Close();

            //close Connection
            db.close();
           
            return positions.ToArray();
        }
    }
}
