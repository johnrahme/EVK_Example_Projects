using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using MySql.Data.MySqlClient;
using System.Diagnostics;
namespace uwb
{
    public class Database
    {
        private MySqlConnection connection;
        private string server;
        private string database;
        private string uid;
        private string password;
        public Database()
        {
            server = "localhost";
            database = "uwb";
            uid = "root";
            password = "";
            string connectionString;
            connectionString = "SERVER=" + server + ";" + "DATABASE=" +
            database + ";" + "UID=" + uid + ";" + "PASSWORD=" + password + ";";

            connection = new MySqlConnection(connectionString);
        }

        public MySqlDataReader getReader(string query)
        {
            
            //Create Command
            MySqlCommand cmd = new MySqlCommand(query, connection);
            MySqlDataReader dataReader = cmd.ExecuteReader();
            return dataReader;
        }

        public void open()
        {
            try
            {
                connection.Open();
            }
            catch (MySql.Data.MySqlClient.MySqlException ex)
            {
                Debug.WriteLine(ex.Message);
            }
        }
        public void close()
        {
            try
            {
                connection.Close();
            }
            catch (MySql.Data.MySqlClient.MySqlException ex)
            {
                Debug.WriteLine(ex.Message);
            }
        }
    }
}