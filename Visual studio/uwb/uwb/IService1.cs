using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.ServiceModel.Web;
using System.Text;

namespace uwb
{
    // NOTE: You can use the "Rename" command on the "Refactor" menu to change the interface name "IService1" in both code and config file together.
    [ServiceContract]
    public interface IService1
    {

        [OperationContract]
        [WebInvoke(Method = "GET",  ResponseFormat = WebMessageFormat.Json,
                                    BodyStyle = WebMessageBodyStyle.Bare,
                                    UriTemplate ="getData/")]
        string GetData();

        //[OperationContract]
        [OperationContract]
        [WebInvoke(Method = "GET", ResponseFormat = WebMessageFormat.Json,
                                   BodyStyle = WebMessageBodyStyle.Bare,
                                   UriTemplate = "getPositions/{sessionId}")]
        Position[] GetPositions(string sessionId);
    }


    // Use a data contract as illustrated in the sample below to add composite types to service operations.
    [DataContract]
    public class Position
    {
        float x = 0;
        float y = 0;
        string dateTime = "not sets";
        int sessionId = 0;

        [DataMember]
        public float X
        {
            get { return x; }
            set { x = value; }
        }
        [DataMember]
        public float Y
        {
            get { return y; }
            set { y = value; }
        }

        [DataMember]
        public string DateTime
        {
            get { return dateTime; }
            set { dateTime = value; }
        }

        [DataMember]
        public int SessionId
        {
            get { return sessionId; }
            set { sessionId = value; }
        }
    }
}
