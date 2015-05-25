#include <ros/ros.h>
#include <mysql/mysql.h>
#include "humans_msgs/DatabaseSrv.h"
#include "humans_msgs/HumanImgSrv.h"
#include <visualization_msgs/Marker.h>
#include "humans_msgs/PersonPoseImgArray.h"
#include "okao_client/OkaoStack.h"
#include "humans_msgs/Int32Srv.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "picojson.h"

using namespace std;
using namespace picojson;

class Relate
{
public:
  int state;
  int okao_id;
  double hist;
  string time_stamp;
  geometry_msgs::Point p;
};


class RelationMining
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;
  
  ros::ServiceServer relation_srv; 
  ros::Publisher relation_pub;
  /*
  ros::ServiceClient joints_last;
  ros::ServiceServer joints_stream;
  ros::Publisher viz_pub;
  ros::Publisher pps_pub;
  */

  MYSQL *connector;
  MYSQL_RES *res_o, *res_t, *res_d;
  MYSQL_ROW row_o, row_t, row_d;

  //int date_req_num;
  vector< vector<Relate> >mining_data;

  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbtable;

  time_t now;
  struct tm *pnow;


public:
  RelationMining()
  /*:
    nh_param("~"),
    dbhost("127.0.0.1"),
    dbuser("root"),
    dbpass("robot15"),
    dbname("data1"),
    dbtable("201551")*/
  {
    //init parameter
    /*
    nh_param.param("dbhost", dbhost, dbhost);
    nh_param.param("dbuser", dbuser, dbuser);
    nh_param.param("dbpass", dbpass, dbpass);
    nh_param.param("dbname", dbname, dbname);
    nh_param.param("dbtable", dbtable, dbtable);
    */
    nh_param.param("dbhost", dbhost, string("127.0.0.1"));
    nh_param.param("dbuser", dbuser, string("root"));
    nh_param.param("dbpass", dbpass, string("robot15"));
    nh_param.param("dbname", dbname, string("data1"));
    nh_param.param("dbtable", dbtable, string("201551"));

    //db_sub = nh.subscribe("/humans/recog_info", 1, 
    //			  &PeoplePositionDatabase::dbCallback, this);

    relation_srv
      = nh.advertiseService("relation",
			    &RelationMining::relationRequest, this);
    relation_pub
      = nh.advertise<std_msgs::String>("/json_listener",10);

    //date_req_num = 0;

    init();

  }

  ~RelationMining()
  {
    ROS_ASSERT(shutdownDBConnector());
  }

private:
  void init()
  {
    //dbtable = date();
    ROS_ASSERT(initDBConnector());
  }
  
  string date()
  {
    now = time(NULL);
    pnow = localtime(&now);
    stringstream date_t;
    
    date_t << pnow->tm_year+1900 
	   << pnow->tm_mon+1 
	   << pnow->tm_mday;

    return date_t.str();
  }

  bool initDBConnector()
  {
    connector = mysql_init(NULL);
    if(!mysql_real_connect(connector, dbhost.c_str(), dbuser.c_str(), 
			   dbpass.c_str(), dbname.c_str(), 3306, 
			   NULL, CLIENT_MULTI_STATEMENTS)) 
      {
	fprintf(stderr, "%s\n", mysql_error(connector));
	return false;
      }
    ROS_INFO("connetor : init ok");
    return true;
  }

  bool shutdownDBConnector()
  {
    mysql_close(connector);
    ROS_INFO("MySQL closed.");
    return true;
  }

  /*
    SELECT time_stamp 

    select_query << "SELECT * FROM " 
		 << dbtable.c_str() 
		 << " WHERE state = 2 "  
		 << " ORDER BY time_stamp DESC LIMIT 1 ;";

   */

  bool relationRequest(humans_msgs::Int32Srv::Request &rq,
		       humans_msgs::Int32Srv::Response &rs)
  {
    cout << "okao_id: " << rq.n <<endl;
    stringstream okao_query;

    okao_query << "SELECT DISTINCT tracking_id FROM "
	       << "`"
	       << dbtable.c_str()
	       << "`"  
	       <<" where state = 2 and okao_id = "
	       << rq.n
	       <<";";

    if( mysql_query( connector, okao_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
	return false;
      }

    ROS_INFO("QUERY: %s", okao_query.str().c_str());

    if( res_o = mysql_store_result( connector ) )
      {
	//map< int , map<long long, Relate> >mining_data;

	int num_fields = mysql_num_rows( res_o );
	cout << "n:" << num_fields << endl;
	while( row_o = mysql_fetch_row( res_o ) )
	  {
	    trackingRequest( row_o );
	  }
	std_msgs::String msg;
	msg.data = jsonInput( mining_data, rq.n );
	cout << msg << endl;
	relation_pub.publish( msg );
	mining_data.clear();
      }
    return true;
  }

  string jsonInput(vector< vector<Relate> > data, int master)
  {
    string output;
    picojson::object elem, nodes, edges;
    picojson::array node_array, edge_array;
    vector< vector<Relate> >::iterator it1 = data.begin();

    while( it1 != data.end() )
      {
	vector<Relate>::iterator it2 = it1->begin();
	//stringstream node_data;
	//data << "["; 

	while( it2 != it1->end() )
	  {
	    //node_data << "{ 'data': { 'id': '" << it2->okao_id <<"' } },";
	    stringstream ss;
	    ss << it2->okao_id;
	    picojson::object node_data, a_node;
	    node_data.insert(make_pair("id", ss.str()));
	    a_node.insert(make_pair("data",node_data));
	    node_array.push_back(value(a_node));

	    if(master!=it2->okao_id)
	      {
		stringstream edgeId, masterId;
		masterId << master;
		edgeId << masterId.str() << "-" << ss.str();
		picojson::object edge_data, a_edge;
		edge_data.insert(make_pair("id", edgeId.str()));
		edge_data.insert(make_pair("source", masterId.str()));
		edge_data.insert(make_pair("target", ss.str()));
		a_edge.insert(make_pair("data",edge_data));
		edge_array.push_back(value(a_edge));
	      }
	    ++it2;
	  }
	++it1;
      }

    elem.insert(make_pair("nodes", node_array));
    elem.insert(make_pair("edges", edge_array));
    value json = value(elem);
    return json.serialize();
  }

  void trackingRequest( MYSQL_ROW row )
  {
    long long tracking_id = atoll( row[0] );
    cout << "tracking_id: "<< tracking_id << endl;
    stringstream tracking_query;
    tracking_query << "SELECT DISTINCT time_stamp FROM "
		   << "`"
		   << dbtable.c_str()
		   << "`"
		   << " WHERE tracking_id = "
		   << tracking_id 
		   << " ORDER BY time_stamp"
		   << ";"; 

    if( mysql_query( connector, tracking_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    ROS_INFO("QUERY: %s", tracking_query.str().c_str());

    if( res_t = mysql_store_result( connector ) )
      {
	int num_fields = mysql_num_rows( res_t );//取得した列の数
	cout << "n:" << num_fields << endl;
	int i = 0;
	string start, end;
	if( num_fields )
	  {
	    while(row_t = mysql_fetch_row(res_t))
	      {
		if( i == 0 )
		  {
		    start = row_t[0];
		  }
		if( i == num_fields - 1)
		  {
		    end = row_t[0];
		  }
		++i;
	      }
	    datetimeRequest(start, end);
	  }
      }
  }

  void datetimeRequest(string start, string end)
  {
    cout << "between ("<< start << "---" << end << ")" << endl;
    stringstream datetime_query;
    datetime_query << "SELECT * FROM "
		   << "`"
		   << dbtable.c_str()
		   << "`"
		   << " WHERE time_stamp BETWEEN "
		   << "'"
		   << start.c_str()
		   << "'"
		   << " and "
		   << "'"
		   << end.c_str()
		   << "'"
		   << ";"; 

    ROS_INFO("QUERY: %s", datetime_query.str().c_str());

    if( mysql_query( connector, datetime_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    map< long long, Relate > id_relation;

    if( res_d = mysql_store_result( connector ) )
      {
	int num_fields = mysql_num_rows( res_d );//取得した列の数
	cout << "n:" << num_fields << endl;
      	if( num_fields )
	  { 
	    while(row_d = mysql_fetch_row( res_d ))
	      {
		//cout << row_d[0] << endl;
		Relate a_relation;
 
		cout << row_d[0] << ", " << row_d[1] << ", "
		     << row_d[2] << ", " << row_d[3] << ", "
		     << row_d[4] << ", " << row_d[5] << ", " 
		     << row_d[7] << endl;

		//if( atoi(row[0]) )
		//  {
		    long long t_id = atoll(row_d[7]);
		    
		    a_relation.state = atoi(row_d[0]);
		    a_relation.okao_id = atoi(row_d[1]);
		    a_relation.hist = atof(row_d[2]);
		    a_relation.time_stamp = row_d[3];
		    
		    if(id_relation[t_id].hist < a_relation.hist)
		      {
			id_relation[t_id] = a_relation;
		      }
		    //  }
		//if(id_relation[row_d].hist < a_relation.hist)

	      }
	    //cout << "id: "<< 

	    map< long long, Relate >::iterator it = id_relation.begin();
	    vector<Relate> relate_array;
	    while( it != id_relation.end())
	      {
		cout <<"tracking_id: " <<it->first 
		     << ", okao_id: " <<it->second.okao_id 
		     <<", hist: " << it->second.hist << endl; 

		relate_array.push_back( it->second );
		++it;
	      }

	    mining_data.push_back( relate_array );
	  }
      }
    else
      cout << "result: " << mysql_store_result( connector ) << endl;
  } 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "relation_mining");
  RelationMining rm;

  ros::spin();
  /*
  //ros::Rate loop(10);

  while(ros::ok())
    { 
      jc.jointsCall();
      ros::spinOnce();
      //loop.sleep();
    }
  */
  return 0;
}
