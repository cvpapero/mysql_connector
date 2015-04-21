/*
2014.4.7----------------
つねに何かをパブリッシュするモジュール

送られてきたサービスによって、その出力を変える。

一旦クリアにする


2015.4.6----------------
データベースから検索を行うモジュール

どうしようかな。どういう仕組みにしようかな。
パラメータのオプションで、表示するデータを変えるようなモジュールにする

0.記録した人物のすべての位置
1.すべての人物の最新の位置
2.特定の個人の位置
3.時間軸的な位置(この時間帯には誰がいたかというような)

複数の検索にも対応したい
つまり、特定の個人の位置と、その個人はある時間帯にはどこにいたかという情報

だが、まずは適当にパブリッシュする機能から

*/

#include <ros/ros.h>
#include <iostream>
#include <mysql/mysql.h>
#include <time.h>

#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "picojson.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>

#include "humans_msgs/PersonPoseImgArray.h"
#include "humans_msgs/Humans.h"
#include "humans_msgs/HumanImgSrv.h"
#include "humans_msgs/HumansSrv.h"

#include "humans_msgs/DatabaseSrv.h"


//#include "Util.hpp"

#include <std_msgs/String.h>
#define MAXOKAO 13
#define JOINT_NUM 25

using namespace std;

humans_msgs::PersonPoseImgArray ppia;
map<int, sensor_msgs::Image> img_stack;
map<int, sensor_msgs::Image> img_stack_gray;
humans_msgs::Humans humans;

class PositionDatabaseServer
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;
  ros::Publisher pps_pub;
  ros::ServiceServer server_srv;
  ros::ServiceServer humans_srv;
  ros::ServiceServer track_srv;
  ros::ServiceServer okao_srv;
  ros::ServiceServer name_srv;

  //ros::ServiceServer particular_person_srv;

  MYSQL *connector;
  MYSQL_RES *res;
  MYSQL_ROW row;

  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbtable;

  time_t now;
  struct tm *pnow;

public:
  PositionDatabaseServer():
    nh_param("~"),
    dbhost("127.0.0.1"),
    dbuser("root"),
    dbpass("robot15"),
    dbname("data1"),
    dbtable("mapping_db_test")
  {
    //init parameter
    nh_param.param("dbhost", dbhost, dbhost);
    nh_param.param("dbuser", dbuser, dbuser);
    nh_param.param("dbpass", dbpass, dbpass);
    nh_param.param("dbname", dbname, dbname);
    nh_param.param("dbtable", dbtable, dbtable);

    pps_pub = nh.advertise<humans_msgs::PersonPoseImgArray
			   >("/a_human", 10);

    server_srv 
      = nh.advertiseService("database_server_srv", 
			    &PositionDatabaseServer::ServerService, this);
    humans_srv 
      = nh.advertiseService("humans_srv", 
			    &PositionDatabaseServer::particularPersonPositionSrv, this);
    track_srv 
      = nh.advertiseService("track_srv", 
			  &PositionDatabaseServer::resTrackingId, this);
    okao_srv 
      = nh.advertiseService("okao_srv", 
			   &PositionDatabaseServer::resOkaoId, this);
     name_srv 
       = nh.advertiseService("name_srv", 
			    &PositionDatabaseServer::resName, this);

    ROS_ASSERT(initDBConnector());
    ROS_ASSERT(initDBImage());
  }
  ~PositionDatabaseServer()
  {
    ROS_ASSERT(shutdownDBConnector());
    ROS_ASSERT(shutdownDBImage());
  }

private:
  bool initDBConnector()
  {
    ROS_INFO("connetor : init ok");

    connector = mysql_init(NULL);
    if(!mysql_real_connect(connector, 
			   dbhost.c_str(), 
			   dbuser.c_str(), 
			   dbpass.c_str(), 
			   dbname.c_str(), 
			   3306, 
			   NULL, 
			   CLIENT_MULTI_STATEMENTS)) 
      {
	fprintf(stderr, "%s\n", mysql_error(connector));
	return false;
      }
    ROS_INFO("MySQL opend.");
    return true;
  }

  bool initDBImage()
  {
    for(int i = 1 ; i <= MAXOKAO ; ++i)
      {
	try
	  {
	    stringstream image_name;
	    image_name 
	      <<"/home/uema/outimage/images/okao/okao" 
	      << i << ".jpg";
	    cv::Mat src = cv::imread(image_name.str());
	    //cout << "input: " << image_name.str() << endl;
	    sensor_msgs::Image output, output_gray;
	    //cv::resize(output, output, cv::Size(128,128));
	    output.height = src.rows; 
	    output.width = src.cols;
	    output.encoding = "bgr8";
	    output.step 
	      = src.cols * src.elemSize();
	    output.data.assign(src.data, src.data + size_t(src.rows*src.step));

	    img_stack[ i ] = output;
	    
	    cv::Mat src_gray;
	    cvtColor(src, src_gray, CV_BGR2GRAY);

	    output_gray.height = src_gray.rows; 
	    output_gray.width = src_gray.cols;
	    output_gray.encoding = "mono8";
	    output_gray.step 
	      = src_gray.cols * src_gray.elemSize();
	    output_gray.data.assign(src_gray.data, src_gray.data + size_t(src_gray.rows*src_gray.step));

	    img_stack_gray[ i ] = output_gray;
	  }
	catch(cv::Exception& e)
	  {
	    std::cout << e.what() << std::endl;
	    return false; 
	  }
      }
    return true; 
  }

  bool shutdownDBConnector()
  {
    mysql_close(connector);
    ROS_INFO("MySQL closed.");
    return true;
  }

  bool shutdownDBImage()
  {
    img_stack.clear();
    img_stack_gray.clear();
    return true;
  }

public:
  //サービスが呼び出されたら次の関数をハブにする
  bool ServerService(humans_msgs::DatabaseSrv::Request &req,
		     humans_msgs::DatabaseSrv::Response &res)
  {
    /*
      特定の個人について、
      条件(name, labo, grade, time)による検索について
      すべての人物の最新の位置について
    */
    if( req.rule == "p" )
      {
	particularPersonPosition( req.person.okao_id ); 
      }
    else if( req.rule == "l" )
      {
	lastPeoplePosition();
      }
    else
      {
	ROS_INFO("no match rule!");
	return false;
      }

    return true;
  }

  //すべての人物の最新位置
  void lastPeoplePosition()
  {
    clearPeoplePoseImgArray();

    for(int i = 1; i <= MAXOKAO ; ++i)
      {
	stringstream select_query;
	select_query << "SELECT hist, max(time_stamp)," 
		     <<" name, laboratory, grade, tracking_id, px, py, pz, magni FROM "
		     << dbtable.c_str() 
		     << " WHERE okao_id = " << i << " ;";
	
	if( mysql_query( connector, select_query.str().c_str() ) )
	  {
	    ROS_ERROR("%s", mysql_error(connector));
	    ROS_ERROR("DB select error.");
	  }
	else
	  {
	    ROS_INFO("QUERY: %s", select_query.str().c_str());	
    
	    res = mysql_store_result( connector );
	    while( (row = mysql_fetch_row(res)) )
	      {
		//rowの中身が存在するかどうか
		if(row[0])
		  {
		    dataStore( row, i );
		  }
	      }
	  }
      }
  }

  //特定の人物位置
  void particularPersonPosition(int okao_id) 
  {
    clearPeoplePoseImgArray();

    stringstream select_query;
    select_query << "SELECT hist, time_stamp, "
		 << "name, laboratory, grade, tracking_id, px, py, pz, magni FROM " 
		 << dbtable.c_str() 
		 << " WHERE okao_id = " << okao_id << " ;";
    //humans_msgs::PersonPoseImg ppi;
    
    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    ROS_INFO("QUERY: %s", select_query.str().c_str());
    //int id = 1;

    res = mysql_store_result( connector );
    //int num_fields = mysql_num_rows( res );//取得した列の数

    while( (row = mysql_fetch_row(res)) )
      {
	if(row[0])
	  dataStore( row, okao_id );
      }

  }

  void clearPeoplePoseImgArray()
  {
    ppia.ppis.clear();
  }

  void dataStore( MYSQL_ROW row, int okao_id )
  {
	  
    //cout << "datastr in" << endl; 
    humans_msgs::PersonPoseImg ppi;
    
    //hist, time_stamp, name, laboratory, grade, tracking_id, px, py, magni 	    
    ppi.person.name = row[2];
    ppi.person.okao_id = okao_id;
    ppi.person.hist = atof( row[0] );
    ppi.pose.position.x = atof( row[6] );
    ppi.pose.position.y = atof( row[7] );
    
    ppi.header.frame_id = "map";
    ppi.header.stamp = ros::Time::now();
    ppi.image = img_stack[ okao_id ];
    ppi.pose.orientation.w = 1;   
    ppia.ppis.push_back( ppi );

    //cout << "datastr out" << endl;    
  }

  void dataStoreSrv( MYSQL_ROW row, humans_msgs::Human *h_res )
  {

    //cout<< "database store" << endl;

    h_res->header.frame_id = "map";
    h_res->header.stamp = ros::Time::now();
    h_res->d_id = atoi( row[0] );
    h_res->max_okao_id = atoi( row[1] );
    h_res->max_hist = atoi( row[2] );
    h_res->p.x = atof( row[8] );
    h_res->p.y = atof( row[9] );
    h_res->p.z = atof( row[10] );
    h_res->magni = atof( row[11] );
    h_res->body.tracking_id = atoll( row[7] );

    humans_msgs::Person per;
    per.name = row[4];
    per.laboratory = row[5];
    per.grade = row[6];
    h_res->face.persons.push_back( per );

    humans_msgs::Body body_tmp;
    jsonToMsg(row[12], &body_tmp);
    h_res->body.joints = body_tmp.joints;
    //cout << "row[11]:";
    //cout << row[11] << endl;
  }

  void jsonToMsg(string row, humans_msgs::Body* body)
  {
    //cout<< "json to msg" << endl;
    picojson::value v;
    string err;
    picojson::parse(v, row.begin(), row.end(), &err);
    
    if( err.empty() )
      {
	picojson::object &obj = v.get<picojson::object>();

	for(int j_num = 0; j_num < JOINT_NUM; ++j_num)
	  {
	    humans_msgs::Joints joint;
	    stringstream j_name;
	    j_name << "joint" << j_num;
	    picojson::object &oj = obj[j_name.str()].get<picojson::object>();

	    joint.joint_name = (string)oj["j_name"].get<string>();
	    joint.tracking_state = (double)oj["t_state"].get<double>();

	    picojson::object &pos = oj["position"].get<picojson::object>();
	    joint.position.x = (double)pos["x"].get<double>();
	    joint.position.y = (double)pos["y"].get<double>();
	    joint.position.z = (double)pos["z"].get<double>();

	    /*
	    picojson::object &ori = oj["orientation"].get<picojson::object>();
	    joint.orientation.x = (double)ori["x"].get<double>();
	    joint.orientation.y = (double)ori["y"].get<double>();
	    joint.orientation.z = (double)ori["z"].get<double>();
	    joint.orientation.w = (double)ori["w"].get<double>();
	    */
	    joint.orientation.w = 1;
	    body->joints.push_back( joint );	  
	  }
      }
  }

  //人物位置のパブリッシュ(メッセージの中のデータを出力する)
  void peoplePositionPublisher()
  {
    //cout << "publish:" << ppia <<endl; 
    ppia.header.stamp = ros::Time::now();
    ppia.header.frame_id = "map";
    pps_pub.publish( ppia );
  }

  //特定の人物位置
  bool particularPersonPositionSrv(humans_msgs::DatabaseSrv::Request &request,
				   humans_msgs::DatabaseSrv::Response &response) 
  {
    //clearPeoplePoseImgArray();
    humans.human.clear();
    stringstream select_query;
    select_query << "SELECT * FROM " 
		 << dbtable.c_str() 
		 << " WHERE okao_id = " << request.person.okao_id << " ;";
    
    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
	return false;
      }

    ROS_INFO("QUERY: %s", select_query.str().c_str());

    if( res = mysql_store_result( connector ) )
      {
	int num_fields = mysql_num_rows( res );//取得した列の数

	//cout<< "get num: "<< num_fields << endl;
	//    int i = 0;
	while(row = mysql_fetch_row(res))
	  {
	    humans_msgs::Human human_res;
	    dataStoreSrv( row, &human_res );
	    response.humans.human.push_back( human_res ); 
	  }
	//response.humans = humans;
	return true;
      }
    else
      {
	return false;
      }
  }

  //クエリを使って探す（まだできていない）
  bool resTrackingId(humans_msgs::HumanImgSrv::Request &request,
		     humans_msgs::HumanImgSrv::Response &response)
  {
    cout<<"tracking_id:"<< request.src.body.tracking_id << endl;
    
    stringstream select_query;
    
    select_query << "SELECT * FROM " 
		 << dbtable.c_str() 
		 << " WHERE okao_id = " << request.src.body.tracking_id 
		 << " ORDER BY time_stamp DESC LIMIT 1 ;";

    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    ROS_INFO("QUERY: %s", select_query.str().c_str());

    if(res = mysql_store_result( connector ))
      {
	int num_fields;
	if( num_fields= mysql_num_rows( res ) )
	  {//取得した列の数
	    //cout << "num fields: "<< num_fields <<endl;
	    //int num_fields = mysql_num_rows( res );//取得した列の数
	    humans_msgs::Human human_res;
	    row = mysql_fetch_row(res);
	    dataStoreSrv( row, &human_res );
	    response.dst = human_res;
	    response.img = img_stack[ request.src.max_okao_id ];
	    return true;
	  }
	else
	  {
	    cout << "tracking_id: "<< request.src.body.tracking_id << ", row_num: "<<num_fields<<endl;
	    return false;
	  }

      }
    else
      {
	cout<<"no tracking_id:"<< request.src.body.tracking_id << endl;
	return false;
      }

  }

  bool resOkaoId(humans_msgs::HumanImgSrv::Request &request,
		 humans_msgs::HumanImgSrv::Response &response)
  {
    cout<<"okao_id:"<< request.src.max_okao_id << endl;

    stringstream select_query;
    
    select_query << "SELECT * FROM " 
		 << dbtable.c_str() 
		 << " WHERE okao_id = " << request.src.max_okao_id
		 << " ORDER BY time_stamp DESC LIMIT 1 ;";
      /*
		 << " WHERE okao_id = " << request.src.max_okao_id
		 << " AND time_stamp = ( select max(time_stamp) from "
		 << dbtable.c_str() 
		 << " );";
      */
    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    ROS_INFO("QUERY: %s", select_query.str().c_str());

    if(res = mysql_store_result( connector ))
      {
	int num_fields;
	if( num_fields= mysql_num_rows( res ) )
	  {//取得した列の数
	    //cout << "num fields: "<< num_fields <<endl;
	    humans_msgs::Human human_res;
	    row = mysql_fetch_row(res);
	    
	    dataStoreSrv( row, &human_res );
	    response.dst = human_res;
	    response.img = img_stack[ request.src.max_okao_id ];
	    cout<<"res okao_id:"<< request.src.max_okao_id << endl;
	    return true;
	  }
	else
	  {
	    cout << "okao_id: "<< request.src.max_okao_id << ", row_num: "<<num_fields<<endl;
	    return false;
	  }
      }
    else
      {
	cout<<"no okao_id:"<< request.src.max_okao_id << endl;
	return false;
      }
  }
 
  
  bool resName(humans_msgs::HumanImgSrv::Request &req,
	       humans_msgs::HumanImgSrv::Response &res)
    
  {

  }
  

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_database_server");
  PositionDatabaseServer PDSObj;
  ros::Rate loop(0.5);
  //PDSObj.lastPeoplePosition();
  while(ros::ok())
    {
      //PDSObj.lastPeoplePosition();
      PDSObj.peoplePositionPublisher();
      ros::spinOnce();
      loop.sleep();
    }
  //

  return 0;
}
