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

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>

#include "humans_msgs/PersonPoseImgArray.h"
#include "humans_msgs/Humans.h"
#include "humans_msgs/HumanSrv.h"
#include "humans_msgs/HumansSrv.h"

#include "humans_msgs/DatabaseSrv.h"


#include "Util.hpp"

#include <std_msgs/String.h>
#define MAXOKAO 13

using namespace std;

humans_msgs::PersonPoseImgArray ppia;
map<int, sensor_msgs::Image> img_stack;
map<int, sensor_msgs::Image> img_stack_gray;

class PositionDatabaseServer
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;
  ros::Publisher pps_pub;
  ros::ServiceServer server_srv;
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
    track_srv 
      = n.advertiseService("track_srv", 
			  &PositionDatabaseServer::resTrackingId, this);
    okao_srv 
      = n.advertiseService("okao_srv", 
			   &PositionDatabaseServer::resOkaoId, this);
     name_srv 
       = n.advertiseService("name_srv", 
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
	      <<"/home/uema/catkin_ws/src/okao_client/src/images/okao" 
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
	select_query << "SELECT hist, max(time_stamp), name, laboratory, grade, tracking_id, px, py, magni FROM "
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
    select_query << "SELECT hist, time_stamp, name, laboratory, grade, tracking_id, px, py, magni FROM " 
		 << dbtable.c_str() 
		 << " WHERE okao_id = " << okao_id << " ;";
    //humans_msgs::PersonPoseImg ppi;
    
    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }

    ROS_INFO("QUERY: %s", select_query.str().c_str());
    int id = 1;

    res = mysql_store_result( connector );
    //int num_fields = mysql_num_rows( res );//取得した列の数

    while( (row = mysql_fetch_row(res)) )
      {
	dataStore( row, id );
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

  //人物位置のパブリッシュ(メッセージの中のデータを出力する)
  void peoplePositionPublisher()
  {
    //cout << "publish:" << ppia <<endl; 
    ppia.header.stamp = ros::Time::now();
    ppia.header.frame_id = "map";
    pps_pub.publish( ppia );
  }


  //クエリを使って探す（まだできていない）
  bool resTrackingId(humans_msgs::HumanSrv::Request &req,
		     humans_msgs::HumanSrv::Response &res)
  {
    cout<<"tracking_id:"<< req.src.body.tracking_id << endl;
    
    /*
    //o_DBHuman内から、tracking_idをキーにして検索
    map<int, humans_msgs::Human>::iterator it_o = o_DBHuman.begin();
    while( it_o != o_DBHuman.end() )
      {
	if( it_o->second.body.tracking_id == req.src.body.tracking_id )
	  {
	    humans_msgs::Human h_res;
	    humans_msgs::Body b_res;
	    humans_msgs::Face f_res;
	    h_res.header.frame_id = req.src.header.frame_id;
	    getPerson(it_o->second, &f_res);
	    h_res.max_okao_id = it_o->second.max_okao_id;
	    h_res.max_hist = it_o->second.max_hist;
	    h_res.header = it_o->second.header;
	    
	    h_res.p = it_o->second.p;
	    h_res.face = f_res;
	    res.dst = h_res;
	    cout <<"response name: " << f_res.persons[0].name << endl;
	    return true;
	  }
	++it_o;
      }

    */
    cout<<"no tracking_id:"<< req.src.body.tracking_id << endl;
    return false;
  }

  bool resOkaoId(humans_msgs::HumanSrv::Request &req,
		 humans_msgs::HumanSrv::Response &res)
  {

  }
 
  bool resName(humans_msgs::HumanSrv::Request &req,
	       humans_msgs::HumanSrv::Response &res)
    
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
      PDSObj.peoplePositionPublisher();
      ros::spinOnce();
      loop.sleep();
    }
  //

  return 0;
}
