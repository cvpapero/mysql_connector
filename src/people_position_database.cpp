/*
2015.4.3----------------
メッセージをサブスクライブして、データベースに書き込むモジュール
現時点では書き込むのみ

*/

#include <ros/ros.h>
#include <iostream>
#include <mysql/mysql.h>
#include <time.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include "picojson.h"

#include "humans_msgs/PersonPoseImgArray.h"
#include "humans_msgs/Humans.h"
#include "humans_msgs/HumanSrv.h"
#include "humans_msgs/HumansSrv.h"

#include "Util.hpp"

#include "okao_client/OkaoStack.h"
//#include "MsgToMsg.hpp"

#include <std_msgs/String.h>

using namespace std;


class PeoplePositionDatabase
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;

  ros::Subscriber db_sub;
  //ros::ServiceServer tracking_id_srv;
  //ros::ServiceServer okao_id_srv;
  //ros::ServiceServer name_srv;

  MYSQL *connector;

  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbtable;
  //string dbdata[100];

  time_t now;
  struct tm *pnow;

public:
  PeoplePositionDatabase():
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

    db_sub = nh.subscribe("/humans/recog_info", 1, 
			  &PeoplePositionDatabase::dbCallback, this);
    /*
    tracking_id_srv 
      = nh.advertiseService("tracking_id_srv", 
			  &PeoplePositionDatabase::resTrackingId, this);
    okao_id_srv 
      = nh.advertiseService("okao_id_srv", 
			    &PeoplePositionDatabase::resOkaoId, this);
    name_srv
      = nh.advertiseService("name_srv", 
			    &PeoplePositionDatabase::resName, this);
    */
    ROS_ASSERT(initDBConnector());
  }
  ~PeoplePositionDatabase()
  {
    ROS_ASSERT(shutdownDBConnector());
  }

private:
  bool initDBConnector()
  {
    ROS_INFO("connetor : init ok");

    connector = mysql_init(NULL);
    if(!mysql_real_connect(connector, dbhost.c_str(), dbuser.c_str(), dbpass.c_str(), dbname.c_str(), 3306, NULL, CLIENT_MULTI_STATEMENTS)) 
      {
	fprintf(stderr, "%s\n", mysql_error(connector));
	return false;
      }

    ROS_INFO("MySQL opend.");

    return true;
  }

  bool shutdownDBConnector()
  {
    mysql_close(connector);
    ROS_INFO("MySQL closed.");
    return true;
  }

  //現在はbody.pのみ
  void transformPosition(humans_msgs::Human hsrc, humans_msgs::Human *hdst)
  {
    //cout << "now looking: " << it_o->second.max_okao_id << endl;
    ros::Time t = ros::Time::now();
    geometry_msgs::PointStamped src;
    src.point = hsrc.p; 
    src.header.stamp = t;
    src.header.frame_id = hsrc.header.frame_id;
    geometry_msgs::PointStamped dst;
    dst.header.stamp = t;
    dst.header.frame_id = hdst->header.frame_id;

    Util::transformHead(src, &dst);
    hdst->p = dst.point;
    hdst->header.stamp = t;
    hdst->header.frame_id = dst.header.frame_id;
    hdst->d_id = hsrc.d_id;
    hdst->max_okao_id = hsrc.max_okao_id;
    hdst->max_hist = hsrc.max_hist;

    //tracking_id
    hdst->body.tracking_id = hsrc.body.tracking_id;

    /*
    for(int j = 0; j < hsrc.body.joints.size() ; ++j)
      hdst->body.joints.push_back(hsrc.body.joints[j]);
    */
    //joints
    hdst->body.joints = hsrc.body.joints;
  }

  void dbCallback(const humans_msgs::HumansConstPtr& msg)
  {
    //cout <<"callback"<<endl;
    now = time(NULL);
    pnow = localtime(&now);

    stringstream time_buff;
    time_buff << pnow->tm_year+1900 
	      <<"-"<< pnow->tm_mon+1 
	      << "-" << pnow->tm_mday
	      << " " << pnow->tm_hour 
	      <<":" << pnow->tm_min 
	      << ":" << pnow->tm_sec; 

    for(int i = 0; i < msg->human.size(); ++i)
      {
	//人物位置の座標変換
	humans_msgs::Human ah;
	ah.header.frame_id = "map";
	ah.header.stamp = msg->header.stamp;

	transformPosition( msg->human[i], &ah );

	//name, labo, gradeの取り出し
	//この取り出しを、上手いこと確定させることはできないだろうか？
	//あるいは、入ってなかったら、そのデータは記録しないとか。
	if(msg->human[i].face.persons.size() != 0)
	  {
	    query_execute( time_buff.str(), ah, msg->human[i] );
	  }
	/*
	else
	  {
	    name = "Unknown";
	    laboratory = "Unknown";
	    grade = "Unknown";
	  }
	*/
      }
  }

  void query_execute(string time_buff, humans_msgs::Human ah, humans_msgs::Human src)
  {
    //cout <<"exe" << endl;
    /*
    string name, laboratory, grade;

    name = msg->human[i].face.persons[0].name;
    laboratory = msg->human[i].face.persons[0].laboratory;
    grade = msg->human[i].face.persons[0].grade;
    */

    string joints_data;
    picojson::object obj_joints;
    //picojson::array positions;
    //stringstream joints_query, joints_data;
    for(int j = 0; j < ah.body.joints.size() ; ++j)
      {
	//picojson::array position, orientation;
	picojson::object position, orientation, joint;//, jx, jy,jz;
	
	joint.insert(make_pair("j_name", ah.body.joints[j].joint_name));
	joint.insert(make_pair("t_state", ah.body.joints[j].tracking_state));

	position.insert(make_pair("x",ah.body.joints[j].position.x));
	position.insert(make_pair("y",ah.body.joints[j].position.y));
	position.insert(make_pair("z",ah.body.joints[j].position.z));
	
	orientation.insert(make_pair("x",ah.body.joints[j].orientation.x));
	orientation.insert(make_pair("y",ah.body.joints[j].orientation.y));
	orientation.insert(make_pair("z",ah.body.joints[j].orientation.z));
	orientation.insert(make_pair("w",ah.body.joints[j].orientation.w));

	joint.insert(make_pair("position", position));
	joint.insert(make_pair("orientation", orientation));

	stringstream j_name;
	j_name << "joint" << j;

	obj_joints.insert(make_pair(j_name.str(), joint));

      }
    picojson::value joints = picojson::value(obj_joints);
    joints_data = joints.serialize();

    stringstream insert_query;
    insert_query << "INSERT INTO "<< dbname.c_str() << "."
		 << dbtable.c_str() 
		 <<" (d_id, okao_id, hist, time_stamp, name, " 
		 << "laboratory, grade, tracking_id, px, py, magni, "
		 << "joints"
		 << ") VALUES ("
		 << ah.d_id<< ", "
		 << ah.max_okao_id << ", " 
		 << ah.max_hist << ", "
		 << "'" << time_buff.c_str() << "'" << ", " 
		 << "'" << src.face.persons[0].name.c_str() << "'" << ", "
		 << "'" << src.face.persons[0].laboratory.c_str() << "'" << ", "
		 << "'" << src.face.persons[0].grade.c_str() << "'" <<", "
		 << ah.body.tracking_id << ", "
		 << ah.p.x <<", " 
		 << ah.p.y <<", "
		 << src.magni << ", "
		 << "'" << joints_data << "'"
		 << ");"; 
    
    if(mysql_query( connector, insert_query.str().c_str()))
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB write error.");
      }
    ROS_INFO("QUERY: %s", insert_query.str().c_str());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_position_database");
  PeoplePositionDatabase PPDObj;
  ros::spin();

  return 0;
}
