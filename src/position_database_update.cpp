/*
2015.4.20----------------
tracking_idとokao_idを1対1対応させるように書き換えて
別テーブルに保存するモジュール

なぜ必要かというと、検索する際にばらつきがあると、必要なデータを取得できない
もちろん、取得された生データは残しておく

アルゴリズム
1.tracking_idを取得する
2.取得していたトラッキングIDが取得されなくなったら3へ
3.tracking_idをキーにしてそれに結びついたすべての情報を取得
4.それぞれのokao_idについてmagniの最大値を求める
5.最大のmagniを出したokao_idに書き換えて保存

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

//#include "Util.hpp"

#include "okao_client/OkaoStack.h"
//#include "MsgToMsg.hpp"

#include <std_msgs/String.h>
#define HEAD 3
#define THRESHOLD 1.5

using namespace std;

//vector< long long > now;
vector< long long > past;

class PeoplePositionDatabase
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;

  ros::Subscriber db_sub;
  tf::TransformListener tl;

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
  PeoplePositionDatabase():
    nh_param("~"),
    dbhost("127.0.0.1"),
    dbuser("root"),
    dbpass("robot15"),
    dbname("data1")
  {

    //init parameter
    nh_param.param("dbhost", dbhost, dbhost);
    nh_param.param("dbuser", dbuser, dbuser);
    nh_param.param("dbpass", dbpass, dbpass);
    nh_param.param("dbname", dbname, dbname);

    db_sub = nh.subscribe("/humans/recog_info", 1, 
			  &PeoplePositionDatabase::dbCallback, this);

    now = time(NULL);
    pnow = localtime(&now);
    stringstream dbtable_name;
    
    dbtable_name << pnow->tm_year+1900 
		 << pnow->tm_mon+1 
		 << pnow->tm_mday;

    dbtable = dbtable_name.str();

    ROS_ASSERT(initDBConnector());
    ROS_ASSERT(initDBTable());
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
    if(!mysql_real_connect(connector, dbhost.c_str(), 
			   dbuser.c_str(), dbpass.c_str(), 
			   dbname.c_str(), 3306, 
			   NULL, CLIENT_MULTI_STATEMENTS)) 
      {
	fprintf(stderr, "%s\n", mysql_error(connector));
	return false;
      }
    ROS_INFO("MySQL opend.");
    return true;
  }

 bool initDBTable()
  {
    if( monitorTable() )
      {
	ROS_INFO("table exist!!");
      }
    else
      {
	ROS_INFO("miss...");
	ros::shutdown();
      }   
    ROS_INFO("MySQL opend.");
    return true;
  }

  //テーブルが存在するかどうか
  bool monitorTable()
  {
    stringstream show_query;
    show_query << "SHOW TABLES FROM " << dbname.c_str() 
	       << " LIKE '" << dbtable.c_str() << "';";

    if(mysql_query( connector, show_query.str().c_str()))
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB write error.");
	return false;
      }
    if(res = mysql_store_result( connector ))
      {
	int num_fields;
	if( num_fields = mysql_num_rows(res) )
	  return true;
	else
	  return false;
      }  
  }


  bool shutdownDBConnector()
  {
    past.clear();
    mysql_close(connector);
    ROS_INFO("MySQL closed.");
    return true;
  }


  void dbCallback(const humans_msgs::HumansConstPtr& msg)
  {
    vector<long long> now;
    for(int i = 0; i < msg->human.size(); ++i)
      {
	now.push_back(msg->human[i].body.tracking_id);
      }

    for(int i = 0; i < past.size(); ++i)
      {
	//nowの中にpastのtracking_idが存在すれば、true
	//もしなければ、そのpastのIDに結びついたデータはチェックされる
	vector<long long>::iterator check = find(now.begin(), now.end(), past[i]);
	if(check == now.end())
	  {
	    //データチェック関数
	    cout << "tracking_id: "<<past[i]<<" is lost!" << endl;
	    select( past[i] );
	  }
      }
    past.clear();
    past = now;
    now.clear();
  }

  /*
  void update( MYSQL_ROW row )
  {
    //ここでアップデートをかける
    //もしmagni=1.5が過半なら、unknownに。それ以外なら何もしない
    stringstream update_query;
    update_query << "UPDATE "
		 << dbname.c_str()<<"."<<dbtable.c_str()
		 << " SET okao_id = "
		 << okao_id << ", "
		 << "name = "
		 << name << ", "
		 << "laboratory = "
		 << laboratory << ", "
		 << "grade = "
		 << grade 
		 << " WHERE tracking_id = "
		 << tracking_id << ";";
  }
  */
  void dataStore()
  {

  }

  void select(long long tracking_id)
  {
    int unknown = 0;
    map< int, int >okaoId;
    map< int, humans_msgs::Person >per_stack;
    //ここでtracking_idに結びついたすべてのokao_id, name, labo, grade, magniをselectする
    stringstream select_query;
    select_query << "SELECT okao_id, time_stamp, name, laboratory, grade, magni FROM "
		 << dbname.c_str()
		 <<"."
		 << dbtable.c_str()
		 << " WHERE tracking_id = "
		 << tracking_id
		 << " ;";

    if( mysql_query( connector, select_query.str().c_str() ) )
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB select error.");
      }
    ROS_INFO("QUERY: %s", select_query.str().c_str());

    vector<string> datatime;
    humans_msgs::Person tmp;
    double max_magni = 0.0;

    if(res = mysql_store_result( connector ))
      {
	int num_fields;
	if( num_fields = mysql_num_rows( res ) )
	  {
	    while( (row = mysql_fetch_row( res )) )
	      {
		datatime.push_back(row[1]);
		//cout << "row[]" << row[0] << "," << row[2] << "," << row[4] <<endl;
		if( atof(row[5]) > max_magni )
		  {
		    max_magni = atof(row[5]);
		    tmp.okao_id = atoi(row[0]);
		    tmp.name = row[2];
		    tmp.laboratory = row[3];
		    tmp.grade = row[4];
		  }
	      }

	    //アップデート
	    for(int i = 0; i<datatime.size(); ++i)
	      {
		stringstream update_query;
		
		update_query << "UPDATE "
			     << dbname.c_str()<<"."<<dbtable.c_str()
			     << " SET okao_id = "
			     << tmp.okao_id << ", "
			     << "name = '"
			     << tmp.name.c_str() << "', "
			     << "laboratory = '"
			     << tmp.laboratory.c_str() << "', "
			     << "grade = '"
			     << tmp.grade.c_str() 
			     << "' WHERE tracking_id = "
			     << tracking_id << " AND "
			     << "time_stamp = '"
			     << datatime[i].c_str()
			     <<"';";

		if( mysql_query( connector, update_query.str().c_str() ) )
		  {
		    ROS_ERROR("%s", mysql_error(connector));
		    ROS_ERROR("DB select error.");
		  }
		ROS_INFO("QUERY: %s", update_query.str().c_str());
	      }

	  }
      }
    else
      {
	cout<<"no tracking_id:"<< tracking_id << endl;
      }
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_database_update");
  PeoplePositionDatabase PPDObj;
  ros::spin();

  return 0;
}
