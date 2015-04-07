/*
2015.4.3----------------
メッセージをサブスクライブして、データベースに書き込むテストモジュール

*/

#include <ros/ros.h>
#include <iostream>
#include <mysql/mysql.h>

#include <std_msgs/String.h>

using namespace std;


class DBConnector
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;

  ros::Subscriber db_sub;

  MYSQL *connector;

  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbdata[100];

public:
  DBConnector():
    nh_priv("~"),
    dbhost("127.0.0.1"),
    dbuser("root"),
    dbpass("robot15"),
    dbname("data1")
  {
    db_sub = nh.subscribe("test_database", 1, &DBConnector::DBCallback, this);
    ROS_ASSERT(initDBConnector());
  }
  ~DBConnector()
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
    /*
    if(mysql_query(connector, "show tables"))
      {
	fprintf(stderr, "%s\n", mysql_error(connector));
	return false;	
      }
    */
    return true;
  }

  bool shutdownDBConnector()
  {
    mysql_close(connector);
    ROS_INFO("MySQL closed.");
    return true;
  }

  void DBCallback(const std_msgs::StringConstPtr& msg)
  {
    char insert_query[1024];
    int i = 1, j = 2;
    sprintf(insert_query,
	    "INSERT INTO data1.table1 (id, name) VALUES (%d, '%s');",
	    i, msg->data.c_str());

    if(mysql_query(connector, insert_query))
      {
	ROS_ERROR("%s", mysql_error(connector));
	ROS_ERROR("DB write error.");
      }

    ROS_INFO("QUERY: %s", insert_query);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_connector");
  DBConnector DBCObj;
  ros::spin();

  return 0;
}
