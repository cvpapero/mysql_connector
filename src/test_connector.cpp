#include <iostream>
#include <ros/ros.h>
#include <mysql/mysql.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sql_test");
  ros::NodeHandle n;
  MYSQL *mysql;
  mysql=mysql_init(NULL);
  if(NULL==mysql)
    {
      cout << "init miss!" <<endl;
    }
  string hostname = "127.0.0.1";
  string username = "root";
  string password = "robot15";
  string database = "data1";
  int  portnumber = 3306;
  if (!mysql_real_connect(mysql, hostname.c_str(), username.c_str(), password.c_str(), database.c_str(),3306, NULL, CLIENT_MULTI_STATEMENTS) )
    {
      // 接続エラー
      printf("error: %s\n", mysql_error(mysql));
    } 
    else 
      {
	//  接続成功
      cout << "connect ok!" <<endl;
      }
  return 0;    
}
