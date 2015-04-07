#include <ros/ros.h>
#include "humans_msgs/DatabaseSrv.h"

using namespace std;

int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "database_client");
  ros::NodeHandle n;
  ros::ServiceClient srv;
  ros::Rate loop(10);
  srv = n.serviceClient<humans_msgs::DatabaseSrv>("database_server_srv");

  while(ros::ok())
    { 
      string rule;
      int okao_id;
      cout << "input rule: ";
      cin >> rule;
      cout << "input okao_id: ";
      cin >> okao_id;
 
      humans_msgs::DatabaseSrv ds;
      ds.request.rule = rule;
      ds.request.person.okao_id = okao_id;
      
      srv.call( ds );
      loop.sleep();
    }
    return 0;
}
