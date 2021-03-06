#include <ros/ros.h>
#include "humans_msgs/DatabaseSrv.h"
#include "humans_msgs/HumanSrv.h"

using namespace std;

int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "database_client");
  ros::NodeHandle n;
  ros::ServiceClient srv;
  ros::Rate loop(10);
  srv = n.serviceClient<humans_msgs::HumanSrv>("okao_srv");

  while(ros::ok())
    { 
      string rule;
      int okao_id;
      //cout << "input rule: ";
      //cin >> rule;
      cout << "input okao_id: ";
      cin >> okao_id;
 
      humans_msgs::HumanSrv hs;
      //hs.request.rule = rule;
      hs.request.src.max_okao_id = okao_id;
      
      srv.call( hs );

      cout << hs.response.dst << endl;
      loop.sleep();
    }
    return 0;
}
