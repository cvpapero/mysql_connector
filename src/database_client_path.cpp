#include <ros/ros.h>
#include "humans_msgs/DatabaseSrv.h"
#include "humans_msgs/HumanImgSrv.h"
#include <visualization_msgs/MarkerArray.h>
#include "humans_msgs/PersonPoseImgArray.h"
#include "okao_client/OkaoStack.h"
#include "humans_msgs/Int32.h"
#include <nav_msgs/Path.h>

using namespace std;

vector<int> okao_id;
nav_msgs::Path path;
map<int, int> id_and_num;

class PathClient
{
private:
  ros::NodeHandle n;
  ros::ServiceClient srv; 
  ros::ServiceClient okaoStack;
  ros::ServiceServer okaoIdSrv;
  ros::Publisher path_pub;
  ros::Publisher line_pub;

public:
  PathClient()
  {
    srv 
      = n.serviceClient<humans_msgs::DatabaseSrv>("humans_srv");
    path_pub 
      = n.advertise<nav_msgs::Path>("/human_path", 1);
    line_pub 
      = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    okaoIdSrv
      = n.advertiseService("okao_id_srv",
			   &PathClient::Request, this);
  }

  ~PathClient()
  {

  }



  bool Request(humans_msgs::Int32::Request &req,
	       humans_msgs::Int32::Response &res)
  {
    okao_id.clear();
    path.poses.clear();
    for(int i = 0;  i < req.n.size(); ++i)
      {
	//okao_id.push_back(req.n[i]);
	//cout << "input okao_id: " << okao_id[i] << endl;

	humans_msgs::DatabaseSrv hs;
	
	hs.request.person.okao_id = req.n[i];

	if( srv.call( hs ) )
	  {	
	    pathPublisher( hs.response.humans );
	  }
	else
	  {
	    cout << "not found!"<<endl;
	  }
      }

    return true;
  }    

  void pathCall()
  {
    for(int i = 0; i < okao_id.size(); ++i)
      {
	humans_msgs::DatabaseSrv hs;
	hs.request.person.okao_id = okao_id[i];
	if( srv.call( hs ) )
	  {	
	    pathPublisher( hs.response.humans );
	  }
	else
	  {
	    cout << "not found!"<<endl;
	  }
      }
  }


  void pathPublisher(humans_msgs::Humans hms)
  {

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    visualization_msgs::MarkerArray line_paths;

    visualization_msgs::Marker line_path;

    line_path.header.frame_id = "map";
    line_path.header.stamp = ros::Time::now();
    line_path.ns = "line_path";
    line_path.action = visualization_msgs::Marker::ADD;
    line_path.pose.orientation.w = 1;
    line_path.type = visualization_msgs::Marker::LINE_STRIP;
    line_path.scale.x = 0.01;
    //line_path.color.b = 1.0;
    line_path.color.a = 1.0;

    int now_d_id = 0;
    int d_id_num = 0;
    for(int i = 0; i < hms.human.size(); ++i)
      {
	if(now_d_id == hms.human[i].d_id)
	  ++id_and_num[hms.human[i].d_id];
	else
	  ++d_id_num;
      }

    for(int i = 0; i < hms.human.size(); ++i)
      {
	int tmp = i % 3;
	//cout << "d_id_add: "<< d_id_add << ", tmp: "<< tmp << endl;
	if(tmp == 0)
	  {
	    line_path.color.r = 1;
	    line_path.color.g = 0;
	    line_path.color.b = 0;
	  }
	else if(tmp == 1)
	  {
	    line_path.color.r = 0;
	    line_path.color.g = 1;
	    line_path.color.b = 0;
	  }
	else
	  {
	    line_path.color.r = 0;
	    line_path.color.g = 0;
	    line_path.color.b = 1;
	  }

	for(int j = 0; j < id_and_num[ hms.human[i].d_id ]; ++j)
	  {
	    geometry_msgs::Point pt;
	    pt.x = hms.human[ i*d_id_num + j ].p.x;
	    pt.y = hms.human[ i*d_id_num + j ].p.y;
	    pt.z = 0.0;
	    line_path.points.push_back( pt );
	    line_path.id = i;
	  }

	line_paths.markers.push_back(line_path);

      }

	/*
	geometry_msgs::PoseStamped pose;

	pose.header.frame_id = "map";
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = hms.human[ i ].p.x;
	pose.pose.position.y = hms.human[ i ].p.y;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.w = 1.0;
	path.poses.push_back( pose );
	*/
	//joints.push_back( hm.body.joints[ i ].position );

	//id change
	/*
	if(now_d_id == hms.human[i].d_id)
	  {
	    //
	    geometry_msgs::Point pt;
	    pt.x = hms.human[ i ].p.x;
	    pt.y = hms.human[ i ].p.y;
	    pt.z = 0.0;
	    line_path.points.push_back( pt );
	  }
	else
	  {
	    //line_paths.marker.push_back( line_path );

	  }

	*/  /*
	    cout << "d_id: "<< now_d_id << endl;
	    now_d_id = hms.human[i].d_id;
	    ++d_id_add;

	    int tmp = d_id_add % 3;

	    cout << "d_id_add: "<< d_id_add << ", tmp: "<< tmp << endl;
	    if(tmp == 0)
	      {
		line_path.color.r = 1;
		line_path.color.g = 0;
		line_path.color.b = 0;
	      }
	    else if(tmp == 1)
	      {
		line_path.color.r = 0;
		line_path.color.g = 1;
		line_path.color.b = 0;
	      }
	    else
	      {
		line_path.color.r = 0;
		line_path.color.g = 0;
		line_path.color.b = 1;
	      }

	    line_paths.markers.push_back( line_path );
	    */

	  
	    //line_path.id = d_id_add;

	  


    
    
    sleep(1);
    //path_pub.publish( path );
    line_pub.publish( line_paths );

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "database_client_path");
  PathClient pc;

  /*
  while(ros::ok())
    { 
      pc.pathCall();
      ros::spinOnce();
    }
  */
  ros::spin();
  return 0;
}
