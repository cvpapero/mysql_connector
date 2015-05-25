#include <ros/ros.h>
#include "humans_msgs/DatabaseSrv.h"
#include "humans_msgs/HumanImgSrv.h"
#include <visualization_msgs/Marker.h>
#include "humans_msgs/PersonPoseImgArray.h"
#include "okao_client/OkaoStack.h"
#include "humans_msgs/Int32.h"

using namespace std;

vector<int> okao_id;

class JointsClient
{
private:
  ros::NodeHandle n;
  ros::ServiceClient srv; 
  ros::ServiceServer joints_last;
  ros::ServiceServer joints_stream;
  ros::Publisher viz_pub;
  ros::Publisher pps_pub;

  vector<geometry_msgs::Point> joints;

public:
  JointsClient()
  {
    //ros::Rate loop(10);
    srv = n.serviceClient<humans_msgs::HumanImgSrv>("okao_srv");

    viz_pub 
      = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    pps_pub 
      = n.advertise<humans_msgs::PersonPoseImgArray>("/v_human", 10);

    joints_last
      = n.advertiseService("joints_last_srv",
			   &JointsClient::jointsLastRequest, this);
    /*
    okao_id.push_back(1);
    okao_id.push_back(2);
    okao_id.push_back(3);
    okao_id.push_back(4);
    okao_id.push_back(5);
    okao_id.push_back(6);
    okao_id.push_back(7);
    okao_id.push_back(8);
    okao_id.push_back(9);
    okao_id.push_back(10);
    okao_id.push_back(11);
    okao_id.push_back(12);
    okao_id.push_back(13);
    */
    //joints_stream
    //  = n.advertiseService("joints_stream_srv",
    //			   &JointsClient::JointsStreamRequest, this);
    //okaoStack
    //  = n.serviceClient<okao_client::OkaoStack>("stack_send");
  }

  ~JointsClient()
  {

  }



  bool jointsLastRequest(humans_msgs::Int32::Request &req,
			 humans_msgs::Int32::Response &res)
  {
    okao_id.clear();
    
    for(int i = 0;  i < req.n.size(); ++i)
      {
	okao_id.push_back(req.n[i]);
	cout << "input okao_id: " << okao_id[i] << endl;
      }
    
    
    return true;
  }    

  void jointsCall()
  {
    //ここをどうするか？
    for(int i = 0; i < okao_id.size(); ++i)
      {

	humans_msgs::HumanImgSrv hs;
	//hs.request.rule = rule;
       	
	hs.request.src.max_okao_id = okao_id[i];
	
	//okao_client::OkaoStack okao_img;
	//okao_img.request.person.okao_id = okao_id;
	
	if( srv.call( hs ) )
	  {	
	    //cout << "okao_id:" << okao_id[i] << ", x,y =" 
	    //	 << hs.response.dst.p.x << ","<< hs.response.dst.p.y <<  endl ;
	    markerPublisher( hs.response.dst, hs.response.img );
	  }
	else
	  {
	    cout << "not found!"<<endl;
	  }
      }


  }


  void jointInput(vector<geometry_msgs::Point> joints, vector<geometry_msgs::Point> *line_points)
  {

    line_points->push_back(joints[0]);
    line_points->push_back(joints[1]);

    line_points->push_back(joints[1]);
    line_points->push_back(joints[20]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[2]);

    line_points->push_back(joints[2]);
    line_points->push_back(joints[3]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[8]);

    line_points->push_back(joints[8]);
    line_points->push_back(joints[9]);

    line_points->push_back(joints[9]);
    line_points->push_back(joints[10]);

    line_points->push_back(joints[10]);
    line_points->push_back(joints[11]);

    line_points->push_back(joints[11]);
    line_points->push_back(joints[23]);

    line_points->push_back(joints[11]);
    line_points->push_back(joints[24]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[4]);

    line_points->push_back(joints[4]);
    line_points->push_back(joints[5]);

    line_points->push_back(joints[5]);
    line_points->push_back(joints[6]);

    line_points->push_back(joints[6]);
    line_points->push_back(joints[7]);

    line_points->push_back(joints[7]);
    line_points->push_back(joints[21]);

    line_points->push_back(joints[7]);
    line_points->push_back(joints[22]);

    line_points->push_back(joints[0]);
    line_points->push_back(joints[16]);

    line_points->push_back(joints[16]);
    line_points->push_back(joints[17]);

    line_points->push_back(joints[17]);
    line_points->push_back(joints[18]);

    line_points->push_back(joints[18]);
    line_points->push_back(joints[19]);

    line_points->push_back(joints[0]);
    line_points->push_back(joints[12]);

    line_points->push_back(joints[12]);
    line_points->push_back(joints[13]);

    line_points->push_back(joints[13]);
    line_points->push_back(joints[14]);

    line_points->push_back(joints[14]);
    line_points->push_back(joints[15]);   

  }

  void markerPublisher(humans_msgs::Human hm, sensor_msgs::Image img)
  {
    //cout << hm << endl;
    joints.clear();
    visualization_msgs::Marker points, line_list;

    points.header.frame_id = line_list.header.frame_id  = "map";
    points.header.stamp = line_list.header.stamp = ros::Time::now();

    points.ns = "points";
    line_list.ns = "lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.id = hm.max_okao_id;
    line_list.id = hm.max_okao_id;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.r = 1.0f;
    points.color.a = 1.0;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0f;
    line_list.color.a = 1.0;
    
    for(int i = 0; i < hm.body.joints.size(); ++i)
      {
	points.points.push_back( hm.body.joints[ i ].position );
	joints.push_back( hm.body.joints[ i ].position );
      }

    vector<geometry_msgs::Point> line_points;
    jointInput( joints, &line_points );

    line_list.points = line_points; 
    

    humans_msgs::PersonPoseImgArray ppia;
    humans_msgs::PersonPoseImg ppi;
    ppi.person.okao_id = hm.max_okao_id;
    ppi.person.hist = hm.max_hist;
    ppi.person.name = hm.face.persons[0].name;
    ppi.pose.position = hm.p;
    ppi.pose.orientation.w = 1;
    ppi.image = img;
    ppi.header.stamp = ros::Time::now();
    ppi.header.frame_id = "map";
    ppia.ppis.push_back( ppi );
    ppia.header.stamp = ros::Time::now();
    ppia.header.frame_id = "map";

    sleep(1);
    viz_pub.publish( points );
    viz_pub.publish( line_list );

    pps_pub.publish( ppia );

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "database_client_joints");
  JointsClient jc;
  //ros::Rate loop(10);

  while(ros::ok())
    { 
      jc.jointsCall();
      ros::spinOnce();
      //loop.sleep();
    }
    return 0;
}
