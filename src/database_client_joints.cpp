#include <ros/ros.h>
#include "humans_msgs/DatabaseSrv.h"
#include "humans_msgs/HumanImgSrv.h"
#include <visualization_msgs/Marker.h>
#include "humans_msgs/PersonPoseImgArray.h"
#include "okao_client/OkaoStack.h"

using namespace std;

class JointsClient
{
private:
  ros::NodeHandle n;
  ros::ServiceClient srv; 
  ros::ServiceClient okaoStack;
  ros::Publisher viz_pub;
  ros::Publisher pps_pub;

  vector<geometry_msgs::Point> joints;

public:
  JointsClient()
  {
    //ros::Rate loop(10);
    srv = n.serviceClient<humans_msgs::HumanImgSrv>("okao_srv");

    viz_pub = 
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    pps_pub 
      = n.advertise<humans_msgs::PersonPoseImgArray>("/v_human", 10);

    //okaoStack
    //  = n.serviceClient<okao_client::OkaoStack>("stack_send");
  }

  ~JointsClient()
  {

  }



  void jointsRequest(int okao_id)
  {
    string rule;
    //int okao_id;
    //cout << "input rule: ";
    //cin >> rule;
    cout << "input okao_id: " << okao_id << endl;
    //cin >> okao_id;
    
    humans_msgs::HumanImgSrv hs;
    //hs.request.rule = rule;
    hs.request.src.max_okao_id = okao_id;
    
    //okao_client::OkaoStack okao_img;
    //okao_img.request.person.okao_id = okao_id;

    if( srv.call( hs ) )
      {	
	markerPublisher( hs.response.dst, hs.response.img );
      }
    else
      {
	cout << "not found!"<<endl;
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
    joints.clear();
    visualization_msgs::Marker points, line_list;

    points.header.frame_id = line_list.header.frame_id  = "map";
    points.header.stamp = line_list.header.stamp = ros::Time::now();

    points.ns = "points";
    line_list.ns = "lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.id = 0;
    line_list.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.r = 1.0f;
    points.color.a = 1.0;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0f;
    line_list.color.a = 1.0;

    /*
    double diff_x = hm.p.x - hm.body.joints[3].position.x;
    double diff_y = hm.p.y - hm.body.joints[3].position.y;
    double diff_z = hm.p.z - hm.body.joints[3].position.z;
    */
    /*
    cout <<"p: x,y,z = "<<hm.p.x << ", "<< hm.p.y << ", "<< hm.p.z << endl;
    cout <<"head: x,y,z = "
	 <<hm.body.joints[3].position.x<<","
	 <<hm.body.joints[3].position.y<<","
	 <<hm.body.joints[3].position.z<<endl; 
    cout << "diff: x,y,z = " << diff_x << ", " << diff_y << ", " << diff_z << endl;
    */
    
    for(int i = 0; i < hm.body.joints.size(); ++i)
      {
	/*
	geometry_msgs::Point bp;
	bp.x = hm.body.joints[ i ].position.x + diff_x;
	bp.y = hm.body.joints[ i ].position.y + diff_y;
	bp.z = hm.body.joints[ i ].position.z + diff_z;

	points.points.push_back( bp );
	joints.push_back( bp );
	*/
	points.points.push_back( hm.body.joints[ i ].position );
	joints.push_back( hm.body.joints[ i ].position );
	//cout << "i:"<< i << endl;
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
      jc.jointsRequest(12);
      ros::spinOnce();
      //loop.sleep();
    }
    return 0;
}
