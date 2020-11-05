#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <iostream>   
#include <string>     

int main(int argc, char** argv){

  std::string name = argv[1];
  std::cout << "add path for " << argv[1] << std::endl;
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle node;

  std::cout << "getting information" << std::endl;

  std::vector<double> xx;
  std::vector<double> yy;
  std::vector<double> color;
  std::string frame_id;
  int freq;
  float mark_size;

  node.getParam("x", xx);
  node.getParam("y", yy);
  node.getParam("color",color);
  node.getParam("freq",freq);
  node.getParam("mark_size",mark_size);
  node.getParam("frame_id",frame_id);

  if(yy.size() != xx.size()){
    ROS_INFO("the lengths of the coordinates are different");
    return 1;
  }

  ros::Publisher vis_pub = node.advertise<visualization_msgs::MarkerArray>( "visualization_marker_list", 0 );

  ros::Rate rate(freq);

  visualization_msgs::MarkerArray maker_list;
  int time = 0;
  while (node.ok() && time < xx.size()){
      time++;
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.ns = name;
      marker.id = time;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = xx[time];
      marker.pose.position.y = yy[time];
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = mark_size;
      marker.scale.y = mark_size;
      marker.scale.z = mark_size;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      //only if using a MESH_RESOURCE marker type:
      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      maker_list.markers.push_back(marker);
      vis_pub.publish(maker_list);
      rate.sleep();
  }
  return 0;
};