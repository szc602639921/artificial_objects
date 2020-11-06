#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <iostream>   
#include <string>     

int main(int argc, char** argv){
  
  
  ros::init(argc, argv, "artificial_object",ros::init_options::AnonymousName);
  std::cout << "getting information" << std::endl;
  std::string node_name = ros::this_node::getName();  
  std::vector<double> xx;
  std::vector<double> yy;
  std::vector<double> color;
  std::string frame_id;
  std::string name_space;

  int freq;
  float mark_size;

  ros::NodeHandle node;

  node.param<std::string>(node_name +"/name_space", name_space, "default");
  node.param<std::string>(node_name +"/frame_id",frame_id,"map");

  node.getParam(node_name +"/x", xx);
  node.getParam(node_name +"/y", yy);
  node.getParam(node_name +"/color",color);
  node.getParam(node_name +"/freq",freq);
  node.getParam(node_name +"/mark_size",mark_size);
  

  std::cout << name_space << std::endl;

  if(yy.size() != xx.size()){
    ROS_INFO("the lengths of the coordinates are different");
    return 1;
  }

  ros::Publisher vis_pub = node.advertise<visualization_msgs::MarkerArray>( "visualization_marker_list", 0 );

  ros::Rate rate(freq);

  visualization_msgs::MarkerArray maker_list;
  int time = 0;
  while (node.ok() && time < xx.size()){
      
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.ns = name_space;
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
      time++;
  }
  ros::spin();
  return 0;
};