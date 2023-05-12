#include "camera/camera_node.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "daheng_camera_node");
  daheng_galaxy::DahengCameraNode camera_node;
  while (ros::ok()) {
    if (!camera_node.isOpen()) {
      camera_node.open();
      if (!camera_node.isOpen()) {
        ros::Duration(1).sleep();
        ROS_WARN("Reopen the camera");
      }
    }
    ros::spinOnce();
  }
  camera_node.close();
}