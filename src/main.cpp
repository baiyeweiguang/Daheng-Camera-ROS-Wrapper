#include "camera/camera_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "daheng_camera_node");
  daheng_galaxy::DahengCameraNode camera_node;
  while (ros::ok()) {
    if (!camera_node.isOpen()) {
      camera_node.open();
      if (!camera_node.isOpen()) {
        ros::Duration(1).sleep();
        ROS_WARN("Camera is not opened, try to open again");
      }
    }
    if ((ros::Time::now() - camera_node.getLatestFrameStamp()).toSec() > 2.0) {
      ROS_WARN("The camera is not publishing image, reopen the camera");
      camera_node.close();
      camera_node.open();
    }

    ros::spinOnce();
  }
  camera_node.close();
}