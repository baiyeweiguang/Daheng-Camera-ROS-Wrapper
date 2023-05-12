// Copyright 2023 Chengfu Zou

#ifndef DAHENG_GALAXY__CAMERA_NODE_HPP_
#define DAHENG_GALAXY__CAMERA_NODE_HPP_

// STD
#include <memory>
// ROS
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "daheng_galaxy/CameraConfig.h"
// Daheng Galaxy
#include "camera/DxImageProc.h"
#include "camera/GxIAPI.h"

namespace daheng_galaxy {
class DahengCameraNode {
 public:
  DahengCameraNode();

  ~DahengCameraNode();

  void open();
  void close();
  bool isOpen();

 private:
  // Init GxDevice
  bool init();
  // Callback of dynamic reconfigure
  void reconfigureCallback(const CameraConfig &config, uint32_t level);
  // onFrameCallback
  static void GX_STDC onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

  // ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  using reconfigure_type = dynamic_reconfigure::Server<CameraConfig>;
  reconfigure_type reconfigure_srv_;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::Image image_msg_;
  static sensor_msgs::CameraInfo camera_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_;

  // Daheng Galaxy API
  static GX_DEV_HANDLE m_hDevice;
  static unsigned char* m_pBufferRaw;
  static GX_STATUS emStatus;
  static int64_t m_nPixelformat;
  static int64_t m_nPayLoadSize;
  static int64_t m_nBayerType;

  // General
  bool is_open_ = false;
  int resolution_width_;
  int resolution_height_;
  int auto_white_balance_;
  int frame_rate_;
  int exposure_time_;
  int gain_;
  const int MAX_RESOLUTION_WIDTH = 1280;
  const int MAX_RESOLUTION_HEIGHT = 1024;
};

}  // namespace daheng_galaxy

#endif  // endif DAHENG_GALAXY__CAMERA_NODE_HPP_
