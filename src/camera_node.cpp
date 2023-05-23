// Copyright 2023 Chengfu Zou

#include "camera/camera_node.hpp"

namespace daheng_galaxy {
DahengCameraNode::DahengCameraNode() : it_(nh_) {
  ROS_INFO("Initializing Daheng Galaxy Camera Driver");

  nh_.param("/camera_name_", camera_name_, std::string("daheng"));
  nh_.param("/camera_info_url", camera_info_url_, std::string(""));
  nh_.param("/camera_frame_id", frame_id_,
            std::string("camera_optical_frame"));
  nh_.param("/pixel_format", pixel_format_, std::string("bgr8"));
  nh_.param("/resolution_width", resolution_width_, 1280);
  nh_.param("/resolution_height", resolution_height_, 1024);
  nh_.param("/auto_white_balance", auto_white_balance_, 1);
  nh_.param("/offset_x", offset_x_, 0);
  nh_.param("/offset_y", offset_y_, 0);
  nh_.param("/frame_rate", frame_rate_, 210);
  nh_.param("/exposure_time", exposure_time_, 2000);
  nh_.param("/gain", gain_, 5);

  //为存储原始图像数据申请空间
  image_msg_.header.frame_id = frame_id_;
  image_msg_.header.stamp = ros::Time::now();
  image_msg_.encoding = pixel_format_;
  image_msg_.height = resolution_height_;
  image_msg_.width = resolution_width_;
  image_msg_.step = resolution_width_ * 3;
  image_msg_.data.resize(image_msg_.height * image_msg_.step);
  m_pBufferRaw =
      new unsigned char[(size_t)(image_msg_.height * image_msg_.step)];

  if (pixel_format_ == "mono8") {
    m_nPixelformat = GX_PIXEL_FORMAT_MONO8;
  } else if (pixel_format_ == "mono16") {
    m_nPixelformat = GX_PIXEL_FORMAT_MONO16;
  } else if (pixel_format_ == "bgr8") {
    m_nPixelformat = GX_PIXEL_FORMAT_BGR8;
    m_nBayerType = BAYERBG;
  } else if (pixel_format_ == "rgb8") {
    m_nPixelformat = GX_PIXEL_FORMAT_RGB8;
    m_nBayerType = BAYERRG;
  } else if (pixel_format_ == "bgra8") {
    m_nPixelformat = GX_PIXEL_FORMAT_BGRA8;
  } else {
    ROS_ERROR("Illegal pixel format");
  }

  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(
          nh_, camera_name_, camera_info_url_);
  if (!camera_info_manager_->isCalibrated()) {
    camera_info_manager_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = image_msg_.header.frame_id;
    camera_info.width = image_msg_.width;
    camera_info.height = image_msg_.height;
    camera_info_manager_->setCameraInfo(camera_info);
  }
  camera_info_ = camera_info_manager_->getCameraInfo();

  pub_ = it_.advertiseCamera(
      nh_.getNamespace() + "/" + camera_name_ + "/image_raw", 1);

  reconfigure_type::CallbackType f_;
  f_ = boost::bind(&DahengCameraNode::reconfigureCallback, this, _1, _2);
  reconfigure_srv_.setCallback(f_);
}

DahengCameraNode::~DahengCameraNode() {
  close();
  if (m_pBufferRaw != NULL) {
    delete[] m_pBufferRaw;
  }
}

ros::Time DahengCameraNode::getLatestFrameStamp() const {
  return image_msg_.header.stamp;
}

void DahengCameraNode::open() {
  // Init Daheng Galaxy Camera Device
  ROS_INFO("Try to open camera");
  is_open_ = init();
  if (is_open_) {
    ROS_INFO("Camera Opened");
  } else {
    ROS_ERROR("Failed to open camera");
  }
}

void DahengCameraNode::close() {
  printf("Try to close camera\n");
  if (is_open_) {
    // Stream off
    GXStreamOff(m_hDevice);
    // Unregister callback and close device
    GXUnregisterCaptureCallback(m_hDevice);
    GXCloseDevice(m_hDevice);
    GXCloseLib();
    is_open_ = false;
  }
}

bool DahengCameraNode::isOpen() {
  return is_open_;
}

bool DahengCameraNode::init() {
  GX_OPEN_PARAM openParam;
  uint32_t nDeviceNum = 0;
  openParam.accessMode = GX_ACCESS_EXCLUSIVE;
  openParam.openMode = GX_OPEN_INDEX;
  openParam.pszContent = (char*)"1";
  //  Init lib
  emStatus = GXInitLib();
  if (emStatus != GX_STATUS_SUCCESS) {
    ROS_ERROR("Can't init lib");
    return false;
  }
  // Find device
  emStatus = GXUpdateDeviceList(&nDeviceNum, 1000);
  if ((emStatus != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
    ROS_ERROR("Can't find camera");
    return false;
  }
  ROS_INFO("Camera found");
  // Open device
  emStatus = GXOpenDevice(&openParam, &m_hDevice);
  if (emStatus != GX_STATUS_SUCCESS) {
    ROS_ERROR("Camera Open Failed");
    return false;
  }
  // Set pixel format
  GXSetEnum(m_hDevice, GX_ENUM_PIXEL_FORMAT, m_nPixelformat);
  // Set resolution
  GXSetInt(m_hDevice, GX_INT_WIDTH, resolution_width_);
  GXSetInt(m_hDevice, GX_INT_HEIGHT, resolution_height_);

  // Set offset
  // int64_t nOffsetX = (MAX_RESOLUTION_WIDTH - resolution_width_) / 2;
  // int64_t nOffsetY = (MAX_RESOLUTION_HEIGHT - resolution_height_) / 2;
  //  GXSetEnum(m_hDevice, GX_ENUM_RREGION_SELECTOR, GX_REGION_SELECTOR_REGION0);
  GXSetInt(m_hDevice, GX_INT_OFFSET_X, offset_x_);
  GXSetInt(m_hDevice, GX_INT_OFFSET_Y, offset_y_);

  // Get payload size
  GXGetInt(m_hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize);
  // Set white balance
  if (auto_white_balance_) {
    GXSetEnum(m_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, 1);
  } else {
    GXSetEnum(m_hDevice, GX_ENUM_LIGHT_SOURCE_PRESET,
              GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K);
  }
  // Set frame rate
  GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
            GX_ENUM_COVER_FRAMESTORE_MODE_ON);
  GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate_);

  // Set exposure time and gain
  GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  GXSetFloat(m_hDevice, GX_FLOAT_GAIN, gain_);

  // Set trigger mode, continuous acquisition
  GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);

  // Register callback
  GXRegisterCaptureCallback(m_hDevice, nullptr, onFrameCallbackFun);

  // Stream on
  GXStreamOn(m_hDevice);

  return true;
}

void GX_STDC
DahengCameraNode::onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
    // RGB转换
    DxRaw8toRGB24((void*)pFrame->pImgBuf, m_pBufferRaw, pFrame->nWidth,
                  pFrame->nHeight, RAW2RGB_NEIGHBOUR,
                  static_cast<DX_PIXEL_COLOR_FILTER>(m_nBayerType), false);

    image_msg_.header.stamp = camera_info_.header.stamp = ros::Time::now();
    memcpy((unsigned char*)(&image_msg_.data[0]), m_pBufferRaw,
           image_msg_.step * image_msg_.height);
    pub_.publish(image_msg_, camera_info_);
  }
}

void DahengCameraNode::reconfigureCallback(const CameraConfig &config,
                                           uint32_t level) {
  if (exposure_time_ != config.exposure_time && is_open_) {
    exposure_time_ = config.exposure_time;
    GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  }
  if (gain_ != config.gain && is_open_) {
    gain_ = config.gain;
    GXSetFloat(m_hDevice, GX_FLOAT_GAIN, gain_);
  }
}

// Define static members
image_transport::CameraPublisher DahengCameraNode::pub_;
sensor_msgs::Image DahengCameraNode::image_msg_;
sensor_msgs::CameraInfo DahengCameraNode::camera_info_;
GX_DEV_HANDLE DahengCameraNode::m_hDevice;
unsigned char* DahengCameraNode::m_pBufferRaw;
GX_STATUS DahengCameraNode::emStatus;
int64_t DahengCameraNode::m_nPixelformat;
int64_t DahengCameraNode::m_nPayLoadSize;
int64_t DahengCameraNode::m_nBayerType;

}  // namespace daheng_galaxy
