#include "hotspotDetection_pyrosens/thermalHotspotNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;


ThermalHotspotNode::ThermalHotspotNode() : rclcpp::Node("thermal_hotspot_node")
{
  // Declare parameters
  this->declare_parameter<std::string>("thermal_topic", thermal_topic_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<double>("temp_gain", temp_gain_);
  this->declare_parameter<double>("temp_offset", temp_offset_);

  // Get initial parameter values
  this->get_parameter("thermal_topic", thermal_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("temp_gain", temp_gain_);
  this->get_parameter("temp_offset", temp_offset_);

  // Configure detector
  thermdetect::DetectorConfig cfg;
  cfg.temp_gain = temp_gain_;
  cfg.temp_offset = temp_offset_;
  detector_ = thermdetect::HotspotDetector(cfg);

  // Subscribe to thermal image topic
  thermalImage_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ThermalHotspotNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "ThermalHotspotNode initalised, Subscribing to: %s (frame_id=%s, gain=%.6f, offset=%.6f)",
              thermal_topic_.c_str(), frame_id_.c_str(), temp_gain_, temp_offset_);
}

ThermalHotspotNode::~ThermalHotspotNode() {
  RCLCPP_INFO(get_logger(), "ThermalHotspotNode shutting down.");
}

void ThermalHotspotNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  // Cache geometry
  img_width_  = static_cast<int>(img_msg->width);
  img_height_ = static_cast<int>(img_msg->height);

  // Convert to cv::Mat (no forced encoding; we accept mono8 or mono16)
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const auto& mat = cv_ptr->image;
  const auto& enc = img_msg->encoding;

  if (mat.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Empty thermal image received.");
    return;
  }

  if (enc != sensor_msgs::image_encodings::MONO8 &&
      enc != sensor_msgs::image_encodings::MONO16) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                         "Unsupported encoding '%s'. Expect mono8 or mono16.",
                         enc.c_str());
    return;
  }

  // Use detector to find the hottest pixel
  const thermdetect::HotspotResult result = detector_.detectHottestPixel(mat);

  if (result.u >= 0 && result.v >= 0) {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
                         "Hotspot: (u=%d, v=%d) raw=%.1f temp=%.2f°C (img %dx%d, %s)",
                         result.u, result.v, result.raw, result.temp_c, img_width_, img_height_, enc.c_str());
  } else {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 2000,
                          "No hotspot found in current frame.");
  }
}