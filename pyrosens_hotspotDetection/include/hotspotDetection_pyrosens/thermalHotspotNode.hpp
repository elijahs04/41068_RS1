#pragma once

#include <memory>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Project
#include "hotspotDetection_pyrosens/hotspotDetector.hpp"

class ThermalHotspotNode : public rclcpp::Node {
public:
  ThermalHotspotNode();
  ~ThermalHotspotNode() override;

private:
  // ROS callback
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  // Params and state
  std::string thermal_topic_ = "/thermal_camera/image";
  std::string frame_id_ = "thermal_camera_frame";
  double temp_gain_ = 1.0;
  double temp_offset_ = 0.0;

  // Cached geometry (optional convenience)
  int img_width_ = 0;
  int img_height_ = 0;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalImage_sub_;

  // Algorithm
  thermdetect::HotspotDetector detector_;
};
