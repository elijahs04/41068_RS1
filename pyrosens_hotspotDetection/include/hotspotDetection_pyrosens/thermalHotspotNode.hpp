#pragma once

#include <memory>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

// Project
#include "hotspotDetection_pyrosens/hotspotDetector.hpp"

class ThermalHotspotNode : public rclcpp::Node {
public:
  ThermalHotspotNode();
  ~ThermalHotspotNode() override;

private:
  // ROS callback
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalImage_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalOverlay_pub_;

  // Algorithm
  thermdetect::HotspotDetector detector_;

  // Params and state
  std::string thermal_topic_ = "/thermal_camera/image";
  std::string frame_id_ = "thermal_camera_frame";
  double temp_gain_ = 1.0;
  double temp_offset_ = 0.0;
  bool publish_overlay_ = true;
  double hot_temp_c_ = 150.0;
  bool   use_percentile_ = false;
  double hot_percentile_ = 98.0;
  int morphology_kernel_ = 0;   // 0=off, else odd (3,5)
  int min_area_px_ = 50;
  int max_area_px_ = 0;         // 0=off
  int max_regions_draw_ = 5;    // just for overlay sanity

  // Cached geometry (optional convenience)
  int img_width_ = 0;
  int img_height_ = 0;

};
