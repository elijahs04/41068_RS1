#pragma once

#include <memory>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Project
#include "hotspotDetection_pyrosens/hotspotDetector.hpp"
#include "hotspotDetection_pyrosens/hotspotTransform.hpp"

class ThermalHotspotNode : public rclcpp::Node {
public:
  ThermalHotspotNode();
  ~ThermalHotspotNode() override;

private:
  // === ROS callback -> Synced callback: thermal + depth ===
  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& thermal_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);


  // ROS I/O
  // Subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> thermal_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;

  // Synchronizer
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalOverlay_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hotspots_world_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr hot_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hot_cloud_pub_;

  // Algorithm
  thermdetect::HotspotDetector detector_;

  // Params and state
  std::string thermal_topic_ = "/thermal_camera/image";
  std::string frame_id_ = "thermal_camera_optical_frame";
  std::string camera_frame_ = "thermal_camera_optical_frame";
  std::string target_frame_ = "map";
  double plane_z_ = 0.0;
  double hfov_rad_ = 2.0944;

  // Depth camera
  std::string depth_topic_ = "/camera/depth/image";
  std::string depth_frame_ = "camera_depth_optical_frame";
  int depth_width_{0}, depth_height_{0};

  // Depth intrinsics (from hfov or CameraInfo if you later add it)
  double d_fx_{0}, d_fy_{0}, d_cx_{0}, d_cy_{0};
  double depth_hfov_rad_{2.0944};
  int depth_stride_{3};           // sample every N pixels to keep it light
  double depth_min_{0.2};         // meters
  double depth_max_{10.0};        // meters (matches your SDF far clip)
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
  int img_width_ = 0;
  int img_height_ = 0;

  // TF + projector
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  hstrfm::HotspotTransform projector_;
  bool intrinsics_ready_{false};

  // Thermal intrinsics we’ll keep around for projection check
  double t_fx_{0}, t_fy_{0}, t_cx_{0}, t_cy_{0};
  bool thermal_intrinsics_ready_{false};

  void setThermalIntrinsicsFromHFOV_();
  void setDepthIntrinsicsFromHFOV_(int w, int h, double hfov);
};
