#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

class HotspotMapperNode : public rclcpp::Node {
public:
  HotspotMapperNode();
  ~HotspotMapperNode() override = default;

private:
  // Callbacks
  void pointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Timer to apply decay and republish
  void tick();

  // Utils
  bool worldToCell(double x, double y, int& cx, int& cy) const;
  void splatGaussian(int cx, int cy);
  void publishOutputs();

  // Params
  std::string frame_id_ = "map";
  double origin_x_ = -6.25;    // meters
  double origin_y_ = -6.25;    // meters
  double resolution_ = 0.01;   // meters/cell
  int width_ = 1250;            // cells
  int height_ = 1250;           // cells
  float clear_threshold_ = 0.05f;    // when value falls below this, mark unseen
  bool publish_unknown_as_unseen_ = false;  // publish -1 for never/cleared cells

  // Splat kernel
  double sigma_m_ = 0.01;      // meters
  int kernel_radius_cells_ = 0;
  cv::Mat kernel_;             // float32 kernel

  // Decay + publish rate
  double decay_rate_hz_ = 2.0;      // how often to apply decay
  double decay_half_life_s_ = 30.0; // seconds to halve intensity
  float decay_factor_per_tick_ = 0.99f;
  float add_weight_ = 1.0f;         // weight added per splat center

  // State grid (float accumulation)
  std::vector<float> grid_;          // width_*height_, heat values
  std::vector<uint8_t> seen_;        // width_*height_, 0/1: has content?

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_pt_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF (kept for future, not required if inputs already in map)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Helpers
  void buildKernel_();
  void reset_();
  void handleReset_(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>);
};
