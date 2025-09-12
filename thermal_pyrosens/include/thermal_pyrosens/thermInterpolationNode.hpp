#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class ThermInterpolationNode : public rclcpp::Node {
public:
  ThermInterpolationNode();

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr samples_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state (scaffold)
  nav_msgs::msg::OccupancyGrid heatmap_;

  // Callback for new thermal samples
  void onSamples(const sensor_msgs::msg::PointCloud & msg);

  // Periodic publish
  void publishHeatmap();

  // Parameters
  double grid_resolution_;   // m/cell
  int grid_width_;           // cells
  int grid_height_;          // cells
  double origin_x_;          // world coords
  double origin_y_;
  double publish_rate_hz_;
  std::string kernel_;       // "IDW", "Gaussian", etc.
  double decay_tau_s_;
  double ambient_temp_c_;
};

