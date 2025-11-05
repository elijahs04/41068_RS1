#pragma once

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class ThermSensNode : public rclcpp::Node {
public:
  ThermSensNode();

private:
  // -------- Publishers / Subscribers / Timers --------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr samples_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sim_points_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // -------- Callbacks --------
  // For incoming simulated thermal points
  void onSimPoints(const sensor_msgs::msg::PointCloud & msg);

  // -------- Helpers --------
  void tick();

  // Parameters
  std::string mode_;       
  double sample_rate_hz_;
  double fov_deg_;
  double max_range_m_;
  double noise_std_c_;
  double dropout_prob_;
};
