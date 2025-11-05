#pragma once

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class ThermSensNode : public rclcpp::Node {
public:
  ThermSensNode();

private:
  // Parameters
  std::string mode_;             // "SIM" or "CAMERA"
  double sample_rate_hz_;
  double fov_deg_;
  double max_range_m_;
  double noise_std_c_;
  double dropout_prob_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr samples_pub_;

  // Subscriber (optional, only in SIM mode if SimulationNode publishes points)
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sim_points_sub_;

  // Timer (for publishing samples in SIM mode)
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback for incoming simulated thermal points
  void onSimPoints(const sensor_msgs::msg::PointCloud & msg);

  // Periodic tick (e.g. generate or forward samples)
  void tick();
};
