#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

class WindSimNode : public rclcpp::Node {
public:
  explicit WindSimNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
  void tick();

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params/state
  int    grid_n_{3};    
  double spacing_{0.5};
  double extent_{2.0};
  double base_u_{1.2}, base_v_{0.4};
  double swirl_gamma_{0.4}, core2_{0.05};
  double noise_amp_{0.2};
  double dt_{0.15};
  int    period_ms_{500};
  double t_{0.0};
};
