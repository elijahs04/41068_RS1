#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>

class SensorNode : public rclcpp::Node {
public:
  SensorNode();  // ctor only; definitions live in .cpp

private:
  void onTimer();

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
