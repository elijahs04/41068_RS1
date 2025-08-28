#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"      
#include "sensor_msgs/msg/channel_float32.hpp"
#include <chrono>

class SensorNode : public rclcpp::Node {
public:
  SensorNode();  

private:
  // functions
  void onTimer();
  void onTestCloud(const sensor_msgs::msg::PointCloud & msg);

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr test_cloud_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;
};
