#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"   
#include <chrono>

class SensorNode : public rclcpp::Node {
public:
  SensorNode();  

private:
  void onTestCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr w_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr test_cloud_sub_;
};
