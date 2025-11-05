#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class PredictionSim : public rclcpp::Node {
public:
  PredictionSim();

private:
  void publish_data();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sim_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heat_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_period_;
  size_t sample_index_;
};

