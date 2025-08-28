#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <optional>

class MapperNode : public rclcpp::Node {
public:
  MapperNode();

private:
  void onPoint(const geometry_msgs::msg::PointStamped & msg);
  void onTemp(const sensor_msgs::msg::Temperature & msg);

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  std::optional<geometry_msgs::msg::PointStamped> last_point_;
};
