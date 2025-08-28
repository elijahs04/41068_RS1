#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"   
#include <optional>

class MapperNode : public rclcpp::Node {
public:
  MapperNode();

private:
  void onPoint(const geometry_msgs::msg::PointStamped & msg);
  void onTemp(const sensor_msgs::msg::Temperature & msg);

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // grid state
  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<float> heat_;   // floating-point "true" temperatures
  std::optional<geometry_msgs::msg::PointStamped> last_point_;

  // grid param
  double resolution_ = 0.25;
  int width_ = 200; 
  int height_ = 200;
  double origin_x_ = -25.0;
  double origin_y_ = -25.0;

  // map param
  float alpha_ = 0.2f;
  double t_min_ = 1.0f;
  double t_max_ = 100.0f;
};
