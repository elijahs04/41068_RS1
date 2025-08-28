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

  // state
  std::optional<geometry_msgs::msg::PointStamped> last_point_;

  // grid data (fixed atm)
  nav_msgs::msg::OccupancyGrid grid_;
  double resolution_ = 1.0;
  int width_ = 100; 
  int height_ = 100;
  double origin_x_ = -50.0;
  double origin_y_ = -50.0;
  double t_min_ = 15.0;
  double t_max_ = 80.0;
};
