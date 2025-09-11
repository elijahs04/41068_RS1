#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"  
#include <optional>
#include <vector>
#include <unordered_map>
#include <cstdint>


class MapperNode : public rclcpp::Node {
public:
  MapperNode();

private:
  void onPoint(const geometry_msgs::msg::PointStamped & msg);
  void onWind(const geometry_msgs::msg::Vector3Stamped & msg);

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr w_vel_sub_;

  // publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // state
  std::unordered_map<int64_t, geometry_msgs::msg::PointStamped> point_cache_;

  // params for visuals
  double arrow_scale_base_;   // base shaft length (m per m/s)
  double arrow_diam_;         // shaft diameter (m)
  double head_diam_;          // head diameter (m)
  double head_len_;           // head length (m)
  double speed_clip_max_;     // cap for arrow scaling
  double speed_min_;          // m/s mapped to "cool" color / minimum size
  double speed_max_;          // m/s mapped to "hot" color / maximum size
  double length_gain_;        // extra gain on arrow length (multiplies current length)

};
