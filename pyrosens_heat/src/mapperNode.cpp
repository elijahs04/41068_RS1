#include "pyrosens_heat/mapperNode.hpp"

#include <algorithm>
#include <cmath>

MapperNode::MapperNode() : rclcpp::Node("mapper_node") {
  // sub
  point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/heat/sensed_point", 10,
    std::bind(&MapperNode::onPoint, this, std::placeholders::_1));

  temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
    "/heat/temperature", 10,
    std::bind(&MapperNode::onTemp, this, std::placeholders::_1));

  // pub
  // new (latched-style)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/heat/heatmap", qos);

  // init grid (fixed v0)
  grid_.header.frame_id = "map";
  grid_.info.resolution = resolution_;
  grid_.info.width = width_;
  grid_.info.height = height_;
  grid_.info.origin.position.x = origin_x_;
  grid_.info.origin.position.y = origin_y_;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(width_ * height_, -1); // -1 = unknown
}

void MapperNode::onPoint(const geometry_msgs::msg::PointStamped & msg) {
  last_point_ = msg;
}

void MapperNode::onTemp(const sensor_msgs::msg::Temperature & msg) {
  if (!last_point_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No point received yet; ignoring temperature");
    return;
  }

  // 1) index into grid
  const auto & p = last_point_.value().point;
  int i = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
  int j = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
  if (i < 0 || j < 0 || i >= width_ || j >= height_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Point (%.2f, %.2f) out of grid", p.x, p.y);
    return;
  }
  size_t idx = static_cast<size_t>(j) * static_cast<size_t>(width_) + static_cast<size_t>(i);

  // 2) scale temperature (°C) → 0..100
  double a = (msg.temperature - t_min_) / (t_max_ - t_min_);
  a = std::clamp(a, 0.0, 1.0);
  int8_t val = static_cast<int8_t>(std::round(a * 100.0));

  // v0: overwrite cell
  grid_.data[idx] = val;

  RCLCPP_INFO_THROTTLE(
  get_logger(), *get_clock(), 1000,
  "Set cell (i=%d, j=%d, idx=%zu) to %d (temp=%.1f, point=(%.2f,%.2f))",
  i, j, idx, val, msg.temperature, p.x, p.y);

  // 3) publish
  grid_.header.stamp = now();
  map_pub_->publish(grid_);
}
