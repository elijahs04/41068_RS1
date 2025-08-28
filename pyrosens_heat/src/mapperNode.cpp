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
  heat_.assign(width_ * height_, std::numeric_limits<float>::quiet_NaN());
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

  const auto & p = last_point_.value().point;
  int i = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
  int j = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
  if (i < 0 || j < 0 || i >= width_ || j >= height_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Point (%.2f, %.2f) out of grid", p.x, p.y);
    return;
  }

  // --- 3x3 Gaussian-ish kernel ---
  const float K[3][3] = {
    {0.05f, 0.10f, 0.05f},
    {0.10f, 0.40f, 0.10f},
    {0.05f, 0.10f, 0.05f}
  };

  float T = static_cast<float>(msg.temperature);

  // loop around the center cell
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      int ii = i + dx;
      int jj = j + dy;
      if (ii < 0 || jj < 0 || ii >= width_ || jj >= height_) continue;

      size_t k = static_cast<size_t>(jj) * width_ + static_cast<size_t>(ii);
      float &H = heat_[k];
      float w = K[dy + 1][dx + 1];

      // EMA update with kernel weight
      if (std::isnan(H)) H = T * w;
      else               H = (1.0f - alpha_ * w) * H + (alpha_ * w) * T;

      // scale to [0,100] for publishing
      float a = (H - t_min_) / (t_max_ - t_min_);
      a = std::clamp(a, 0.0f, 1.0f);
      grid_.data[k] = static_cast<int8_t>(std::round(a * 100.0f));
    }
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Painted around (i=%d, j=%d) for temp=%.1f at point=(%.2f, %.2f)",
    i, j, msg.temperature, p.x, p.y);

  grid_.header.stamp = now();
  map_pub_->publish(grid_);
}
