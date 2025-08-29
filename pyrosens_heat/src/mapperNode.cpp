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

  // ---- find the center cell for this sample ----
  const auto & p = last_point_.value().point;
  int i = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
  int j = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
  if (i < 0 || j < 0 || i >= width_ || j >= height_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Point (%.2f, %.2f) out of grid", p.x, p.y);
    return;
  }

  // ---- radial brush parameters (meters → cells) ----
  // Brush radius R_m = cutoff_sigma_ * sigma_m_ (e.g., 3 * 0.5m = 1.5m)
  double R_m = cutoff_sigma_ * sigma_m_;
  int R = std::max(1, static_cast<int>(std::ceil(R_m / resolution_)));
  double sigma2 = sigma_m_ * sigma_m_;
  double R2 = R_m * R_m;

  float T = static_cast<float>(msg.temperature);

  // ---- paint a disk around (i,j) with Gaussian/top-hat weights ----
  for (int dy = -R; dy <= R; ++dy) {
    int jj = j + dy;
    if (jj < 0 || jj >= height_) continue;

    double y_m = dy * resolution_;  // meters

    for (int dx = -R; dx <= R; ++dx) {
      int ii = i + dx;
      if (ii < 0 || ii >= width_) continue;

      double x_m = dx * resolution_;  // meters
      double r2 = x_m * x_m + y_m * y_m;
      if (r2 > R2) continue;  // outside the circular brush

      // weight at this offset
      float w;
      if (use_top_hat_) {
        w = 1.0f;  // flat disk
      } else {
        // Gaussian: peak 1.0 at center, smooth falloff
        w = static_cast<float>(std::exp(-0.5 * (r2 / sigma2)));
      }

      size_t k = static_cast<size_t>(jj) * static_cast<size_t>(width_)
               + static_cast<size_t>(ii);

      float &Hk = heat_[k];
      if (std::isnan(Hk)) {
        Hk = T * w;
      } else {
        // EMA toward T with kernel weight
        Hk = (1.0f - alpha_ * w) * Hk + (alpha_ * w) * T;
      }

      // scale to [0,100] for publishing
      float a = (Hk - t_min_) / (t_max_ - t_min_);
      a = std::clamp(a, 0.0f, 1.0f);
      grid_.data[k] = static_cast<int8_t>(std::round(a * 100.0f));
    }
  }

  grid_.header.frame_id = "map";
  grid_.header.stamp = now();
  map_pub_->publish(grid_);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Painted circular brush at (i=%d,j=%d) temp=%.1f (sigma=%.2fm, cutoff=%.1fσ)",
    i, j, msg.temperature, sigma_m_, cutoff_sigma_);
}
