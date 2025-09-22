#include "wind_pyrosens/windInterpolationNode.hpp"
#include <algorithm>
#include <cmath>

WindInterpolationNode::WindInterpolationNode() : rclcpp::Node("wind_interpolation_node") {
  // sub
  point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/wind/point", 10,
    std::bind(&WindInterpolationNode::onPoint, this, std::placeholders::_1));

  w_vel_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/wind/velocity", 10,
    std::bind(&WindInterpolationNode::onWind, this, std::placeholders::_1));

  // pub
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/wind/markers", qos);

  // Params (with sensible defaults)
  arrow_scale_base_ = declare_parameter<double>("arrow_scale_base", 0.3); // m per (m/s)
  arrow_diam_       = declare_parameter<double>("arrow_diam",       0.05);
  head_diam_        = declare_parameter<double>("head_diam",        0.08);
  head_len_         = declare_parameter<double>("head_len",         0.06);
  speed_clip_max_   = declare_parameter<double>("speed_clip_max",   10.0);
  speed_min_        = declare_parameter<double>("speed_min",        0.0);   // m/s
  speed_max_        = declare_parameter<double>("speed_max",        10.0);  // m/s
  length_gain_      = declare_parameter<double>("length_gain",      1.0); // dimensionless

}

void WindInterpolationNode::onPoint(const geometry_msgs::msg::PointStamped & msg) {
  const int64_t key = rclcpp::Time(msg.header.stamp).nanoseconds();
  point_cache_[key] = msg;
}

void WindInterpolationNode::onWind(const geometry_msgs::msg::Vector3Stamped & msg) {
  // 1) Find matching point by identical stamp (sensor publishes both with same stamp)
  const int64_t key = rclcpp::Time(msg.header.stamp).nanoseconds();
  auto it = point_cache_.find(key);
  if (it == point_cache_.end()) {
    return; // no matching point yet
  }
  const auto & p = it->second;
  point_cache_.erase(it);

  // 2) Build arrow
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp    = this->now();
  arrow.type   = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  // Position at the sample point
  arrow.pose.position = p.point;

  // Velocity -> orientation (yaw about Z)
  const double u = msg.vector.x;
  const double v = msg.vector.y;
  const double speed = std::hypot(u, v);
  const double yaw = std::atan2(v, u);
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = std::sin(yaw * 0.5);
  arrow.pose.orientation.w = std::cos(yaw * 0.5);

  // 3) Stable ID per grid cell (0.5 m grid => *2 and round)
  const int xi = static_cast<int>(std::llround(p.point.x * 2.0));
  const int yi = static_cast<int>(std::llround(p.point.y * 2.0));
  arrow.ns = "wind_arrows";
  arrow.id = yi * 10000 + xi;           // unique per cell; new publish updates the same marker
  arrow.lifetime = rclcpp::Duration(0, 0); // keep visible; updated each tick

  // 4) Size from speed (length + thickness)
  const double t_clip = std::min(speed, speed_clip_max_);
  const double L = std::max(t_clip * arrow_scale_base_, 0.05);
  arrow.scale.x = L;                                   // shaft length
  arrow.scale.y = std::max(arrow_diam_ * (0.6 + 0.8 * (t_clip / std::max(1e-6, speed_clip_max_))), 0.01);
  arrow.scale.z = std::max(head_diam_  * (0.6 + 0.8 * (t_clip / std::max(1e-6, speed_clip_max_))), arrow.scale.y);

  // 5) Colour from speed (blue→cyan→green→yellow→red)
  auto clamp01 = [](double x){ return std::max(0.0, std::min(1.0, x)); };
  const double t = clamp01(speed / std::max(1e-6, speed_clip_max_));
  auto speedToColor = [](double t01, float &r, float &g, float &b) {
    t01 = std::max(0.0, std::min(1.0, t01));
    if (t01 < 0.25) { double k=t01/0.25; r=0.0f; g=float(k); b=1.0f; }
    else if (t01 < 0.5) { double k=(t01-0.25)/0.25; r=0.0f; g=1.0f; b=float(1.0-k); }
    else if (t01 < 0.75){ double k=(t01-0.5)/0.25; r=float(k); g=1.0f; b=0.0f; }
    else { double k=(t01-0.75)/0.25; r=1.0f; g=float(1.0-k); b=0.0f; }
  };
  float r, g, b;
  speedToColor(t, r, g, b);
  arrow.color.r = r; arrow.color.g = g; arrow.color.b = b; arrow.color.a = 1.0f;

  // 6) Publish
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(arrow);
  marker_pub_->publish(arr);
}

