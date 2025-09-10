#include "wind_pyrosens/mapperNode.hpp"
#include <algorithm>
#include <cmath>

// namespace {
//   inline double clamp01(double x) { 
//     return std::max(0.0, std::min(1.0, x)); 
//   }

//   inline float  f32(double x) { 
//     return static_cast<float>(x); 
//   }

//   inline void speedToColor(double t01, float &r, float &g, float &b) {
//     // 0..0.25: blue→cyan, 0.25..0.5: cyan→green, 0.5..0.75: green→yellow, 0.75..1: yellow→red
//     if (t01 < 0.25) {
//       double k = t01 / 0.25;        r = 0.0f;         g = f32(k);      b = 1.0f;
//     } else if (t01 < 0.5) {
//       double k = (t01-0.25)/0.25;   r = 0.0f;         g = 1.0f;        b = f32(1.0 - k);
//     } else if (t01 < 0.75) {
//       double k = (t01-0.5) /0.25;   r = f32(k);       g = 1.0f;        b = 0.0f;
//     } else {
//       double k = (t01-0.75)/0.25;   r = 1.0f;         g = f32(1.0 - k); b = 0.0f;
//     }
//   }
// }

MapperNode::MapperNode() : rclcpp::Node("mapper_node") {
  // sub
  point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/wind/point", 10,
    std::bind(&MapperNode::onPoint, this, std::placeholders::_1));

  w_vel_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/wind/velocity", 10,
    std::bind(&MapperNode::onWind, this, std::placeholders::_1));

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

void MapperNode::onPoint(const geometry_msgs::msg::PointStamped & msg) {
  last_point_ = msg;
}

void MapperNode::onWind(const geometry_msgs::msg::Vector3Stamped & msg) {
  if (!last_point_) return;

  static int arrow_counter = 0;

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp    = this->now();
  arrow.ns   = "wind_arrows";
  arrow.id   = arrow_counter++;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  // base at the sample point
  arrow.pose.position = last_point_->point;

  // velocity -> yaw
  const double u = msg.vector.x;
  const double v = msg.vector.y;
  const double speed = std::hypot(u, v);
  const double yaw = std::atan2(v, u);

  // quaternion about Z
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = std::sin(yaw * 0.5);
  arrow.pose.orientation.w = std::cos(yaw * 0.5);

  // ---- size from speed ----
  // normalize 0..1 using speed_clip_max_ so big gusts don't explode sizes
  auto clamp01 = [](double x){ return std::max(0.0, std::min(1.0, x)); };
  const double t = clamp01(speed / std::max(1e-6, speed_clip_max_));   // 0 (calm) … 1 (clipped max)

  // shaft length scales with speed (you already have arrow_scale_base_)
  const double L = std::max(std::min(speed, speed_clip_max_) * arrow_scale_base_, 0.05);
  arrow.scale.x = L;

  // also thicken arrow with speed (optional but nice)
  arrow.scale.y = std::max(arrow_diam_ * (0.6 + 0.8 * t), 0.01); // shaft diameter
  arrow.scale.z = std::max(head_diam_  * (0.6 + 0.8 * t), arrow.scale.y); // head diameter

  // ---- colour from speed (blue->cyan->green->yellow->red) ----
  auto speedToColor = [](double t01, float &r, float &g, float &b) {
    t01 = std::max(0.0, std::min(1.0, t01));
    if (t01 < 0.25) {           // blue -> cyan
      double k = t01 / 0.25; r = 0.0f;        g = static_cast<float>(k); b = 1.0f;
    } else if (t01 < 0.5) {     // cyan -> green
      double k = (t01-0.25)/0.25; r = 0.0f;   g = 1.0f;                  b = static_cast<float>(1.0 - k);
    } else if (t01 < 0.75) {    // green -> yellow
      double k = (t01-0.5)/0.25; r = static_cast<float>(k); g = 1.0f;     b = 0.0f;
    } else {                    // yellow -> red
      double k = (t01-0.75)/0.25; r = 1.0f;   g = static_cast<float>(1.0 - k); b = 0.0f;
    }
  };
  float r, g, b;
  speedToColor(t, r, g, b);
  arrow.color.r = r;
  arrow.color.g = g;
  arrow.color.b = b;
  arrow.color.a = 1.0f;

  // keep visible while testing (set to finite later if you want fading)
  arrow.lifetime = rclcpp::Duration(0, 0);

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(arrow);
  marker_pub_->publish(arr);

  last_point_.reset();
}

