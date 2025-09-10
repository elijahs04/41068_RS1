#include "wind_pyrosens/mapperNode.hpp"
#include <algorithm>
#include <cmath>

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
}

void MapperNode::onPoint(const geometry_msgs::msg::PointStamped & msg) {
  last_point_ = msg;
}

void MapperNode::onWind(const geometry_msgs::msg::Vector3Stamped & msg) {
    if (!last_point_) return;

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp    = this->now();
  arrow.ns   = "wind_arrows";
  arrow.id   = static_cast<int>(this->get_clock()->now().nanoseconds() % 1000000);
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;

  // position: base at sample point
  arrow.pose.position = last_point_->point;

  // orientation from velocity (z-rotation)
  const double u = msg.vector.x;
  const double v = msg.vector.y;
  const double speed = std::hypot(u, v);
  const double yaw = std::atan2(v, u);
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = std::sin(yaw * 0.5);
  arrow.pose.orientation.w = std::cos(yaw * 0.5);

  // scale: shaft length ∝ speed (capped)
  const double L = std::min(speed, speed_clip_max_) * arrow_scale_base_;
  arrow.scale.x = std::max(L, 0.2);
  arrow.scale.y = arrow_diam_;
  arrow.scale.z = std::max(head_diam_, arrow_diam_);

  // color
  arrow.color.r = 1.0f;
  arrow.color.g = 1.0f;
  arrow.color.b = 1.0f;
  arrow.color.a = 1.0f;

  // short lifetime
  // arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
  
  // infinite lifetime
  arrow.lifetime = rclcpp::Duration(0, 0);

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(arrow);
  marker_pub_->publish(arr);

  last_point_.reset();
}
