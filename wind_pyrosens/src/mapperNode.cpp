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
  arrow_diam_       = declare_parameter<double>("arrow_diam",       0.03);
  head_diam_        = declare_parameter<double>("head_diam",        0.06);
  head_len_         = declare_parameter<double>("head_len",         0.06);
  speed_clip_max_   = declare_parameter<double>("speed_clip_max",   10.0);
}

void MapperNode::onPoint(const geometry_msgs::msg::PointStamped & msg) {
  last_point_ = msg;
}

void MapperNode::onWind(const geometry_msgs::msg::Vector3Stamped & msg) {
  
}
