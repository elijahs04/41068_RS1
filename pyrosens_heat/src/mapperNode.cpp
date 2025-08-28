#include "pyrosens_heat/mapperNode.hpp"

MapperNode::MapperNode() : rclcpp::Node("mapper_node") {
  point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/heat/sensed_point", 10,
    std::bind(&MapperNode::onPoint, this, std::placeholders::_1));

  temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
    "/heat/temperature", 10,
    std::bind(&MapperNode::onTemp, this, std::placeholders::_1));
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
  RCLCPP_INFO(get_logger(), "Got temp %.2f C at (%.2f, %.2f)",
              msg.temperature, p.x, p.y);
}
