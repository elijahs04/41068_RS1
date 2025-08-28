#include "pyrosens_heat/sensorNode.hpp"

using namespace std::chrono_literals;

SensorNode::SensorNode() : rclcpp::Node("sensor_node") {
  temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/heat/temperature", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/heat/sensed_point", 10);
  timer_ = create_wall_timer(100ms, std::bind(&SensorNode::onTimer, this));
}

void SensorNode::onTimer() {
  auto stamp = this->now();

  sensor_msgs::msg::Temperature t;
  t.header.stamp = stamp;
  t.header.frame_id = "sensor_frame";
  t.temperature = 25.0;
  t.variance = 0.25;
  temp_pub_->publish(t);

  geometry_msgs::msg::PointStamped p;
  p.header.stamp = stamp;
  p.header.frame_id = "map";
  p.point.x = 0.0; p.point.y = 0.0; p.point.z = 0.0;
  point_pub_->publish(p);
}
