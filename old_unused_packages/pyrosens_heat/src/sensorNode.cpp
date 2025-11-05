#include "pyrosens_heat/sensorNode.hpp"

#include <cmath>

using namespace std::chrono_literals;

SensorNode::SensorNode() : rclcpp::Node("sensor_node") {
    // pub
  temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/heat/temperature", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/heat/sensed_point", 10);

    // test subscriber
  test_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud>(
    "/heat/test_cloud", 10,
    std::bind(&SensorNode::onTestCloud, this, std::placeholders::_1));

  timer_ = create_wall_timer(100ms, std::bind(&SensorNode::onTimer, this));
}

void SensorNode::onTimer() {
  // keep fake motion, or comment out while feeding test clouds
//   t_ += 0.1;
//   double x = std::fmod(t_, 20.0) - 10.0;
//   double y = 0.0;

//   double background = 20.0, amp = 60.0, sx = 2.0, sy = 2.0;
//   double dx = x - 2.0, dy = y - 0.0;
//   double temp = background + amp * std::exp(-0.5 * ((dx*dx)/(sx*sx) + (dy*dy)/(sy*sy)));

//   auto stamp = this->now();

//   sensor_msgs::msg::Temperature t;
//   t.header.stamp = stamp;
//   t.header.frame_id = "sensor_frame";
//   t.temperature = static_cast<float>(temp);
//   t.variance = 0.25;
//   temp_pub_->publish(t);

//   geometry_msgs::msg::PointStamped p;
//   p.header.stamp = stamp;
//   p.header.frame_id = "map";
//   p.point.x = x; p.point.y = y; p.point.z = 0.0;
//   point_pub_->publish(p);
}

void SensorNode::onTestCloud(const sensor_msgs::msg::PointCloud & cloud) {
  // Find the "temperature" channel
  const sensor_msgs::msg::ChannelFloat32* temp_channel = nullptr;
  for (const auto & ch : cloud.channels) {
    if (ch.name == "temperature") { temp_channel = &ch; break; }
  }
  if (!temp_channel) {
    RCLCPP_WARN(get_logger(), "PointCloud missing 'temperature' channel");
    return;
  }
  if (temp_channel->values.size() != cloud.points.size()) {
    RCLCPP_WARN(get_logger(), "Channel size != points size (%zu vs %zu)",
                temp_channel->values.size(), cloud.points.size());
    return;
  }

  // Replay each point+temperature as our standard topics
  for (size_t k = 0; k < cloud.points.size(); ++k) {
    auto stamp = this->now();  // simple: same stamp for both messages
    const auto & pt = cloud.points[k];
    float temp = temp_channel->values[k];

    sensor_msgs::msg::Temperature t;
    t.header.stamp = stamp;
    t.header.frame_id = "sensor_frame";
    t.temperature = temp;
    t.variance = 0.25f;
    temp_pub_->publish(t);

    geometry_msgs::msg::PointStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = cloud.header.frame_id.empty() ? "map" : cloud.header.frame_id;
    p.point.x = pt.x; p.point.y = pt.y; p.point.z = pt.z;
    point_pub_->publish(p);
  }

  RCLCPP_INFO(get_logger(), "Replayed %zu heat samples from test cloud", cloud.points.size());
}
