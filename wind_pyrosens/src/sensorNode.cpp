#include "wind_pyrosens/sensorNode.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

using namespace std::chrono_literals;

SensorNode::SensorNode() : rclcpp::Node("sensor_node") {
    // pub
  w_vel_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/wind/velocity", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/wind/point", 10);

    // test subscriber
  test_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/wind/test_cloud", 10,
    std::bind(&SensorNode::onTestCloud, this, std::placeholders::_1));

}

void SensorNode::onTestCloud(const sensor_msgs::msg::PointCloud2 & cloud) {
  // Expect fields: x, y, z, u, v (w optional/ignored)
  // Iterate through points and publish point + velocity pairs.

  geometry_msgs::msg::PointStamped p_msg;
  geometry_msgs::msg::Vector3Stamped v_msg;

  p_msg.header.frame_id = "map";
  v_msg.header.frame_id = "map";

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> it_u(cloud, "u");
  sensor_msgs::PointCloud2ConstIterator<float> it_v(cloud, "v");

  // Iterate all points
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_u, ++it_v) {
    // stamp both with the SAME time for logical pairing
    const auto stamp = this->now();
    p_msg.header.stamp = stamp;
    v_msg.header.stamp = stamp;

    p_msg.point.x = *it_x;
    p_msg.point.y = *it_y;
    p_msg.point.z = *it_z;   // likely 0

    v_msg.vector.x = *it_u;
    v_msg.vector.y = *it_v;
    v_msg.vector.z = 0.0;

    point_pub_->publish(p_msg);
    w_vel_pub_->publish(v_msg);
  }
}
