#include "wind_pyrosens/windSensNode.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

using namespace std::chrono_literals;

WindSensNode::WindSensNode() : rclcpp::Node("wind_sensor_node") {
    // pub
  w_vel_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/wind/velocity", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/wind/point", 10);

    // test subscriber
  test_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/wind/test_cloud", 10,
    std::bind(&WindSensNode::onTestCloud, this, std::placeholders::_1));

}

void WindSensNode::onTestCloud(const sensor_msgs::msg::PointCloud2 & cloud) {
  using sensor_msgs::PointCloud2ConstIterator;

  geometry_msgs::msg::PointStamped    p_msg;
  geometry_msgs::msg::Vector3Stamped  v_msg;
  p_msg.header.frame_id = "map";
  v_msg.header.frame_id = "map";

  PointCloud2ConstIterator<float> it_x(cloud, "x");
  PointCloud2ConstIterator<float> it_y(cloud, "y");
  PointCloud2ConstIterator<float> it_z(cloud, "z");
  PointCloud2ConstIterator<float> it_u(cloud, "u");
  PointCloud2ConstIterator<float> it_v(cloud, "v");

  // NEW: unique stamp per sample
  const rclcpp::Time base = this->now();
  int64_t i = 0;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_u, ++it_v, ++i) {
    const rclcpp::Time stamp = base + rclcpp::Duration(0, i);  // add i nanoseconds

    p_msg.header.stamp = stamp;
    v_msg.header.stamp = stamp;

    p_msg.point.x = *it_x;
    p_msg.point.y = *it_y;
    p_msg.point.z = *it_z;

    v_msg.vector.x = *it_u;
    v_msg.vector.y = *it_v;
    v_msg.vector.z = 0.0;

    point_pub_->publish(p_msg);
    w_vel_pub_->publish(v_msg);

    // RCLCPP_INFO(this->get_logger(), "Point: (%.2f, %.2f, %.2f)  Velocity: (%.2f, %.2f)", p_msg.point.x, p_msg.point.y, p_msg.point.z, v_msg.vector.x, v_msg.vector.y);

    // delay between samples
    std::this_thread::sleep_for(10ms); // uncomment to simulate slower sensor

  }
}
