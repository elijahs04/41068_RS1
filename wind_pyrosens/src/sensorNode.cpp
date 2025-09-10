#include "wind_pyrosens/sensorNode.hpp"
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
  // TO DO 
}
