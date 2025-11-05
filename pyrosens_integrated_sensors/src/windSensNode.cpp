#include "wind_pyrosens/windSensNode.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace
{
std::string bearingToCardinal(double bearing_deg)
{
  static const std::array<const char*, 8> cardinals{
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
  };
  const double wrapped = std::fmod(bearing_deg + 360.0, 360.0);
  const std::size_t idx = static_cast<std::size_t>(
    std::llround(wrapped / 45.0)) % cardinals.size();
  return cardinals[idx];
}
}  // namespace

WindSensNode::WindSensNode() : rclcpp::Node("wind_sensor_node") {
    // pub
  w_vel_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/wind/velocity", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/wind/point", 10);
  wind_text_pub_ = create_publisher<std_msgs::msg::String>("/sim/wind", 10);

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

  double sum_u = 0.0;
  double sum_v = 0.0;
  double max_speed = 0.0;
  std::size_t count = 0;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_u, ++it_v, ++i) {
    const rclcpp::Time stamp = base + rclcpp::Duration(0, i);  // add i nanoseconds

    p_msg.header.stamp = stamp;
    v_msg.header.stamp = stamp;

    const double x = static_cast<double>(*it_x);
    const double y = static_cast<double>(*it_y);
    const double z = static_cast<double>(*it_z);
    const double u = static_cast<double>(*it_u);
    const double v = static_cast<double>(*it_v);

    p_msg.point.x = static_cast<float>(x);
    p_msg.point.y = static_cast<float>(y);
    p_msg.point.z = static_cast<float>(z);

    v_msg.vector.x = static_cast<float>(u);
    v_msg.vector.y = static_cast<float>(v);
    v_msg.vector.z = 0.0;

    point_pub_->publish(p_msg);
    w_vel_pub_->publish(v_msg);

    sum_u += u;
    sum_v += v;
    max_speed = std::max(max_speed, std::hypot(u, v));
    ++count;

    // RCLCPP_INFO(this->get_logger(), "Point: (%.2f, %.2f, %.2f)  Velocity: (%.2f, %.2f)", p_msg.point.x, p_msg.point.y, p_msg.point.z, v_msg.vector.x, v_msg.vector.y);

    // delay between samples
    std::this_thread::sleep_for(10ms); // uncomment to simulate slower sensor

  }

  if (count > 0 && wind_text_pub_) {
    const double mean_u = sum_u / static_cast<double>(count);
    const double mean_v = sum_v / static_cast<double>(count);
    const double mean_speed = std::hypot(mean_u, mean_v);
    const double heading_rad = std::atan2(mean_v, mean_u);  // 0 rad = +X (East)
    double bearing_deg = 90.0 - (heading_rad * 180.0 / M_PI);  // convert to 0° = North
    if (!std::isfinite(bearing_deg)) {
      bearing_deg = 0.0;
    }
    bearing_deg = std::fmod(bearing_deg + 360.0, 360.0);
    const std::string cardinal = bearingToCardinal(bearing_deg);

    std_msgs::msg::String summary;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Wind: " << mean_speed << " m/s (peak " << max_speed
        << " m/s) @ " << std::setprecision(0) << bearing_deg << "° " << cardinal;
    summary.data = oss.str();
    wind_text_pub_->publish(summary);
  }
}
