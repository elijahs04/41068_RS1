#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <atomic>
#include <string>
#include <thread>

class PredictionSim : public rclcpp::Node {
public:
  PredictionSim();
  ~PredictionSim() override;

private:
  enum class PointMode {
    Disabled,
    Manual,
    Cloud
  };

  void publish_fake_data();
  void manual_input_loop();
  void handle_manual_line(const std::string & line);
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sim_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heat_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> running_{false};
  std::thread input_thread_;
  PointMode point_mode_{PointMode::Disabled};
  std::string cloud_topic_;
  std::string manual_point_frame_;
  bool enable_fake_sensors_{false};
  double publish_period_;
  size_t sample_index_;
};
