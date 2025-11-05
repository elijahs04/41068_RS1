#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <array>
#include <random>
#include <string>

class ThermalSimNode : public rclcpp::Node {
public:
  explicit ThermalSimNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
  // Helpers
  void init_blob_from_points_(const std::array<std::pair<double,double>,4>& pts);
  double temperature_at_(double x, double y) const;
  void tick_();

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr      temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   point_pub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  // Params / state
  std::string frame_id_;
  double spacing_{0.5}, extent_{10.0}, z_height_{0.0};
  int period_ms_{60};

  // Temperature model
  double T_ambient_{25.0}, T_peak_{550.0}, noise_std_{1.5};

  // RNG
  std::mt19937 rng_;
  std::normal_distribution<double> distN_{0.0, 1.0};

  // Superellipse blob (axis-aligned)
  double blob_cx_{0.0}, blob_cy_{0.0};   // centre
  double blob_a_{1.0}, blob_b_{1.0};     // semi-axes
  double blob_n_{2.6};                   // shape exponent
  double blob_beta_{8.0};                // edge sharpness

  // Starburst scan params/state
  double center_x_{0.0}, center_y_{0.0};
  double dr_{0.5};
  double dtheta_deg_{2.0};
  double omega_rps_{0.0};
  double scan_r_max_{10.0};
  int    samples_per_tick_{5};

  double scan_theta_deg_{0.0};           // [deg]
  double r_{0.0};                         // [m]
};
