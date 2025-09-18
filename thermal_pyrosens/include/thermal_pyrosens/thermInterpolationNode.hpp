#pragma once

#include <deque>
#include <vector>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class ThermInterpolationNode : public rclcpp::Node {
public:
  /// Interpolates single-pixel thermal samples into a rolling cost map.
  /// Subscribes: /heat/sample_point (PointStamped), /heat/temperature (Temperature)
  /// Publishes : /heat/costmap (OccupancyGrid), /heat/samples (PointCloud2 with 'temp' field)
  ThermInterpolationNode();

private:
  // -------- Types --------
  struct Sample { double x, y, z, T; };

  // -------- Parameters (declared in ctor) --------
  std::string frame_id_;
  // Map geometry
  double resolution_{0.5};   // m/cell
  double extent_x_{10.0};    // half-width [m]
  double extent_y_{10.0};    // half-height [m]
  double origin_x_{-10.0};   // lower-left X [m]
  double origin_y_{-10.0};   // lower-left Y [m]

  // Temperature scaling (°C) → cost 0..100
  double T_ambient_{25.0};
  double T_peak_{550.0};

  // Interpolation kernel
  std::string kernel_{"idw"};  // "idw" or "gaussian"
  double radius_{1.5};         // [m] influence radius
  double idw_power_{2.0};      // IDW exponent
  double gauss_sigma_{0.6};    // Gaussian sigma [m]
  double wsum_cap_{100.0};     // per-cell cap on accumulated weight

  // Timing / buffers
  double publish_hz_{5.0};     // costmap publish rate
  int pc_max_points_{30000};   // rolling sample cloud limit
  int pair_tol_ms_{50};        // point/temperature stamp pairing tolerance [ms]
  double decay_tau_s_{0.0};    // <=0 disables exponential time decay

  // -------- Map storage --------
  int width_{0}, height_{0};
  std::vector<float> wsum_;            // accumulated weights per cell
  std::vector<float> tsum_;            // accumulated (weight * temperature) per cell
  nav_msgs::msg::OccupancyGrid grid_;  // published costmap

  // -------- Rolling samples & pairing --------
  std::deque<Sample> samples_;         // for /heat/samples PointCloud2
  std::deque<geometry_msgs::msg::PointStamped> points_q_;
  std::deque<sensor_msgs::msg::Temperature>    temps_q_;

  // -------- Publishers / Subscribers / Timers --------
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr    sub_temp_;

  rclcpp::TimerBase::SharedPtr pub_timer_;

  // -------- Callbacks --------
  void onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onTemp(const sensor_msgs::msg::Temperature::SharedPtr msg);
  void publishOutputs();          // publishes /heat/costmap and /heat/samples
  void tryPair();                 // pair newest messages within tolerance and process

  // Apply one matched measurement (x,y,T) to accumulators
  void processMeasurement(const geometry_msgs::msg::PointStamped& pt,
                          const sensor_msgs::msg::Temperature& temp);

  // -------- Helpers (implemented in .cpp) --------
  float  kernelWeight(double d) const;  // IDW or Gaussian weight
  void   publishSampleCloud();          // builds PointCloud2 from samples_

  int    worldToIndexX(double x) const;
  int    worldToIndexY(double y) const;
  double indexToWorldX(int i) const;
  double indexToWorldY(int j) const;

  void   trimPointsDeque(std::size_t max_keep = 200);
  void   trimTempsDeque(std::size_t max_keep = 200);
};


