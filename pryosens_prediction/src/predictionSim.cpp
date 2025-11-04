#include "pyrosens_prediction/predictionSim.hpp"

#include <cmath>

using namespace std::chrono_literals;

PredictionSim::PredictionSim() : Node("prediction_sim"), publish_period_(1.0), sample_index_(0)
{
  this->declare_parameter("publish_period", publish_period_);
  this->get_parameter("publish_period", publish_period_);

  // NOTE(sim_data_contract): replace dummy publishers with realistic sample grid.
  // Suggested fields per PointCloud2:
  //   - XYZ: lattice center (meters, map frame)
  //   - wind_x|wind_y|wind_z: local wind vector (m/s)
  //   - temperature: degrees Celsius or Kelvin
  //   - fuel_coeff (optional): local fuel load scaling factor.
  // When ready, this node should iterate through analytical/simulation outputs and fill
  // the PointCloud2 binary buffer so PredictionNode can interpolate meaningful values.

  sim_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sim_cloud", 10);
  wind_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("wind", 10);
  heat_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("heat_cloud", 10);
  point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(publish_period_),
    std::bind(&PredictionSim::publish_data, this));

  RCLCPP_INFO(this->get_logger(), "PredictionSim started with period %.2f s", publish_period_);
}

void PredictionSim::publish_data()
{
  const auto stamp = this->now();

  // TODO(sim_data_contract): populate cloud with structured grid data. Consider storing
  // metadata (resolution, bounds) on parameters so PredictionNode can reconstruct
  // neighbouring voxels for interpolation.
  sensor_msgs::msg::PointCloud2 sim_cloud_msg;
  sim_cloud_msg.header.stamp = stamp;
  sim_cloud_msg.header.frame_id = "map";
  sim_cloud_msg.height = 1;
  sim_cloud_msg.width = 1;
  sim_cloud_msg.is_bigendian = false;
  sim_cloud_msg.is_dense = true;
  sim_cloud_msg.point_step = 0;
  sim_cloud_msg.row_step = 0;
  sim_cloud_pub_->publish(sim_cloud_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy sim cloud sample %zu", sample_index_);

  geometry_msgs::msg::Vector3Stamped wind_msg;
  wind_msg.header.stamp = stamp;
  wind_msg.header.frame_id = "map";
  wind_msg.vector.x = std::cos(static_cast<double>(sample_index_) / 4.0);
  wind_msg.vector.y = std::sin(static_cast<double>(sample_index_) / 4.0);
  wind_msg.vector.z = 0.25 * static_cast<double>(sample_index_ % 4);
  wind_pub_->publish(wind_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy wind (%.2f, %.2f, %.2f)",
              wind_msg.vector.x, wind_msg.vector.y, wind_msg.vector.z);

  sensor_msgs::msg::PointCloud2 heat_cloud_msg = sim_cloud_msg;
  heat_cloud_pub_->publish(heat_cloud_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy heat cloud sample %zu", sample_index_);

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.stamp = stamp;
  point_msg.header.frame_id = "map";
  point_msg.point.x = 1.0 + 0.5 * static_cast<double>(sample_index_);
  point_msg.point.y = -1.0 + 0.25 * static_cast<double>(sample_index_);
  point_msg.point.z = 0.5;
  // NOTE(prediction_math): eventually drive this ignition point from mission planner
  // or observation data. Keep history so prediction node can emit marker trail.
  point_pub_->publish(point_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy point (%.2f, %.2f, %.2f)",
              point_msg.point.x, point_msg.point.y, point_msg.point.z);

  ++sample_index_;
}
