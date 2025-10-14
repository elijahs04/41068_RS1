#include "wind_pyrosens/windSimNode.hpp"
#include <cmath>

WindSimNode::WindSimNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("wind_sim_node", opts), t_(0.0)
{
  // Tunable params (override with ros2 param set)
  grid_n_      = declare_parameter<int>("grid_n", 3);
  spacing_     = declare_parameter<double>("spacing", 0.5);
  extent_      = declare_parameter<double>("extent", 2.0);
  base_u_      = declare_parameter<double>("base_u", 1.2);
  base_v_      = declare_parameter<double>("base_v", 0.4);
  swirl_gamma_ = declare_parameter<double>("swirl_gamma", 0.4);
  core2_       = declare_parameter<double>("swirl_core2", 0.05);
  noise_amp_   = declare_parameter<double>("noise_amp", 0.2);
  dt_          = declare_parameter<double>("dt", 0.15);
  period_ms_   = declare_parameter<int>("period_ms", 500);

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/wind/test_cloud", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_),
                             std::bind(&WindSimNode::tick, this));
}

void WindSimNode::tick() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = now();

  // Area from -extent_ .. +extent_ at spacing_
  const int nx = static_cast<int>(std::floor((extent_ * 2.0) / spacing_)) + 1;
  const int ny = nx;
  const int N  = nx * ny;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "u", 1, sensor_msgs::msg::PointField::FLOAT32,
      "v", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  mod.resize(N);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_u(cloud, "u");
  sensor_msgs::PointCloud2Iterator<float> it_v(cloud, "v");

  const double t = this->now().seconds();
  for (int j = 0; j < ny; ++j) {
    const double y = -extent_ + j * spacing_;
    for (int i = 0; i < nx; ++i, ++it_x, ++it_y, ++it_z, ++it_u, ++it_v) {
      const double x = -extent_ + i * spacing_;
      const double r2 = x*x + y*y;

      // Base flow
      double ux = base_u_;
      double vy = base_v_;

      // Swirl around origin (circulation), softened by core2_
      const double denom = (r2 + core2_);
      ux += (-y) * (swirl_gamma_ / denom);
      vy += ( x) * (swirl_gamma_ / denom);

      // Light jitter so speeds vary (drives color/size)
      const double n1 = 0.5*std::sin(1.7*x + 2.1*y + t) + 0.5*std::cos(2.3*x - 1.3*y - 1.5*t);
      const double n2 = 0.5*std::sin(1.1*x - 2.0*y - 0.7*t) + 0.5*std::cos(2.6*x + 1.8*y + 0.9*t);
      ux += noise_amp_ * n1;
      vy += noise_amp_ * n2;

      *it_x = static_cast<float>(x);
      *it_y = static_cast<float>(y);
      *it_z = 0.0f;
      *it_u = static_cast<float>(ux);
      *it_v = static_cast<float>(vy);
    }
  }

  pub_->publish(cloud);
  // phase accumulator (kept for compatibility even if t above uses clock)
  (void)dt_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Published /wind/test_cloud (%dx%d, spacing=%.2f, extent=%.2f)", nx, ny, spacing_, extent_);
}
