/*

*/

#include "pyrosens_prediction/predictionSim.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

using namespace std::chrono_literals;

PredictionSim::PredictionSim() : rclcpp::Node("prediction_sim")
{
  // Declare parameters
  this->declare_parameter("period_s", period_s_);
  this->declare_parameter("frame_id", frame_id_);
  this->declare_parameter("nx", nx_);
  this->declare_parameter("ny", ny_);
  this->declare_parameter("res", res_);
  this->declare_parameter("heat_sigma", heat_sigma_);
  this->declare_parameter("wind_base_x", wind_base_x_);
  this->declare_parameter("wind_base_y", wind_base_y_);
  this->declare_parameter("wind_base_z", wind_base_z_);
  this->declare_parameter("p0x", p0x_);
  this->declare_parameter("p0y", p0y_);
  this->declare_parameter("p0z", p0z_);

  period_s_ = this->get_parameter("period_s").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  nx_ = this->get_parameter("nx").as_int();
  ny_ = this->get_parameter("ny").as_int();
  res_ = this->get_parameter("res").as_double();
  heat_sigma_ = this->get_parameter("heat_sigma").as_double();
  wind_base_x_ = this->get_parameter("wind_base_x").as_double();
  wind_base_y_ = this->get_parameter("wind_base_y").as_double();
  wind_base_z_ = this->get_parameter("wind_base_z").as_double();
  p0x_ = this->get_parameter("p0x").as_double();
  p0y_ = this->get_parameter("p0y").as_double();
  p0z_ = this->get_parameter("p0z").as_double();

  // publishers
  sim_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sim_cloud", 10);
  heat_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("heat_cloud", 10);
  wind_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("wind", 10);
  point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);

  // timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_s_),
    std::bind(&PredictionSim::publish_data, this));

  RCLCPP_INFO(this->get_logger(), "Prediction Sim Node has been started.",
              period_s_, nx_, ny_, res_);
}

static sensor_msgs::msg::PointCloud2 make_cloud_xyz_i(
    const std::string &frame, int n_points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.width = n_points;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(2, "xyz", "intensity");
    mod.resize(n_points);
    return cloud;
}

static sensor_msgs::msg::PointCloud2 make_cloud_xyz_vxvyvz(
    const std::string& frame, int n_points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.width = n_points;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vy", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vz", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(n_points);
    return cloud;
}

void PredictionSim::publish_data()
{
    const rclcp::Time now = this->now();
    t_ += period_s_;

    geometry_msgs::msg::Vector3Stamped wind;
    wind.header.stamp = now;
    wind.header.frame_id = frame_id_;
    wind.vector.x = wind_base_x_ + 0.7 * std::sin(0.2 * t_);
    wind.vector.y = wind_base_y_ + 0.3 * std::cos(0.1 * t_);
    wind.vector.z = wind_base_z_;
    wind_pub_->publish(wind);

    geometry_msgs::msg::PointStamped p;
    p.header.stamp = now;
    p.header.frame_id = frame_id_;
    p.point.x = p0x_;
    p.point.y = p0y_;
    p.point.z = p0z_;
    point_pub_->publish(p);

    const int N = nx_ * ny_;
    auto heat_cloud = make_cloud_xyz_i(frame_id_, N);
    heat_cloud.header.stamp = now;

    sensor_msgs::PointCloud2Iterator<float> hx(heat_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> hy(heat_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> hz(heat_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> hi(heat_cloud, "intensity");

    const double cx = heat_cx_ + 10.0 * std::sin(0.05 * t_);
    const double cy = heat_cy_ + 10.0 * std::cos(0.04 * t_);
    const double two_sigma2 = 2.0 * heat_sigma_ * heat_sigma_;

    int idx = 0; 
    for (int iy = 0; iy < ny_; ++iy) {
        for (int ix = 0; ix < nx_; ++ix, ++idx, ++hx, ++hy, ++hz, ++hi) {
            const double x = (ix - nx_ / 2) * res_;
            const double y = (iy - ny_ / 2) * res_;
            const double r2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            const double temp = 22.0 + 480.0 * std::exp(-r2 / std::max(1e-6, two_sigma2));

            *hx = static_cast<float>(x);
            *hy = static_cast<float>(y);
            *hz = 0.0f;
            *hi = static_cast<float>(temp);

        }
    }
    heat_cloud_pub_->publish(heat_cloud);

    auto wind_cloud = make_cloud_xyz_vxvyvz(frame_id_, N);
    wind_cloud.header.stamp = now;

    sensor_msgs::PointCloud2Iterator<float> wx(wind_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> wy(wind_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> wz(wind_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> wvx(wind_cloud, "vx");
    sensor_msgs::PointCloud2Iterator<float> wvy(wind_cloud, "vy");
    sensor_msgs::PointCloud2Iterator<float> wvz(wind_cloud, "vz");

    idx = 0;
    for (int iy = 0; iy < ny_; ++iy) {
        for (int ix = 0; ix < nx_; ++ix, ++idx, ++wx, ++wy, ++wz, ++wvx, ++wvy, ++wvz) {
            const double x = (ix - nx_ / 2) * res_;
            const double y = (iy - ny_ / 2) * res_;

            const double curl_x = 0.3 * std::sin(0.01 * y);
            const double curl_y = 0.3 * std::cos(0.01 * x);

            *wx = static_cast<float>(x);
            *wy = static_cast<float>(y);
            *wz = 0.0f;

            *wvx = wind.vector.x;
            *wvy = wind.vector.y;
            *wvz = wind.vector.z;
        }
    }
    sim_cloud_pub_->publish(wind_cloud);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Sim tick t=%.1fs | wind=(%.2f, %.2f) | hotspot≈(%.1f,%.1f)",
      t_, wind.vector.x, wind.vector.y, cx, cy);
}
