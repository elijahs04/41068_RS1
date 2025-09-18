#include "thermal_pyrosens/thermInterpolationNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

ThermInterpolationNode::ThermInterpolationNode()
: rclcpp::Node("therm_interpolation_node")
{
  // -------- Parameters --------
  frame_id_    = this->declare_parameter<std::string>("frame_id", "map");

  // Map geometry
  resolution_  = this->declare_parameter<double>("resolution", 0.25);
  extent_x_    = this->declare_parameter<double>("extent_x", 10.0);
  extent_y_    = this->declare_parameter<double>("extent_y", 10.0);
  origin_x_    = this->declare_parameter<double>("origin_x", -extent_x_);
  origin_y_    = this->declare_parameter<double>("origin_y", -extent_y_);

  // Temperature scaling
  T_ambient_   = this->declare_parameter<double>("T_ambient", 25.0);
  T_peak_      = this->declare_parameter<double>("T_peak", 550.0);

  // Interpolation kernel
  kernel_      = this->declare_parameter<std::string>("kernel", "idw");   // "idw" or "gaussian"
  radius_      = this->declare_parameter<double>("radius", 1.5);
  idw_power_   = this->declare_parameter<double>("idw_power", 2.0);
  gauss_sigma_ = this->declare_parameter<double>("gauss_sigma", 0.6);
  wsum_cap_    = this->declare_parameter<double>("wsum_cap", 100.0);

  // Timing / buffers
  publish_hz_  = this->declare_parameter<double>("publish_hz", 5.0);
  pc_max_points_ = this->declare_parameter<int>("pc_max_points", 30000);
  pair_tol_ms_ = this->declare_parameter<int>("pair_tolerance_ms", 50);
  decay_tau_s_ = this->declare_parameter<double>("decay_tau_s", 0.0); // <=0 disables

  // -------- Map init --------
  width_  = static_cast<int>(std::floor((2.0 * extent_x_) / resolution_)) + 1;
  height_ = static_cast<int>(std::floor((2.0 * extent_y_) / resolution_)) + 1;
  const size_t N = static_cast<size_t>(width_) * static_cast<size_t>(height_);

  wsum_.assign(N, 0.0f);
  tsum_.assign(N, 0.0f);

  grid_.info.resolution = resolution_;
  grid_.info.width  = width_;
  grid_.info.height = height_;
  grid_.info.origin.position.x = origin_x_;
  grid_.info.origin.position.y = origin_y_;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.header.frame_id = frame_id_;
  grid_.data.assign(N, -1);

  // -------- Publishers --------
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/heat/costmap", 10);
  pc_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/heat/samples", 10);

  // -------- Subscribers --------
  sub_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/heat/sample_point", rclcpp::SensorDataQoS(),
      std::bind(&ThermInterpolationNode::onPoint, this, _1));

  sub_temp_  = this->create_subscription<sensor_msgs::msg::Temperature>(
      "/heat/temperature", rclcpp::SensorDataQoS(),
      std::bind(&ThermInterpolationNode::onTemp, this, _1));

  // -------- Timers --------
  const auto pub_period_ms = static_cast<int>(1000.0 / std::max(1e-6, publish_hz_));
  pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pub_period_ms),
      std::bind(&ThermInterpolationNode::publishOutputs, this));

  RCLCPP_INFO(this->get_logger(),
    "ThermInterpolation: map %dx%d @ %.2fm, kernel=%s R=%.2fm (idw=%.1f, sigma=%.2f), T[%.1f..%.1f]",
    width_, height_, resolution_, kernel_.c_str(), radius_, idw_power_, gauss_sigma_, T_ambient_, T_peak_);
}

// ------------------------------------------------------------
// Callbacks
// ------------------------------------------------------------
void ThermInterpolationNode::onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  points_q_.push_back(*msg);
  trimPointsDeque();
  tryPair();
}

void ThermInterpolationNode::onTemp(const sensor_msgs::msg::Temperature::SharedPtr msg) {
  temps_q_.push_back(*msg);
  trimTempsDeque();
  tryPair();
}

void ThermInterpolationNode::tryPair() {
  if (points_q_.empty() || temps_q_.empty()) return;

  const rclcpp::Duration tol(0, static_cast<long>(pair_tol_ms_) * 1000000L);

  // Match newest temperature with closest point
  auto &temp_latest = temps_q_.back();
  auto best_pt_it = points_q_.end();
  rclcpp::Duration best_dt = rclcpp::Duration::from_nanoseconds(std::numeric_limits<int64_t>::max());

  for (auto it = points_q_.begin(); it != points_q_.end(); ++it) {
    rclcpp::Time t1(temp_latest.header.stamp);
      rclcpp::Time t2(it->header.stamp);
      rclcpp::Duration dt = (t1 > t2) ? (t1 - t2) : (t2 - t1);
    if (dt < best_dt) { best_dt = dt; best_pt_it = it; }
  }

  if (best_pt_it != points_q_.end() && best_dt <= tol) {
    processMeasurement(*best_pt_it, temp_latest);
    points_q_.erase(best_pt_it);
    temps_q_.pop_back();
    return;
  }

  // Or match newest point with closest temperature
  auto &pt_latest = points_q_.back();
  auto best_temp_it = temps_q_.end();
  best_dt = rclcpp::Duration::from_nanoseconds(std::numeric_limits<int64_t>::max());

  for (auto it = temps_q_.begin(); it != temps_q_.end(); ++it) {
    rclcpp::Time t1(temp_latest.header.stamp);
      rclcpp::Time t2(it->header.stamp);
      rclcpp::Duration dt = (t1 > t2) ? (t1 - t2) : (t2 - t1);
    if (dt < best_dt) { best_dt = dt; best_temp_it = it; }
  }

  if (best_temp_it != temps_q_.end() && best_dt <= tol) {
    processMeasurement(pt_latest, *best_temp_it);
    points_q_.pop_back();
    temps_q_.erase(best_temp_it);
  }
}

// ------------------------------------------------------------
// Core processing
// ------------------------------------------------------------
void ThermInterpolationNode::processMeasurement(const geometry_msgs::msg::PointStamped& pt,
                                                const sensor_msgs::msg::Temperature& temp)
{
  const double x = pt.point.x;
  const double y = pt.point.y;
  const double T = temp.temperature;

  // 1) Append to rolling samples
  samples_.push_back({x, y, 0.0, T});
  if (static_cast<int>(samples_.size()) > pc_max_points_) samples_.pop_front();

  // 2) Incremental weighted update within radius
  const int i_min = worldToIndexX(x - radius_);
  const int i_max = worldToIndexX(x + radius_);
  const int j_min = worldToIndexY(y - radius_);
  const int j_max = worldToIndexY(y + radius_);

  for (int j = j_min; j <= j_max; ++j) {
    if (j < 0 || j >= height_) continue;
    const double cy = indexToWorldY(j);
    for (int i = i_min; i <= i_max; ++i) {
      if (i < 0 || i >= width_) continue;
      const double cx = indexToWorldX(i);
      const double dx = cx - x;
      const double dy = cy - y;
      const double d  = std::hypot(dx, dy);
      if (d > radius_) continue;

      const float w = kernelWeight(d);
      if (w <= 0.0f) continue;

      const size_t idx = static_cast<size_t>(j) * width_ + static_cast<size_t>(i);

      // Effective weight limited by remaining capacity
      const float remaining = static_cast<float>(wsum_cap_) - wsum_[idx];
      if (remaining <= 0.0f) continue;
      const float w_eff = std::min(w, remaining);

      tsum_[idx] += w_eff * static_cast<float>(T);
      wsum_[idx] += w_eff;
    }
  }
}

// ------------------------------------------------------------
// Publishing
// ------------------------------------------------------------
void ThermInterpolationNode::publishOutputs() {
  // Optional decay of history (approx dt = 1/publish_hz_)
  if (decay_tau_s_ > 0.0) {
    const double dt = 1.0 / std::max(1e-6, publish_hz_);
    const float alpha = static_cast<float>(std::exp(-dt / decay_tau_s_));
    const size_t N = wsum_.size();
    for (size_t k = 0; k < N; ++k) {
      wsum_[k] *= alpha;
      tsum_[k] *= alpha;
    }
  }

  // OccupancyGrid (0..100 cost, -1 unknown)
  grid_.header.stamp = this->now();
  grid_.header.frame_id = frame_id_;
  grid_.info.map_load_time = grid_.header.stamp;

  const size_t N = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  grid_.data.resize(N);

  const double denom = std::max(1e-6, (T_peak_ - T_ambient_));

  for (size_t k = 0; k < N; ++k) {
    if (wsum_[k] > 0.0f) {
      const float Tcell = tsum_[k] / wsum_[k];
      float norm = static_cast<float>((Tcell - T_ambient_) / denom);
      norm = std::clamp(norm, 0.0f, 1.0f);
      grid_.data[k] = static_cast<int8_t>(std::lround(norm * 100.0f)); // 0..100
    } else {
      grid_.data[k] = -1; // unknown
    }
  }
  grid_pub_->publish(grid_);

  // Rolling samples cloud
  publishSampleCloud();
}

void ThermInterpolationNode::publishSampleCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id_;
  cloud.header.stamp = this->now();

  const size_t N = samples_.size();

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "temp", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  mod.resize(N);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_T(cloud, "temp");

  for (const auto& s : samples_) {
    *it_x = static_cast<float>(s.x);
    *it_y = static_cast<float>(s.y);
    *it_z = static_cast<float>(s.z);
    *it_T = static_cast<float>(s.T);
    ++it_x; ++it_y; ++it_z; ++it_T;
  }

  pc_pub_->publish(cloud);
}

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
float ThermInterpolationNode::kernelWeight(double d) const {
  if (d <= 1e-6) return 1.0f;
  if (kernel_ == "gaussian") {
    const double s2 = gauss_sigma_ * gauss_sigma_;
    return static_cast<float>(std::exp(-0.5 * (d*d) / s2));
  }
  // IDW
  return static_cast<float>(1.0 / std::pow(std::max(1e-6, d), idw_power_));
}

int ThermInterpolationNode::worldToIndexX(double x) const {
  return static_cast<int>(std::floor((x - origin_x_) / resolution_));
}
int ThermInterpolationNode::worldToIndexY(double y) const {
  return static_cast<int>(std::floor((y - origin_y_) / resolution_));
}
double ThermInterpolationNode::indexToWorldX(int i) const {
  return origin_x_ + (i + 0.5) * resolution_;
}
double ThermInterpolationNode::indexToWorldY(int j) const {
  return origin_y_ + (j + 0.5) * resolution_;
}

void ThermInterpolationNode::trimPointsDeque(std::size_t max_keep) {
  while (points_q_.size() > max_keep) points_q_.pop_front();
}
void ThermInterpolationNode::trimTempsDeque(std::size_t max_keep) {
  while (temps_q_.size() > max_keep) temps_q_.pop_front();
}


