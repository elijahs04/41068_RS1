#include "thermal_pyrosens/thermalSimNode.hpp"
#include <algorithm>
#include <cmath>

ThermalSimNode::ThermalSimNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("thermal_sim_node", opts)
{
  // ----- Parameters -----
  frame_id_   = declare_parameter<std::string>("frame_id", "map");
  spacing_    = declare_parameter<double>("spacing", 0.5);
  extent_     = declare_parameter<double>("extent", 10.0);
  period_ms_  = declare_parameter<int>("period_ms", 60);
  z_height_   = declare_parameter<double>("z", 0.0);

  T_ambient_  = declare_parameter<double>("T_ambient", 25.0);
  T_peak_     = declare_parameter<double>("T_peak",    550.0);
  noise_std_  = declare_parameter<double>("noise_std", 1.5);

  center_x_   = declare_parameter<double>("center_x", 0.0);
  center_y_   = declare_parameter<double>("center_y", 0.0);
  dr_         = declare_parameter<double>("dr", 0.25);
  dtheta_deg_ = declare_parameter<double>("dtheta_deg", 2.0);
  omega_rps_  = declare_parameter<double>("omega_rps", 0.0);
  scan_r_max_ = declare_parameter<double>("scan_r_max", 10.0);
  samples_per_tick_ = declare_parameter<int>("samples_per_tick", 5);

  blob_n_     = declare_parameter<double>("blob_n",    2.6);
  blob_beta_  = declare_parameter<double>("blob_beta", 8.0);

  init_blob_from_points_({
    std::make_pair(-5.0, 6.0),
    std::make_pair( 5.0, 7.0),
    std::make_pair(-3.0, 3.0),
    std::make_pair( 4.0, 2.0)
  });

  rng_.seed(std::random_device{}());
  distN_ = std::normal_distribution<double>(0.0, noise_std_);

  temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/heat/temperature", 10);
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/heat/sample_point", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms_),
    std::bind(&ThermalSimNode::tick_, this)
  );

  RCLCPP_INFO(
    get_logger(),
    "ThermalSim starburst: r_max=%.1fm, dr=%.2fm, dtheta=%.2fdeg (ω=%.2f rps), blob_c=(%.2f,%.2f) a=%.2f b=%.2f",
    scan_r_max_, dr_, dtheta_deg_, omega_rps_, blob_cx_, blob_cy_, blob_a_, blob_b_);
}

void ThermalSimNode::init_blob_from_points_(const std::array<std::pair<double,double>,4>& pts) {
  double sx=0.0, sy=0.0;
  for (auto& p : pts) { sx += p.first; sy += p.second; }
  blob_cx_ = sx / pts.size();
  blob_cy_ = sy / pts.size();

  double minx=pts[0].first, maxx=minx, miny=pts[0].second, maxy=miny;
  for (auto& p : pts) {
    minx = std::min(minx, p.first);  maxx = std::max(maxx, p.first);
    miny = std::min(miny, p.second); maxy = std::max(maxy, p.second);
  }
  blob_a_ = std::max(0.5, 0.5 * (maxx - minx));
  blob_b_ = std::max(0.5, 0.5 * (maxy - miny));
}

double ThermalSimNode::temperature_at_(double x, double y) const {
  const double xc = x - blob_cx_;
  const double yc = y - blob_cy_;
  const double rx = std::pow(std::abs(xc) / blob_a_, blob_n_);
  const double ry = std::pow(std::abs(yc) / blob_b_, blob_n_);
  const double rho = rx + ry;                 // <=1 inside blob
  const double edge = std::max(0.0, rho - 1.0);
  const double gain = std::exp(-blob_beta_ * edge * edge);
  return T_ambient_ + (T_peak_ - T_ambient_) * gain;
}

void ThermalSimNode::tick_() {
  for (int n = 0; n < samples_per_tick_; ++n) {
    const double th = scan_theta_deg_ * M_PI / 180.0;
    const double x  = center_x_ + r_ * std::cos(th);
    const double y  = center_y_ + r_ * std::sin(th);

    double T = temperature_at_(x, y) + distN_(rng_);
    if (T < T_ambient_) T = T_ambient_;
    if (T > T_peak_)    T = T_peak_;

    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = now();
    pt.header.frame_id = frame_id_;
    pt.point.x = x; pt.point.y = y; pt.point.z = z_height_;
    point_pub_->publish(pt);

    sensor_msgs::msg::Temperature msg;
    msg.header = pt.header;
    msg.temperature = T;
    msg.variance = noise_std_ * noise_std_;
    temp_pub_->publish(msg);

    r_ += dr_;
    if (r_ > scan_r_max_) {
      r_ = 0.0;
      double dtheta_now_deg = dtheta_deg_;
      if (omega_rps_ > 0.0) {
        const double dt = static_cast<double>(period_ms_) / 1000.0;
        dtheta_now_deg = omega_rps_ * 360.0 * dt;
      }
      scan_theta_deg_ += dtheta_now_deg;
      if (scan_theta_deg_ >= 360.0) scan_theta_deg_ -= 360.0;
    }
  }

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "Starburst @ θ=%.1f r=%.2f (burst=%d)", scan_theta_deg_, r_, samples_per_tick_);
}
