#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <random>
#include <cmath>

class ThermalSimNode : public rclcpp::Node {
public:
  ThermalSimNode() : rclcpp::Node("thermal_sim_node") {
    // ----- Parameters (override via ros2 param set) -----
    frame_id_     = declare_parameter<std::string>("frame_id", "map");
    spacing_      = declare_parameter<double>("spacing", 0.5);          // [m] (kept for reference)
    extent_       = declare_parameter<double>("extent", 5.0);           // [m] half-size => 10x10m area
    period_ms_    = declare_parameter<int>("period_ms", 80);            // [ms] sample period (sensor rate)
    z_height_     = declare_parameter<double>("z", 0.0);                // [m] sensor/ground plane height

    // Elliptical "big fire spot" (rotated Gaussian)
    hot_x_             = declare_parameter<double>("hot_x", 0.0);       // [m]
    hot_y_             = declare_parameter<double>("hot_y", 0.0);       // [m]
    sigma_x_           = declare_parameter<double>("sigma_x", 1.2);     // [m] ellipse axis
    sigma_y_           = declare_parameter<double>("sigma_y", 0.6);     // [m] ellipse axis
    ellipse_theta_deg_ = declare_parameter<double>("theta_deg", 25.0);  // [deg] ellipse rotation (CCW)

    T_ambient_    = declare_parameter<double>("T_ambient", 25.0);       // [°C]
    T_peak_       = declare_parameter<double>("T_peak", 550.0);         // [°C]
    noise_std_    = declare_parameter<double>("noise_std", 1.5);        // [°C] sensor noise (1σ)

    // Radar scan params
    center_x_     = declare_parameter<double>("center_x", 0.0);
    center_y_     = declare_parameter<double>("center_y", 0.0);
    dr_           = declare_parameter<double>("dr", 0.5);               // radial spacing per revolution
    dtheta_deg_   = declare_parameter<double>("dtheta_deg", 2.0);       // per-tick angle step [deg]
    omega_rps_    = declare_parameter<double>("omega_rps", 0.0);        // >0 => compute dtheta from ω·dt

    // --- Precompute ellipse rotation terms ---
    const double th = ellipse_theta_deg_ * M_PI / 180.0;
    c_ = std::cos(th);
    s_ = std::sin(th);
    inv2sx2_ = 1.0 / (2.0 * sigma_x_ * sigma_x_);
    inv2sy2_ = 1.0 / (2.0 * sigma_y_ * sigma_y_);

    // RNG
    rng_.seed(std::random_device{}());
    distN_ = std::normal_distribution<double>(0.0, noise_std_);

    // Publishers
    temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/heat/temperature", 10);
    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/heat/sample_point", 10);

    // Timer (sensor cadence)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&ThermalSimNode::tick_, this)
    );

    RCLCPP_INFO(get_logger(),
      "ThermalSim radar: area=%.1fm x %.1fm, dr=%.2f, dtheta=%.2fdeg, T_peak=%.1fC, ambient=%.1fC",
      2*extent_, 2*extent_, dr_, (omega_rps_>0? -1.0 : dtheta_deg_), T_peak_, T_ambient_);
  }

private:
  // Elliptical rotated Gaussian temperature model
  inline double temperature_at_(double x, double y) const {
    const double dx = x - hot_x_;
    const double dy = y - hot_y_;
    // rotate into ellipse frame
    const double xr =  c_*dx + s_*dy;
    const double yr = -s_*dx + c_*dy;
    const double expo = -(xr*xr)*inv2sx2_ - (yr*yr)*inv2sy2_;
    return T_ambient_ + (T_peak_ - T_ambient_) * std::exp(expo);
  }

  void tick_() {
    // Compute angular step
    double dtheta_now_deg = dtheta_deg_;
    if (omega_rps_ > 0.0) {
      const double dt = static_cast<double>(period_ms_) / 1000.0;
      dtheta_now_deg = omega_rps_ * 360.0 * dt;
    }

    // Advance angle; when we wrap, move outward one ring
    scan_theta_deg_ += dtheta_now_deg;
    bool wrapped = false;
    if (scan_theta_deg_ >= 360.0) {
      scan_theta_deg_ -= 360.0;
      wrapped = true;
    }
    if (wrapped) {
      r_ += dr_;
      if (r_ > extent_) {
        r_ = 0.0;               // restart from centre after full coverage
        scan_theta_deg_ = 0.0;
      }
    }

    // Convert polar -> Cartesian (map frame)
    const double th = scan_theta_deg_ * M_PI / 180.0;
    const double x = center_x_ + r_ * std::cos(th);
    const double y = center_y_ + r_ * std::sin(th);
    const double Traw = temperature_at_(x, y) + distN_(rng_);

    // clamp to ambient..peak
    double T = Traw;
    if (T < T_ambient_) T = T_ambient_;
    if (T > T_peak_)    T = T_peak_;

    // Publish point and temperature
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = now();
    pt.header.frame_id = frame_id_;
    pt.point.x = x;
    pt.point.y = y;
    pt.point.z = z_height_;
    point_pub_->publish(pt);

    sensor_msgs::msg::Temperature msg;
    msg.header = pt.header;
    msg.temperature = T;
    msg.variance = noise_std_*noise_std_;
    temp_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Radar sample r=%.2f th=%.1f -> (x=%.2f,y=%.2f) T=%.1f°C",
      r_, scan_theta_deg_, x, y, T);
  }

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params / state
  std::string frame_id_;
  double spacing_{0.5}, extent_{5.0}, z_height_{0.0};
  int period_ms_{80};

  // Fire spot (elliptical Gaussian) + rotation
  double hot_x_{0.0}, hot_y_{0.0}, sigma_x_{1.2}, sigma_y_{0.6}, ellipse_theta_deg_{25.0};
  double T_ambient_{25.0}, T_peak_{550.0}, noise_std_{1.5};

  // Precomputed for ellipse rotation
  double c_{1.0}, s_{0.0}, inv2sx2_{1.0}, inv2sy2_{1.0};

  // RNG
  std::mt19937 rng_;
  std::normal_distribution<double> distN_;

  // --- Radar scan params ---
  double center_x_{0.0}, center_y_{0.0};  // map centre
  double dr_{0.5};                         // radial step per revolution [m]
  double dtheta_deg_{2.0};                 // angular resolution per tick [deg]
  double omega_rps_{0.0};                  // angular speed [rev/sec] (0 => use dtheta_deg)

  // --- Radar scan state ---
  double scan_theta_deg_{0.0};             // current angle [deg]
  double r_{0.0};                          // current radius [m]
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSimNode>());
  rclcpp::shutdown();
  return 0;
}

