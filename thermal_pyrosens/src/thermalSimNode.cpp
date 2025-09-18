#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <random>
#include <cmath>
#include <array>
#include <algorithm>  // std::clamp

class ThermalSimNode : public rclcpp::Node {
public:
  ThermalSimNode() : rclcpp::Node("thermal_sim_node") {
    // ----- Parameters (override via ros2 param set) -----
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    spacing_    = declare_parameter<double>("spacing", 0.5);         // [m] reference sampling step
    extent_     = declare_parameter<double>("extent", 10.0);         // [m] half-size of map display
    period_ms_  = declare_parameter<int>("period_ms", 60);           // [ms] sensor cadence
    z_height_   = declare_parameter<double>("z", 0.0);               // [m] sample plane height

    // Temperature model (hot blob vs ambient)
    T_ambient_  = declare_parameter<double>("T_ambient", 25.0);      // [°C]
    T_peak_     = declare_parameter<double>("T_peak", 550.0);        // [°C]
    noise_std_  = declare_parameter<double>("noise_std", 1.5);       // [°C] 1σ

    // Starburst scan params (angle fixed -> radii 0..R, then next angle)
    center_x_   = declare_parameter<double>("center_x", 0.0);
    center_y_   = declare_parameter<double>("center_y", 0.0);
    dr_         = declare_parameter<double>("dr", 0.5);              // [m] radial step
    dtheta_deg_ = declare_parameter<double>("dtheta_deg", 2.0);      // [deg] angle step per sweep
    omega_rps_  = declare_parameter<double>("omega_rps", 0.0);       // [rev/s] if >0, overrides dtheta
    scan_r_max_ = declare_parameter<double>("scan_r_max", 10.0);     // [m] max radius for each ray

    // --- Superellipse blob spanning 4 points: (-5,6) (5,7) (-3,3) (4,2) ---
    // You can tune the "roundness" and edge sharpness at runtime:
    blob_n_     = declare_parameter<double>("blob_n",    2.6);       // 2=ellipse, higher=round-box
    blob_beta_  = declare_parameter<double>("blob_beta", 8.0);       // edge falloff strength

    init_blob_from_points_({
      std::make_pair(-5.0, 6.0),
      std::make_pair( 5.0, 7.0),
      std::make_pair(-3.0, 3.0),
      std::make_pair( 4.0, 2.0)
    });

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
      "ThermalSim starburst: r_max=%.1fm, dr=%.2fm, dtheta=%.2fdeg (ω=%.2f rps), blob_c=(%.2f,%.2f) a=%.2f b=%.2f",
      scan_r_max_, dr_, dtheta_deg_, omega_rps_, blob_cx_, blob_cy_, blob_a_, blob_b_);
  }

private:
  // Build a rounded hot region from 4 points (axis-aligned superellipse)
  void init_blob_from_points_(const std::array<std::pair<double,double>,4>& pts) {
    // centre = average of points
    double sx=0.0, sy=0.0;
    for (auto& p : pts) { sx += p.first; sy += p.second; }
    blob_cx_ = sx / pts.size();
    blob_cy_ = sy / pts.size();

    // extents -> semi-axes (≥0.5 m for stability)
    double minx=pts[0].first, maxx=minx, miny=pts[0].second, maxy=miny;
    for (auto& p : pts) {
      minx = std::min(minx, p.first);  maxx = std::max(maxx, p.first);
      miny = std::min(miny, p.second); maxy = std::max(maxy, p.second);
    }
    blob_a_ = std::max(0.5, 0.5 * (maxx - minx));
    blob_b_ = std::max(0.5, 0.5 * (maxy - miny));
  }

  // Superellipse level-set field -> temperature (smooth blob with soft edges)
  inline double temperature_at_(double x, double y) const {
    const double xc = x - blob_cx_;
    const double yc = y - blob_cy_;

    const double rx = std::pow(std::abs(xc) / blob_a_, blob_n_);
    const double ry = std::pow(std::abs(yc) / blob_b_, blob_n_);
    const double rho = rx + ry;              // <=1 inside the blob

    const double edge = std::max(0.0, rho - 1.0);
    const double gain = std::exp(-blob_beta_ * edge * edge);

    return T_ambient_ + (T_peak_ - T_ambient_) * gain;
  }

  void tick_() {
    // Current sample (polar -> Cartesian)
    const double th = scan_theta_deg_ * M_PI / 180.0;
    const double x  = center_x_ + r_ * std::cos(th);
    const double y  = center_y_ + r_ * std::sin(th);

    double T = temperature_at_(x, y) + distN_(rng_);
    T = std::clamp(T, T_ambient_, T_peak_);

    // Publish point + temperature
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

    // Advance radius first; when radius passes max, reset and bump angle
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

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Starburst r=%.2f th=%.1f -> (%.2f,%.2f) T=%.1f°C",
      r_, scan_theta_deg_, x, y, T);
  }

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params / state
  std::string frame_id_;
  double spacing_{0.5}, extent_{10.0}, z_height_{0.0};
  int period_ms_{60};

  // Temperature model
  double T_ambient_{25.0}, T_peak_{550.0}, noise_std_{1.5};

  // RNG
  std::mt19937 rng_;
  std::normal_distribution<double> distN_;

  // Superellipse blob (axis-aligned)
  double blob_cx_{0.0}, blob_cy_{0.0};  // centre
  double blob_a_{1.0}, blob_b_{1.0};    // semi-axes
  double blob_n_{2.6};                  // shape exponent
  double blob_beta_{8.0};               // edge sharpness
  // (points are only used at init; not stored)

  // Starburst scan params/state
  double center_x_{0.0}, center_y_{0.0};
  double dr_{0.5};
  double dtheta_deg_{2.0};
  double omega_rps_{0.0};
  double scan_r_max_{10.0};

  double scan_theta_deg_{0.0};          // [deg]
  double r_{0.0};                        // [m]
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSimNode>());
  rclcpp::shutdown();
  return 0;
}
