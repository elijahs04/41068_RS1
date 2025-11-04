/*
make a constructor 

add ros publisher and subscriber
add functions/parameters 
*/

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <vector>
#include <string>

class PredictionNode : public rclcpp::Node {
public:
  PredictionNode();
private:

// -------- Callbacks --------
    void onTimer();
    void onWindPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onWindVelocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void onHeatSamples(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onPredict();
    void publish_visualization();

// -------- Publishers / Subscribers --------
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr predict_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wind_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr heat_samples_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

// -------- Parameters --------
    double timer_period_;
    double predict_time_;
    double predict_step_;
    double wind_sample_retention_sec_;
    std::size_t wind_sample_limit_;
    std::size_t path_visualization_limit_;
    std::string wind_point_topic_;
    std::string wind_velocity_topic_;
    std::string heat_samples_topic_;
    std::string prediction_marker_topic_;
    std::string point_topic_;
    std::string frame_id_;
    double wind_x_;
    double wind_y_;
    double wind_z_;
    bool wind_ready_;
    bool heat_ready_;
    bool point_ready_;
    geometry_msgs::msg::PointStamped::SharedPtr point_;

// -------- Functions --------
    void predict_step();
    float interpolate_heat(float x, float y, float z);
    float interpolate_wind_x(float x, float y, float z);
    float interpolate_wind_y(float x, float y, float z);
    float interpolate_wind_z(float x, float y, float z);
    float trilinear_interpolation(float x, float y, float z,
                                  float x0, float x1,
                                  float y0, float y1,
                                  float z0, float z1,
                                  float c000, float c100,
                                  float c010, float c110,
                                  float c001, float c101,
                                  float c011, float c111);

    struct WindSample {
      geometry_msgs::msg::Point point;
      geometry_msgs::msg::Vector3 velocity;
      rclcpp::Time stamp;
    };

    struct HeatSample {
      geometry_msgs::msg::Point point;
      float temperature;
    };

    std::optional<WindSample> nearest_wind_sample(const geometry_msgs::msg::Point & point) const;
    std::optional<float> nearest_heat_sample(const geometry_msgs::msg::Point & point) const;
    void prune_wind_samples(const rclcpp::Time & now);
    void try_pair_wind(int64_t stamp);

    mutable std::mutex data_mutex_;
    std::deque<WindSample> wind_samples_;
    std::vector<HeatSample> heat_samples_;
    std::map<int64_t, geometry_msgs::msg::PointStamped> pending_wind_points_;
    std::map<int64_t, geometry_msgs::msg::Vector3Stamped> pending_wind_velocities_;
    std::vector<geometry_msgs::msg::Point> predicted_path_;
};
