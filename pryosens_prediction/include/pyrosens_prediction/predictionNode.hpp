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
#include <chrono>

class PredictionNode : public rclcpp::Node {
public:
  PredictionNode();
private:

// -------- Callbacks --------
    void onTimer();
    void onSimCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onWind(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void onHeat(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onPredict();

// -------- Publishers / Subscribers --------
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr predict_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sim_cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr heat_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

// -------- Parameters --------
    double timer_period_;
    double predict_time_;
    double predict_step_;
    double wind_x_;
    double wind_y_;
    double wind_z_;
    bool wind_ready_;
    bool heat_ready_;
    bool point_ready_;
    sensor_msgs::msg::PointCloud2::SharedPtr sim_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr heat_cloud_;
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
};
