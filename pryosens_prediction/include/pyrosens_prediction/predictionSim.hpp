/*
the prediction sim is the source of heat, wind, and point cloud data for the prediction node to work with 
it needs to publish this data to topics that the prediction node can subscribe to
the prediction node then takes this data, interpolates it, and outputs a prediction topic
the prediction node is the logic of the data it is fed (wind, heat, point stamps)
the wind is running (vector), heat is (here), so it should go here to here
basic flow: it will move from point in direction
advanced flow: it will move from point to point
*/

#pragma once 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>

class PredictionSim : public rclcpp::Node {
public:
  PredictionSim();
private:
    void publish_data();

    // publishers + timer
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sim_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heat_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // parameters
    double period_s_{0.5};
    std::string frame_id_{"map"};
    int nx_{40}, ny_{40};
    double res_{5.0}; 

    // hotspot 
    double heat_cx_{60.0}, heat_cy_{0};
    double heat_sigma_{25.0}; // gaussian width (m)

    // wind
    double wind_base_x_{2.5}, wind_base_y_{0.8}, wind_base_z_{0.0};

    // point 
    double p0x_{0.0}, p0y_{0.0}, p0z_{0.0};

    // timer
    double t_{0.0};
}; 

    


