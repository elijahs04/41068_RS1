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
    


