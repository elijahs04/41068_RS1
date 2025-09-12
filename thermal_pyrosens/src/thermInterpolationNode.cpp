#include "thermal_pyrosens/thermInterpolationNode.hpp"

ThermInterpolationNode::ThermInterpolationNode()
: Node("therm_interpolation_node")
{
  // declare parameters
  this->declare_parameter<double>("grid_resolution", 0.5);
  this->declare_parameter<int>("grid_width", 40);
  this->declare_parameter<int>("grid_height", 40);
  this->declare_parameter<double>("origin_x", 0.0);
  this->declare_parameter<double>("origin_y", 0.0);
  this->declare_parameter<double>("publish_rate_hz", 1.0);
  this->declare_parameter<std::string>("kernel", "IDW");
  this->declare_parameter<double>("decay_tau_s", 30.0);
  this->declare_parameter<double>("ambient_temp_c", 22.0);

  // TODO: init subscribers, publishers, timers
}

void ThermInterpolationNode::onSamples(const sensor_msgs::msg::PointCloud & msg)
{
  // TODO: process incoming samples into heatmap
}

void ThermInterpolationNode::publishHeatmap()
{
  // TODO: publish occupancy grid
}


