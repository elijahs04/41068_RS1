#include "thermal_pyrosens/thermSensNode.hpp"

ThermSensNode::ThermSensNode() 
: Node("therm_sens_node")
{
  // declare parameters
  this->declare_parameter<std::string>("mode", "SIM");
  this->declare_parameter<double>("sample_rate_hz", 5.0);
  this->declare_parameter<double>("fov_deg", 60.0);
  this->declare_parameter<double>("max_range_m", 20.0);
  this->declare_parameter<double>("noise_std_c", 0.0);
  this->declare_parameter<double>("dropout_prob", 0.0);

  // TODO: init publishers, subscribers, timers
}

void ThermSensNode::onSimPoints(const sensor_msgs::msg::PointCloud & msg)
{
  (void)msg;
  // TODO: handle incoming sim data
}

void ThermSensNode::tick()
{
  // TODO: periodic sampling or publishing
}
