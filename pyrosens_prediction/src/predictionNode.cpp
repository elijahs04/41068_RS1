/* 
source code for predictionNode.hpp
needs to break down heat, wind, point cloud data, and interpolate sensor data to output a prediction topic
takes in predictionSim data, interpolates, and then outputs/publishes to the terminal
*/

#include "pyrosens_prediction/predictionNode.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Constructor
PredictionNode::PredictionNode() : Node("prediction_node")
{
  // Declare parameters
  this->declare_parameter("timer_period", 1.0);
  this->declare_parameter("predict_time", 10.0);
  this->declare_parameter("predict_step", 1.0);

  // Get parameters
  this->get_parameter("timer_period", timer_period_);
  this->get_parameter("predict_time", predict_time_);
  this->get_parameter("predict_step", predict_step_);

  // Initialize variables
  wind_x_ = 0.0;
  wind_y_ = 0.0;
  wind_z_ = 0.0;
  wind_ready_ = false;
  heat_ready_ = false;
  point_ready_ = false;

  // Create publisher
  predict_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prediction", 10);

  // Create subscribers
  sim_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "sim_cloud", 10, std::bind(&PredictionNode::onSimCloud, this, _1));
  
  wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "wind", 10, std::bind(&PredictionNode::onWind, this, _1));
  
  heat_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "heat_cloud", 10, std::bind(&PredictionNode::onHeat, this, _1));
  
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "point", 10, std::bind(&PredictionNode::onPoint, this, _1));

  // Create timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_), std::bind(&PredictionNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Prediction Node has been started.");
}

// Timer callback
void PredictionNode::onTimer()
{
  if (wind_ready_ && heat_ready_ && point_ready_ && sim_cloud_) {
    RCLCPP_INFO(this->get_logger(), "All data ready, starting prediction.");
    onPredict();
  } else {
    RCLCPP_WARN(this->get_logger(), "Waiting for all data to be ready...");
  }
}

// Simulation cloud callback
void PredictionNode::onSimCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sim_cloud_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received simulation cloud data.");
}

// Wind callback
void PredictionNode::onWind(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  wind_x_ = msg->vector.x;
  wind_y_ = msg->vector.y;
  wind_z_ = msg->vector.z;
  wind_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Received wind data: (%.2f, %.2f, %.2f)", wind_x_, wind_y_, wind_z_);
}
// Heat callback
void PredictionNode::onHeat(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  heat_cloud_ = msg;
  heat_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Received heat cloud data.");
}
// Point callback
void PredictionNode::onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  point_ = msg;
  point_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Received point data: (%.2f, %.2f, %.2f)", point_->point.x, point_->point.y, point_->point.z);
}
// Prediction callback
void PredictionNode::onPredict()
{
  double current_time = 0.0;
  while (current_time < predict_time_) {
    predict_step();
    current_time += predict_step_;
  }
  RCLCPP_INFO(this->get_logger(), "Prediction completed.");
}
// Perform a single prediction step
void PredictionNode::predict_step()
{
  if (!point_) {
    RCLCPP_ERROR(this->get_logger(), "Point data not available for prediction step.");
    return;
  }

  // Interpolate wind at current point
  float wind_x = interpolate_wind_x(point_->point.x, point_->point.y, point_->point.z);
  float wind_y = interpolate_wind_y(point_->point.x, point_->point.y, point_->point.z);
  float wind_z = interpolate_wind_z(point_->point.x, point_->point.y, point_->point.z);

  // Update point position based on wind
  point_->point.x += wind_x * predict_step_;
  point_->point.y += wind_y * predict_step_;
  point_->point.z += wind_z * predict_step_;

  // Interpolate heat at new position
  float heat = interpolate_heat(point_->point.x, point_->point.y, point_->point.z);

  RCLCPP_INFO(this->get_logger(), "Predicted Point: (%.2f, %.2f, %.2f) with Heat: %.2f", 
              point_->point.x, point_->point.y, point_->point.z, heat);

  // Publish prediction as a PointCloud2 message
  auto prediction_msg = sensor_msgs::msg::PointCloud2();
  prediction_msg.header.stamp = this->now();
  prediction_msg.header.frame_id = "map";
  // Fill in the rest of the PointCloud2 message fields as needed
  // For simplicity, we are not populating the full PointCloud2 structure here

  predict_pub_->publish(prediction_msg);
}
// Trilinear interpolation function
float PredictionNode::trilinear_interpolation(float x, float y, float z,
                                                float x0, float x1,
                                                float y0, float y1,
                                                float z0, float z1,
                                                float c000, float c100,
                                                float c010, float c110,
                                                float c001, float c101,
                                                float c011, float c111)
    {
    float xd = (x - x0) / (x1 - x0);
    float yd = (y - y0) / (y1 - y0);
    float zd = (z - z0) / (z1 - z0);
    
    float c00 = c000 * (1 - xd) + c100 * xd;
    float c10 = c010 * (1 - xd) + c110 * xd;
    float c01 = c001 * (1 - xd) + c101 * xd;
    float c11 = c011 * (1 - xd) + c111 * xd;
    
    float c0 = c00 * (1 - yd) + c10 * yd;
    float c1 = c01 * (1 - yd) + c11 * yd;
    
    return c0 * (1 - zd) + c1 * zd;
    }
// Interpolate heat at a given point
float PredictionNode::interpolate_heat(float x, float y, float z)
{
  // Placeholder implementation
  // In a real implementation, you would extract the relevant points from heat_cloud_ and perform trilinear interpolation
  return 25.0; // Dummy value
}
// Interpolate wind x component at a given point
float PredictionNode::interpolate_wind_x(float x, float y, float z)
{
  // Placeholder implementation
  // In a real implementation, you would extract the relevant points from sim_cloud_ and perform trilinear interpolation
  return wind_x_; // Using the last received wind value as a dummy
}
// Interpolate wind y component at a given point
float PredictionNode::interpolate_wind_y(float x, float y, float z)
{
  // Placeholder implementation
  // In a real implementation, you would extract the relevant points from sim_cloud_ and perform trilinear interpolation
  return wind_y_; // Using the last received wind value as a dummy
}
// Interpolate wind z component at a given point
float PredictionNode::interpolate_wind_z(float x, float y, float z)
{
  // Placeholder implementation
  // In a real implementation, you would extract the relevant points from sim_cloud_ and perform trilinear interpolation
  return wind_z_; // Using the last received wind value as a dummy
}
/*
main file for prediction node and simulation node
*/ 
