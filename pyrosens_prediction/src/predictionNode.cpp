#include "pyrosens_prediction/predictionNode.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

// Constructor
PredictionNode::PredictionNode() : Node("prediction_node")
{
  // Declare parameters
  timer_period_ = this->declare_parameter("timer_period", 1.0);
  predict_time_ = this->declare_parameter("predict_time", 10.0);
  predict_step_ = this->declare_parameter("predict_step", 1.0);
  wind_sample_retention_sec_ = this->declare_parameter("wind_sample_retention_sec", 5.0);
  wind_sample_limit_ = static_cast<std::size_t>(this->declare_parameter<int>("wind_sample_limit", 500));
  path_visualization_limit_ = static_cast<std::size_t>(this->declare_parameter<int>("path_visualization_limit", 200));
  wind_point_topic_ = this->declare_parameter<std::string>("wind_point_topic", "/wind/point");
  wind_velocity_topic_ = this->declare_parameter<std::string>("wind_velocity_topic", "/wind/velocity");
  heat_samples_topic_ = this->declare_parameter<std::string>("heat_samples_topic", "/heat/samples");
  prediction_marker_topic_ = this->declare_parameter<std::string>("prediction_marker_topic", "prediction_markers");
  point_topic_ = this->declare_parameter<std::string>("prediction_point_topic", "point");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

  // Initialize variables
  wind_x_ = 0.0;
  wind_y_ = 0.0;
  wind_z_ = 0.0;
  wind_ready_ = false;
  heat_ready_ = false;
  point_ready_ = false;

  // Create publisher
  predict_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prediction", 10);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(prediction_marker_topic_, 10);

  // Create subscribers
  auto sensor_qos = rclcpp::SensorDataQoS();

  wind_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    wind_point_topic_, sensor_qos,
    std::bind(&PredictionNode::onWindPoint, this, _1));

  wind_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    wind_velocity_topic_, sensor_qos,
    std::bind(&PredictionNode::onWindVelocity, this, _1));

  heat_samples_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    heat_samples_topic_, sensor_qos,
    std::bind(&PredictionNode::onHeatSamples, this, _1));

  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    point_topic_, 10, std::bind(&PredictionNode::onPoint, this, _1));

  // Create timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_), std::bind(&PredictionNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Prediction Node has been started.");
}

// Timer callback
void PredictionNode::onTimer()
{
  bool ready = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ready = wind_ready_ && !wind_samples_.empty() && heat_ready_ && !heat_samples_.empty() && point_ready_;
  }

  if (ready) {
    RCLCPP_INFO(this->get_logger(), "All data ready, starting prediction.");
    onPredict();
  } else {
    RCLCPP_WARN(this->get_logger(), "Waiting for all data to be ready...");
  }
}

// Wind point callback
void PredictionNode::onWindPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const int64_t key = rclcpp::Time(msg->header.stamp).nanoseconds();
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_wind_points_[key] = *msg;
  try_pair_wind(key);
}

// Wind velocity callback
void PredictionNode::onWindVelocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  const int64_t key = rclcpp::Time(msg->header.stamp).nanoseconds();
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_wind_velocities_[key] = *msg;
  try_pair_wind(key);
}

// Heat callback
void PredictionNode::onHeatSamples(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  heat_samples_.clear();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_temp(*msg, "temp");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_temp) {
      HeatSample sample;
      sample.point.x = *it_x;
      sample.point.y = *it_y;
      sample.point.z = *it_z;
      sample.temperature = *it_temp;
      heat_samples_.push_back(sample);
    }
    heat_ready_ = !heat_samples_.empty();
    RCLCPP_INFO(this->get_logger(), "Received %zu heat samples.", heat_samples_.size());
  } catch (const std::runtime_error & ex) {
    heat_ready_ = false;
    RCLCPP_WARN(this->get_logger(), "Failed to parse heat samples: %s", ex.what());
  }
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

void PredictionNode::publish_visualization()
{
  if (!viz_pub_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> path_copy;
  geometry_msgs::msg::Point tail;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (predicted_path_.empty()) {
      return;
    }
    path_copy = predicted_path_;
    tail = predicted_path_.back();
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = frame_id_;
  path_marker.header.stamp = this->now();
  path_marker.ns = "prediction";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.05;
  path_marker.color.r = 0.1f;
  path_marker.color.g = 0.8f;
  path_marker.color.b = 0.2f;
  path_marker.color.a = 1.0f;
  path_marker.points = path_copy;
  markers.markers.push_back(path_marker);

  visualization_msgs::msg::Marker head_marker = path_marker;
  head_marker.id = 1;
  head_marker.type = visualization_msgs::msg::Marker::SPHERE;
  head_marker.scale.x = 0.2;
  head_marker.scale.y = 0.2;
  head_marker.scale.z = 0.2;
  head_marker.pose.orientation.w = 1.0;
  head_marker.points.clear();
  head_marker.pose.position = tail;
  head_marker.color.r = 0.9f;
  head_marker.color.g = 0.2f;
  head_marker.color.b = 0.1f;
  head_marker.color.a = 0.9f;
  markers.markers.push_back(head_marker);

  viz_pub_->publish(markers);
}

// Perform a single prediction step
void PredictionNode::predict_step()
{
  geometry_msgs::msg::Point current_point;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!point_) {
      RCLCPP_ERROR(this->get_logger(), "Point data not available for prediction step.");
      return;
    }
    current_point = point_->point;
  }

  auto wind_sample = nearest_wind_sample(current_point);
  if (!wind_sample) {
    RCLCPP_WARN(this->get_logger(), "No wind samples available for interpolation.");
    return;
  }

  geometry_msgs::msg::Point predicted_point;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!point_) {
      return;
    }
    point_->point.x += wind_sample->velocity.x * predict_step_;
    point_->point.y += wind_sample->velocity.y * predict_step_;
    point_->point.z += wind_sample->velocity.z * predict_step_;
    predicted_point = point_->point;

    wind_x_ = wind_sample->velocity.x;
    wind_y_ = wind_sample->velocity.y;
    wind_z_ = wind_sample->velocity.z;

    predicted_path_.push_back(predicted_point);
    if (predicted_path_.size() > path_visualization_limit_) {
      predicted_path_.erase(predicted_path_.begin());
    }
  }

  // Interpolate heat at new position
  float heat = interpolate_heat(predicted_point.x, predicted_point.y, predicted_point.z);

  RCLCPP_INFO(this->get_logger(),
              "Predicted Point: (%.2f, %.2f, %.2f)  Wind: (%.2f, %.2f, %.2f)  Heat: %.2f",
              predicted_point.x, predicted_point.y, predicted_point.z,
              wind_sample->velocity.x, wind_sample->velocity.y, wind_sample->velocity.z,
              heat);

  publish_visualization();

  // Publish prediction as a PointCloud2 message
  auto prediction_msg = sensor_msgs::msg::PointCloud2();
  prediction_msg.header.stamp = this->now();
  prediction_msg.header.frame_id = frame_id_;

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
  geometry_msgs::msg::Point query;
  query.x = x;
  query.y = y;
  query.z = z;

  auto heat = nearest_heat_sample(query);
  if (heat) {
    return *heat;
  }

  // Fallback to ambient temperature assumption
  return 25.0f;
}
// Interpolate wind x component at a given point
float PredictionNode::interpolate_wind_x(float x, float y, float z)
{
  geometry_msgs::msg::Point query;
  query.x = x;
  query.y = y;
  query.z = z;

  auto wind = nearest_wind_sample(query);
  if (wind) {
    return static_cast<float>(wind->velocity.x);
  }
  return static_cast<float>(wind_x_);
}
// Interpolate wind y component at a given point
float PredictionNode::interpolate_wind_y(float x, float y, float z)
{
  geometry_msgs::msg::Point query;
  query.x = x;
  query.y = y;
  query.z = z;

  auto wind = nearest_wind_sample(query);
  if (wind) {
    return static_cast<float>(wind->velocity.y);
  }
  return static_cast<float>(wind_y_);
}
// Interpolate wind z component at a given point
float PredictionNode::interpolate_wind_z(float x, float y, float z)
{
  geometry_msgs::msg::Point query;
  query.x = x;
  query.y = y;
  query.z = z;

  auto wind = nearest_wind_sample(query);
  if (wind) {
    return static_cast<float>(wind->velocity.z);
  }
  return static_cast<float>(wind_z_);
}

std::optional<PredictionNode::WindSample> PredictionNode::nearest_wind_sample(const geometry_msgs::msg::Point & point) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (wind_samples_.empty()) {
    return std::nullopt;
  }

  const WindSample * closest = nullptr;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto & sample : wind_samples_) {
    const double dx = sample.point.x - point.x;
    const double dy = sample.point.y - point.y;
    const double dz = sample.point.z - point.z;
    const double dist = dx * dx + dy * dy + dz * dz;
    if (dist < best_dist) {
      best_dist = dist;
      closest = &sample;
    }
  }

  if (!closest) {
    return std::nullopt;
  }
  return *closest;
}

std::optional<float> PredictionNode::nearest_heat_sample(const geometry_msgs::msg::Point & point) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (heat_samples_.empty()) {
    return std::nullopt;
  }

  const HeatSample * closest = nullptr;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto & sample : heat_samples_) {
    const double dx = sample.point.x - point.x;
    const double dy = sample.point.y - point.y;
    const double dz = sample.point.z - point.z;
    const double dist = dx * dx + dy * dy + dz * dz;
    if (dist < best_dist) {
      best_dist = dist;
      closest = &sample;
    }
  }

  if (!closest) {
    return std::nullopt;
  }
  return closest->temperature;
}

void PredictionNode::prune_wind_samples(const rclcpp::Time & now)
{
  if (wind_sample_retention_sec_ > 0.0) {
    while (!wind_samples_.empty()) {
      const auto & sample = wind_samples_.front();
      const double age = (now - sample.stamp).seconds();
      if (age > wind_sample_retention_sec_) {
        wind_samples_.pop_front();
      } else {
        break;
      }
    }
  }

  while (wind_samples_.size() > wind_sample_limit_) {
    wind_samples_.pop_front();
  }

  wind_ready_ = !wind_samples_.empty();
}

void PredictionNode::try_pair_wind(int64_t stamp)
{
  auto point_it = pending_wind_points_.find(stamp);
  auto vel_it = pending_wind_velocities_.find(stamp);
  if (point_it == pending_wind_points_.end() || vel_it == pending_wind_velocities_.end()) {
    return;
  }

  WindSample sample;
  sample.point = point_it->second.point;
  sample.velocity = vel_it->second.vector;
  sample.stamp = point_it->second.header.stamp;

  wind_samples_.push_back(sample);
  wind_ready_ = true;
  prune_wind_samples(this->now());

  pending_wind_points_.erase(point_it);
  pending_wind_velocities_.erase(vel_it);

  // Prevent unbounded cache growth
  const std::size_t max_pending = std::max<std::size_t>(wind_sample_limit_, 100U);
  while (pending_wind_points_.size() > max_pending) {
    pending_wind_points_.erase(pending_wind_points_.begin());
  }
  while (pending_wind_velocities_.size() > max_pending) {
    pending_wind_velocities_.erase(pending_wind_velocities_.begin());
  }
}
