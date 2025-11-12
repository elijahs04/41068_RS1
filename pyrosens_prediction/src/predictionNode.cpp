#include "pyrosens_prediction/predictionNode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <sstream>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"

using std::placeholders::_1;

// Constructor
PredictionNode::PredictionNode() : Node("prediction_node")
{
  // Declare parameters
  timer_period_ = this->declare_parameter("timer_period", 1.0);
  predict_time_ = this->declare_parameter("predict_time", 10.0);
  predict_step_ = this->declare_parameter("predict_step", 1.0);
  wind_sample_retention_sec_ = this->declare_parameter("wind_sample_retention_sec", 5.0);
  wind_idw_radius_ = this->declare_parameter("wind_idw_radius", 2.0);
  heat_idw_radius_ = this->declare_parameter("heat_idw_radius", 2.5);
  idw_power_ = this->declare_parameter("idw_power", 2.0);
  idw_min_neighbors_ = static_cast<std::size_t>(this->declare_parameter<int>("idw_min_neighbors", 3));
  gradient_arrow_scale_ = this->declare_parameter("gradient_arrow_scale", 0.5);
  heat_color_min_ = this->declare_parameter("heat_color_min", 25.0);
  heat_color_max_ = this->declare_parameter("heat_color_max", 600.0);
  wind_sample_limit_ = static_cast<std::size_t>(this->declare_parameter<int>("wind_sample_limit", 500));
  path_visualization_limit_ = static_cast<std::size_t>(this->declare_parameter<int>("path_visualization_limit", 200));
  wind_point_topic_ = this->declare_parameter<std::string>("wind_point_topic", "/wind/point");
  wind_velocity_topic_ = this->declare_parameter<std::string>("wind_velocity_topic", "/wind/velocity");
  heat_samples_topic_ = this->declare_parameter<std::string>("heat_samples_topic", "/hotspots/points_cloud");
  heat_value_field_ = this->declare_parameter<std::string>("heat_value_field", "temp");
  default_heat_temperature_ = this->declare_parameter("default_heat_temperature", 160.0);
  prediction_marker_topic_ = this->declare_parameter<std::string>("prediction_marker_topic", "prediction_markers");
  point_topic_ = this->declare_parameter<std::string>("prediction_point_topic", "point");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

  // Initialise variables
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

  auto field_available = [&msg](const std::string & name) {
    if (name.empty()) {
      return false;
    }
    return std::any_of(msg->fields.begin(), msg->fields.end(),
                       [&name](const sensor_msgs::msg::PointField & field) {
                         return field.name == name;
                       });
  };

  std::vector<std::string> candidates;
  candidates.reserve(5);
  candidates.push_back(heat_value_field_);
  if (heat_value_field_ != "temp") {
    candidates.push_back("temp");
  }
  if (heat_value_field_ != "temperature") {
    candidates.push_back("temperature");
  }
  if (heat_value_field_ != "intensity") {
    candidates.push_back("intensity");
  }
  if (heat_value_field_ != "i") {
    candidates.push_back("i");
  }

  std::string value_field;
  for (const auto & name : candidates) {
    if (field_available(name)) {
      value_field = name;
      break;
    }
  }

  try {
    if (!value_field.empty()) {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> it_val(*msg, value_field);

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_val) {
        HeatSample sample;
        sample.point.x = *it_x;
        sample.point.y = *it_y;
        sample.point.z = *it_z;
        sample.temperature = *it_val;
        heat_samples_.push_back(sample);
      }
      heat_ready_ = !heat_samples_.empty();
      if (heat_ready_) {
        RCLCPP_INFO(this->get_logger(),
                    "Received %zu heat samples using field '%s'.",
                    heat_samples_.size(), value_field.c_str());
      }
    } else {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
        HeatSample sample;
        sample.point.x = *it_x;
        sample.point.y = *it_y;
        sample.point.z = *it_z;
        sample.temperature = static_cast<float>(default_heat_temperature_);
        heat_samples_.push_back(sample);
      }
      heat_ready_ = !heat_samples_.empty();
      if (heat_ready_) {
        RCLCPP_WARN(this->get_logger(),
                    "Heat samples missing value field; defaulting to %.1f°C for %zu points.",
                    default_heat_temperature_, heat_samples_.size());
      }
    }
  } catch (const std::runtime_error & ex) {
    heat_ready_ = false;
    RCLCPP_WARN(this->get_logger(), "Failed to parse heat samples: %s", ex.what());
  }

  if (!heat_ready_) {
    RCLCPP_WARN(this->get_logger(), "Heat data unavailable or empty.");
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

  std::vector<PredictionSample> history;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (prediction_history_.empty()) {
      return;
    }
    history = prediction_history_;
  }

  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = this->now();

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = frame_id_;
  path_marker.header.stamp = stamp;
  path_marker.ns = "prediction";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.04;
  path_marker.color.r = 0.15f;
  path_marker.color.g = 0.9f;
  path_marker.color.b = 0.3f;
  path_marker.color.a = 0.8f;
  path_marker.points.reserve(history.size());
  for (const auto & sample : history) {
    path_marker.points.push_back(sample.point);
  }
  markers.markers.push_back(path_marker);

  visualization_msgs::msg::Marker heat_marker;
  heat_marker.header = path_marker.header;
  heat_marker.ns = "prediction";
  heat_marker.id = 1;
  heat_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  heat_marker.action = visualization_msgs::msg::Marker::ADD;
  heat_marker.scale.x = 0.15;
  heat_marker.scale.y = 0.15;
  heat_marker.scale.z = 0.15;
  heat_marker.points.reserve(history.size());
  heat_marker.colors.reserve(history.size());
  for (const auto & sample : history) {
    heat_marker.points.push_back(sample.point);
    heat_marker.colors.push_back(heat_to_color(sample.heat));
  }
  markers.markers.push_back(heat_marker);

  const auto & tail = history.back();

  visualization_msgs::msg::Marker head_marker;
  head_marker.header = path_marker.header;
  head_marker.ns = "prediction";
  head_marker.id = 2;
  head_marker.type = visualization_msgs::msg::Marker::SPHERE;
  head_marker.action = visualization_msgs::msg::Marker::ADD;
  head_marker.pose.orientation.w = 1.0;
  head_marker.pose.position = tail.point;
  head_marker.scale.x = 0.24;
  head_marker.scale.y = 0.24;
  head_marker.scale.z = 0.24;
  head_marker.color = heat_to_color(tail.heat);
  head_marker.color.a = 1.0f;
  markers.markers.push_back(head_marker);

  if (gradient_arrow_scale_ > 0.0 && tail.gradient_magnitude > 1e-3) {
    visualization_msgs::msg::Marker gradient_marker;
    gradient_marker.header = path_marker.header;
    gradient_marker.ns = "prediction";
    gradient_marker.id = 3;
    gradient_marker.type = visualization_msgs::msg::Marker::ARROW;
    gradient_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start = tail.point;
    geometry_msgs::msg::Point end = tail.point;
    const double scale = gradient_arrow_scale_ / std::max(tail.gradient_magnitude, 1e-6);
    end.x += tail.gradient.x * scale;
    end.y += tail.gradient.y * scale;
    end.z += tail.gradient.z * scale;

    gradient_marker.points.push_back(start);
    gradient_marker.points.push_back(end);
    gradient_marker.scale.x = 0.05;
    gradient_marker.scale.y = 0.1;
    gradient_marker.scale.z = 0.1;
    gradient_marker.color.r = 1.0f;
    gradient_marker.color.g = 0.5f;
    gradient_marker.color.b = 0.0f;
    gradient_marker.color.a = 0.85f;
    markers.markers.push_back(gradient_marker);
  }

  viz_pub_->publish(markers);
}

void PredictionNode::publish_prediction_cloud()
{
  if (!predict_pub_) {
    return;
  }

  std::vector<PredictionSample> history;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (prediction_history_.empty()) {
      return;
    }
    history = prediction_history_;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = frame_id_;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
    10,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "heat", 1, sensor_msgs::msg::PointField::FLOAT32,
    "wind_speed", 1, sensor_msgs::msg::PointField::FLOAT32,
    "grad_x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "grad_y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "grad_z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "wind_neighbors", 1, sensor_msgs::msg::PointField::UINT32,
    "heat_neighbors", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(history.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_heat(cloud_msg, "heat");
  sensor_msgs::PointCloud2Iterator<float> iter_wind_speed(cloud_msg, "wind_speed");
  sensor_msgs::PointCloud2Iterator<float> iter_grad_x(cloud_msg, "grad_x");
  sensor_msgs::PointCloud2Iterator<float> iter_grad_y(cloud_msg, "grad_y");
  sensor_msgs::PointCloud2Iterator<float> iter_grad_z(cloud_msg, "grad_z");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_wind_neighbors(cloud_msg, "wind_neighbors");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_heat_neighbors(cloud_msg, "heat_neighbors");

  for (const auto & sample : history) {
    *iter_x = static_cast<float>(sample.point.x);
    *iter_y = static_cast<float>(sample.point.y);
    *iter_z = static_cast<float>(sample.point.z);
    *iter_heat = sample.heat;
    *iter_wind_speed = static_cast<float>(sample.wind_speed);
    *iter_grad_x = static_cast<float>(sample.gradient.x);
    *iter_grad_y = static_cast<float>(sample.gradient.y);
    *iter_grad_z = static_cast<float>(sample.gradient.z);
    *iter_wind_neighbors = static_cast<uint32_t>(sample.wind_neighbors);
    *iter_heat_neighbors = static_cast<uint32_t>(sample.heat_neighbors);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_heat;
    ++iter_wind_speed;
    ++iter_grad_x;
    ++iter_grad_y;
    ++iter_grad_z;
    ++iter_wind_neighbors;
    ++iter_heat_neighbors;
  }

  cloud_msg.is_dense = true;
  predict_pub_->publish(cloud_msg);
}

// Perform a single prediction step
void PredictionNode::predict_step()
{
  geometry_msgs::msg::Point current_point;
  std::vector<WindSample> wind_samples_copy;
  std::vector<HeatSample> heat_samples_copy;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!point_) {
      RCLCPP_ERROR(this->get_logger(), "Point data not available for prediction step.");
      return;
    }
    current_point = point_->point;
    wind_samples_copy.assign(wind_samples_.begin(), wind_samples_.end());
    heat_samples_copy = heat_samples_;
  }

  auto wind_result = interpolate_wind_idw(current_point, wind_samples_copy);
  if (!wind_result) {
    RCLCPP_WARN(this->get_logger(), "Insufficient wind samples for interpolation.");
    return;
  }

  geometry_msgs::msg::Point predicted_point = current_point;
  predicted_point.x += wind_result->velocity.x * predict_step_;
  predicted_point.y += wind_result->velocity.y * predict_step_;
  predicted_point.z += wind_result->velocity.z * predict_step_;

  const double wind_speed = std::sqrt(
    wind_result->velocity.x * wind_result->velocity.x +
    wind_result->velocity.y * wind_result->velocity.y +
    wind_result->velocity.z * wind_result->velocity.z);

  auto heat_result = interpolate_heat_idw(predicted_point, heat_samples_copy);

  float heat_value = 25.0f;
  geometry_msgs::msg::Vector3 heat_gradient{};
  std::size_t heat_neighbors = 0;
  if (heat_result) {
    heat_value = heat_result->temperature;
    heat_gradient = heat_result->gradient;
    heat_neighbors = heat_result->neighbor_count;
  } else {
    RCLCPP_WARN(this->get_logger(), "Using ambient temperature estimate due to sparse heat data.");
  }

  const double grad_mag = std::sqrt(
    heat_gradient.x * heat_gradient.x +
    heat_gradient.y * heat_gradient.y +
    heat_gradient.z * heat_gradient.z);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!point_) {
      return;
    }
    point_->point = predicted_point;
    wind_x_ = wind_result->velocity.x;
    wind_y_ = wind_result->velocity.y;
    wind_z_ = wind_result->velocity.z;

    PredictionSample sample;
    sample.point = predicted_point;
    sample.heat = heat_value;
    sample.wind_speed = wind_speed;
    sample.gradient = heat_gradient;
    sample.gradient_magnitude = grad_mag;
    sample.wind_neighbors = wind_result->neighbor_count;
    sample.heat_neighbors = heat_neighbors;

    prediction_history_.push_back(sample);
    if (prediction_history_.size() > path_visualization_limit_) {
      prediction_history_.erase(prediction_history_.begin());
    }
  }

  publish_visualization();
  publish_prediction_cloud();
  log_prediction_step(predicted_point,
                      wind_result->velocity,
                      wind_speed,
                      heat_value,
                      heat_gradient,
                      grad_mag,
                      wind_result->neighbor_count,
                      heat_neighbors);
}

void PredictionNode::log_prediction_step(const geometry_msgs::msg::Point & point,
                                         const geometry_msgs::msg::Vector3 & wind,
                                         double wind_speed,
                                         float heat_value,
                                         const geometry_msgs::msg::Vector3 & gradient,
                                         double gradient_magnitude,
                                         std::size_t wind_neighbors,
                                         std::size_t heat_neighbors)
{
  auto format_line = [](const std::string & label, const std::string & value) {
    constexpr int box_width = 63;
    const int base_width = 18;
    const int padding = box_width - base_width - 3;
    const int value_width = std::max(0, padding);
    std::ostringstream line;
    line << "| " << std::left << std::setw(base_width) << label << " "
         << std::setw(value_width) << value << " |";
    return line.str();
  };

  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss << "\n+---------------------------------------------------------------+\n";
  oss << "|                    Prediction Step Summary                    |\n";
  oss << "+---------------------------------------------------------------+\n";

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(2)
        << "(" << point.x << ", " << point.y << ", " << point.z << ") m";
    oss << format_line("Point", tmp.str()) << "\n";
  }

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(2)
        << "(" << wind.x << ", " << wind.y << ", " << wind.z << ") m/s";
    oss << format_line("Wind vector", tmp.str()) << "\n";
  }

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(2) << wind_speed << " m/s (" << wind_neighbors << " samples)";
    oss << format_line("Wind speed", tmp.str()) << "\n";
  }

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(2) << heat_value << " degC (" << heat_neighbors << " samples)";
    oss << format_line("Heat", tmp.str()) << "\n";
  }

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(3) << gradient_magnitude << " degC/m";
    oss << format_line("Gradient |∇T|", tmp.str()) << "\n";
  }

  {
    std::ostringstream tmp;
    tmp.setf(std::ios::fixed);
    tmp << std::setprecision(3)
        << "(" << gradient.x << ", " << gradient.y << ", " << gradient.z << ") degC/m";
    oss << format_line("Gradient dir", tmp.str()) << "\n";
  }

  oss << "+---------------------------------------------------------------+";

  RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
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
std::optional<PredictionNode::WindInterpolationResult> PredictionNode::interpolate_wind_idw(
  const geometry_msgs::msg::Point & point,
  const std::vector<WindSample> & samples) const
{
  if (samples.empty()) {
    return std::nullopt;
  }

  const double eps = 1e-6;
  const WindSample * nearest = nullptr;
  double nearest_dist2 = std::numeric_limits<double>::max();

  double sum_weight = 0.0;
  double accum_x = 0.0;
  double accum_y = 0.0;
  double accum_z = 0.0;
  double max_dist = 0.0;
  std::size_t used = 0;

  for (const auto & sample : samples) {
    const double dx = sample.point.x - point.x;
    const double dy = sample.point.y - point.y;
    const double dz = sample.point.z - point.z;
    const double dist2 = dx * dx + dy * dy + dz * dz;
    if (dist2 < nearest_dist2) {
      nearest = &sample;
      nearest_dist2 = dist2;
    }

    const double dist = std::sqrt(dist2);
    if (wind_idw_radius_ > 0.0 && dist > wind_idw_radius_) {
      continue;
    }

    if (dist < eps) {
      WindInterpolationResult result;
      result.velocity = sample.velocity;
      result.neighbor_count = 1;
      result.total_weight = std::numeric_limits<double>::infinity();
      result.max_distance = 0.0;
      return result;
    }

    const double weight = 1.0 / std::pow(dist, idw_power_);
    accum_x += weight * sample.velocity.x;
    accum_y += weight * sample.velocity.y;
    accum_z += weight * sample.velocity.z;
    sum_weight += weight;
    ++used;
    if (dist > max_dist) {
      max_dist = dist;
    }
  }

  if (used >= idw_min_neighbors_ && sum_weight > 0.0) {
    WindInterpolationResult result;
    result.velocity.x = accum_x / sum_weight;
    result.velocity.y = accum_y / sum_weight;
    result.velocity.z = accum_z / sum_weight;
    result.neighbor_count = used;
    result.total_weight = sum_weight;
    result.max_distance = max_dist;
    return result;
  }

  if (nearest) {
    WindInterpolationResult result;
    result.velocity = nearest->velocity;
    result.neighbor_count = (nearest_dist2 < std::numeric_limits<double>::infinity()) ? 1 : 0;
    result.total_weight = 0.0;
    result.max_distance = std::sqrt(nearest_dist2);
    return result;
  }

  return std::nullopt;
}

std::optional<PredictionNode::HeatInterpolationResult> PredictionNode::interpolate_heat_idw(
  const geometry_msgs::msg::Point & point,
  const std::vector<HeatSample> & samples) const
{
  if (samples.empty()) {
    return std::nullopt;
  }

  struct Neighbor {
    double dx;
    double dy;
    double dz;
    double dist;
    double weight;
    float temperature;
  };

  const double eps = 1e-6;
  const HeatSample * nearest = nullptr;
  double nearest_dist2 = std::numeric_limits<double>::max();

  std::vector<Neighbor> neighbors;
  neighbors.reserve(samples.size());
  double sum_weight = 0.0;
  double weighted_temp = 0.0;
  double max_dist = 0.0;

  for (const auto & sample : samples) {
    const double dx = sample.point.x - point.x;
    const double dy = sample.point.y - point.y;
    const double dz = sample.point.z - point.z;
    const double dist2 = dx * dx + dy * dy + dz * dz;
    if (dist2 < nearest_dist2) {
      nearest = &sample;
      nearest_dist2 = dist2;
    }

    const double dist = std::sqrt(dist2);
    if (heat_idw_radius_ > 0.0 && dist > heat_idw_radius_) {
      continue;
    }

    if (dist < eps) {
      HeatInterpolationResult result;
      result.temperature = sample.temperature;
      result.gradient.x = 0.0;
      result.gradient.y = 0.0;
      result.gradient.z = 0.0;
      result.neighbor_count = 1;
      result.total_weight = std::numeric_limits<double>::infinity();
      result.max_distance = 0.0;
      return result;
    }

    const double weight = 1.0 / std::pow(dist, idw_power_);
    neighbors.push_back({dx, dy, dz, dist, weight, sample.temperature});
    weighted_temp += weight * sample.temperature;
    sum_weight += weight;
    if (dist > max_dist) {
      max_dist = dist;
    }
  }

  if (neighbors.size() >= idw_min_neighbors_ && sum_weight > 0.0) {
    const float temperature = static_cast<float>(weighted_temp / sum_weight);
    geometry_msgs::msg::Vector3 gradient{};
    const double inv_sum_weight = 1.0 / sum_weight;

    for (const auto & n : neighbors) {
      const double unit_x = n.dx / n.dist;
      const double unit_y = n.dy / n.dist;
      const double unit_z = n.dz / n.dist;
      const double diff = static_cast<double>(n.temperature) - temperature;
      const double scaled_weight = (n.weight * inv_sum_weight);
      const double scale = scaled_weight * diff / std::max(n.dist, eps);
      gradient.x += scale * unit_x;
      gradient.y += scale * unit_y;
      gradient.z += scale * unit_z;
    }

    HeatInterpolationResult result;
    result.temperature = temperature;
    result.gradient = gradient;
    result.neighbor_count = neighbors.size();
    result.total_weight = sum_weight;
    result.max_distance = max_dist;
    return result;
  }

  if (nearest) {
    HeatInterpolationResult result;
    result.temperature = nearest->temperature;
    result.gradient.x = 0.0;
    result.gradient.y = 0.0;
    result.gradient.z = 0.0;
    result.neighbor_count = 1;
    result.total_weight = 0.0;
    result.max_distance = std::sqrt(nearest_dist2);
    return result;
  }

  return std::nullopt;
}

std_msgs::msg::ColorRGBA PredictionNode::heat_to_color(float heat) const
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.85f;

  const double range = std::max(1e-3, heat_color_max_ - heat_color_min_);
  double t = (static_cast<double>(heat) - heat_color_min_) / range;
  t = std::clamp(t, 0.0, 1.0);

  if (t < 0.25) {
    const double k = t / 0.25;
    color.r = 0.0f;
    color.g = static_cast<float>(k);
    color.b = 1.0f;
  } else if (t < 0.5) {
    const double k = (t - 0.25) / 0.25;
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = static_cast<float>(1.0 - k);
  } else if (t < 0.75) {
    const double k = (t - 0.5) / 0.25;
    color.r = static_cast<float>(k);
    color.g = 1.0f;
    color.b = 0.0f;
  } else {
    const double k = (t - 0.75) / 0.25;
    color.r = 1.0f;
    color.g = static_cast<float>(1.0 - k);
    color.b = 0.0f;
  }

  return color;
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
