#include "pyrosens_prediction/predictionSim.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <thread>

#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

PredictionSim::PredictionSim()
: Node("prediction_sim"), publish_period_(1.0), sample_index_(0)
{
  publish_period_ = this->declare_parameter("publish_period", publish_period_);
  enable_fake_sensors_ = this->declare_parameter<bool>("enable_fake_sensors", false);

  const std::string point_mode_param = this->declare_parameter<std::string>("point_mode", "manual");
  cloud_topic_ = this->declare_parameter<std::string>("point_cloud_topic", "/hotspots/points_cloud");
  manual_point_frame_ = this->declare_parameter<std::string>("manual_point_frame", "map");

  point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);

  if (enable_fake_sensors_) {
    sim_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sim_cloud", 10);
    wind_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("wind", 10);
    heat_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("heat_cloud", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&PredictionSim::publish_fake_data, this));

    RCLCPP_INFO(this->get_logger(),
                "PredictionSim fake sensors enabled (period %.2f s).",
                publish_period_);
  }

  if (point_mode_param == "manual") {
    point_mode_ = PointMode::Manual;
  } else if (point_mode_param == "cloud") {
    point_mode_ = PointMode::Cloud;
  } else {
    point_mode_ = PointMode::Disabled;
  }

  switch (point_mode_) {
    case PointMode::Manual:
      running_.store(true);
      input_thread_ = std::thread(&PredictionSim::manual_input_loop, this);
      RCLCPP_INFO(this->get_logger(),
                  "PredictionSim manual point mode active. Enter 'x y z' then press Enter (q to stop).\n");
      break;
    case PointMode::Cloud: {
      rclcpp::SensorDataQoS qos;
      qos.best_effort();
      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos,
        std::bind(&PredictionSim::onCloud, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
                  "PredictionSim cloud point mode listening to %s.",
                  cloud_topic_.c_str());
      break;
    }
    case PointMode::Disabled:
      RCLCPP_INFO(this->get_logger(), "PredictionSim point publishing disabled.");
      break;
  }

  RCLCPP_INFO(this->get_logger(),
              "PredictionSim configured (fake_sensors=%s, point_mode=%s).",
              enable_fake_sensors_ ? "on" : "off",
              point_mode_param.c_str());
}

PredictionSim::~PredictionSim()
{
  running_.store(false);
  if (input_thread_.joinable()) {
    input_thread_.join();
  }
}

void PredictionSim::publish_fake_data()
{
  if (!sim_cloud_pub_ || !wind_pub_ || !heat_cloud_pub_ || !point_pub_) {
    return;
  }

  const auto stamp = this->now();

  sensor_msgs::msg::PointCloud2 sim_cloud_msg;
  sim_cloud_msg.header.stamp = stamp;
  sim_cloud_msg.header.frame_id = "map";
  sim_cloud_msg.height = 1;
  sim_cloud_msg.width = 1;
  sim_cloud_msg.is_bigendian = false;
  sim_cloud_msg.is_dense = true;
  sim_cloud_msg.point_step = 0;
  sim_cloud_msg.row_step = 0;
  sim_cloud_pub_->publish(sim_cloud_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy sim cloud sample %zu", sample_index_);

  geometry_msgs::msg::Vector3Stamped wind_msg;
  wind_msg.header.stamp = stamp;
  wind_msg.header.frame_id = "map";
  wind_msg.vector.x = std::cos(static_cast<double>(sample_index_) / 4.0);
  wind_msg.vector.y = std::sin(static_cast<double>(sample_index_) / 4.0);
  wind_msg.vector.z = 0.25 * static_cast<double>(sample_index_ % 4);
  wind_pub_->publish(wind_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy wind (%.2f, %.2f, %.2f)",
              wind_msg.vector.x, wind_msg.vector.y, wind_msg.vector.z);

  sensor_msgs::msg::PointCloud2 heat_cloud_msg = sim_cloud_msg;
  heat_cloud_pub_->publish(heat_cloud_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy heat cloud sample %zu", sample_index_);

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.stamp = stamp;
  point_msg.header.frame_id = "map";
  point_msg.point.x = 1.0 + 0.5 * static_cast<double>(sample_index_);
  point_msg.point.y = -1.0 + 0.25 * static_cast<double>(sample_index_);
  point_msg.point.z = 0.5;
  point_pub_->publish(point_msg);
  RCLCPP_INFO(this->get_logger(), "Published dummy point (%.2f, %.2f, %.2f)",
              point_msg.point.x, point_msg.point.y, point_msg.point.z);

  ++sample_index_;
}

void PredictionSim::manual_input_loop()
{
  std::string buffer;

  while (rclcpp::ok() && running_.load()) {
    if (!std::cin.good()) {
      std::this_thread::sleep_for(200ms);
      continue;
    }

    std::streambuf * buf = std::cin.rdbuf();
    if (!buf) {
      std::this_thread::sleep_for(200ms);
      continue;
    }

    std::streamsize available = buf->in_avail();
    if (available > 0) {
      for (; available > 0; --available) {
        char c = static_cast<char>(std::cin.get());
        if (c == '\r') {
          continue;
        }
        if (c == '\n') {
          const std::string line = buffer;
          buffer.clear();
          handle_manual_line(line);
          if (!running_.load()) {
            return;
          }
        } else {
          buffer.push_back(c);
        }
      }
    } else {
      std::this_thread::sleep_for(50ms);
    }
  }
}

void PredictionSim::handle_manual_line(const std::string & line)
{
  auto is_not_space = [](unsigned char ch) { return !std::isspace(ch); };

  auto begin = std::find_if(line.begin(), line.end(), is_not_space);
  if (begin == line.end()) {
    return;
  }
  auto end = std::find_if(line.rbegin(), line.rend(), is_not_space).base();
  std::string trimmed(begin, end);

  if (trimmed == "q" || trimmed == "Q" || trimmed == "quit" || trimmed == "QUIT") {
    RCLCPP_INFO(this->get_logger(), "Stopping manual point input on user request.");
    running_.store(false);
    return;
  }

  std::stringstream ss(trimmed);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!(ss >> x >> y >> z)) {
    RCLCPP_WARN(this->get_logger(), "Could not parse manual input '%s'. Expected three numbers.", trimmed.c_str());
    return;
  }

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = manual_point_frame_;
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;
  point_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published manual point (%.2f, %.2f, %.2f)", x, y, z);
}

void PredictionSim::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg || !point_pub_) {
    return;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    std::size_t count = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      sum_x += static_cast<double>(*it_x);
      sum_y += static_cast<double>(*it_y);
      sum_z += static_cast<double>(*it_z);
      ++count;
    }

    if (count == 0) {
      return;
    }

    geometry_msgs::msg::PointStamped out;
    out.header = msg->header;
    out.point.x = sum_x / static_cast<double>(count);
    out.point.y = sum_y / static_cast<double>(count);
    out.point.z = sum_z / static_cast<double>(count);
    point_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Published centroid point from cloud (%zu samples).",
                         count);
  } catch (const std::runtime_error & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Failed to parse point cloud '%s': %s",
                         cloud_topic_.c_str(), ex.what());
  }
}
