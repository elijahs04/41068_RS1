#include "hotspotDetection_pyrosens/hotspotMapperNode.hpp"

using std::placeholders::_1;

HotspotMapperNode::HotspotMapperNode() : rclcpp::Node("hotspot_mapper_node")
{
  // Declare params
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<double>("origin_x", origin_x_);
  this->declare_parameter<double>("origin_y", origin_y_);
  this->declare_parameter<double>("resolution", resolution_);
  this->declare_parameter<int>("width", width_);
  this->declare_parameter<int>("height", height_);
  this->declare_parameter<double>("sigma_m", sigma_m_);
  this->declare_parameter<double>("decay_rate_hz", decay_rate_hz_);
  this->declare_parameter<double>("decay_half_life_s", decay_half_life_s_);
  this->declare_parameter<double>("add_weight", double(add_weight_));
  this->declare_parameter<double>("clear_threshold", double(clear_threshold_));
  this->declare_parameter<bool>("publish_unknown_as_unseen", publish_unknown_as_unseen_);

  // Get params
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("width", width_);
  this->get_parameter("height", height_);
  this->get_parameter("sigma_m", sigma_m_);
  this->get_parameter("decay_rate_hz", decay_rate_hz_);
  this->get_parameter("decay_half_life_s", decay_half_life_s_);
  double addw; this->get_parameter("add_weight", addw); add_weight_ = float(addw);
  double ct; this->get_parameter("clear_threshold", ct); clear_threshold_ = float(ct);
  this->get_parameter("publish_unknown_as_unseen", publish_unknown_as_unseen_);

  // Pre-size grid
  grid_.assign(size_t(width_*height_), 0.0f);
  seen_.assign(size_t(width_*height_), 0u);

  // Build Gaussian kernel
  buildKernel_();

  // Compute per-tick decay factor from half-life
  const double dt = 1.0 / std::max(1.0, decay_rate_hz_);
  const double lambda = std::log(2.0) / std::max(1e-6, decay_half_life_s_);
  decay_factor_per_tick_ = float(std::exp(-lambda * dt));

  // TF (not strictly used if inputs already in frame_id_)
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  rclcpp::QoS qos(10);
  qos.best_effort();
  sub_pt_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "hotspots/points_stamped", qos, std::bind(&HotspotMapperNode::pointCb, this, _1));
  sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "hotspots/points_cloud", qos, std::bind(&HotspotMapperNode::cloudCb, this, _1));

  // Publishers
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("hotspots/heatmap", map_qos);

  auto img_qos = rclcpp::SensorDataQoS().keep_last(1);
  pub_img_  = this->create_publisher<sensor_msgs::msg::Image>("hotspots/heatmap_image", img_qos);

  // Reset service
  srv_reset_ = this->create_service<std_srvs::srv::Empty>(
      "hotspots/reset_heatmap",
      std::bind(&HotspotMapperNode::handleReset_, this, std::placeholders::_1, std::placeholders::_2));

  // Timer for decay + republish
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / std::max(1.0, decay_rate_hz_))),
      std::bind(&HotspotMapperNode::tick, this));

  RCLCPP_INFO(this->get_logger(),
              "Heatmap ready: frame=%s origin=(%.2f,%.2f) res=%.2fm size=%dx%d sigma=%.2fm half_life=%.1fs",
              frame_id_.c_str(), origin_x_, origin_y_, resolution_, width_, height_, sigma_m_, decay_half_life_s_);
}

void HotspotMapperNode::buildKernel_()
{
  const double sigma_cells = sigma_m_ / resolution_;
  kernel_radius_cells_ = std::max(1, int(std::ceil(3.0 * sigma_cells)));
  const int ksz = 2 * kernel_radius_cells_ + 1;

  kernel_ = cv::Mat(ksz, ksz, CV_32F);
  const double two_sigma2 = 2.0 * sigma_cells * sigma_cells;
  float sumv = 0.0f;

  for (int j = -kernel_radius_cells_; j <= kernel_radius_cells_; ++j) {
    for (int i = -kernel_radius_cells_; i <= kernel_radius_cells_; ++i) {
      const double r2 = double(i*i + j*j);
      float v = float(std::exp(-r2 / two_sigma2));
      kernel_.at<float>(j + kernel_radius_cells_, i + kernel_radius_cells_) = v;
      sumv += v;
    }
  }
  // Normalize to peak 1.0 (not unit integral) so add_weight_ is intuitive
  if (sumv > 0.0f) {
    float peak = kernel_.at<float>(kernel_radius_cells_, kernel_radius_cells_);
    kernel_ /= peak;
  }
}

bool HotspotMapperNode::worldToCell(double x, double y, int& cx, int& cy) const
{
  const double gx = (x - origin_x_) / resolution_;
  const double gy = (y - origin_y_) / resolution_;
  cx = int(std::floor(gx));
  cy = int(std::floor(gy));
  return (cx >= 0 && cy >= 0 && cx < width_ && cy < height_);
}

void HotspotMapperNode::splatGaussian(int cx, int cy)
{
  const int x0 = cx - kernel_radius_cells_;
  const int y0 = cy - kernel_radius_cells_;

  for (int ky = 0; ky < kernel_.rows; ++ky) {
    int gy = y0 + ky;
    if (gy < 0 || gy >= height_) continue;

    for (int kx = 0; kx < kernel_.cols; ++kx) {
      int gx = x0 + kx;
      if (gx < 0 || gx >= width_) continue;

      const size_t idx = size_t(gy) * size_t(width_) + size_t(gx);
      const float w = kernel_.at<float>(ky, kx) * add_weight_;
      grid_[idx] += w;
      seen_[idx] = 1u;
    }
  }
}

void HotspotMapperNode::pointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Assume incoming is already in frame_id_, or the caller ensures it
  if (msg->header.frame_id != frame_id_) {
    // If needed, add tf transform here
  }

  int cx, cy;
  if (worldToCell(msg->point.x, msg->point.y, cx, cy)) {
    splatGaussian(cx, cy);
  }
}

void HotspotMapperNode::cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Iterate xyz
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    int cx, cy;
    if (worldToCell(*it_x, *it_y, cx, cy)) {
      splatGaussian(cx, cy);
    }
  }
}

void HotspotMapperNode::tick()
{
  for (size_t i = 0; i < grid_.size(); ++i) {
    grid_[i] *= decay_factor_per_tick_;
    if (grid_[i] < clear_threshold_) {
      grid_[i] = 0.0f;
      if (publish_unknown_as_unseen_) seen_[i] = 0u;  // let it disappear
    }
  }
  publishOutputs();
}

void HotspotMapperNode::publishOutputs()
{
  // Build a list of active values for percentile cap
  float cap = 0.0f;
  {
    std::vector<float> vals; vals.reserve(grid_.size());
    for (size_t i = 0; i < grid_.size(); ++i) {
      if (seen_[i] && grid_[i] > 0.0f) vals.push_back(grid_[i]);
    }
    if (!vals.empty()) {
      const size_t k = std::max<size_t>(0, vals.size() * 9 / 10);
      std::nth_element(vals.begin(), vals.begin() + k, vals.end());
      cap = std::max(1e-3f, vals[k]);
    } else {
      cap = 1.0f; // avoid div-by-zero; nothing to show
    }
  }
  const float inv_cap = 100.0f / cap;

  // OccupancyGrid
  nav_msgs::msg::OccupancyGrid og;
  og.header.stamp = now();
  og.header.frame_id = frame_id_;
  og.info.resolution = float(resolution_);
  og.info.width = width_;
  og.info.height = height_;
  og.info.origin.position.x = origin_x_;
  og.info.origin.position.y = origin_y_;
  og.info.origin.orientation.w = 1.0;
  og.data.resize(size_t(width_) * size_t(height_));

  og.data.resize(size_t(width_) * size_t(height_));
  if (publish_unknown_as_unseen_) {
    std::fill(og.data.begin(), og.data.end(), static_cast<int8_t>(-1));
  } else {
    std::fill(og.data.begin(), og.data.end(), static_cast<int8_t>(0)); // free
  }

  for (size_t i = 0; i < grid_.size(); ++i) {
    if (publish_unknown_as_unseen_ && !seen_[i]) continue; // keep unknown
    int v = int(std::round(std::min(100.0f, grid_[i] * inv_cap)));
    og.data[i] = static_cast<int8_t>(v);
  }
  pub_grid_->publish(og);

  size_t hot = 0, seen = 0;
  for (size_t i = 0; i < grid_.size(); ++i) { seen += seen_[i]; if (grid_[i] > 0.f) ++hot; }
  RCLCPP_INFO(this->get_logger(), "heatmap: seen=%zu hot=%zu cap=%.3f", seen, hot, cap);


  // Pretty image (use only seen cells for normalization)
  cv::Mat img(height_, width_, CV_32F, (void*)grid_.data());
  cv::Mat norm_u8(height_, width_, CV_8U, cv::Scalar(0));
  const float scale = 255.0f / cap;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t idx = size_t(y) * size_t(width_) + size_t(x);
      if (seen_[idx]) {
        norm_u8.at<uint8_t>(y, x) = uint8_t(std::min(255.0f, grid_[idx] * scale));
      } // else keep 0 (background)
    }
  }
  cv::Mat color_bgr; cv::applyColorMap(norm_u8, color_bgr, cv::ColormapTypes::COLORMAP_JET);

  sensor_msgs::msg::Image out;
  out.header = og.header;
  out.height = color_bgr.rows;
  out.width  = color_bgr.cols;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.is_bigendian = false;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(color_bgr.step);
  out.data.assign(color_bgr.datastart, color_bgr.dataend);
  pub_img_->publish(out);
}

void HotspotMapperNode::reset_()
{
  std::fill(grid_.begin(), grid_.end(), 0.0f);
  std::fill(seen_.begin(), seen_.end(), uint8_t(0));
}

void HotspotMapperNode::handleReset_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  reset_();
  RCLCPP_INFO(this->get_logger(), "Heatmap reset via /hotspots/reset_heatmap");
}