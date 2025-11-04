#include "hotspotDetection_pyrosens/thermalHotspotNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iomanip>      
#include <sstream> 

using std::placeholders::_1;


ThermalHotspotNode::ThermalHotspotNode() : rclcpp::Node("thermal_hotspot_node")
{
  // Declare parameters
  this->declare_parameter<std::string>("thermal_topic", thermal_topic_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("camera_frame", camera_frame_);
  this->declare_parameter<std::string>("target_frame", target_frame_);

  this->declare_parameter<double>("temp_gain", temp_gain_);
  this->declare_parameter<double>("temp_offset", temp_offset_);
  this->declare_parameter<bool>("publish_overlay", publish_overlay_);
  this->declare_parameter<double>("hot_temp_c", hot_temp_c_);
  this->declare_parameter<bool>("use_percentile", use_percentile_);
  this->declare_parameter<double>("hot_percentile", hot_percentile_);
  this->declare_parameter<int>("morphology_kernel", morphology_kernel_);
  this->declare_parameter<int>("min_area_px", min_area_px_);
  this->declare_parameter<int>("max_area_px", max_area_px_);
  this->declare_parameter<int>("max_regions_draw", max_regions_draw_);
  this->declare_parameter<double>("plane_z", plane_z_);
  this->declare_parameter<double>("hfov_rad", hfov_rad_);

  // Get initial parameter values
  this->get_parameter("thermal_topic", thermal_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("target_frame", target_frame_);

  this->get_parameter("temp_gain", temp_gain_);
  this->get_parameter("temp_offset", temp_offset_);
  this->get_parameter("publish_overlay", publish_overlay_);
  this->get_parameter("hot_temp_c", hot_temp_c_);
  this->get_parameter("use_percentile", use_percentile_);
  this->get_parameter("hot_percentile", hot_percentile_);
  this->get_parameter("morphology_kernel", morphology_kernel_);
  this->get_parameter("min_area_px", min_area_px_);
  this->get_parameter("max_area_px", max_area_px_);
  this->get_parameter("max_regions_draw", max_regions_draw_);
  this->get_parameter("plane_z", plane_z_);  
  this->get_parameter("hfov_rad", hfov_rad_);

  // Configure detector
  thermdetect::DetectorConfig cfg;
  cfg.temp_gain = temp_gain_;
  cfg.temp_offset = temp_offset_;
  cfg.hot_temp_c = hot_temp_c_;
  cfg.use_percentile = use_percentile_;
  cfg.hot_percentile = hot_percentile_;
  cfg.morphology_kernel = morphology_kernel_;
  cfg.min_area_px = min_area_px_;
  cfg.max_area_px = max_area_px_;
  detector_ = thermdetect::HotspotDetector(cfg);

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  projector_.setTFBuffer(tf_buffer_);
  projector_.setFrames(camera_frame_, target_frame_);
  projector_.setPlaneZ(plane_z_); // world plane z

  // Subscribe to thermal image topic
  thermalImage_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ThermalHotspotNode::imageCallback, this, std::placeholders::_1));

  // Publish overlay image outlining hotspots
  if (publish_overlay_) {
    thermalOverlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "thermal_overlay", rclcpp::SystemDefaultsQoS());
  }

  // Publish world-frame hotspot points each frame
  hotspots_world_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "hotspots/world_points", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(),
              "ThermalHotspotNode initalised, Subscribing to: %s (frame_id=%s, gain=%.6f, offset=%.6f)",
              thermal_topic_.c_str(), frame_id_.c_str(), temp_gain_, temp_offset_);
}

ThermalHotspotNode::~ThermalHotspotNode() {
  RCLCPP_INFO(get_logger(), "ThermalHotspotNode shutting down.");
}

void ThermalHotspotNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  // Cache geometry
  img_width_  = static_cast<int>(img_msg->width);
  img_height_ = static_cast<int>(img_msg->height);

  // One-time intrinsics from hfov + size (sim-perfect)
  if (!intrinsics_ready_) {
    projector_.setIntrinsicsFromHFOV(img_width_, img_height_, hfov_rad_);
    intrinsics_ready_ = true;
    RCLCPP_INFO(get_logger(),
      "Hotspot projector intrinsics set from hfov=%.4f rad, size=%dx%d",
      hfov_rad_, img_width_, img_height_);
  }

  // Convert to cv::Mat (no forced encoding; we accept mono8 or mono16)
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const auto& mat = cv_ptr->image;
  const auto& enc = img_msg->encoding;

  if (mat.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Empty thermal image received.");
    return;
  }

  if (enc != sensor_msgs::image_encodings::MONO8 &&
      enc != sensor_msgs::image_encodings::MONO16) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                         "Unsupported encoding '%s'. Expect mono8 or mono16.",
                         enc.c_str());
    return;
  }

  const auto regions = detector_.detectHotspots(mat);
  if (!regions.empty()) {
    // Log the main region (largest) for quick sanity
    const auto& r0 = regions.front();
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
      "Hotspot region: bbox=(%d,%d,%d,%d) area=%d max=%.1fC mean=%.1fC",
      r0.bbox_px.x, r0.bbox_px.y, r0.bbox_px.width, r0.bbox_px.height,
      r0.area_px, r0.max_temp_c, r0.mean_temp_c);
  } else {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 2000, "No hotspots above threshold.");
  }

  // Project to world and publish PoseArray
  geometry_msgs::msg::PoseArray pa;
  pa.header.stamp = img_msg->header.stamp;
  pa.header.frame_id = target_frame_;
  for (const auto& r : regions) {
    double x, y;
    if (projector_.pixelToWorldXY(r.centroid_px.x, r.centroid_px.y,
                                  img_msg->header.stamp, x, y)) {
      geometry_msgs::msg::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = plane_z_;
      // orientation unused; leave identity
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
     "Center pixel projects to (%.2f, %.2f) in %s", x, y, target_frame_.c_str());
  }
  if (!pa.poses.empty()) {
    hotspots_world_pub_->publish(pa);
  }

  if (publish_overlay_ && thermalOverlay_pub_) {
    // mat is mono8 → convert to BGR for colored drawing
    cv::Mat bgr;
    cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_GRAY2BGR);

    const int K = std::min<int>(max_regions_draw_, regions.size());
    for (int i = 0; i < K; ++i) {
      const auto& r = regions[i];
      const auto& b = r.bbox_px;
      cv::rectangle(bgr, b, cv::Scalar(0,0,255), 2);
      // label: max temp
      std::ostringstream oss; oss.setf(std::ios::fixed); oss<<std::setprecision(1)<<r.max_temp_c<<"C";
      auto org = cv::Point(b.x, std::max(0, b.y - 4));
      cv::putText(bgr, oss.str(), org, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }
    cv_bridge::CvImage out;
    out.header = img_msg->header; out.header.frame_id = frame_id_;
    out.encoding = sensor_msgs::image_encodings::BGR8; out.image = bgr;
    thermalOverlay_pub_->publish(*out.toImageMsg());
  }

}