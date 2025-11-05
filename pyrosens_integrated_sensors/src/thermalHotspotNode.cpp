#include "hotspotDetection_pyrosens/thermalHotspotNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iomanip>      
#include <sstream> 

using std::placeholders::_1;
using std::placeholders::_2;


ThermalHotspotNode::ThermalHotspotNode() : rclcpp::Node("thermal_hotspot_node")
{
  // Declare parameters with defaults
  this->declare_parameter<std::string>("thermal_topic", thermal_topic_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("camera_frame", camera_frame_);
  this->declare_parameter<std::string>("target_frame", target_frame_);
  this->declare_parameter<std::string>("depth_topic", depth_topic_);
  this->declare_parameter<std::string>("depth_frame", depth_frame_);

  this->declare_parameter<double>("plane_z", plane_z_);
  this->declare_parameter<double>("hfov_rad", hfov_rad_);
  this->declare_parameter<double>("depth_hfov_rad", depth_hfov_rad_);
  this->declare_parameter<int>("depth_stride", depth_stride_);
  this->declare_parameter<double>("depth_min", depth_min_);
  this->declare_parameter<double>("depth_max", depth_max_);
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

  // Get parameters
  this->get_parameter("thermal_topic", thermal_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("target_frame", target_frame_);
  this->get_parameter("plane_z", plane_z_);
  this->get_parameter("hfov_rad", hfov_rad_);
  this->get_parameter("depth_topic", depth_topic_);
  this->get_parameter("depth_frame", depth_frame_);
  this->get_parameter("depth_hfov_rad", depth_hfov_rad_);
  this->get_parameter("depth_stride", depth_stride_);
  this->get_parameter("depth_min", depth_min_);
  this->get_parameter("depth_max", depth_max_);

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

  // Synced subscribers: thermal + depth
  thermal_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, thermal_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, depth_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *thermal_sub_, *depth_sub_);
  sync_->registerCallback(std::bind(&ThermalHotspotNode::syncCallback, this, _1, _2));

  // Publish overlay image outlining hotspots
  if (publish_overlay_) {
    thermalOverlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "thermal_overlay", rclcpp::SystemDefaultsQoS());
  }
  hot_points_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "hotspots/points_stamped", rclcpp::SystemDefaultsQoS());
  hot_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "hotspots/points_cloud", rclcpp::SystemDefaultsQoS());

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

void ThermalHotspotNode::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& thermal_msg,
                                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
  // Cache image sizes and init intrinsics
  img_width_  = static_cast<int>(thermal_msg->width);
  img_height_ = static_cast<int>(thermal_msg->height);
  if (!thermal_intrinsics_ready_) setThermalIntrinsicsFromHFOV_();

  // Convert thermal to cv::Mat
  cv_bridge::CvImageConstPtr t_ptr;
  try { t_ptr = cv_bridge::toCvShare(thermal_msg); }
  catch (const cv_bridge::Exception& e) { RCLCPP_ERROR(get_logger(),"thermal cv_bridge: %s", e.what()); return; }
  const auto& t_img = t_ptr->image;
  const auto& t_enc = thermal_msg->encoding;
  if (t_img.empty()) return;
  if (t_enc != sensor_msgs::image_encodings::MONO8 &&
      t_enc != sensor_msgs::image_encodings::MONO16) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Thermal enc '%s' not mono8/mono16", t_enc.c_str());
    return;
  }

  // Build thermal hot mask (same rule as detector)
  cv::Mat hotMask;
  {
    double thr_raw = 0.0;
    if (use_percentile_) {
      thr_raw = thermdetect::percentileRawMono8(t_img, hot_percentile_);
    } else {
      if (temp_gain_ <= 0.0) thr_raw = hot_temp_c_;
      else thr_raw = (hot_temp_c_ - temp_offset_) / temp_gain_;
      thr_raw = std::clamp(thr_raw, 0.0, 255.0);
    }
    const double t = std::max(0.0, thr_raw - 1.0);
    cv::threshold(t_img, hotMask, t, 255.0, cv::THRESH_BINARY);
    if (morphology_kernel_ > 0) {
      int k = morphology_kernel_; if ((k % 2) == 0) k += 1;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k,k));
      cv::morphologyEx(hotMask, hotMask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(hotMask, hotMask, cv::MORPH_CLOSE, kernel);
    }
  }

  // Depth image
  cv_bridge::CvImageConstPtr d_ptr;
  try { d_ptr = cv_bridge::toCvShare(depth_msg); }
  catch (const cv_bridge::Exception& e) { RCLCPP_ERROR(get_logger(),"depth cv_bridge: %s", e.what()); return; }
  const auto& d_img = d_ptr->image;
  if (d_img.empty()) return;

  // Initialize depth intrinsics once from size + hfov param
  if (depth_width_ == 0 || depth_height_ == 0) {
    setDepthIntrinsicsFromHFOV_(depth_msg->width, depth_msg->height, depth_hfov_rad_);
    RCLCPP_INFO(get_logger(), "Depth intrinsics from hfov=%.3f, size=%ux%u",
                depth_hfov_rad_, depth_msg->width, depth_msg->height);
  }

  // TF transforms we need:
  //  A) thermal_frame <- depth_frame  (for projecting depth points into thermal image)
  //  B) target_frame  <- depth_frame  (for publishing in world)

  geometry_msgs::msg::TransformStamped T_th_d, T_map_d;
  const rclcpp::Time stamp(thermal_msg->header.stamp);
   try {
    T_th_d  = tf_buffer_->lookupTransform(frame_id_,    depth_frame_,  stamp);
    T_map_d = tf_buffer_->lookupTransform(target_frame_, depth_frame_,  stamp);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return;
  }

  // Convert geometry_msgs Transform to Eigen
  auto toEig = [](const geometry_msgs::msg::Transform& t){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    T.linear() = q.toRotationMatrix();
    return T;
  };
  const Eigen::Isometry3d E_th_d  = toEig(T_th_d.transform);
  const Eigen::Isometry3d E_map_d = toEig(T_map_d.transform);

  // Prepare cloud (optional but nice)
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = thermal_msg->header.stamp;
  cloud.header.frame_id = target_frame_;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(0);

  // Iterate depth pixels with stride
  const int H = static_cast<int>(depth_msg->height);
  const int W = static_cast<int>(depth_msg->width);
  const bool depth_is_32f = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  const bool depth_is_16u = (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1);

  int kept = 0;
  for (int v = 0; v < H; v += depth_stride_) {
    for (int u = 0; u < W; u += depth_stride_) {
      // Read depth (meters)
      float Zm = std::numeric_limits<float>::quiet_NaN();
      if (depth_is_32f) {
        Zm = d_img.at<float>(v, u);
      } else if (depth_is_16u) {
        uint16_t raw = d_img.at<uint16_t>(v, u);
        Zm = 0.001f * raw; // mm -> m
      } else {
        continue; // unsupported
      }
      if (!std::isfinite(Zm) || Zm < depth_min_ || Zm > depth_max_) continue;

      // Back-project to 3D (depth optical frame)
      const double Xd = ( (u - d_cx_) / d_fx_ ) * Zm;
      const double Yd = ( (v - d_cy_) / d_fy_ ) * Zm;
      const double Zd = Zm;
      Eigen::Vector3d Pd(Xd, Yd, Zd);

      // Project this 3D point into thermal image
      Eigen::Vector3d Pt = E_th_d * Pd; // in thermal camera frame
      if (Pt.z() <= 1e-6) continue;     // behind camera or at origin

      const double ut = t_fx_ * (Pt.x() / Pt.z()) + t_cx_;
      const double vt = t_fy_ * (Pt.y() / Pt.z()) + t_cy_;

      if (ut < 0 || vt < 0 || ut >= img_width_ || vt >= img_height_) continue;

      // Check hot mask
      if (hotMask.at<uint8_t>(static_cast<int>(vt), static_cast<int>(ut)) == 0) continue;

      // Transform to world (map) and publish
      Eigen::Vector3d Pm = E_map_d * Pd;

      // PointStamped
      geometry_msgs::msg::PointStamped ps;
      ps.header.stamp = thermal_msg->header.stamp;
      ps.header.frame_id = target_frame_;
      ps.point.x = Pm.x();
      ps.point.y = Pm.y();
      ps.point.z = plane_z_; // flatten to plane for your 2D visualization
      hot_points_pub_->publish(ps);

      RCLCPP_INFO(get_logger(), "Hot pt depth_px=(%d,%d) depth=%.3f m world=(%.3f,%.3f,%.3f)",
                   u, v, Zm, Pm.x(), Pm.y(), plane_z_);

      // Append to cloud
      mod.resize(mod.size() + 1);
      sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
      // Advance to last element (PointCloud2Modifier only appends; re-create iter each time)
      it_x += (mod.size() - 1);
      it_y += (mod.size() - 1);
      it_z += (mod.size() - 1);
      *it_x = static_cast<float>(Pm.x());
      *it_y = static_cast<float>(Pm.y());
      *it_z = static_cast<float>(plane_z_);
      kept++;
    }
  }

  if (kept > 0) {
    hot_cloud_pub_->publish(cloud);
  }

  // Optional: still publish PoseArray of centroids for continuity with your earlier viz
  const auto regions = detector_.detectHotspots(t_img);
  if (!regions.empty()) {
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = thermal_msg->header.stamp;
    pa.header.frame_id = target_frame_;
    for (const auto& r : regions) {
      double x, y;
      if (projector_.pixelToWorldXY(r.centroid_px.x, r.centroid_px.y, thermal_msg->header.stamp, x, y)) {
        geometry_msgs::msg::Pose p; p.position.x = x; p.position.y = y; p.position.z = plane_z_; p.orientation.w = 1.0;
        pa.poses.push_back(p);
      }
    }
    if (!pa.poses.empty()) hotspots_world_pub_->publish(pa);
  }

  // Overlay with bounding boxes
  if (publish_overlay_ && thermalOverlay_pub_) {
    cv::Mat bgr; cv::cvtColor(t_img, bgr, cv::COLOR_GRAY2BGR);
    for (const auto& r : regions) {
      const auto& b = r.bbox_px;
      cv::rectangle(bgr, b, cv::Scalar(0,0,255), 2);
    }
    cv_bridge::CvImage out;
    out.header = thermal_msg->header; out.header.frame_id = frame_id_;
    out.encoding = sensor_msgs::image_encodings::BGR8; out.image = bgr;
    thermalOverlay_pub_->publish(*out.toImageMsg());
  }
}

void ThermalHotspotNode::setThermalIntrinsicsFromHFOV_() {
  // Based on current img_width_/img_height_
  const double fx = 0.5 * double(img_width_) / std::tan(0.5 * hfov_rad_);
  const double fy = fx;
  t_fx_ = fx; t_fy_ = fy;
  t_cx_ = 0.5 * (double(img_width_)  - 1.0);
  t_cy_ = 0.5 * (double(img_height_) - 1.0);
  thermal_intrinsics_ready_ = true;
}

void ThermalHotspotNode::setDepthIntrinsicsFromHFOV_(int w, int h, double hfov) {
  const double fx = 0.5 * double(w) / std::tan(0.5 * hfov);
  const double fy = fx;
  d_fx_ = fx; d_fy_ = fy;
  d_cx_ = 0.5 * (double(w) - 1.0);
  d_cy_ = 0.5 * (double(h) - 1.0);
  depth_width_ = w; depth_height_ = h;
}