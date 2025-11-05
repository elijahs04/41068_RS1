#include "hotspotDetection_pyrosens/hotspotTransform.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hstrfm {

void HotspotTransform::setFrames(const std::string& camera_frame,
                                 const std::string& target_frame) {
  camera_frame_ = camera_frame;
  target_frame_ = target_frame;
}

void HotspotTransform::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  tf_buffer_ = std::move(tf_buffer);
}

void HotspotTransform::setIntrinsicsFromHFOV(int width, int height, double hfov_rad) {
  const double fx = 0.5 * static_cast<double>(width) / std::tan(0.5 * hfov_rad);
  const double fy = fx;
  const double cx = 0.5 * (static_cast<double>(width)  - 1.0);
  const double cy = 0.5 * (static_cast<double>(height) - 1.0);

  Eigen::Matrix3d Ke;
  Ke << fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0;
  Kinv_ = Ke.inverse();
  have_intrinsics_ = true;
}


void HotspotTransform::setIntrinsics(const cv::Mat& Kcv) {
  // Ensure 3x3 and convert to double
  CV_Assert(Kcv.rows == 3 && Kcv.cols == 3);
  cv::Mat K64;
  if (Kcv.type() != CV_64F) {
    Kcv.convertTo(K64, CV_64F);
  } else {
    K64 = Kcv;
  }

  // Copy into Eigen and invert with Eigen
  Eigen::Matrix3d Ke;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Ke(r, c) = K64.at<double>(r, c);

  Kinv_ = Ke.inverse();
  have_intrinsics_ = true;
}

void HotspotTransform::setPlaneZ(double plane_z) {
  n_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  d_ = -plane_z;
}

void HotspotTransform::setPlaneND(const Eigen::Vector3d& n_target, double d) {
  n_ = n_target.normalized();
  d_ = d;
}

Eigen::Isometry3d HotspotTransform::toEigen(const geometry_msgs::msg::Transform& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  T.linear() = q.toRotationMatrix();
  return T;
}

bool HotspotTransform::pixelToWorldXY(double u, double v,
                                      const rclcpp::Time& stamp,
                                      double& x, double& y) const
{
  if (!tf_buffer_) return false;
  if (!have_intrinsics_) return false;

  // 1) pixel to direction in camera frame
  Eigen::Vector3d ph(u, v, 1.0);
  Eigen::Vector3d dir_cam = (Kinv_ * ph).normalized();

  // 2) camera pose in target frame at stamp
  geometry_msgs::msg::TransformStamped T_target_cam_msg;
  try {
    // Use exact time from stamp
    T_target_cam_msg =
      tf_buffer_->lookupTransform(target_frame_, camera_frame_,
                                  tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
  } catch (const tf2::TransformException&) {
    return false;
  }

  Eigen::Isometry3d T_target_cam = toEigen(T_target_cam_msg.transform);
  Eigen::Vector3d cam_o = T_target_cam.translation();
  Eigen::Vector3d cam_dir = (T_target_cam.linear() * dir_cam).normalized();

  // 3) intersect with plane n^T X + d = 0
  const double denom = n_.dot(cam_dir);
  if (std::abs(denom) < 1e-9) return false; // nearly parallel

  const double t = -(n_.dot(cam_o) + d_) / denom;
  if (t <= 0.0) return false; // plane is behind camera

  Eigen::Vector3d hit = cam_o + t * cam_dir;
  x = hit.x();
  y = hit.y();
  return true;
}

} // namespace hstrfm
