#pragma once

#include <string>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hstrfm {

    class HotspotTransform {
    public:
    HotspotTransform() = default;
    ~HotspotTransform() = default;

    // Required: set frames (camera optical frame that images come from, and target world frame)
    void setFrames(const std::string& camera_frame,
                    const std::string& target_frame);

    // Required: provide a TF buffer
    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    // Intrinsics from hfov and image size (good for sim)
    void setIntrinsicsFromHFOV(int width, int height, double hfov_rad);

    // Or directly set K (3x3) if you have CameraInfo
    void setIntrinsics(const cv::Mat& K);

    // Plane z = constant in target frame
    void setPlaneZ(double plane_z);

    // General plane n^T X + d = 0 in target frame
    void setPlaneND(const Eigen::Vector3d& n_target, double d);

    // Project a pixel (u, v) at a given timestamp to world XY on the plane
    // Returns true on success and fills x,y (meters in target_frame)
    bool pixelToWorldXY(double u, double v,
                        const rclcpp::Time& stamp,
                        double& x, double& y) const;

    private:
    // helpers
    static Eigen::Isometry3d toEigen(const geometry_msgs::msg::Transform& t);

    std::string camera_frame_{"thermal_camera_link"};
    std::string target_frame_{"map"};

    // Intrinsics
    Eigen::Matrix3d Kinv_ = Eigen::Matrix3d::Identity();
    bool have_intrinsics_{false};

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Plane in target frame: n^T X + d = 0
    Eigen::Vector3d n_{0.0, 0.0, 1.0};
    double d_{0.0}; // for z = z0 plane, n=[0,0,1], d = -z0
    };

} // namespace hstrfm