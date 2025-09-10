#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

class TestCloudPub : public rclcpp::Node {
public:
  TestCloudPub() : rclcpp::Node("test_cloud_pub") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/wind/test_cloud", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestCloudPub::tick, this));
  }

private:
  void tick() {
    // Build a small cloud with fields: x, y, z, u, v
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = now();

    // Describe fields
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    // Add custom fields u, v
    mod.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "u", 1, sensor_msgs::msg::PointField::FLOAT32,
      "v", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    const int N = 5;
    mod.resize(N);

    sensor_msgs::PointCloud2Iterator<float> it_x (cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y (cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z (cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_u (cloud, "u");
    sensor_msgs::PointCloud2Iterator<float> it_v (cloud, "v");

    // Simple pattern: a few points in a line with varying wind
    float pts[N][5] = {
      {0.0f, 0.0f, 0.0f,  1.0f, 0.0f},  // (x,y,z,u,v)
      {1.0f, 0.0f, 0.0f,  0.7f, 0.7f},
      {2.0f, 0.5f, 0.0f,  0.0f, 1.2f},
      {3.0f, 1.0f, 0.0f, -0.8f, 0.3f},
      {4.0f, 1.5f, 0.0f, -1.2f,-0.2f}
    };

    for (int i = 0; i < N; ++i, ++it_x, ++it_y, ++it_z, ++it_u, ++it_v) {
      *it_x = pts[i][0];
      *it_y = pts[i][1];
      *it_z = pts[i][2];
      *it_u = pts[i][3];
      *it_v = pts[i][4];
    }

    pub_->publish(cloud);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Published /wind/test_cloud");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestCloudPub>());
  rclcpp::shutdown();
  return 0;
}
