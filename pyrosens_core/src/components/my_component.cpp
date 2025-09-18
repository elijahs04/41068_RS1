// filename: src/components/my_component.cpp (note *_component.cpp)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace pkg_ns {

class MyComponent : public rclcpp::Node {
public:
  explicit MyComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("my_component", options) {
    // TODO: your existing logic (as a class)
  }
};

} // namespace pkg_ns

RCLCPP_COMPONENTS_REGISTER_NODE(pkg_ns::MyComponent)
