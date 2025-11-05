/**
 * @file mission_cmd_bridge.cpp
 *
 * Simple adapter that converts textual mission commands from the GUI
 * (published on /mission/cmd) into Trigger service calls understood
 * by the mission manager.
 */

#include <algorithm>
#include <chrono>
#include <cctype>
#include <exception>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace
{
std::string to_lower_trimmed(const std::string &input)
{
  auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c){ return std::isspace(c); });
  auto last  = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c){ return std::isspace(c); }).base();
  std::string result = (first < last) ? std::string(first, last) : std::string();
  std::transform(result.begin(), result.end(), result.begin(),
    [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return result;
}
}  // namespace

class MissionCmdBridge : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;

  MissionCmdBridge()
  : rclcpp::Node("mission_cmd_bridge")
  {
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/mission/cmd");
    auto service_ns = declare_parameter<std::string>("service_namespace", "/mission");

    if (!service_ns.empty() && service_ns.back() == '/') {
      service_ns.pop_back();
    }

    const std::vector<std::pair<std::string, std::string>> command_to_service = {
      {"start", "start"},
      {"resume", "resume"},
      {"stop", "stop"},
      {"pause", "stop"},
      {"abort", "abort"},
      {"estop", "estop"},
      {"commit_goals", "commit_goals"},
    };

    std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> clients;
    for (const auto & mapping : command_to_service) {
      const auto & service_name = mapping.second;
      auto & client = clients[service_name];
      if (!client) {
        client = create_client<Trigger>(service_ns + "/" + service_name);
      }
      service_clients_.emplace(mapping.first, client);
    }

    subscription_ = create_subscription<std_msgs::msg::String>(
      cmd_topic, rclcpp::QoS(10),
      std::bind(&MissionCmdBridge::on_command, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MissionCmdBridge listening on %s, bridging to namespace %s",
      cmd_topic.c_str(), service_ns.c_str());
  }

private:
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const auto key = to_lower_trimmed(msg->data);
    auto it = service_clients_.find(key);
    if (it == service_clients_.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received unsupported mission command '%s'", msg->data.c_str());
      return;
    }

    const auto & client = it->second;
    if (!client->wait_for_service(100ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Service %s not available yet", client->get_service_name());
      return;
    }

    auto req = std::make_shared<Trigger::Request>();
    const std::string service_name(client->get_service_name());
    client->async_send_request(
      req,
      [this, command = key, service_name](rclcpp::Client<Trigger>::SharedFuture future_resp) {
        std::string status = "unknown";
        std::string detail;
        try {
          const auto response = future_resp.get();
          if (response) {
            status = response->success ? "success" : "failure";
            detail = response->message;
          } else {
            status = "no response";
          }
        } catch (const std::exception &e) {
          status = "error";
          detail = e.what();
        }

        RCLCPP_INFO(
          get_logger(),
          "Mission command '%s' completed with %s via %s (%s)",
          command.c_str(), status.c_str(), service_name.c_str(), detail.c_str());
      });
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> service_clients_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionCmdBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
