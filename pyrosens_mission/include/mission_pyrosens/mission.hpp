#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <rmw/qos_profiles.h>  // for rmw_qos_profile_services_default

namespace mission_pyrosens
{

enum class MissionState {
  IDLE,
  RUNNING,
  PAUSED,
  COMPLETED,
  ABORTED,
  ESTOPPED
};

class Mission : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ClientHandle   = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ServerHandle   = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  Mission();
  ~Mission() override;

private:
  // ---------------- Lifecycle ----------------
  void startWorker_();
  void stopWorker_();
  void runLoop_();  // main mission loop

  // ---------------- Inputs (topics) ----------------
  void onGoals_(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onPath_(const nav_msgs::msg::Path::SharedPtr msg);
  void onPoseStream_(const geometry_msgs::msg::PoseStamped::SharedPtr ps);

  // ---------------- Services ----------------
  void srvStart_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvResume_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvAbort_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvEStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvCommitGoals_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ---------------- NavigateToPose Action SERVER (proxy/inbox) ----------------
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp_action::GoalResponse navSrvHandleGoal_(const rclcpp_action::GoalUUID&,
                      std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse navSrvHandleCancel_(const std::shared_ptr<ServerHandle> goal_handle);
  void navSrvHandleAccepted_(const std::shared_ptr<ServerHandle> goal_handle);
  void navSrvExecute_(const std::shared_ptr<ServerHandle> goal_handle);

  // ---------------- NavigateToPose Action CLIENT (downstream to Nav2) ----------------
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  ClientHandle::SharedPtr active_goal_handle_;
  bool waitForActionServer_(const rclcpp::Duration& timeout);
  bool sendGoal_(const geometry_msgs::msg::PoseStamped& goal_pose);
  void cancelActiveGoalNoThrow_();

  // ---------------- Status/Progress/Safety ----------------
  void publishStatus_(const std::optional<std::string>& extra = std::nullopt);
  void publishProgress_();
  void publishCurrentGoal_();
  void publishGoalList_();
  void safeDisarm_();

private:
  // Mission data
  std::mutex mtx_;
  MissionState state_{MissionState::IDLE};

  // Simple FIFO queue of waypoints (PoseStamped)
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::size_t current_index_{0};

  // Stream buffer for PoseStamped intake
  std::vector<geometry_msgs::msg::PoseStamped> stream_buffer_;

  // Worker state
  std::thread worker_;
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> paused_{false};

  // ROS pubs/subs/services
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_current_goal_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_goal_list_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_goals_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_stream_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_resume_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_abort_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_commit_goals_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cbg_services_;
  rclcpp::CallbackGroup::SharedPtr cbg_topics_;
  rclcpp::CallbackGroup::SharedPtr cbg_action_;

  // Parameters
  std::string goals_topic_{"/mission/load_goals"};
  bool allow_path_input_{false};
  std::string path_topic_{"/goals/path"};
  bool allow_pose_stream_{false};
  std::string pose_stream_topic_{"/goals/pose_stream"};

  // Downstream Nav2 NavigateToPose action name (client forwards to this)
  std::string downstream_nav_action_name_{"/nav2/navigate_to_pose"};

  // Optional tracking for server-side goal (not strictly required)
  std::mutex navsrv_mtx_;
  std::weak_ptr<ServerHandle> navsrv_active_;
  std::unordered_map<ServerHandle*, std::size_t> navsrv_goal_index_;
};

} // namespace mission_pyrosens
