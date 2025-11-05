#include "mission_pyrosens/mission.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using mission_pyrosens::MissionState;

namespace mission_pyrosens
{

Mission::Mission() : rclcpp::Node("mission_manager")
{
  // ---- Callback groups ----
  cbg_services_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbg_topics_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_action_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ---- Publishers ----
  pub_status_       = this->create_publisher<std_msgs::msg::String>("/mission/status", 10);
  pub_progress_     = this->create_publisher<std_msgs::msg::Float32>("/mission/progress", 10);
  pub_current_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mission/current_goal", 10);
  pub_goal_list_    = this->create_publisher<std_msgs::msg::String>("/mission/goals", 10);

  // ---- Parameters ----
  goals_topic_        = this->declare_parameter<std::string>("goals_topic", "/mission/load_goals");
  allow_path_input_   = this->declare_parameter<bool>("allow_path_input", false);
  path_topic_         = this->declare_parameter<std::string>("path_topic", "/goals/path");
  allow_pose_stream_  = this->declare_parameter<bool>("allow_pose_stream", false);
  pose_stream_topic_  = this->declare_parameter<std::string>("pose_stream_topic", "/goals/pose_stream");
  downstream_nav_action_name_ = this->declare_parameter<std::string>(
      "downstream_nav_action_name", "/nav2/navigate_to_pose");

  // ---- Subscriptions ----
  {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg_topics_;
    sub_goals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        goals_topic_, 10,
        std::bind(&Mission::onGoals_, this, std::placeholders::_1), sub_opts);
  }
  if (allow_path_input_) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg_topics_;
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10,
        std::bind(&Mission::onPath_, this, std::placeholders::_1), sub_opts);
    RCLCPP_INFO(get_logger(), "Path intake enabled on %s", path_topic_.c_str());
  }
  if (allow_pose_stream_) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg_topics_;
    sub_stream_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_stream_topic_, 50,
        std::bind(&Mission::onPoseStream_, this, std::placeholders::_1), sub_opts);

    // ---- Commit-from-stream service (no rclcpp::ServiceOptions) ----
    srv_commit_goals_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/commit_goals",
        std::bind(&Mission::srvCommitGoals_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,   // QoS profile
        cbg_services_                       // callback group
    );

    RCLCPP_INFO(get_logger(),
      "Pose stream intake enabled on %s (use /mission/commit_goals to load).",
      pose_stream_topic_.c_str());
  }

  // ---- Services (Start/Stop/Resume/Abort/E-Stop) ----
    srv_start_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/start",
        std::bind(&Mission::srvStart_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_services_
    );

    srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/stop",
        std::bind(&Mission::srvStop_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_services_
    );

    srv_resume_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/resume",
        std::bind(&Mission::srvResume_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_services_
    );

    srv_abort_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/abort",
        std::bind(&Mission::srvAbort_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_services_
    );

    srv_estop_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/estop",
        std::bind(&Mission::srvEStop_, this,
                std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_services_
    );


  // ---- Action CLIENT (downstream Nav2) ----
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, downstream_nav_action_name_, cbg_action_);

  // ---- Action SERVER (proxy inbox for teammates) ----
  nav_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "navigate_to_pose",  // keep default; remap if needed
      std::bind(&Mission::navSrvHandleGoal_,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Mission::navSrvHandleCancel_,   this, std::placeholders::_1),
      std::bind(&Mission::navSrvHandleAccepted_, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      cbg_action_);

  RCLCPP_INFO(get_logger(),
    "MissionManager up. Proxy server on /navigate_to_pose -> forwards to %s",
    downstream_nav_action_name_.c_str());

  publishStatus_();
}

Mission::~Mission()
{
  stopWorker_();
}

// ---------------- Topic goal intake ----------------

void Mission::onGoals_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::size_t loaded = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED) {
      RCLCPP_WARN(get_logger(), "Cannot load goals while mission is active. Abort/complete first.");
      return;
    }
    goals_.clear();
    goals_.reserve(msg->poses.size());
    for (const auto& p : msg->poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg->header;
      ps.pose   = p;
      goals_.push_back(ps);
    }
    current_index_ = 0;
    state_ = MissionState::IDLE;
    loaded = goals_.size();
  }
  RCLCPP_INFO(get_logger(), "Loaded %zu goals from PoseArray.", loaded);
  publishGoalList_();
  publishStatus_();
  publishProgress_();
}

void Mission::onPath_(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::size_t loaded = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED) {
      RCLCPP_WARN(get_logger(), "Ignoring Path: mission active. Abort/complete first.");
      return;
    }
    goals_.clear();
    goals_.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) {
      goals_.push_back(ps);
    }
    current_index_ = 0;
    state_ = MissionState::IDLE;
    loaded = goals_.size();
  }
  RCLCPP_INFO(get_logger(), "Loaded %zu goals from nav_msgs/Path.", loaded);
  publishGoalList_();
  publishStatus_();
  publishProgress_();
}

void Mission::onPoseStream_(const geometry_msgs::msg::PoseStamped::SharedPtr ps)
{
  std::lock_guard<std::mutex> lk(mtx_);
  stream_buffer_.push_back(*ps);
}

void Mission::srvCommitGoals_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::size_t committed = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED) {
      res->success = false; res->message = "Cannot commit: mission active."; return;
    }
    if (stream_buffer_.empty()) {
      res->success = false; res->message = "No poses buffered from stream."; return;
    }
    goals_ = stream_buffer_;
    committed = goals_.size();
    stream_buffer_.clear();
    current_index_ = 0;
    state_ = MissionState::IDLE;
  }
  publishGoalList_();
  publishStatus_();
  publishProgress_();
  RCLCPP_INFO(get_logger(), "Committed %zu streamed poses as mission goals.", committed);
  res->success = true; res->message = "Goals committed from stream.";
}

// ---------------- Services ----------------

void Mission::srvStart_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ == MissionState::ESTOPPED) { res->success=false; res->message="E-Stop asserted."; return; }
    if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED) { res->success=false; res->message="Already active."; return; }
    if (state_ == MissionState::COMPLETED || state_ == MissionState::ABORTED) { res->success=false; res->message="Ended; load new goals."; return; }
    if (goals_.empty()) { res->success=false; res->message="No goals loaded."; return; }

    cancel_requested_ = false;
    paused_ = false;
    state_ = MissionState::RUNNING;
  }

  startWorker_();
  res->success = true; res->message = "Mission started.";
  publishStatus_();
}

void Mission::srvStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != MissionState::RUNNING) { res->success=false; res->message="Not RUNNING."; return; }
    paused_ = true;
    state_  = MissionState::PAUSED;
  }
  cancelActiveGoalNoThrow_();
  res->success = true; res->message = "Mission paused.";
  publishStatus_();
}

void Mission::srvResume_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != MissionState::PAUSED) { res->success=false; res->message="Not PAUSED."; return; }
    paused_ = false;
    state_  = MissionState::RUNNING;
  }
  res->success = true; res->message = "Mission resumed.";
  publishStatus_();
}

void Mission::srvAbort_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ == MissionState::IDLE || state_ == MissionState::COMPLETED ||
        state_ == MissionState::ABORTED || state_ == MissionState::ESTOPPED) {
      res->success=false; res->message="Abort not applicable."; return;
    }
    cancel_requested_ = true;
    state_ = MissionState::ABORTED;
  }
  cancelActiveGoalNoThrow_();
  res->success = true; res->message = "Mission aborted (cannot resume).";
  publishStatus_();
}

void Mission::srvEStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cancel_requested_ = true;
    paused_ = false;
    state_ = MissionState::ESTOPPED;
  }
  cancelActiveGoalNoThrow_();
  safeDisarm_();
  res->success = true; res->message = "E-Stop asserted.";
  publishStatus_();
}

// ---------------- NavigateToPose SERVER (proxy) ----------------

rclcpp_action::GoalResponse Mission::navSrvHandleGoal_(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!goal) { RCLCPP_WARN(get_logger(), "[Proxy] Reject: null goal."); return rclcpp_action::GoalResponse::REJECT; }
  if (state_ == MissionState::ESTOPPED) { RCLCPP_WARN(get_logger(), "[Proxy] Reject: E-Stop."); return rclcpp_action::GoalResponse::REJECT; }
  // Accept even if RUNNING/PAUSED — we will enqueue it
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Mission::navSrvHandleCancel_(
    const std::shared_ptr<ServerHandle> /*goal_handle*/)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cancel_requested_ = true;
    paused_ = false;
    if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED)
      state_ = MissionState::ABORTED;
    cancelActiveGoalNoThrow_();
  }
  publishStatus_();
  RCLCPP_WARN(get_logger(), "[Proxy] Client requested cancel -> Mission ABORTED.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Mission::navSrvHandleAccepted_(const std::shared_ptr<ServerHandle> goal_handle)
{
  {
    std::lock_guard<std::mutex> g(navsrv_mtx_);
    navsrv_active_ = goal_handle;
  }
  std::thread{std::bind(&Mission::navSrvExecute_, this, goal_handle)}.detach();
}

void Mission::navSrvExecute_(const std::shared_ptr<ServerHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  if (!goal) {
    goal_handle->abort(std::make_shared<NavigateToPose::Result>());
    return;
  }

  // 1) Prepare mission queue with the new pose
  bool had_worker = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    had_worker = worker_running_.load();
    cancel_requested_ = true;
    paused_ = false;
  }

  if (had_worker) {
    stopWorker_();
  } else {
    cancelActiveGoalNoThrow_();
  }

  std::size_t queue_size = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    goals_.clear();
    goals_.push_back(goal->pose);
    current_index_ = 0;
    cancel_requested_ = false;
    state_ = MissionState::IDLE;
    queue_size = goals_.size();
  }
  publishGoalList_();
  publishProgress_();
  publishStatus_("awaiting start");
  RCLCPP_INFO(get_logger(), "[Proxy] Enqueued goal. Queue size: %zu", queue_size);
  RCLCPP_INFO(get_logger(),
    "[Proxy] Goal enqueued and waiting. Use Mission Start to begin navigation.");

  // 3) Keep server goal active until our enqueued index is passed by the worker
  const std::size_t my_index = 0;
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(std::make_shared<NavigateToPose::Result>());
      RCLCPP_WARN(get_logger(), "[Proxy] Server goal canceled by client.");
      return;
    }

    bool finished = false;
    MissionState s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      s = state_;
      finished = (current_index_ > my_index) || (current_index_ >= goals_.size());
      if (s == MissionState::ABORTED || s == MissionState::ESTOPPED) {
        goal_handle->abort(std::make_shared<NavigateToPose::Result>());
        RCLCPP_WARN(get_logger(), "[Proxy] Aborted/Estopped while waiting for completion.");
        return;
      }
    }

    if (finished) {
      goal_handle->succeed(std::make_shared<NavigateToPose::Result>());
      RCLCPP_INFO(get_logger(), "[Proxy] Server goal reported success.");
      return;
    }

    rate.sleep();
  }

  if (!rclcpp::ok()) {
    goal_handle->abort(std::make_shared<NavigateToPose::Result>());
  }
}

// ---------------- Action CLIENT helpers (downstream) ----------------

bool Mission::waitForActionServer_(const rclcpp::Duration& timeout)
{
  if (!nav_client_) return false;
  bool ok = nav_client_->wait_for_action_server(std::chrono::nanoseconds(timeout.nanoseconds()));
  if (!ok) {
    RCLCPP_ERROR(get_logger(), "Downstream NavigateToPose server not available at '%s'.",
                 downstream_nav_action_name_.c_str());
  }
  return ok;
}

bool Mission::sendGoal_(const geometry_msgs::msg::PoseStamped& goal_pose)
{
  if (!waitForActionServer_(2s)) {
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose = goal_pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
      [this](std::shared_ptr<ClientHandle> handle){
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Downstream goal rejected.");
        } else {
          std::lock_guard<std::mutex> lk(mtx_);
          active_goal_handle_ = handle;
          RCLCPP_INFO(get_logger(), "Downstream goal accepted.");
        }
      };

  opts.result_callback =
      [this](const ClientHandle::WrappedResult& result){
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Downstream goal succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Downstream goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Downstream goal canceled.");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Downstream unknown result code.");
            break;
        }
      };

  auto future_handle = nav_client_->async_send_goal(goal, opts);

  auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (future_handle.wait_for(100ms) != std::future_status::ready) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "Interrupted while waiting for downstream goal handle.");
      return false;
    }
    if (std::chrono::steady_clock::now() > wait_deadline) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for downstream goal handle.");
      return false;
    }
  }

  auto handle = future_handle.get();
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "Downstream goal not accepted.");
    return false;
  }

  auto result_future = nav_client_->async_get_result(handle);
  while (rclcpp::ok())
  {
    if (cancel_requested_.load() || paused_.load() || state_ == MissionState::ESTOPPED) {
      cancelActiveGoalNoThrow_();
      return false;
    }
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      auto result = result_future.get();
      return result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }
  }
  return false;
}

void Mission::cancelActiveGoalNoThrow_()
{
  try {
    auto handle = active_goal_handle_;
    if (handle) {
      nav_client_->async_cancel_goal(handle);
      active_goal_handle_.reset();
    } else if (nav_client_) {
      nav_client_->async_cancel_all_goals();
    }
  } catch (...) {
    // best-effort
  }
}

// ---------------- Status/Progress/Safety ----------------

void Mission::publishCurrentGoal_()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (current_index_ < goals_.size()) {
    pub_current_goal_->publish(goals_[current_index_]);
  }
}

void Mission::publishGoalList_()
{
  if (!pub_goal_list_) {
    return;
  }

  std_msgs::msg::String msg;
  std::ostringstream oss;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (goals_.empty()) {
      msg.data = "(no goals loaded)";
    } else {
      constexpr double kRadToDeg = 57.29577951308232;  // 180/pi
      for (std::size_t i = 0; i < goals_.size(); ++i) {
        const auto & goal = goals_[i];
        const auto & pos = goal.pose.position;
        const auto & q   = goal.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        const double yaw_deg = yaw * kRadToDeg;

        if (i > 0) {
          oss << "; ";
        }
        oss << "Goal " << (i + 1) << ": " << std::fixed
            << "x=" << std::setprecision(2) << pos.x
            << " y=" << std::setprecision(2) << pos.y
            << " yaw_deg=" << std::setprecision(1) << yaw_deg;
      }
      msg.data = oss.str();
    }
  }
  if (msg.data.empty()) {
    msg.data = "(no goals loaded)";
  }
  pub_goal_list_->publish(msg);
}

void Mission::publishProgress_()
{
  std_msgs::msg::Float32 msg;
  float total = goals_.empty() ? 1.0f : static_cast<float>(goals_.size());
  {
    std::lock_guard<std::mutex> lk(mtx_);
    msg.data = static_cast<float>(current_index_) / total;
  }
  pub_progress_->publish(msg);
}

void Mission::publishStatus_(const std::optional<std::string>& extra)
{
  std_msgs::msg::String out;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::string state_str =
      (state_==MissionState::IDLE     ? "IDLE" :
       state_==MissionState::RUNNING  ? "RUNNING" :
       state_==MissionState::PAUSED   ? "PAUSED" :
       state_==MissionState::COMPLETED? "COMPLETED" :
       state_==MissionState::ABORTED  ? "ABORTED" : "ESTOPPED");
    out.data = "state=" + state_str +
               " idx=" + std::to_string(current_index_) +
               " total=" + std::to_string(goals_.size());
    if (extra) out.data += " extra=" + *extra;
  }
  pub_status_->publish(out);
}

void Mission::safeDisarm_()
{
  // TODO: Implement vehicle-specific hard stop/disarm (topic/service/action)
  RCLCPP_WARN(get_logger(), "[SAFETY] E-Stop hook called. Implement vehicle-specific disarm/kill.");
}

// ---------------- Worker loop ----------------

void Mission::startWorker_()
{
  if (worker_running_.exchange(true)) return;
  worker_ = std::thread([this](){ this->runLoop_(); });
}

void Mission::stopWorker_()
{
  if (!worker_running_.exchange(false)) return;
  cancel_requested_ = true;
  cancelActiveGoalNoThrow_();
  if (worker_.joinable()) worker_.join();
}

void Mission::runLoop_()
{
  RCLCPP_INFO(get_logger(), "Mission loop started.");
  rclcpp::Rate r(10.0);

  while (rclcpp::ok() && worker_running_.load())
  {
    MissionState s;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      s = state_;
    }

    if (s == MissionState::ESTOPPED || s == MissionState::ABORTED) break;
    if (s == MissionState::PAUSED) { r.sleep(); continue; }
    if (s != MissionState::RUNNING) { r.sleep(); continue; }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (goals_.empty()) {
        state_ = MissionState::IDLE;
        publishStatus_();
        break;
      }
      if (current_index_ >= goals_.size()) {
        state_ = MissionState::COMPLETED;
        publishStatus_();
        break;
      }
    }

    publishCurrentGoal_();
    geometry_msgs::msg::PoseStamped goal_copy;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal_copy = goals_.at(current_index_);
    }
    bool ok = sendGoal_(goal_copy);

    if (cancel_requested_.load()) break;
    if (paused_.load()) continue;

    if (!ok) {
      std::lock_guard<std::mutex> lk(mtx_);
      state_ = MissionState::ABORTED;
      publishStatus_(std::string("Navigation failed"));
      break;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      ++current_index_;
    }
    publishProgress_();

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (current_index_ >= goals_.size()) {
        state_ = MissionState::COMPLETED;
        publishStatus_();
        break;
      }
    }
  }

  worker_running_ = false;
  RCLCPP_INFO(get_logger(), "Mission loop stopped.");
}

} // namespace mission_pyrosens
