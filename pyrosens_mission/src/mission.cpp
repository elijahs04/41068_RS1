#include "mission_pyrosens/mission.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>

// Mission node acts as a NavigateToPose proxy: it ingests waypoints from
// topics/services/action clients, keeps a local queue with state tracking,
// and forwards one goal at a time to the downstream Nav2 action server
// exposed under `downstream_nav_action_name_`.

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
      "downstream_nav_action_name", "/navigate_to_pose");
  upstream_nav_action_name_ = this->declare_parameter<std::string>(
      "upstream_nav_action_name", "/mission/navigate_to_pose");
  downstream_wait_timeout_sec_ = this->declare_parameter<double>(
      "downstream_wait_timeout_sec", 15.0);

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
      upstream_nav_action_name_,
      std::bind(&Mission::navSrvHandleGoal_,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Mission::navSrvHandleCancel_,   this, std::placeholders::_1),
      std::bind(&Mission::navSrvHandleAccepted_, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      cbg_action_);

  RCLCPP_INFO(get_logger(),
    "MissionManager up. Proxy server on %s -> forwards to %s",
    upstream_nav_action_name_.c_str(),
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
     goal_statuses_.clear();
    goals_.reserve(msg->poses.size());
    goal_statuses_.reserve(msg->poses.size());
    for (const auto& p : msg->poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg->header;
      ps.pose   = p;
      goals_.push_back(ps);
      goal_statuses_.push_back(GoalStatus::PENDING);
    }
    current_index_ = 0;
    state_ = MissionState::IDLE;
    updateGoalStatusesLocked_();
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
    goal_statuses_.clear();
    goals_.reserve(msg->poses.size());
    goal_statuses_.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) {
      goals_.push_back(ps);
      goal_statuses_.push_back(GoalStatus::PENDING);
    }
    current_index_ = 0;
    state_ = MissionState::IDLE;
    updateGoalStatusesLocked_();
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
    goal_statuses_.assign(goals_.size(), GoalStatus::PENDING);
    committed = goals_.size();
    stream_buffer_.clear();
    current_index_ = 0;
    state_ = MissionState::IDLE;
    updateGoalStatusesLocked_();
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
    updateGoalStatusesLocked_();
  }

  startWorker_();
  res->success = true; res->message = "Mission started.";
  publishStatus_();
  publishGoalList_();
}

void Mission::srvStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != MissionState::RUNNING) { res->success=false; res->message="Not RUNNING."; return; }
    paused_ = true;
    state_  = MissionState::PAUSED;
    updateGoalStatusesLocked_();
  }
  cancelActiveGoalNoThrow_();
  res->success = true; res->message = "Mission paused.";
  publishStatus_();
  publishGoalList_();
}

void Mission::srvResume_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != MissionState::PAUSED) { res->success=false; res->message="Not PAUSED."; return; }
    paused_ = false;
    state_  = MissionState::RUNNING;
    updateGoalStatusesLocked_();
  }
  res->success = true; res->message = "Mission resumed.";
  publishStatus_();
  publishGoalList_();
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
    updateGoalStatusesLocked_();
  }
  cancelActiveGoalNoThrow_();
  res->success = true; res->message = "Mission aborted (cannot resume).";
  publishStatus_();
  publishGoalList_();
}

void Mission::srvEStop_(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cancel_requested_ = true;
    paused_ = false;
    state_ = MissionState::ESTOPPED;
    updateGoalStatusesLocked_();
  }
  cancelActiveGoalNoThrow_();
  safeDisarm_();
  res->success = true; res->message = "E-Stop asserted.";
  publishStatus_();
  publishGoalList_();
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
    const std::shared_ptr<ServerHandle> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "[Proxy] Cancel requested with null handle.");
    return rclcpp_action::CancelResponse::REJECT;
  }

  std::size_t canceled_index = 0;
  bool have_index = false;
  {
    std::lock_guard<std::mutex> g(navsrv_mtx_);
    auto it = navsrv_goal_index_.find(goal_handle.get());
    if (it != navsrv_goal_index_.end()) {
      canceled_index = it->second;
      have_index = true;
      navsrv_goal_index_.erase(it);
    }
  }

  if (have_index) {
    RCLCPP_INFO(get_logger(),
      "[Proxy] Cancel request acknowledged for queued goal idx=%zu. Mission state unchanged.",
      canceled_index);
  } else {
    RCLCPP_INFO(get_logger(),
      "[Proxy] Cancel request acknowledged for unknown goal handle. Mission state unchanged.");
  }
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

  auto raw_handle = goal_handle.get();
  auto cleanup_tracking = [this, raw_handle]() {
    std::lock_guard<std::mutex> g(navsrv_mtx_);
    navsrv_goal_index_.erase(raw_handle);
  };

  // 1) Decide whether we are appending or replacing the queue
  bool mission_active = false;
  bool reset_queue = false;
  bool append_only = false;
  bool should_stop_worker = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mission_active = (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED);
    if (mission_active) {
      cancel_requested_ = true;
      paused_ = false;
      should_stop_worker = worker_running_.load();
      reset_queue = true;
    } else if (state_ == MissionState::COMPLETED ||
               state_ == MissionState::ABORTED   ||
               state_ == MissionState::ESTOPPED) {
      reset_queue = true;
    }
    append_only = !mission_active && !reset_queue;
  }

  if (mission_active) {
    if (should_stop_worker) {
      stopWorker_();
    } else {
      cancelActiveGoalNoThrow_();
    }
  }

  std::size_t queue_size = 0;
  std::size_t my_index = 0;
  bool auto_start = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (reset_queue) {
      goals_.clear();
      goal_statuses_.clear();
      current_index_ = 0;
    }
    goals_.push_back(goal->pose);
    goal_statuses_.push_back(GoalStatus::PENDING);
    my_index = goals_.size() - 1;
    cancel_requested_ = false;
    paused_ = false;
    queue_size = goals_.size();
    auto_start = (queue_size == 1);
    state_ = auto_start ? MissionState::RUNNING : MissionState::IDLE;
    updateGoalStatusesLocked_();
  }
  {
    std::lock_guard<std::mutex> g(navsrv_mtx_);
    navsrv_goal_index_[raw_handle] = my_index;
  }
  publishGoalList_();
  publishProgress_();
  if (auto_start) {
    publishStatus_();
  } else {
    publishStatus_("awaiting start");
  }
  if (auto_start) {
    startWorker_();
  }
  if (append_only) {
    RCLCPP_INFO(get_logger(), "[Proxy] Appended goal. Queue size: %zu", queue_size);
  } else if (mission_active) {
    RCLCPP_WARN(get_logger(), "[Proxy] Active mission preempted. Queue reset to new goal.");
  } else {
    RCLCPP_INFO(get_logger(), "[Proxy] Loaded goal. Queue size: %zu", queue_size);
  }
  if (auto_start) {
    RCLCPP_INFO(get_logger(), "[Proxy] Auto-starting mission for single goal from upstream client.");
  } else {
    RCLCPP_INFO(get_logger(),
      "[Proxy] Goal enqueued and waiting. Use Mission Start to begin navigation.");
  }

  // 3) Keep server goal active until our enqueued index is passed by the worker
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling()) {
      cleanup_tracking();
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
        cleanup_tracking();
        goal_handle->abort(std::make_shared<NavigateToPose::Result>());
        RCLCPP_WARN(get_logger(), "[Proxy] Aborted/Estopped while waiting for completion.");
        return;
      }
    }

    if (finished) {
      cleanup_tracking();
      goal_handle->succeed(std::make_shared<NavigateToPose::Result>());
      RCLCPP_INFO(get_logger(), "[Proxy] Server goal reported success.");
      return;
    }

    rate.sleep();
  }

  if (!rclcpp::ok()) {
    cleanup_tracking();
    goal_handle->abort(std::make_shared<NavigateToPose::Result>());
  }
}

// ---------------- Action CLIENT helpers (downstream) ----------------

bool Mission::waitForActionServer_(const rclcpp::Duration& timeout)
{
  if (!nav_client_) return false;
  bool ok = nav_client_->wait_for_action_server(std::chrono::nanoseconds(timeout.nanoseconds()));
  if (!ok) {
    RCLCPP_ERROR(get_logger(),
                 "Downstream NavigateToPose server not available at '%s' after waiting %.1f s.",
                 downstream_nav_action_name_.c_str(), downstream_wait_timeout_sec_);
  }
  return ok;
}

bool Mission::sendGoal_(const geometry_msgs::msg::PoseStamped& goal_pose)
{
  const auto wait_timeout = rclcpp::Duration::from_seconds(downstream_wait_timeout_sec_);
  if (!waitForActionServer_(wait_timeout)) {
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

const char* Mission::goalStatusToString(GoalStatus status)
{
  switch (status)
  {
    case GoalStatus::PENDING:   return "PENDING";
    case GoalStatus::ACTIVE:    return "ACTIVE";
    case GoalStatus::COMPLETED: return "COMPLETED";
  }
  return "UNKNOWN";
}

void Mission::updateGoalStatusesLocked_()
{
  if (goal_statuses_.size() != goals_.size()) {
    goal_statuses_.assign(goals_.size(), GoalStatus::PENDING);
  }

  if (goals_.empty()) {
    goal_statuses_.clear();
    return;
  }

  if (current_index_ >= goals_.size()) {
    std::fill(goal_statuses_.begin(), goal_statuses_.end(), GoalStatus::COMPLETED);
    return;
  }

  for (std::size_t i = 0; i < goal_statuses_.size(); ++i) {
    if (i < current_index_) {
      goal_statuses_[i] = GoalStatus::COMPLETED;
    } else if (i == current_index_) {
      if (state_ == MissionState::RUNNING || state_ == MissionState::PAUSED) {
        goal_statuses_[i] = GoalStatus::ACTIVE;
      } else {
        goal_statuses_[i] = GoalStatus::PENDING;
      }
    } else {
      goal_statuses_[i] = GoalStatus::PENDING;
    }
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
      constexpr double kHalfPi   = 1.5707963267948966; // pi/2
      for (std::size_t i = 0; i < goals_.size(); ++i) {
        const auto & goal = goals_[i];
        const auto & pos = goal.pose.position;
        const auto & q   = goal.pose.orientation;
        const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        const double roll = std::atan2(sinr_cosp, cosr_cosp);

        const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        double pitch = 0.0;
        if (std::abs(sinp) >= 1.0) {
          pitch = std::copysign(kHalfPi, sinp);
        } else {
          pitch = std::asin(sinp);
        }

        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        const double roll_deg  = roll  * kRadToDeg;
        const double pitch_deg = pitch * kRadToDeg;
        const double yaw_deg   = yaw   * kRadToDeg;

        const char* status_str =
            (i < goal_statuses_.size()) ? goalStatusToString(goal_statuses_[i]) : "PENDING";

        if (i > 0) {
          oss << "; ";
        }
        oss << "Goal " << (i + 1) << " [" << status_str << "]: " << std::fixed
            << "pos=(" << std::setprecision(2) << pos.x << ","
            << std::setprecision(2) << pos.y << ","
            << std::setprecision(2) << pos.z << ") "
            << "rpy_deg=(" << std::setprecision(1) << roll_deg << ","
            << std::setprecision(1) << pitch_deg << ","
            << std::setprecision(1) << yaw_deg << ") "
            << "quat=(" << std::setprecision(3) << q.x << ","
            << std::setprecision(3) << q.y << ","
            << std::setprecision(3) << q.z << ","
            << std::setprecision(3) << q.w << ")";
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
  std::optional<std::string> extra_copy;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const std::string state_str =
      (state_==MissionState::IDLE     ? "IDLE" :
       state_==MissionState::RUNNING  ? "RUNNING" :
       state_==MissionState::PAUSED   ? "PAUSED" :
       state_==MissionState::COMPLETED? "COMPLETED" :
       state_==MissionState::ABORTED  ? "ABORTED" : "ESTOPPED");
    out.data = state_str;
    if (extra && !extra->empty()) {
      extra_copy = *extra;
    }
  }
  if (extra_copy) {
    RCLCPP_INFO(get_logger(), "Mission status detail: %s", extra_copy->c_str());
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
  const bool was_running = worker_running_.exchange(false);
  cancel_requested_ = true;
  cancelActiveGoalNoThrow_();

  if (worker_.joinable()) {
    if (!was_running) {
      // Worker already finished; just join to reclaim resources.
      worker_.join();
    } else {
      worker_.join();
    }
  }
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

    bool no_goals = false;
    bool completed = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (goals_.empty()) {
        state_ = MissionState::IDLE;
        updateGoalStatusesLocked_();
        no_goals = true;
      } else if (current_index_ >= goals_.size()) {
        state_ = MissionState::COMPLETED;
        updateGoalStatusesLocked_();
        completed = true;
      } else {
        updateGoalStatusesLocked_();
      }
    }

    if (no_goals) {
      publishGoalList_();
      publishStatus_();
      break;
    }

    if (completed) {
      publishGoalList_();
      publishStatus_();
      break;
    }

    publishGoalList_();
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
      {
        std::lock_guard<std::mutex> lk(mtx_);
        state_ = MissionState::ABORTED;
        updateGoalStatusesLocked_();
      }
      publishGoalList_();
      publishStatus_(std::string("Navigation failed"));
      break;
    }

    bool reached_end = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (current_index_ < goals_.size()) {
        ++current_index_;
      }
      if (current_index_ >= goals_.size()) {
        state_ = MissionState::COMPLETED;
        reached_end = true;
      }
      updateGoalStatusesLocked_();
    }
    publishProgress_();
    publishGoalList_();

    if (reached_end) {
      publishStatus_();
      break;
    }
  }

  worker_running_ = false;
  RCLCPP_INFO(get_logger(), "Mission loop stopped.");
}

} // namespace mission_pyrosens
