#include "pyrosens_gui/pyrosens_gui.hpp"
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <QQmlContext>
#include <ignition/plugin/Register.hh>

using namespace pyrosens;

PyroSENSGui::PyroSENSGui()
{
  uiTimer_ = new QTimer(this);
  connect(uiTimer_, &QTimer::timeout, this, &PyroSENSGui::OnUiTick);
  uiTimer_->start(50);
}

PyroSENSGui::~PyroSENSGui()
{
  statusSub_.reset();
  goalsSub_.reset();
  windSub_.reset();
  cmdPub_.reset();
  node_.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
}

void PyroSENSGui::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  Q_UNUSED(_pluginElem);
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  node_ = rclcpp::Node::make_shared("pyrosens_gz_gui");

  statusTopic_ = ParamOr("status_topic", std::string("/mission/status"));
  goalsTopic_  = ParamOr("goals_topic",  std::string("/mission/goals"));
  cmdTopic_    = ParamOr("cmd_topic",    std::string("/mission/cmd"));
  windTopic_   = ParamOr("wind_topic",   std::string("/sim/wind"));

  cmdPub_ = node_->create_publisher<std_msgs::msg::String>(cmdTopic_, 10);

  statusSub_ = node_->create_subscription<std_msgs::msg::String>(
    statusTopic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr msg){
      QMutexLocker lk(&mtx_);
      pendingStatus_ = QString::fromStdString(msg->data);
      statusDirty_ = true;
    });

  goalsSub_ = node_->create_subscription<std_msgs::msg::String>(
    goalsTopic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr msg){
      QMutexLocker lk(&mtx_);
      pendingGoals_.clear();
      const QString q = QString::fromStdString(msg->data);
      for (const auto &item : q.split(';', Qt::SkipEmptyParts))
        pendingGoals_ << item.trimmed();
      goalsDirty_ = true;
    });

  windSub_ = node_->create_subscription<std_msgs::msg::String>(
    windTopic_, rclcpp::QoS(10).best_effort(),
    [this](const std_msgs::msg::String::SharedPtr msg){
      QMutexLocker lk(&mtx_);
      pendingWind_ = QString::fromStdString(msg->data);
      windDirty_ = true;
    });

  if (auto app = ignition::gui::App())
  {
    auto engine = app->Engine();
    if (engine)
      engine->rootContext()->setContextProperty("PyroSENSGui", this);
  }
}

void PyroSENSGui::OnUiTick()
{
  if (node_) rclcpp::spin_some(node_);
  QMutexLocker lk(&mtx_);
  if (statusDirty_) { missionStatus_ = pendingStatus_; statusDirty_ = false; emit missionStatusChanged(); }
  if (goalsDirty_)  { goals_ = pendingGoals_; goalsDirty_ = false; emit goalsChanged(); }
  if (windDirty_)   { windText_ = pendingWind_; windDirty_ = false; emit windTextChanged(); }
}

void PyroSENSGui::publishCmd(const std::string &cmd)
{
  if (!cmdPub_) return;
  std_msgs::msg::String msg; msg.data = cmd; cmdPub_->publish(msg);
}

void PyroSENSGui::startMission(){ publishCmd("start"); }
void PyroSENSGui::resumeMission(){ publishCmd("resume"); }
void PyroSENSGui::abortMission() { publishCmd("abort"); }
void PyroSENSGui::estopMission() { publishCmd("pause"); }

IGNITION_ADD_PLUGIN(pyrosens::PyroSENSGui, ignition::gui::Plugin)
