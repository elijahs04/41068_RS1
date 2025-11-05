#pragma once
#include <ignition/gui/Plugin.hh>
#include <QQmlEngine>
#include <QQmlContext>
#include <QTimer>
#include <QMutex>
#include <QString>
#include <QStringList>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pyrosens {

class PyroSENSGui : public ignition::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString missionStatus READ missionStatus NOTIFY missionStatusChanged)
  Q_PROPERTY(QString windText      READ windText      NOTIFY windTextChanged)
  Q_PROPERTY(QStringList goals     READ goals         NOTIFY goalsChanged)

public:
  PyroSENSGui();
  ~PyroSENSGui() override;
  void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;

  Q_INVOKABLE void startMission();
  Q_INVOKABLE void resumeMission();
  Q_INVOKABLE void abortMission();
  Q_INVOKABLE void estopMission();

  QString missionStatus() const { return missionStatus_; }
  QString windText() const      { return windText_; }
  QStringList goals() const     { return goals_; }

signals:
  void missionStatusChanged();
  void windTextChanged();
  void goalsChanged();

private slots:
  void OnUiTick();

private:
  template<typename T>
  T ParamOr(const std::string&, const T& fallback) { return fallback; }
  void publishCmd(const std::string &cmd);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmdPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr statusSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goalsSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr windSub_;

  std::string statusTopic_, goalsTopic_, cmdTopic_, windTopic_;
  QTimer *uiTimer_{nullptr};
  QMutex mtx_;

  QString pendingStatus_{"(waiting)"};
  QStringList pendingGoals_;
  QString pendingWind_{"Wind: --"};
  bool statusDirty_{true}, goalsDirty_{false}, windDirty_{true};

  QString missionStatus_{"(waiting)"};
  QStringList goals_;
  QString windText_{"Wind: --"};
};

} // namespace pyrosens
