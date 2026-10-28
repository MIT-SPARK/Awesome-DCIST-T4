#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <dcist_launch_system_msgs/srv/clear_robots.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dcist_launch_system_rviz {

/**
 * @brief Starts, stops and resets active window change detection on one or all robots, via the
 * <robot>/change_detection/{start,stop,reset} services of khronos' RosChangeDetectionController.
 * "Clear Base Station" drops the selected robot(s)' cached changes at the AWCD base station.
 */
class ChangeDetectionPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit ChangeDetectionPanel(QWidget* parent = nullptr);
  ~ChangeDetectionPanel() override = default;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

 private Q_SLOTS:
  void discoverRobots();
  void onStart() { sendToSelected("start"); }
  void onStop() { sendToSelected("stop"); }
  void onReset() { sendToSelected("reset"); }
  void onResetAndStart() { sendToSelected("reset", "start"); }
  void onClearBaseStation();

 private:
  using Trigger = std_srvs::srv::Trigger;
  using ClearRobots = dcist_launch_system_msgs::srv::ClearRobots;

  struct Robot {
    std::string ns;  // e.g. /hamilton/change_detection
    std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> clients;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    std::optional<std::string> state;  // "running", "paused" or "reset"
  };

  //! Calls `command` on the selected robot(s); `then` (if set) is called after it succeeds.
  void sendToSelected(const std::string& command, const std::string& then = "");
  void send(const QString& robot, const std::string& command, const std::string& then);
  void addRobot(const QString& name, const std::string& ns);
  //! Runs `fn` on the Qt thread (ROS callbacks run on the executor thread).
  void postToGui(std::function<void()> fn);
  void setResult(const QString& text);
  void updateStatus();

  rclcpp::Node::SharedPtr node_;
  std::map<QString, Robot> robots_;  // only touched on the Qt thread
  std::string clear_service_;        // discovered <ns>/awcd_base_station/clear_robots
  rclcpp::Client<ClearRobots>::SharedPtr clear_client_;
  QString pending_selection_;

  QComboBox* robot_box_;
  QLabel* status_label_;
  QLabel* result_label_;
  QTimer* discovery_timer_;
};

}  // namespace dcist_launch_system_rviz
