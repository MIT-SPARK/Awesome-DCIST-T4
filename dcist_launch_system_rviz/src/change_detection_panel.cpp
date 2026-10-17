#include "dcist_launch_system_rviz/change_detection_panel.hpp"

#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace dcist_launch_system_rviz {
namespace {

const std::string kStartSuffix = "/change_detection/start";
const QString kAll = "All";

}  // namespace

ChangeDetectionPanel::ChangeDetectionPanel(QWidget* parent) : Panel(parent) {
  auto* robot_layout = new QHBoxLayout;
  robot_layout->addWidget(new QLabel("Robot"));
  robot_box_ = new QComboBox;
  robot_box_->addItem(kAll);
  robot_layout->addWidget(robot_box_, 1);
  auto* refresh_button = new QPushButton("Refresh");
  robot_layout->addWidget(refresh_button);

  auto* button_layout = new QHBoxLayout;
  auto* start_button = new QPushButton("Start");
  auto* stop_button = new QPushButton("Stop");
  auto* reset_button = new QPushButton("Reset");
  auto* reset_start_button = new QPushButton("Reset && Start");
  button_layout->addWidget(start_button);
  button_layout->addWidget(stop_button);
  button_layout->addWidget(reset_button);
  button_layout->addWidget(reset_start_button);

  status_label_ = new QLabel("No robots found.");
  status_label_->setTextFormat(Qt::RichText);
  result_label_ = new QLabel;
  result_label_->setWordWrap(true);

  auto* layout = new QVBoxLayout;
  layout->addLayout(robot_layout);
  layout->addLayout(button_layout);
  layout->addWidget(status_label_);
  layout->addWidget(result_label_);
  setLayout(layout);

  connect(refresh_button, &QPushButton::clicked, this, &ChangeDetectionPanel::discoverRobots);
  connect(start_button, &QPushButton::clicked, this, &ChangeDetectionPanel::onStart);
  connect(stop_button, &QPushButton::clicked, this, &ChangeDetectionPanel::onStop);
  connect(reset_button, &QPushButton::clicked, this, &ChangeDetectionPanel::onReset);
  connect(reset_start_button, &QPushButton::clicked, this, &ChangeDetectionPanel::onResetAndStart);

  discovery_timer_ = new QTimer(this);
  connect(discovery_timer_, &QTimer::timeout, this, &ChangeDetectionPanel::discoverRobots);
}

void ChangeDetectionPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  discoverRobots();
  discovery_timer_->start(2000);
}

void ChangeDetectionPanel::discoverRobots() {
  if (!node_) {
    return;
  }

  for (const auto& [service, types] : node_->get_service_names_and_types()) {
    if (service.size() <= kStartSuffix.size() ||
        service.compare(service.size() - kStartSuffix.size(), kStartSuffix.size(), kStartSuffix)) {
      continue;
    }
    const auto robot_ns = service.substr(0, service.size() - kStartSuffix.size());
    const auto ns = robot_ns + "/change_detection";
    auto name = QString::fromStdString(robot_ns);
    if (name.startsWith('/')) {
      name.remove(0, 1);
    }
    if (name.isEmpty()) {
      name = "/";
    }
    if (!robots_.count(name)) {
      addRobot(name, ns);
    }
  }
}

void ChangeDetectionPanel::addRobot(const QString& name, const std::string& ns) {
  auto& robot = robots_[name];
  robot.ns = ns;
  for (const auto& command : {"start", "stop", "reset"}) {
    robot.clients[command] = node_->create_client<Trigger>(ns + "/" + command);
  }
  robot.state_sub = node_->create_subscription<std_msgs::msg::Bool>(
      ns + "/enabled",
      rclcpp::QoS(1).transient_local().reliable(),
      [this, name](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool enabled = msg->data;
        postToGui([this, name, enabled]() {
          robots_[name].enabled = enabled;
          updateStatus();
        });
      });

  robot_box_->addItem(name);
  if (name == pending_selection_) {
    robot_box_->setCurrentText(name);
  }
  updateStatus();
}

void ChangeDetectionPanel::sendToSelected(const std::string& command, const std::string& then) {
  if (robots_.empty()) {
    setResult("No robots found.");
    return;
  }
  const auto selected = robot_box_->currentText();
  if (selected == kAll) {
    for (const auto& [name, robot] : robots_) {
      send(name, command, then);
    }
  } else {
    send(selected, command, then);
  }
}

void ChangeDetectionPanel::send(const QString& robot_name,
                                const std::string& command,
                                const std::string& then) {
  const auto label = robot_name + "/" + QString::fromStdString(command);
  auto it = robots_.find(robot_name);
  if (it == robots_.end()) {
    setResult(label + ": unknown robot");
    return;
  }
  auto client = it->second.clients.at(command);
  if (!client->service_is_ready()) {
    setResult(label + ": service not available");
    return;
  }

  // Never block the GUI: the response arrives on the executor thread.
  client->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, robot_name, label, then](rclcpp::Client<Trigger>::SharedFuture future) {
        const auto response = future.get();
        const auto text = label + ": " + QString::fromStdString(response->message);
        const bool success = response->success;
        postToGui([this, robot_name, text, success, then]() {
          setResult(text);
          if (success && !then.empty()) {
            send(robot_name, then, "");
          }
        });
      });
}

void ChangeDetectionPanel::postToGui(std::function<void()> fn) {
  QMetaObject::invokeMethod(this, std::move(fn), Qt::QueuedConnection);
}

void ChangeDetectionPanel::setResult(const QString& text) { result_label_->setText(text); }

void ChangeDetectionPanel::updateStatus() {
  if (robots_.empty()) {
    status_label_->setText("No robots found.");
    return;
  }
  QString text;
  for (const auto& [name, robot] : robots_) {
    QString state = "<span style='color:gray'>? unknown</span>";
    if (robot.enabled) {
      state = *robot.enabled ? "<span style='color:green'>&#9679; running</span>"
                             : "<span style='color:gray'>&#9675; stopped</span>";
    }
    text += "<b>" + name.toHtmlEscaped() + "</b>&nbsp;&nbsp;" + state + "<br>";
  }
  status_label_->setText(text);
}

void ChangeDetectionPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  config.mapSetValue("robot", robot_box_->currentText());
}

void ChangeDetectionPanel::load(const rviz_common::Config& config) {
  Panel::load(config);
  QString robot;
  if (config.mapGetString("robot", &robot)) {
    pending_selection_ = robot;
    if (robot_box_->findText(robot) >= 0) {
      robot_box_->setCurrentText(robot);
    }
  }
}

}  // namespace dcist_launch_system_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dcist_launch_system_rviz::ChangeDetectionPanel, rviz_common::Panel)
