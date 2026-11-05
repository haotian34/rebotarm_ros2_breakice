#include <QApplication>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rebotarm_msgs/msg/arm_status.hpp>
#include <iostream>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panel_smoke_backend");
  const auto prefix = std::string("/") + qEnvironmentVariable("REBOTARM_SAFETY_NAMESPACE", "rebotarm").toStdString();
  int homes = 0, stops = 0;
  auto home = node->create_service<std_srvs::srv::Trigger>(prefix + "/stop_and_home",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { ++homes; response->success = true; response->message = "home accepted"; });
  auto stop = node->create_service<std_srvs::srv::Trigger>(prefix + "/safety_stop",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { ++stops; response->success = true; response->message = "stop accepted"; });
  auto publisher = node->create_publisher<rebotarm_msgs::msg::ArmStatus>(prefix + "/arm_status", rclcpp::QoS(1).transient_local());
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("rebotarm_rviz_panel/SafetyPanel");
  panel->resize(360, 500);
  panel->show();
  QPushButton *home_button = nullptr, *stop_button = nullptr;
  for (auto button : panel->findChildren<QPushButton *>()) {
    if (button->text().contains(QString::fromUtf8("停止并"))) home_button = button;
    else stop_button = button;
  }
  if (!home_button || !stop_button) return 2;
  int ticks = 0;
  int phase = 0;
  bool success = false;
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(node);
    rebotarm_msgs::msg::ArmStatus status;
    status.state_machine = stops ? "SAFETY_STOPPED" : "SAFETY_HOMING";
    publisher->publish(status);
    if (phase == 0 && home_button->isEnabled()) { home_button->click(); phase = 1; }
    if (phase == 1 && homes == 1 && stop_button->isEnabled()) { stop_button->click(); phase = 2; }
    if (phase == 2 && stops == 1) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("stop accepted")) phase = 3;
      }
    }
    if (phase == 3) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("SAFETY_STOPPED")) {
          panel->grab().save("/tmp/rebotarm_rviz_safety_panel.png");
          success = true;
          app.quit();
        }
      }
    }
    if (++ticks > 300) app.quit();
  });
  timer.start(30);
  app.exec();
  panel.reset();
  rclcpp::shutdown();
  std::cout << "home requests=" << homes << " stop requests=" << stops << " success=" << success << '\n';
  return success ? 0 : 1;
}
