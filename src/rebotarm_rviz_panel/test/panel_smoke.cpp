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
  int homes = 0, stops = 0, releases = 0;
  auto home = node->create_service<std_srvs::srv::Trigger>(prefix + "/stop_and_home",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { ++homes; response->success = true; response->message = "home accepted"; });
  auto stop = node->create_service<std_srvs::srv::Trigger>(prefix + "/safety_stop",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { ++stops; response->success = true; response->message = "stop accepted"; });
  auto release = node->create_service<std_srvs::srv::Trigger>(prefix + "/safety_release",
    [&](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { ++releases; response->success = true; response->message = "release accepted"; });
  auto publisher = node->create_publisher<rebotarm_msgs::msg::ArmStatus>(prefix + "/arm_status", rclcpp::QoS(1).transient_local());
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("rebotarm_rviz_panel/SafetyPanel");
  panel->resize(360, 500);
  panel->show();
  QPushButton *home_button = nullptr, *stop_button = nullptr, *release_button = nullptr;
  for (auto button : panel->findChildren<QPushButton *>()) {
    if (button->text().contains(QString::fromUtf8("停止并"))) home_button = button;
    else if (button->text().contains(QString::fromUtf8("解除锁止"))) release_button = button;
    else stop_button = button;
  }
  if (!home_button || !stop_button || !release_button || release_button->isEnabled()) return 2;
  int ticks = 0;
  int phase = 0;
  bool success = false;
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(node);
    rebotarm_msgs::msg::ArmStatus status;
    status.state_machine = releases ? "IDLE" : (stops ? "SAFETY_STOPPED" : "SAFETY_HOMING");
    status.enabled = true;
    status.control_loop_active = true;
    publisher->publish(status);
    if (phase == 0 && home_button->isEnabled()) {
      if (release_button->isEnabled()) { app.quit(); return; }
      home_button->click(); phase = 1;
    }
    if (phase == 1 && homes == 1 && stop_button->isEnabled()) { stop_button->click(); phase = 2; }
    if (phase == 2 && stops == 1) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("stop accepted")) phase = 3;
      }
    }
    if (phase == 3) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("SAFETY_STOPPED") && release_button->isEnabled()) {
          panel->grab().save("/tmp/rebotarm_rviz_safety_panel.png");
          release_button->click(); phase = 4;
        }
      }
    }
    if (phase == 4 && releases == 1) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("release accepted")) phase = 5;
      }
    }
    if (phase == 5) {
      for (auto label : panel->findChildren<QLabel *>()) {
        if (label->text().contains("IDLE") && !release_button->isEnabled()) {
          success = true; app.quit();
        }
      }
    }
    if (++ticks > 300) app.quit();
  });
  timer.start(30);
  app.exec();
  panel.reset();
  rclcpp::shutdown();
  std::cout << "home requests=" << homes << " stop requests=" << stops << " releases=" << releases << " success=" << success << '\n';
  return success ? 0 : 1;
}
