#include <chrono>
#include <memory>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rebotarm_msgs/msg/arm_status.hpp>

namespace rebotarm_rviz_panel {
class SafetyPanel : public rviz_common::Panel {
public:
  explicit SafetyPanel(QWidget *parent = nullptr) : Panel(parent) {
    auto *layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(QString::fromUtf8("机械臂停止与复位")));
    namespace_ = new QLineEdit(qEnvironmentVariable("REBOTARM_SAFETY_NAMESPACE", "rebotarm"));
    layout->addWidget(new QLabel(QString::fromUtf8("机械臂命名空间")));
    layout->addWidget(namespace_);
    stop_ = new QPushButton(QString::fromUtf8("仅停止 / 中断复位"));
    stop_->setMinimumHeight(52);
    stop_->setStyleSheet("background:#b71c1c;color:white;font-size:18px;font-weight:bold;");
    home_ = new QPushButton(QString::fromUtf8("停止并缓慢复位"));
    home_->setMinimumHeight(48);
    layout->addWidget(stop_);
    layout->addWidget(home_);
    state_ = new QLabel(QString::fromUtf8("等待控制器状态"));
    state_->setWordWrap(true);
    result_ = new QLabel;
    result_->setWordWrap(true);
    layout->addWidget(state_);
    layout->addWidget(result_);
    auto *hint = new QLabel(QString::fromUtf8("回零前确保路径无人、无障碍物。\n软件停止不能替代硬件急停。\n触发后锁止运动，检查后重启控制器。"));
    hint->setWordWrap(true);
    layout->addWidget(hint);
    layout->addStretch();
    node_ = std::make_shared<rclcpp::Node>("rebotarm_rviz_safety", rclcpp::NodeOptions().use_global_arguments(false));
    executor_.add_node(node_);
    connect(namespace_, &QLineEdit::editingFinished, this, [this]() {
      if (namespace_->text() != bound_namespace_) { bind(); Q_EMIT configChanged(); }
    });
    connect(stop_, &QPushButton::clicked, this, [this]() { send(stop_client_); });
    connect(home_, &QPushButton::clicked, this, [this]() { send(home_client_); });
    bind();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      executor_.spin_some(std::chrono::milliseconds(2));
      stop_->setEnabled(stop_client_ && stop_client_->service_is_ready());
      home_->setEnabled(home_client_ && home_client_->service_is_ready());
      if (std::chrono::steady_clock::now() - last_state_ > std::chrono::seconds(2)) {
        state_->setText(QString::fromUtf8("控制器状态不可用或已过期"));
      }
      if (pending_ && std::chrono::steady_clock::now() > deadline_) {
        pending_client_->remove_pending_request(pending_id_);
        pending_ = false;
        result_->setText(QString::fromUtf8("请求超时，未确认停止；必要时使用硬件急停"));
      }
    });
    timer_->start(30);
  }
  ~SafetyPanel() override {
    // QWidget teardown can emit editingFinished after ROS members are destroyed.
    QObject::disconnect(namespace_, nullptr, this, nullptr);
    QObject::disconnect(timer_, nullptr, this, nullptr);
    timer_->stop();
    executor_.remove_node(node_);
  }
  void save(rviz_common::Config config) const override {
    Panel::save(config);
    config.mapSetValue("Arm Namespace", namespace_->text());
  }
  void load(const rviz_common::Config &config) override {
    Panel::load(config);
    QString value;
    if (!qEnvironmentVariableIsSet("REBOTARM_SAFETY_NAMESPACE") && config.mapGetString("Arm Namespace", &value)) {
      namespace_->setText(value);
      bind();
    }
  }
private:
  using Trigger = std_srvs::srv::Trigger;
  using Client = rclcpp::Client<Trigger>;
  void bind() {
    if (pending_) { pending_client_->remove_pending_request(pending_id_); pending_ = false; }
    auto ns = namespace_->text().trimmed();
    while (ns.startsWith('/')) ns.remove(0, 1);
    while (ns.endsWith('/')) ns.chop(1);
    const auto prefix = std::string("/") + ns.toStdString();
    try {
      auto stop_client = node_->create_client<Trigger>(prefix + "/safety_stop");
      auto home_client = node_->create_client<Trigger>(prefix + "/stop_and_home");
      auto subscription = node_->create_subscription<rebotarm_msgs::msg::ArmStatus>(
        prefix + "/arm_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](rebotarm_msgs::msg::ArmStatus::ConstSharedPtr msg) {
          last_state_ = std::chrono::steady_clock::now();
          state_->setText(QString::fromUtf8("状态：") + QString::fromStdString(msg->state_machine));
        });
      stop_client_ = stop_client;
      home_client_ = home_client;
      subscription_ = subscription;
      bound_namespace_ = ns;
      namespace_->setText(ns);
      state_->setText(QString::fromUtf8("等待控制器状态"));
      result_->clear();
      last_state_ = std::chrono::steady_clock::time_point{};
    } catch (const std::exception &e) {
      namespace_->setText(bound_namespace_);
      result_->setText(QString::fromUtf8("命名空间无效，保留原目标：") + e.what());
    }
    stop_->setEnabled(stop_client_ && stop_client_->service_is_ready());
    home_->setEnabled(home_client_ && home_client_->service_is_ready());
  }
  void send(Client::SharedPtr client) {
    if (!client || !client->service_is_ready()) {
      result_->setText(QString::fromUtf8("停止服务不可用")); return;
    }
    if (pending_) pending_client_->remove_pending_request(pending_id_);
    result_->setText(QString::fromUtf8("请求已发送，等待控制器确认…"));
    pending_client_ = client;
    pending_ = true;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    auto request = client->async_send_request(std::make_shared<Trigger::Request>(),
      [this](Client::SharedFuture future) {
        pending_ = false;
        try {
          auto response = future.get();
          result_->setText((response->success ? QString::fromUtf8("已受理：") : QString::fromUtf8("失败：")) +
                          QString::fromStdString(response->message));
        } catch (const std::exception &e) { result_->setText(e.what()); }
      });
    pending_id_ = request.request_id;
  }
  QString bound_namespace_;
  QLineEdit *namespace_;
  QPushButton *stop_, *home_;
  QLabel *state_, *result_;
  QTimer *timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  Client::SharedPtr stop_client_, home_client_, pending_client_;
  rclcpp::Subscription<rebotarm_msgs::msg::ArmStatus>::SharedPtr subscription_;
  bool pending_ = false;
  int64_t pending_id_ = 0;
  std::chrono::steady_clock::time_point last_state_{}, deadline_{};
};
}
PLUGINLIB_EXPORT_CLASS(rebotarm_rviz_panel::SafetyPanel, rviz_common::Panel)
