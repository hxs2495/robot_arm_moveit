#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMetaObject>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QVBoxLayout>
#include <QWidget>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kDegreesToRadians = M_PI / 180.0;

QString nowText()
{
  return QDateTime::currentDateTime().toString("HH:mm:ss");
}
}  // namespace

class MoveItWorker : public QObject
{
  Q_OBJECT

public:
  explicit MoveItWorker(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

public slots:
  void initialize()
  {
    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
      move_group_->setPlanningTime(8.0);
      ready_.store(true);
      emit backendReady(true);
      emit logMessage("MoveIt 控制接口已连接，可以开始规划");
    } catch (const std::exception& error) {
      emit logMessage(QString("MoveIt 初始化失败：%1").arg(error.what()));
      emit backendReady(false);
    }
  }

  void planTarget(
    const QVector<double>& target_degrees,
    int velocity_percent,
    int acceleration_percent,
    double planning_seconds,
    bool execute_after_planning)
  {
    if (!ready_.load() || !move_group_) {
      emit logMessage("控制接口尚未就绪，请确认 move_group 已启动");
      return;
    }

    emit busyChanged(true);
    emit planAvailabilityChanged(false);
    has_plan_ = false;

    std::vector<double> target_radians;
    target_radians.reserve(static_cast<std::size_t>(target_degrees.size()));
    for (const double degree : target_degrees) {
      target_radians.push_back(degree * kDegreesToRadians);
    }

    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(planning_seconds);
    move_group_->setMaxVelocityScalingFactor(velocity_percent / 100.0);
    move_group_->setMaxAccelerationScalingFactor(acceleration_percent / 100.0);

    if (!move_group_->setJointValueTarget(target_radians)) {
      emit logMessage("目标关节角无效或超出机械臂限制");
      emit busyChanged(false);
      return;
    }

    emit logMessage("正在规划关节轨迹……");
    moveit::planning_interface::MoveGroupInterface::Plan candidate;
    const bool success = static_cast<bool>(move_group_->plan(candidate));
    if (!success) {
      emit logMessage("轨迹规划失败，请调整目标角度后重试");
      emit busyChanged(false);
      return;
    }

    latest_plan_ = std::move(candidate);
    has_plan_ = true;
    emit planAvailabilityChanged(true);
    emit logMessage("轨迹规划成功");

    if (execute_after_planning) {
      executeLatestPlanInternal();
    } else {
      emit busyChanged(false);
    }
  }

  void executeLatestPlan()
  {
    if (!ready_.load() || !move_group_) {
      emit logMessage("控制接口尚未就绪");
      return;
    }
    if (!has_plan_) {
      emit logMessage("当前没有可执行轨迹，请先点击“规划轨迹”");
      return;
    }

    emit busyChanged(true);
    executeLatestPlanInternal();
  }

  void stopMovement()
  {
    if (ready_.load() && move_group_) {
      move_group_->stop();
      emit logMessage("已发送停止当前运动指令");
    }
  }

signals:
  void backendReady(bool ready);
  void busyChanged(bool busy);
  void planAvailabilityChanged(bool available);
  void logMessage(const QString& text);

private:
  void executeLatestPlanInternal()
  {
    emit logMessage("正在执行轨迹……");
    const bool success = static_cast<bool>(move_group_->execute(latest_plan_));
    emit logMessage(success ? "轨迹执行完成" : "轨迹执行失败或已停止");
    emit busyChanged(false);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan latest_plan_;
  std::atomic_bool ready_{false};
  bool has_plan_{false};
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    setWindowTitle("机械臂智能控制中心");
    resize(1180, 820);
    setMinimumSize(980, 720);
    buildInterface();
    applyStyle();
    createJointStateSubscription();
    startMoveItWorker();
    appendLog("界面已启动，正在等待机械臂状态和 MoveIt 服务");
  }

  ~MainWindow() override
  {
    worker_thread_.quit();
    worker_thread_.wait();
  }

signals:
  void requestPlan(QVector<double> target, int velocity, int acceleration, double time, bool execute);
  void requestExecute();
  void requestStop();

private:
  void buildInterface()
  {
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(22, 20, 22, 20);
    root->setSpacing(12);

    auto* header = new QWidget;
    header->setObjectName("headerPanel");
    auto* header_layout = new QHBoxLayout(header);
    header_layout->setContentsMargins(22, 16, 22, 16);
    auto* title_column = new QVBoxLayout;
    auto* title = new QLabel("机械臂智能控制中心");
    title->setObjectName("titleLabel");
    auto* subtitle = new QLabel("六轴协作机械臂 · 运动规划 · 仿真控制");
    subtitle->setObjectName("subtitleLabel");
    title_column->addWidget(title);
    title_column->addWidget(subtitle);
    header_layout->addLayout(title_column);
    header_layout->addStretch();
    auto* mode_badge = new QLabel("◆  数字孪生仿真");
    mode_badge->setObjectName("modeBadge");
    mode_badge->setAlignment(Qt::AlignCenter);
    header_layout->addWidget(mode_badge);
    status_badge_ = new QLabel("●  正在连接");
    status_badge_->setObjectName("statusBadge");
    status_badge_->setAlignment(Qt::AlignCenter);
    status_badge_->setMinimumWidth(170);
    header_layout->addWidget(status_badge_);
    root->addWidget(header);

    auto* settings_group = new QGroupBox("运动规划参数");
    auto* settings = new QHBoxLayout(settings_group);
    velocity_spin_ = new QSpinBox;
    velocity_spin_->setRange(1, 100);
    velocity_spin_->setValue(35);
    velocity_spin_->setSuffix(" %");
    acceleration_spin_ = new QSpinBox;
    acceleration_spin_->setRange(1, 100);
    acceleration_spin_->setValue(35);
    acceleration_spin_->setSuffix(" %");
    planning_time_spin_ = new QDoubleSpinBox;
    planning_time_spin_->setRange(1.0, 30.0);
    planning_time_spin_->setValue(8.0);
    planning_time_spin_->setSingleStep(1.0);
    planning_time_spin_->setSuffix(" 秒");
    settings->addWidget(new QLabel("最大速度："));
    settings->addWidget(velocity_spin_);
    settings->addSpacing(18);
    settings->addWidget(new QLabel("最大加速度："));
    settings->addWidget(acceleration_spin_);
    settings->addSpacing(18);
    settings->addWidget(new QLabel("规划时间："));
    settings->addWidget(planning_time_spin_);
    settings->addStretch();
    root->addWidget(settings_group);

    auto* joint_group = new QGroupBox("关节控制台  ·  实时反馈与目标设定（单位：度）");
    auto* grid = new QGridLayout(joint_group);
    grid->setHorizontalSpacing(18);
    grid->setVerticalSpacing(9);
    const QStringList headers{"轴号 / 功能", "实时反馈", "目标设定", "关节调节范围"};
    for (int column = 0; column < headers.size(); ++column) {
      auto* label = new QLabel(headers[column]);
      label->setObjectName("tableHeader");
      grid->addWidget(label, 0, column);
    }

    const std::array<double, 6> lower_limits{-180.0, -134.0, -149.0, -180.0, -126.0, -180.0};
    const std::array<double, 6> upper_limits{180.0, 134.0, 149.0, 180.0, 126.0, 180.0};
    const std::array<QString, 6> joint_names{
      "J1  基座回转", "J2  肩部俯仰", "J3  肘部俯仰",
      "J4  腕部回转", "J5  腕部俯仰", "J6  末端回转"};
    for (int index = 0; index < 6; ++index) {
      auto* name = new QLabel(joint_names[index]);
      name->setObjectName("jointName");
      current_labels_[index] = new QLabel("--.-°");
      current_labels_[index]->setObjectName("currentValue");
      current_labels_[index]->setMinimumWidth(90);

      target_spins_[index] = new QDoubleSpinBox;
      target_spins_[index]->setRange(lower_limits[index], upper_limits[index]);
      target_spins_[index]->setDecimals(1);
      target_spins_[index]->setSingleStep(1.0);
      target_spins_[index]->setSuffix("°");
      target_spins_[index]->setMinimumWidth(120);

      target_sliders_[index] = new QSlider(Qt::Horizontal);
      target_sliders_[index]->setRange(
        static_cast<int>(lower_limits[index] * 10.0),
        static_cast<int>(upper_limits[index] * 10.0));
      target_sliders_[index]->setSingleStep(10);
      target_sliders_[index]->setPageStep(50);

      connect(target_spins_[index], qOverload<double>(&QDoubleSpinBox::valueChanged),
        this, [this, index](double value) {
          target_sliders_[index]->setValue(static_cast<int>(std::round(value * 10.0)));
          plan_available_ = false;
          refreshButtonState();
        });
      connect(target_sliders_[index], &QSlider::valueChanged,
        this, [this, index](int value) {
          target_spins_[index]->setValue(value / 10.0);
        });

      grid->addWidget(name, index + 1, 0);
      grid->addWidget(current_labels_[index], index + 1, 1);
      grid->addWidget(target_spins_[index], index + 1, 2);
      grid->addWidget(target_sliders_[index], index + 1, 3);
    }
    grid->setColumnStretch(3, 1);
    root->addWidget(joint_group, 1);

    auto* preset_row = new QHBoxLayout;
    auto* current_button = new QPushButton("同步当前姿态");
    auto* home_button = new QPushButton("载入零位");
    auto* ready_button = new QPushButton("载入准备位");
    preset_row->addWidget(current_button);
    preset_row->addWidget(home_button);
    preset_row->addWidget(ready_button);
    preset_row->addStretch();
    root->addLayout(preset_row);

    connect(current_button, &QPushButton::clicked, this, [this]() {
      if (!has_joint_state_) {
        appendLog("尚未收到关节状态，无法读取当前姿态");
        return;
      }
      setTargetValues(latest_degrees_);
      appendLog("已将当前姿态复制为目标姿态");
    });
    connect(home_button, &QPushButton::clicked, this, [this]() {
      setTargetValues({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      appendLog("已载入零位目标");
    });
    connect(ready_button, &QPushButton::clicked, this, [this]() {
      setTargetValues({0.0, -0.65 * kRadiansToDegrees, 1.15 * kRadiansToDegrees,
        0.0, -0.50 * kRadiansToDegrees, 0.0});
      appendLog("已载入准备位目标");
    });

    auto* action_row = new QHBoxLayout;
    plan_button_ = new QPushButton("轨迹规划");
    plan_button_->setObjectName("primaryButton");
    execute_button_ = new QPushButton("执行轨迹");
    plan_execute_button_ = new QPushButton("规划并执行");
    plan_execute_button_->setObjectName("successButton");
    stop_button_ = new QPushButton("立即停止");
    stop_button_->setObjectName("stopButton");
    action_row->addWidget(plan_button_);
    action_row->addWidget(execute_button_);
    action_row->addWidget(plan_execute_button_);
    action_row->addStretch();
    action_row->addWidget(stop_button_);
    root->addLayout(action_row);

    connect(plan_button_, &QPushButton::clicked, this, [this]() { sendPlanRequest(false); });
    connect(plan_execute_button_, &QPushButton::clicked, this, [this]() { sendPlanRequest(true); });
    connect(execute_button_, &QPushButton::clicked, this, &MainWindow::requestExecute);
    connect(stop_button_, &QPushButton::clicked, this, &MainWindow::requestStop);

    auto* log_group = new QGroupBox("系统运行日志");
    auto* log_layout = new QVBoxLayout(log_group);
    log_view_ = new QTextEdit;
    log_view_->setReadOnly(true);
    log_view_->setMaximumHeight(165);
    log_layout->addWidget(log_view_);
    root->addWidget(log_group);

    setCentralWidget(central);
    refreshButtonState();
  }

  void applyStyle()
  {
    setFont(QFont("Noto Sans CJK SC", 10));
    setStyleSheet(R"(
      QMainWindow { background-color: #08111f; }
      QWidget { background: transparent; color: #dce7f5; }
      QWidget#headerPanel {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #0e3d70, stop:0.58 #11558b, stop:1 #087b8d);
        border: 1px solid #25799d; border-radius: 12px;
      }
      QLabel#titleLabel { color: #ffffff; font-size: 25px; font-weight: 700; }
      QLabel#subtitleLabel { color: #b8d8eb; font-size: 12px; }
      QLabel#modeBadge { color: #69ddec; background: #0d314b; border: 1px solid #2b7890;
                         border-radius: 14px; padding: 7px 14px; font-weight: 600; }
      QLabel#statusBadge { color: #63c7ff; background: #123047; border: 1px solid #265a7c;
                           border-radius: 14px; padding: 7px 14px; font-weight: 600; }
      QGroupBox { background-color: #101c2d; border: 1px solid #243750; border-radius: 9px;
                  margin-top: 12px; padding: 14px; font-weight: 600; color: #dce7f5; }
      QGroupBox::title { subcontrol-origin: margin; left: 15px; padding: 0 7px;
                         color: #8fadd0; background: #101c2d; }
      QLabel#tableHeader { color: #6f8baa; font-size: 11px; font-weight: 600; }
      QLabel#jointName { color: #edf5ff; font-weight: 600; }
      QLabel#currentValue { color: #44d5e6; font-size: 15px; font-weight: 700; }
      QDoubleSpinBox, QSpinBox { color: #f1f7ff; background: #16263a;
                                border: 1px solid #304966; border-radius: 6px;
                                padding: 6px 9px; selection-background-color: #168fa8; }
      QDoubleSpinBox:hover, QSpinBox:hover { border-color: #3bbbd0; }
      QDoubleSpinBox:focus, QSpinBox:focus { border: 1px solid #47cee0; }
      QSlider::groove:horizontal { height: 5px; background: #263a52; border-radius: 2px; }
      QSlider::sub-page:horizontal { background: #18a9c0; border-radius: 2px; }
      QSlider::add-page:horizontal { background: #263a52; border-radius: 2px; }
      QSlider::handle:horizontal { background: #dffbff; border: 2px solid #20b9ce;
                                   width: 16px; margin: -7px 0; border-radius: 9px; }
      QPushButton { color: #cfdded; background: #17273b; border: 1px solid #334b66;
                    border-radius: 7px; padding: 9px 18px; font-weight: 600; }
      QPushButton:hover { color: white; background: #203650; border-color: #4a6e91; }
      QPushButton:pressed { background: #0f1d2e; }
      QPushButton:disabled { color: #56677a; background: #111b29; border-color: #223044; }
      QPushButton#primaryButton { color: white; background: #176fb3; border-color: #2b91d4; }
      QPushButton#primaryButton:hover { background: #2082c8; }
      QPushButton#successButton { color: white; background: #087f72; border-color: #18aa98; }
      QPushButton#successButton:hover { background: #0d9485; }
      QPushButton#stopButton { color: white; background: #a92f43; border-color: #d14b5e; }
      QPushButton#stopButton:hover { background: #c33a50; }
      QTextEdit { color: #a9d9dc; background: #080f19; border: 1px solid #21334a;
                  border-radius: 6px; font-family: "Noto Sans Mono", monospace; padding: 8px; }
      QScrollBar:vertical { background: #0d1725; width: 9px; border-radius: 4px; }
      QScrollBar::handle:vertical { background: #314860; min-height: 25px; border-radius: 4px; }
      QToolTip { color: white; background: #16263a; border: 1px solid #3b607d; }
    )");
  }

  void startMoveItWorker()
  {
    worker_ = new MoveItWorker(node_);
    worker_->moveToThread(&worker_thread_);
    connect(&worker_thread_, &QThread::started, worker_, &MoveItWorker::initialize);
    connect(&worker_thread_, &QThread::finished, worker_, &QObject::deleteLater);
    connect(this, &MainWindow::requestPlan, worker_, &MoveItWorker::planTarget);
    connect(this, &MainWindow::requestExecute, worker_, &MoveItWorker::executeLatestPlan);
    connect(this, &MainWindow::requestStop, worker_, &MoveItWorker::stopMovement, Qt::DirectConnection);
    connect(worker_, &MoveItWorker::backendReady, this, [this](bool ready) {
      backend_ready_ = ready;
      refreshStatus();
      refreshButtonState();
    });
    connect(worker_, &MoveItWorker::busyChanged, this, [this](bool busy) {
      busy_ = busy;
      refreshButtonState();
    });
    connect(worker_, &MoveItWorker::planAvailabilityChanged, this, [this](bool available) {
      plan_available_ = available;
      refreshButtonState();
    });
    connect(worker_, &MoveItWorker::logMessage, this, &MainWindow::appendLog);
    worker_thread_.start();
  }

  void createJointStateSubscription()
  {
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        QVector<double> values(6, 0.0);
        std::array<bool, 6> found{false, false, false, false, false, false};
        for (std::size_t msg_index = 0;
          msg_index < message->name.size() && msg_index < message->position.size(); ++msg_index)
        {
          for (int joint_index = 0; joint_index < 6; ++joint_index) {
            if (message->name[msg_index] == "joint_" + std::to_string(joint_index + 1)) {
              values[joint_index] = message->position[msg_index] * kRadiansToDegrees;
              found[joint_index] = true;
              break;
            }
          }
        }
        for (const bool present : found) {
          if (!present) {
            return;
          }
        }
        QMetaObject::invokeMethod(this, [this, values]() { updateJointState(values); },
          Qt::QueuedConnection);
      });
  }

  void updateJointState(const QVector<double>& degrees)
  {
    for (int index = 0; index < 6; ++index) {
      latest_degrees_[index] = degrees[index];
      current_labels_[index]->setText(QString::number(degrees[index], 'f', 1) + "°");
    }
    if (!has_joint_state_) {
      has_joint_state_ = true;
      setTargetValues(latest_degrees_);
      appendLog("已收到六个关节的实时状态");
      refreshStatus();
    }
  }

  void setTargetValues(const std::array<double, 6>& values)
  {
    for (int index = 0; index < 6; ++index) {
      target_spins_[index]->setValue(values[index]);
    }
    plan_available_ = false;
    refreshButtonState();
  }

  QVector<double> targetValues() const
  {
    QVector<double> result;
    result.reserve(6);
    for (const auto* spin : target_spins_) {
      result.push_back(spin->value());
    }
    return result;
  }

  void sendPlanRequest(bool execute_after_planning)
  {
    if (!backend_ready_ || !has_joint_state_) {
      appendLog("后台尚未就绪，请等待状态指示变为“系统已就绪”");
      return;
    }
    emit requestPlan(targetValues(), velocity_spin_->value(), acceleration_spin_->value(),
      planning_time_spin_->value(), execute_after_planning);
  }

  void refreshStatus()
  {
    if (backend_ready_ && has_joint_state_) {
      status_badge_->setText("●  系统已就绪");
      status_badge_->setStyleSheet("color:#5de2a5; background:#12382f; border:1px solid #266b55;");
    } else if (backend_ready_) {
      status_badge_->setText("●  等待关节状态");
      status_badge_->setStyleSheet("color:#f0c66a; background:#3a2f18; border:1px solid #705c29;");
    } else {
      status_badge_->setText("●  正在连接 MoveIt");
      status_badge_->setStyleSheet("color:#63c7ff; background:#123047; border:1px solid #265a7c;");
    }
  }

  void refreshButtonState()
  {
    const bool can_control = backend_ready_ && has_joint_state_ && !busy_;
    plan_button_->setEnabled(can_control);
    plan_execute_button_->setEnabled(can_control);
    execute_button_->setEnabled(can_control && plan_available_);
    stop_button_->setEnabled(backend_ready_);
  }

  void appendLog(const QString& text)
  {
    log_view_->append(QString("[%1] %2").arg(nowText(), text));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  QThread worker_thread_;
  MoveItWorker* worker_{nullptr};

  QLabel* status_badge_{nullptr};
  std::array<QLabel*, 6> current_labels_{};
  std::array<QDoubleSpinBox*, 6> target_spins_{};
  std::array<QSlider*, 6> target_sliders_{};
  QSpinBox* velocity_spin_{nullptr};
  QSpinBox* acceleration_spin_{nullptr};
  QDoubleSpinBox* planning_time_spin_{nullptr};
  QPushButton* plan_button_{nullptr};
  QPushButton* execute_button_{nullptr};
  QPushButton* plan_execute_button_{nullptr};
  QPushButton* stop_button_{nullptr};
  QTextEdit* log_view_{nullptr};

  std::array<double, 6> latest_degrees_{};
  bool backend_ready_{false};
  bool has_joint_state_{false};
  bool plan_available_{false};
  bool busy_{false};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  application.setApplicationName("机械臂智能控制中心");

  auto node = rclcpp::Node::make_shared(
    "simple_arm_gui", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() { executor.spin(); });

  int result = 0;
  {
    MainWindow window(node);
    window.show();
    QTimer shutdown_timer;
    QObject::connect(&shutdown_timer, &QTimer::timeout, &application, [&application]() {
      if (!rclcpp::ok()) {
        application.quit();
      }
    });
    shutdown_timer.start(100);
    result = application.exec();
    executor.cancel();
    ros_thread.join();
  }

  rclcpp::shutdown();
  return result;
}

#include "main.moc"
