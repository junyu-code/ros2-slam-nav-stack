#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSplitter>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_frame.hpp"

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void requestShutdown(int)
{
  shutdown_requested = 1;
}

QString workspaceRoot()
{
  const auto env = qgetenv("SLAM_NAV_WS");
  if (!env.isEmpty()) {
    return QString::fromLocal8Bit(env);
  }
  return QStringLiteral("/home/junyu/slam_nav_ws");
}

QString shellQuote(const QString & value)
{
  QString escaped = value;
  escaped.replace(QStringLiteral("'"), QStringLiteral("'\"'\"'"));
  return QStringLiteral("'") + escaped + QStringLiteral("'");
}

class OperatorWindow : public QMainWindow
{
public:
  OperatorWindow(
    QApplication * app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node)
  : workspace_(workspaceRoot())
  {
    setWindowTitle(QStringLiteral("SLAM Nav Operator"));
    resize(1500, 920);

    auto * root = new QWidget(this);
    auto * root_layout = new QVBoxLayout(root);
    root_layout->setContentsMargins(10, 10, 10, 10);
    root_layout->setSpacing(8);

    auto * title = new QLabel(QStringLiteral("SLAM Nav Operator 控制台"), root);
    title->setStyleSheet(QStringLiteral("font-size: 22px; font-weight: 700;"));
    root_layout->addWidget(title);

    auto * main_splitter = new QSplitter(Qt::Horizontal, root);
    main_splitter->addWidget(buildLeftPanel());
    main_splitter->addWidget(buildCenterPanel(app, rviz_node));
    main_splitter->addWidget(buildRightPanel());
    main_splitter->setStretchFactor(0, 0);
    main_splitter->setStretchFactor(1, 1);
    main_splitter->setStretchFactor(2, 0);
    root_layout->addWidget(main_splitter, 1);

    log_view_ = new QPlainTextEdit(root);
    log_view_->setReadOnly(true);
    log_view_->setMaximumHeight(180);
    log_view_->setPlaceholderText(QStringLiteral("运行日志会显示在这里。"));
    root_layout->addWidget(log_view_);

    setCentralWidget(root);

    // 定时刷新轻量状态，不阻塞 RViz 渲染线程。
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() { refreshStatus(); });
    timer->start(2500);
    refreshStatus();
  }

private:
  QWidget * buildLeftPanel()
  {
    auto * panel = new QGroupBox(QStringLiteral("运行任务"), this);
    auto * layout = new QVBoxLayout(panel);

    addRunButton(layout, QStringLiteral("静态仿真后端"), QStringLiteral("sim-static"), {QStringLiteral("gui:=false")});
    addRunButton(layout, QStringLiteral("打开 Gazebo 窗口"), QStringLiteral("gazebo-client"));
    addRunButton(layout, QStringLiteral("关闭 Gazebo 窗口"), QStringLiteral("gazebo-client-stop"));
    addRunButton(layout, QStringLiteral("默认导航"), QStringLiteral("nav"));
    addRunButton(layout, QStringLiteral("一键导航演示"), QStringLiteral("demo-nav"));
    addRunButton(layout, QStringLiteral("自动建图"), QStringLiteral("auto-mapping"));
    addRunButton(layout, QStringLiteral("Piper 可视化"), QStringLiteral("piper-viz"));
    addRunButton(layout, QStringLiteral("图形自检"), QStringLiteral("ui-gui-check"));
    addRunButton(layout, QStringLiteral("地图检查"), QStringLiteral("task1-map-check"));
    addRunButton(layout, QStringLiteral("清理残留"), QStringLiteral("clean"));

    layout->addStretch(1);
    return panel;
  }

  QWidget * buildCenterPanel(
    QApplication * app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node)
  {
    auto * tabs = new QTabWidget(this);

    rviz_frame_ = new rviz_common::VisualizationFrame(rviz_node, tabs);
    rviz_frame_->setApp(app);
    rviz_frame_->setSplashPath(QString());
    rviz_frame_->initialize(rviz_node);
    const QString rviz_config = workspace_ + QStringLiteral("/src/slam_nav_bringup/rviz/nav2_debug.rviz");
    if (QFileInfo::exists(rviz_config)) {
      rviz_frame_->loadDisplayConfig(rviz_config);
    }
    tabs->addTab(rviz_frame_, QStringLiteral("RViz"));

    map_label_ = new QLabel(tabs);
    map_label_->setAlignment(Qt::AlignCenter);
    map_label_->setText(QStringLiteral("地图文件未加载"));
    loadMapPreview();
    tabs->addTab(map_label_, QStringLiteral("地图"));

    center_log_ = new QPlainTextEdit(tabs);
    center_log_->setReadOnly(true);
    center_log_->setPlaceholderText(QStringLiteral("中心日志视图"));
    tabs->addTab(center_log_, QStringLiteral("日志"));

    return tabs;
  }

  QWidget * buildRightPanel()
  {
    auto * panel = new QGroupBox(QStringLiteral("参数与状态"), this);
    auto * layout = new QVBoxLayout(panel);

    world_combo_ = new QComboBox(panel);
    world_combo_->addItem(QStringLiteral("静态验收场地"), QStringLiteral("static"));
    world_combo_->addItem(QStringLiteral("动态障碍场地"), QStringLiteral("dynamic"));
    world_combo_->addItem(QStringLiteral("大场地"), QStringLiteral("large_arena"));
    world_combo_->addItem(QStringLiteral("碰撞扰动场地"), QStringLiteral("large_arena_collision"));
    layout->addWidget(new QLabel(QStringLiteral("仿真场景"), panel));
    layout->addWidget(world_combo_);

    gui_check_ = new QCheckBox(QStringLiteral("启动 Gazebo 图形客户端"), panel);
    layout->addWidget(gui_check_);

    extra_args_ = new QLineEdit(panel);
    extra_args_->setPlaceholderText(QStringLiteral("附加参数，例如 use_sim_time:=true"));
    layout->addWidget(new QLabel(QStringLiteral("附加参数"), panel));
    layout->addWidget(extra_args_);

    status_list_ = new QListWidget(panel);
    layout->addWidget(new QLabel(QStringLiteral("系统状态"), panel));
    layout->addWidget(status_list_, 1);

    auto * stop_button = new QPushButton(QStringLiteral("停止最近任务"), panel);
    connect(stop_button, &QPushButton::clicked, this, [this]() { stopLastProcess(); });
    layout->addWidget(stop_button);

    auto * stop_all_button = new QPushButton(QStringLiteral("停止全部任务"), panel);
    connect(stop_all_button, &QPushButton::clicked, this, [this]() { stopAllProcesses(); });
    layout->addWidget(stop_all_button);

    return panel;
  }

  void addRunButton(
    QVBoxLayout * layout,
    const QString & label,
    const QString & command,
    const QStringList & default_args = {})
  {
    auto * button = new QPushButton(label, this);
    button->setMinimumHeight(38);
    connect(button, &QPushButton::clicked, this, [this, command, default_args]() {
      QStringList args = default_args;
      if (command == QStringLiteral("sim-static")) {
        args.clear();
        args << QStringLiteral("world:=") + world_combo_->currentData().toString();
        args << QStringLiteral("gui:=") + (gui_check_->isChecked() ? QStringLiteral("true") : QStringLiteral("false"));
      }
      const auto extra = extra_args_->text().trimmed();
      if (!extra.isEmpty()) {
        args.append(extra.split(QChar(' '), Qt::SkipEmptyParts));
      }
      runCommand(command, args);
    });
    layout->addWidget(button);
  }

  void runCommand(const QString & command, const QStringList & args = {})
  {
    auto * process = new QProcess(this);
    process->setWorkingDirectory(workspace_);
    process->setProcessChannelMode(QProcess::MergedChannels);

    QString line = QStringLiteral("cd ") + shellQuote(workspace_) + QStringLiteral(" && ./run.sh ") + shellQuote(command);
    for (const auto & arg : args) {
      line += QStringLiteral(" ") + shellQuote(arg);
    }

    appendLog(QStringLiteral("[operator] 启动: ./run.sh %1 %2").arg(command, args.join(QStringLiteral(" "))));

    connect(process, &QProcess::readyReadStandardOutput, this, [this, process]() {
      appendLog(QString::fromLocal8Bit(process->readAllStandardOutput()));
    });
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
      [this, process, command](int code, QProcess::ExitStatus) {
        appendLog(QStringLiteral("[operator] %1 退出，返回码 %2").arg(command).arg(code));
        processes_.removeAll(process);
        process->deleteLater();
        refreshStatus();
      });

    processes_.append(process);
    // 用 setsid 给每个长驻任务独立进程组，停止按钮才能连同 ROS/Gazebo 子进程一起收掉。
    process->start(QStringLiteral("setsid"), {QStringLiteral("bash"), QStringLiteral("-lc"), line});
  }

  void stopLastProcess()
  {
    if (processes_.isEmpty()) {
      appendLog(QStringLiteral("[operator] 没有由 Operator 直接启动的进程。"));
      return;
    }
    auto * process = processes_.last();
    stopProcessGroup(process, QStringLiteral("最近任务"));
  }

  void stopAllProcesses()
  {
    if (processes_.isEmpty()) {
      appendLog(QStringLiteral("[operator] 没有由 Operator 直接启动的进程。"));
      return;
    }
    const auto snapshot = processes_;
    for (auto * process : snapshot) {
      stopProcessGroup(process, QStringLiteral("全部任务"));
    }
  }

  void stopProcessGroup(QProcess * process, const QString & reason)
  {
    if (!process || process->state() == QProcess::NotRunning) {
      return;
    }
    const qint64 pid = process->processId();
    appendLog(QStringLiteral("[operator] 请求停止%1，pid=%2。").arg(reason).arg(pid));
    if (pid <= 0) {
      process->terminate();
      return;
    }

    auto * killer = new QProcess(this);
    connect(killer, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), killer, &QProcess::deleteLater);
    killer->start(QStringLiteral("bash"), {
      QStringLiteral("-lc"),
      QStringLiteral("kill -TERM -- -%1 2>/dev/null || kill -TERM %1 2>/dev/null || true").arg(pid)
    });

    QTimer::singleShot(3500, this, [this, process, pid]() {
      if (!processes_.contains(process) || process->state() == QProcess::NotRunning) {
        return;
      }
      appendLog(QStringLiteral("[operator] 进程未按时退出，强制结束 pid=%1。").arg(pid));
      QProcess::execute(QStringLiteral("bash"), {
        QStringLiteral("-lc"),
        QStringLiteral("kill -KILL -- -%1 2>/dev/null || kill -KILL %1 2>/dev/null || true").arg(pid)
      });
    });
  }

  void refreshStatus()
  {
    status_list_->clear();
    status_list_->addItem(QStringLiteral("Operator 管理任务: %1 个").arg(processes_.size()));
    status_list_->addItem(QStringLiteral("工作区: %1").arg(workspace_));

    auto * probe = new QProcess(this);
    probe->setProcessChannelMode(QProcess::MergedChannels);
    connect(probe, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this, probe]() {
      const auto text = QString::fromLocal8Bit(probe->readAllStandardOutput()).trimmed();
      if (!text.isEmpty()) {
        for (const auto & line : text.split(QChar('\n'), Qt::SkipEmptyParts)) {
          status_list_->addItem(line);
        }
      }
      probe->deleteLater();
    });
    // 只做轻量状态采样，避免影响正在运行的 ROS 图。
    probe->start(QStringLiteral("bash"), {
      QStringLiteral("-lc"),
      QStringLiteral(
        "printf 'Gazebo server: '; pgrep -x gzserver >/dev/null && echo ONLINE || echo OFFLINE; "
        "printf 'Gazebo client: '; pgrep -x gzclient >/dev/null && echo ONLINE || echo OFFLINE; "
        "printf 'RViz: '; pgrep -x rviz2 >/dev/null && echo ONLINE || echo OFFLINE; "
        "printf 'Nav/SLAM/Piper: '; pgrep -af '[n]av2|[s]lam_toolbox|[f]astlio|[p]iper' >/dev/null && echo ONLINE || echo OFFLINE; "
        "pgrep -af '[g]zserver|[g]zclient|[r]viz2|[n]av2|[s]lam_toolbox|[f]astlio|[p]iper' | head -8 || true")
    });
  }

  void loadMapPreview()
  {
    const QString map_path = workspace_ + QStringLiteral("/src/slam_nav_bringup/map/nav_test_map.pgm");
    QImage image(map_path);
    if (image.isNull()) {
      return;
    }
    map_label_->setPixmap(QPixmap::fromImage(image).scaled(
      760, 760, Qt::KeepAspectRatio, Qt::FastTransformation));
  }

  void appendLog(const QString & text)
  {
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    const QString line = QStringLiteral("[%1] %2").arg(stamp, text.trimmed());
    log_view_->appendPlainText(line);
    if (center_log_) {
      center_log_->appendPlainText(line);
    }
  }

  QString workspace_;
  rviz_common::VisualizationFrame * rviz_frame_{nullptr};
  QLabel * map_label_{nullptr};
  QPlainTextEdit * log_view_{nullptr};
  QPlainTextEdit * center_log_{nullptr};
  QComboBox * world_combo_{nullptr};
  QCheckBox * gui_check_{nullptr};
  QLineEdit * extra_args_{nullptr};
  QListWidget * status_list_{nullptr};
  QList<QProcess *> processes_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  QApplication app(argc, argv);

  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);

  QTimer signal_timer;
  QObject::connect(&signal_timer, &QTimer::timeout, &app, [&app]() {
    if (shutdown_requested != 0) {
      app.quit();
    }
  });
  signal_timer.start(50);

  int code = 0;
  {
    auto rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      "slam_nav_operator_rviz");
    OperatorWindow window(&app, rviz_node);
    window.show();
    code = app.exec();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return code;
}
