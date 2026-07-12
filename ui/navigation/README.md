# SLAM Nav Web 主界面

该界面借鉴并适配了 GUI_go2 的导航作业台交互，项目命名、运行流程、ROS 话题和 frame 均以 SLAM Nav 为准。主界面以 2×2 视觉矩阵同时显示导航 RGB、导航深度、Piper 腕部 RGB 和 Piper 腕部深度，并提供截图、导航 RGB 录像、二维地图、代价地图、规划路径、机器人位姿、航点队列和任务控制。

单航点通过 `NavigateToPose` 下发，多航点通过 `NavigateThroughPoses` 下发。录像优先导出 WebM，浏览器只支持 MP4 时自动回退为 MP4。

“开始导航”受 `/navigation_ready` (`std_msgs/msg/Bool`) 门控。界面收到 `true` 时显示“可以开始”，收到 `false` 或尚未收到消息时禁止下发新的导航目标；取消正在执行的导航不受影响。

## 运行

先在一个终端启动仿真与 Nav2：

```bash
./run.sh demo-nav
```

再在另一个终端启动 Web 主界面：

```bash
./run.sh ui
```

默认地址为 `http://127.0.0.1:8765`。`nav-ui`、`dashboard`、`mission-control`、`navigation-ui` 和 `nav-console` 是同一入口的兼容别名。桥接器直接使用 NumPy/OpenCV 解析常见 ROS 图像编码，不依赖 `cv_bridge`。关闭导航相机时仍可使用 Piper 视野、地图、导航与任务控制：

```bash
./run.sh ui --no-camera
```

视觉矩阵默认订阅以下四个项目话题：

```text
/nav_camera/color/image_raw
/nav_camera/depth/image_raw
/piper/arm_camera/color/image_raw
/piper/arm_camera/depth/image_raw
```

深度桥支持 `16UC1` 和 `32FC1`。导航深度默认显示 `0.25–5.0 m`，Piper 深度默认显示 `0.15–2.5 m`。暂时不接机械臂时无需额外配置；Piper 两个视图会保持等待状态。若希望完全关闭 Piper 图像订阅，可执行 `./run.sh ui --no-piper-camera`。

局域网访问需要显式开放监听地址：

```bash
./run.sh ui --host 0.0.0.0
```

该服务可以向 Nav2 下发和取消目标。只应在可信网络中开放，避免将控制端口直接暴露到公网。

主界面点击“专业界面”会在运行工作区的主机桌面启动 Qt/RViz Operator。远程浏览器只发出启动请求，不会把原生窗口传输到远程电脑。也可以继续使用 `./run.sh operator` 直接打开。

旧地址 `/console.html` 由 bridge 重定向到统一主界面的任务控制面板。

## 前端开发

```bash
cd ui/navigation
npm ci
npm test
npm run lint
npm run build
```

前端要求在 WSL/Ubuntu 中安装 Vite 支持的 Linux Node.js（`^20.19.0` 或 `>=22.12.0`）和 Linux npm。启动脚本默认使用仓库内已有的 `dist`；仅当缺少构建产物或指定 `--rebuild` 时才调用 Node.js，并会拒绝误用 Windows npm。

已有 Chromium/Edge 时可运行桌面与移动端横向溢出检查：

```bash
PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH=/path/to/chromium npm run visual:smoke
```

后端单元测试：

```bash
cd ui/navigation/bridge
python3 -m unittest discover -p 'test_*.py'
```

所有话题、frame、监听地址和端口均可通过 `.env.example` 中的 `SLAM_NAV_*` 环境变量覆盖。新的 `SLAM_NAV_UI_*` 名称优先，旧 `SLAM_NAV_NAV_UI_*` 名称仅作为兼容回退。
