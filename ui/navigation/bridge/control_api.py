#!/usr/bin/env python3
"""Process-control services used by the unified SLAM Nav web UI."""

from __future__ import annotations

import os
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_DIR = PROJECT_ROOT / "log" / "ui"
OPERATOR_EXECUTABLE = Path("install/slam_nav_operator/lib/slam_nav_operator/slam_nav_operator")


FLOWS: dict[str, dict[str, Any]] = {
    "sim-static": {
        "label": "静态仿真后端",
        "command": ["sim-static", "gui:=false"],
        "group": "演示启动",
        "level": "primary",
        "summary": "启动 Gazebo server 和机器人，不弹图形窗口。",
    },
    "gazebo-client": {
        "label": "打开 Gazebo 窗口",
        "command": ["gazebo-client"],
        "group": "演示启动",
        "level": "primary",
        "summary": "仿真后端已启动时，单独打开 Gazebo 图形客户端。",
    },
    "gazebo-client-stop": {
        "label": "关闭 Gazebo 窗口",
        "command": ["gazebo-client-stop"],
        "group": "演示启动",
        "level": "utility",
        "summary": "只关闭 Gazebo 图形客户端，保留后端仿真。",
    },
    "demo-nav": {
        "label": "一键导航演示",
        "command": ["demo-nav"],
        "group": "演示启动",
        "level": "primary",
        "summary": "先启动静态仿真，再自动启动 Nav2 和 RViz。",
    },
    "operator": {
        "label": "Qt/RViz Operator",
        "command": ["operator"],
        "group": "演示启动",
        "level": "primary",
        "summary": "打开嵌入 RViz 的 Qt 专业控制界面。",
    },
    "auto-mapping": {
        "label": "自动建图",
        "command": ["auto-mapping"],
        "group": "演示启动",
        "level": "primary",
        "summary": "启动 FAST-LIO、slam_toolbox、RViz 和保守自动探索。",
    },
    "nav": {
        "label": "自主导航",
        "command": ["nav"],
        "group": "演示启动",
        "level": "primary",
        "summary": "仿真已启动时，加载默认地图并启动 Nav2 导航链路。",
    },
    "nav-full": {
        "label": "全量导航",
        "command": ["nav-full"],
        "group": "演示启动",
        "level": "primary",
        "summary": "动态障碍、RGB-D、3D 地形增强一起启动。",
    },
    "piper-viz": {
        "label": "Piper 可视化",
        "command": ["piper-viz"],
        "group": "机械臂",
        "level": "primary",
        "summary": "打开 Piper 官方 URDF、腕部相机和抓取候选 RViz。",
    },
    "piper-task-smoke": {
        "label": "抓取冒烟",
        "command": ["piper-task-smoke"],
        "group": "机械臂",
        "level": "check",
        "summary": "验证假感知、抓取候选和 pick/place action 链路。",
    },
    "task1-status": {
        "label": "交付状态",
        "command": ["task1-status"],
        "group": "检查",
        "level": "check",
        "summary": "快速查看 task1 还缺哪些证据、截图或报告材料。",
    },
    "task1-map-check": {
        "label": "地图检查",
        "command": ["task1-map-check"],
        "group": "检查",
        "level": "check",
        "summary": "检查默认地图 yaml/pgm 元数据和过期状态。",
    },
    "piper-safety-check": {
        "label": "安全检查",
        "command": ["piper-safety-check"],
        "group": "检查",
        "level": "check",
        "summary": "检查 Piper 实机前安全默认值和边界开关。",
    },
    "ui-gui-check": {
        "label": "图形自检",
        "command": ["ui-gui-check"],
        "group": "检查",
        "level": "check",
        "summary": "检查 DISPLAY、Wayland、Gazebo 和 RViz 图形入口。",
    },
    "clean-dry-run": {
        "label": "清理预览",
        "command": ["clean", "--dry-run"],
        "group": "维护",
        "level": "utility",
        "summary": "只预览将清理的 ROS/Gazebo/RViz/Nav2 残留进程。",
    },
    "clean": {
        "label": "清理残留",
        "command": ["clean"],
        "group": "维护",
        "level": "danger",
        "summary": "停止残留进程，为下一次演示整理环境。",
    },
}


class ControlError(Exception):
    """An expected API error with an HTTP status."""

    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


@dataclass
class RunState:
    run_id: str | None = None
    flow_id: str | None = None
    label: str = "空闲"
    command: list[str] = field(default_factory=list)
    started_at: float | None = None
    finished_at: float | None = None
    return_code: int | None = None
    stop_requested: bool = False
    log_path: Path | None = None
    process: subprocess.Popen[bytes] | None = None


def group_flows(flows: Mapping[str, Mapping[str, Any]] = FLOWS) -> list[dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for flow_id, flow in flows.items():
        groups.setdefault(str(flow["group"]), []).append(
            {
                "id": flow_id,
                "label": flow["label"],
                "summary": flow["summary"],
                "level": flow["level"],
                "command": " ".join(["./run.sh", *flow["command"]]),
            }
        )
    return [{"name": name, "items": items} for name, items in groups.items()]


def run_probe(
    command: str,
    *,
    root_dir: Path = PROJECT_ROOT,
    timeout: float = 1.2,
) -> subprocess.CompletedProcess[str] | None:
    """Run a read-only system probe and fail closed on errors or timeouts."""
    try:
        return subprocess.run(
            ["bash", "-lc", command],
            cwd=str(root_dir),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None


def process_online(pattern: str, *, root_dir: Path = PROJECT_ROOT, exact: bool = False) -> bool:
    flag = "-x" if exact else "-f"
    result = run_probe(
        f"pgrep {flag} {shlex.quote(pattern)} >/dev/null",
        root_dir=root_dir,
        timeout=0.6,
    )
    return result is not None and result.returncode == 0


def collect_ros_topics(root_dir: Path = PROJECT_ROOT) -> dict[str, Any]:
    command = (
        "set +u; "
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; "
        "[[ -f install/setup.bash ]] && source install/setup.bash >/dev/null 2>&1 || true; "
        "timeout 2s ros2 topic list 2>/dev/null | head -80"
    )
    result = run_probe(command, root_dir=root_dir, timeout=2.8)
    if result is None or result.returncode not in (0, 124):
        return {"ok": False, "count": 0, "items": []}
    topics = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return {"ok": bool(topics), "count": len(topics), "items": topics[:20]}


def collect_health(root_dir: Path = PROJECT_ROOT) -> dict[str, Any]:
    checks = {
        "runScript": root_dir / "run.sh",
        "defaultMap": root_dir / "src/slam_nav_bringup/map/nav_test_map.yaml",
        "piperBringup": root_dir / "src/slam_nav_piper_bringup",
        "rvizConfig": root_dir / "src/slam_nav_piper_bringup/config/piper_visualization.rviz",
    }
    return {
        key: {"ok": path.exists(), "path": str(path.relative_to(root_dir))}
        for key, path in checks.items()
    }


def collect_runtime(root_dir: Path = PROJECT_ROOT) -> dict[str, Any]:
    processes = {
        "gzserver": process_online("gzserver", root_dir=root_dir, exact=True),
        "gzclient": process_online("gzclient", root_dir=root_dir, exact=True),
        "rviz2": process_online("rviz2", root_dir=root_dir, exact=True),
        "nav2": process_online("[n]av2", root_dir=root_dir),
        "slam": process_online("[s]lam_toolbox|[f]astlio|[l]aserMapping", root_dir=root_dir),
        "piper": process_online("[p]iper", root_dir=root_dir),
        "operator": process_online("[s]lam_nav_operator", root_dir=root_dir),
    }
    display = {
        "DISPLAY": os.environ.get("DISPLAY", ""),
        "WAYLAND_DISPLAY": os.environ.get("WAYLAND_DISPLAY", ""),
    }
    display["ok"] = bool(display["DISPLAY"] or display["WAYLAND_DISPLAY"])
    return {"processes": processes, "ros": collect_ros_topics(root_dir), "display": display}


def clean_log_text(text: str) -> str:
    text = text.replace("\x00", "")
    clean_lines: list[str] = []
    for line in text.splitlines():
        compact = re.sub(r"\s+", "", line).lower()
        has_bad_chars = "�" in line or bool(re.search(r"[\x01-\x08\x0b\x0c\x0e-\x1f]", line))
        if "wsl:" in compact and "localhost" in compact:
            continue
        if "wsl" in compact and "nat" in compact and "localhost" in compact:
            continue
        if ("wsl" in compact or "localhost" in compact) and has_bad_chars:
            continue
        if line.count("�") >= 3 and ("WSL" in line or "localhost" in line):
            continue
        clean_lines.append(line)
    return "\n".join(clean_lines).strip()


def read_log_tail(path: Path | None, lines: int) -> str:
    if path is None or not path.is_file():
        return ""
    max_bytes = 256 * 1024
    with path.open("rb") as handle:
        handle.seek(0, os.SEEK_END)
        size = handle.tell()
        handle.seek(max(0, size - max_bytes), os.SEEK_SET)
        data = handle.read().decode("utf-8", errors="replace")
    return "\n".join(clean_log_text(data).splitlines()[-lines:])


def wsl_path_from_unc(path: Path) -> str | None:
    normalized = str(path).replace("/", "\\")
    prefixes = ("\\\\wsl.localhost\\", "\\\\wsl$\\")
    if not normalized.startswith(prefixes):
        return None
    parts = [part for part in normalized.split("\\") if part]
    if len(parts) < 3:
        return None
    return "/" + "/".join(parts[2:])


def build_run_command(args: list[str], root_dir: Path = PROJECT_ROOT) -> list[str]:
    if os.name == "nt":
        wsl_root = wsl_path_from_unc(root_dir)
        if wsl_root is not None:
            run_line = "./run.sh " + " ".join(shlex.quote(arg) for arg in args)
            shell_line = "cd " + shlex.quote(wsl_root)
            shell_line += " && export COLUMNS=120 LINES=40"
            shell_line += " && script -qefc " + shlex.quote(run_line) + " /dev/null"
            return ["bash", "-lc", shell_line]
    return ["bash", str(root_dir / "run.sh"), *args]


def build_process_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("LANG", "C.UTF-8")
    env.setdefault("LC_ALL", "C.UTF-8")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    return env


def stop_process(process: subprocess.Popen[bytes], timeout: float = 3.0) -> None:
    """Stop one UI-owned process group, escalating after a short grace period."""
    if process.poll() is not None:
        return
    if os.name == "nt":
        process.terminate()
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name == "nt":
        process.kill()
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        return


class ControlService:
    """Thread-safe flow registry and process lifecycle manager."""

    def __init__(
        self,
        root_dir: Path = PROJECT_ROOT,
        log_dir: Path | None = None,
        flows: Mapping[str, Mapping[str, Any]] = FLOWS,
    ) -> None:
        self.root_dir = root_dir.resolve()
        self.log_dir = (log_dir or self.root_dir / "log" / "ui").resolve()
        self.flows = flows
        self.flow_groups = group_flows(flows)
        self.lock = threading.Lock()
        self.current = RunState()
        self.active: list[RunState] = []
        self.history: list[RunState] = []

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            self._refresh_finished_locked()
            current = serialize_run(self.current, self.root_dir)
            active = [serialize_run(item, self.root_dir) for item in self.active]
            history = [serialize_run(item, self.root_dir) for item in self.history[-8:]][::-1]
            managed_operator_running = any(item.flow_id == "operator" for item in self.active)

        # The probes can take several seconds when ROS is unavailable. The HTTP
        # server invokes snapshot through asyncio.to_thread.
        runtime = collect_runtime(self.root_dir)
        operator_running = managed_operator_running or bool(runtime["processes"]["operator"])
        operator = self.operator_status(operator_running)
        return {
            "running": bool(active),
            "current": current,
            "active": active,
            "history": history,
            "flows": self.flow_groups,
            "health": collect_health(self.root_dir),
            "runtime": runtime,
            "operator": operator,
            "now": int(time.time()),
        }

    def operator_status(self, running: bool | None = None) -> dict[str, Any]:
        display_ok = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        executable = self.root_dir / OPERATOR_EXECUTABLE
        built = executable.is_file() and os.access(executable, os.X_OK)
        if running is None:
            running = process_online("[s]lam_nav_operator", root_dir=self.root_dir)
        if not display_ok:
            reason = "未检测到 DISPLAY 或 WAYLAND_DISPLAY"
        elif not built:
            reason = "slam_nav_operator 尚未构建"
        else:
            reason = None
        return {"available": display_ok and built, "running": bool(running), "reason": reason}

    def run_flow(self, flow_id: object, args: object = None) -> dict[str, Any]:
        if not isinstance(flow_id, str) or flow_id not in self.flows:
            raise ControlError(400, "unknown flow")
        if args is None:
            args = []
        if not isinstance(args, list) or not all(isinstance(item, str) for item in args):
            raise ControlError(400, "invalid args")
        if len(args) > 64 or any(len(item) > 4096 for item in args):
            raise ControlError(400, "invalid args")

        flow = self.flows[flow_id]
        with self.lock:
            self._refresh_finished_locked()
            if flow_id == "operator":
                status = self.operator_status()
                managed_running = any(item.flow_id == "operator" for item in self.active)
                if status["running"] or managed_running:
                    raise ControlError(409, "operator already running")
                if not status["available"]:
                    raise ControlError(503, str(status["reason"]))

            self.log_dir.mkdir(parents=True, exist_ok=True)
            run_id = time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6]
            log_path = self.log_dir / f"{run_id}_{flow_id}.log"
            command = build_run_command([*flow["command"], *args], self.root_dir)
            try:
                with log_path.open("wb") as log_file:
                    process = subprocess.Popen(
                        command,
                        cwd=str(self.root_dir),
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        env=build_process_env(),
                        start_new_session=(os.name != "nt"),
                    )
            except OSError as exc:
                raise ControlError(500, f"failed to start flow: {exc}") from exc

            run_state = RunState(
                run_id=run_id,
                flow_id=flow_id,
                label=str(flow["label"]),
                command=command,
                started_at=time.time(),
                log_path=log_path,
                process=process,
            )
            self.current = run_state
            self.active.append(run_state)

        threading.Thread(target=self._watch_process, args=(run_state,), daemon=True).start()
        return {"ok": True, "current": serialize_run(run_state, self.root_dir)}

    def stop(self, run_id: object = None) -> dict[str, Any]:
        if run_id is not None and (not isinstance(run_id, str) or not run_id.strip()):
            raise ControlError(400, "invalid runId")
        with self.lock:
            self._refresh_finished_locked()
            if run_id is None:
                targets = list(self.active)
            else:
                targets = [item for item in self.active if item.run_id == run_id]
                if not targets:
                    known = any(item.run_id == run_id for item in self.history)
                    if known:
                        return {"ok": True, "stopped": 0, "message": "process already stopped"}
                    raise ControlError(404, "unknown runId")
            for item in targets:
                item.stop_requested = True

        for item in targets:
            if item.process is not None:
                stop_process(item.process)
        return {
            "ok": True,
            "stopped": len(targets),
            **({"message": "no running process"} if not targets else {}),
        }

    def read_log(self, run_id: str | None, lines: int) -> dict[str, Any]:
        if not isinstance(lines, int) or isinstance(lines, bool) or not 1 <= lines <= 2000:
            raise ControlError(400, "invalid lines")
        if run_id is not None and (not isinstance(run_id, str) or not run_id.strip()):
            raise ControlError(400, "invalid runId")
        with self.lock:
            self._refresh_finished_locked()
            if run_id is None:
                target = self.current
            else:
                target = next(
                    (item for item in [*self.active, *reversed(self.history)] if item.run_id == run_id),
                    None,
                )
                if target is None:
                    raise ControlError(404, "unknown runId")
            path = target.log_path
            selected_run_id = target.run_id
        return {"runId": selected_run_id, "text": read_log_tail(path, lines)}

    def _watch_process(self, run_state: RunState) -> None:
        process = run_state.process
        if process is None:
            return
        return_code = process.wait()
        with self.lock:
            self._finish_locked(run_state, return_code)

    def _refresh_finished_locked(self) -> None:
        for item in list(self.active):
            if item.process is None:
                continue
            return_code = item.process.poll()
            if return_code is not None:
                self._finish_locked(item, return_code)

    def _finish_locked(self, run_state: RunState, return_code: int) -> None:
        if run_state.finished_at is not None:
            return
        run_state.return_code = return_code
        run_state.finished_at = time.time()
        run_state.process = None
        self.active = [item for item in self.active if item.run_id != run_state.run_id]
        self.history.append(run_state)
        if self.current.run_id == run_state.run_id:
            self.current = self.active[-1] if self.active else RunState()


def serialize_run(state: RunState, root_dir: Path = PROJECT_ROOT) -> dict[str, Any]:
    log_path: str | None = None
    if state.log_path is not None:
        try:
            log_path = str(state.log_path.relative_to(root_dir))
        except ValueError:
            log_path = str(state.log_path)
    return {
        "runId": state.run_id,
        "flowId": state.flow_id,
        "label": state.label,
        "command": state.command,
        "startedAt": state.started_at,
        "finishedAt": state.finished_at,
        "returnCode": state.return_code,
        "stopped": state.stop_requested,
        "logPath": log_path,
    }
