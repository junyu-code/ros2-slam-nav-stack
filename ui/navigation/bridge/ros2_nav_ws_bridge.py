#!/usr/bin/env python3
"""将 ROS2 图像与 Nav2 导航数据桥接到浏览器 WebSocket。"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import mimetypes
import os
import signal
import sys
import threading
import time
from math import atan2, cos, isfinite, sin
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping
from urllib.parse import ParseResult, parse_qs, unquote, urlparse

from control_api import ControlError, ControlService

WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
DEFAULT_TOPIC = "/nav_camera/color/image_raw"
DEFAULT_NAV_DEPTH_TOPIC = "/nav_camera/depth/image_raw"
DEFAULT_PIPER_COLOR_TOPIC = "/piper/arm_camera/color/image_raw"
DEFAULT_PIPER_DEPTH_TOPIC = "/piper/arm_camera/depth/image_raw"
DEFAULT_NAVIGATION_READY_TOPIC = "/navigation_ready"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_NAV_WS_PATH = "/ws/nav"
DEFAULT_NAV_DEPTH_WS_PATH = "/ws/nav/depth"
DEFAULT_PIPER_RGB_WS_PATH = "/ws/piper/rgb"
DEFAULT_PIPER_DEPTH_WS_PATH = "/ws/piper/depth"
DEFAULT_MAP_TOPIC = "/map"
DEFAULT_GLOBAL_COSTMAP_TOPIC = "/global_costmap/costmap"
DEFAULT_LOCAL_COSTMAP_TOPIC = "/local_costmap/costmap"
DEFAULT_GLOBAL_PLAN_TOPIC = "/plan"
DEFAULT_LOCAL_PLAN_TOPIC = "/local_plan"
DEFAULT_LETHAL_COST_THRESHOLD = 253
DEFAULT_MAP_FRAME = "map"
DEFAULT_ROBOT_FRAMES = "base_footprint,base_link"
MAX_HTTP_BODY_BYTES = 64 * 1024
MAX_REASONABLE_CAMERA_LATENCY_MS = 60_000.0


def build_frame_payload(
    *,
    image_base64: str,
    width: int,
    height: int,
    timestamp: float | None,
    fps: float | None,
    latency_ms: float | None,
    topic: str,
) -> str:
    return json.dumps(
        {
            "image": f"data:image/jpeg;base64,{image_base64}",
            "timestamp": timestamp,
            "width": width,
            "height": height,
            "fps": fps,
            "latencyMs": latency_ms,
            "topic": topic,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )


def build_nav_status_payload(
    state: str,
    *,
    detail: str | None = None,
    action: str | None = None,
) -> str:
    payload: dict[str, object] = {"type": "nav_status", "state": state}
    if detail is not None:
        payload["detail"] = detail
    if action is not None:
        payload["action"] = action
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def build_navigation_ready_payload(ready: bool, topic: str) -> str:
    return json.dumps(
        {
            "type": "navigation_ready",
            "ready": bool(ready),
            "topic": topic,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )


def calculate_frame_latency_ms(now: float, timestamp: float | None) -> float | None:
    if timestamp is None:
        return None
    latency_ms = (now - timestamp) * 1000.0
    if not isfinite(latency_ms) or latency_ms < 0.0 or latency_ms > MAX_REASONABLE_CAMERA_LATENCY_MS:
        return None
    return latency_ms


def image_message_to_bgr(message: object, numpy_module: object, cv2_module: object) -> object:
    width = int(getattr(message, "width", 0))
    height = int(getattr(message, "height", 0))
    encoding = str(getattr(message, "encoding", "")).strip().lower()
    if width <= 0 or height <= 0:
        raise ValueError("图像尺寸无效")

    channel_layouts = {
        "rgb8": (3, getattr(cv2_module, "COLOR_RGB2BGR")),
        "bgr8": (3, None),
        "8uc3": (3, None),
        "rgba8": (4, getattr(cv2_module, "COLOR_RGBA2BGR")),
        "bgra8": (4, getattr(cv2_module, "COLOR_BGRA2BGR")),
        "mono8": (1, getattr(cv2_module, "COLOR_GRAY2BGR")),
        "8uc1": (1, getattr(cv2_module, "COLOR_GRAY2BGR")),
        "yuyv": (2, getattr(cv2_module, "COLOR_YUV2BGR_YUY2")),
        "yuyv422": (2, getattr(cv2_module, "COLOR_YUV2BGR_YUY2")),
        "yuv422_yuy2": (2, getattr(cv2_module, "COLOR_YUV2BGR_YUY2")),
        "uyvy": (2, getattr(cv2_module, "COLOR_YUV2BGR_UYVY")),
        "yuv422": (2, getattr(cv2_module, "COLOR_YUV2BGR_UYVY")),
    }
    if encoding not in channel_layouts:
        raise ValueError(f"暂不支持图像编码：{encoding or '<empty>'}")

    channels, color_conversion = channel_layouts[encoding]
    packed_width = width * channels
    step = int(getattr(message, "step", 0)) or packed_width
    if step < packed_width:
        raise ValueError(f"图像步长小于有效行宽：step={step}, row={packed_width}")

    raw = numpy_module.frombuffer(getattr(message, "data", b""), dtype=numpy_module.uint8)
    required_size = height * step
    if raw.size < required_size:
        raise ValueError(f"图像数据不足：需要 {required_size} 字节，实际 {raw.size} 字节")

    rows = raw[:required_size].reshape(height, step)
    pixels = rows[:, :packed_width]
    if channels == 1:
        image = pixels.reshape(height, width)
    else:
        image = pixels.reshape(height, width, channels)
    image = numpy_module.ascontiguousarray(image)
    if color_conversion is None:
        return image
    return cv2_module.cvtColor(image, color_conversion)


def depth_message_to_bgr(
    message: object,
    numpy_module: object,
    cv2_module: object,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> object:
    width = int(getattr(message, "width", 0))
    height = int(getattr(message, "height", 0))
    encoding = str(getattr(message, "encoding", "")).strip().lower()
    if width <= 0 or height <= 0:
        raise ValueError("深度图尺寸无效")
    if not 0.0 <= min_depth_m < max_depth_m:
        raise ValueError("深度显示范围无效")

    if encoding in {"16uc1", "mono16"}:
        bytes_per_pixel = 2
        dtype = numpy_module.dtype(">u2" if bool(getattr(message, "is_bigendian", 0)) else "<u2")
        scale = 0.001
    elif encoding == "32fc1":
        bytes_per_pixel = 4
        dtype = numpy_module.dtype(">f4" if bool(getattr(message, "is_bigendian", 0)) else "<f4")
        scale = 1.0
    else:
        raise ValueError(f"暂不支持深度图编码：{encoding or '<empty>'}")

    packed_width = width * bytes_per_pixel
    step = int(getattr(message, "step", 0)) or packed_width
    if step < packed_width:
        raise ValueError(f"深度图步长小于有效行宽：step={step}, row={packed_width}")

    raw = numpy_module.frombuffer(getattr(message, "data", b""), dtype=numpy_module.uint8)
    required_size = height * step
    if raw.size < required_size:
        raise ValueError(f"深度图数据不足：需要 {required_size} 字节，实际 {raw.size} 字节")

    packed = raw[:required_size].reshape(height, step)[:, :packed_width].copy()
    depth_m = packed.reshape(-1).view(dtype).reshape(height, width).astype(numpy_module.float32)
    depth_m *= scale
    valid = numpy_module.isfinite(depth_m) & (depth_m > 0.0)
    clipped = numpy_module.clip(depth_m, min_depth_m, max_depth_m)
    normalized = (max_depth_m - clipped) / (max_depth_m - min_depth_m)
    intensity = numpy_module.clip(normalized * 255.0, 0.0, 255.0).astype(numpy_module.uint8)
    intensity[~valid] = 0
    colorized = cv2_module.applyColorMap(intensity, cv2_module.COLORMAP_TURBO)
    colorized[~valid] = 0
    return colorized


def filter_costmap_obstacles(
    data: Iterable[int],
    *,
    width: int,
    lethal_threshold: int = DEFAULT_LETHAL_COST_THRESHOLD,
) -> list[dict[str, int]]:
    cells: list[dict[str, int]] = []
    if width <= 0:
        return cells

    for index, value in enumerate(data):
        numeric_value = int(value)
        if numeric_value >= lethal_threshold:
            cells.append({"x": index % width, "y": index // width, "value": numeric_value})
    return cells


def select_navigation_action(waypoints: list[dict[str, float]]) -> str:
    return "NavigateThroughPoses" if len(waypoints) > 1 else "NavigateToPose"


def quaternion_to_yaw(orientation: object) -> float:
    x = float(getattr(orientation, "x", 0.0))
    y = float(getattr(orientation, "y", 0.0))
    z = float(getattr(orientation, "z", 0.0))
    w = float(getattr(orientation, "w", 1.0))
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": sin(yaw / 2.0), "w": cos(yaw / 2.0)}


def pose_origin_payload(pose: object) -> dict[str, float]:
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return {
        "x": float(getattr(position, "x", 0.0)),
        "y": float(getattr(position, "y", 0.0)),
        "yaw": quaternion_to_yaw(orientation),
    }


def message_frame_id(message: object, fallback: str = "map") -> str:
    frame_id = getattr(getattr(message, "header", None), "frame_id", "")
    return str(frame_id).strip() or fallback


def parse_frame_list(value: str) -> list[str]:
    frames = [frame.strip() for frame in value.split(",") if frame.strip()]
    return frames or ["base_link"]


def rotate_point(x: float, y: float, yaw: float) -> tuple[float, float]:
    yaw_cos = cos(yaw)
    yaw_sin = sin(yaw)
    return x * yaw_cos - y * yaw_sin, x * yaw_sin + y * yaw_cos


def transform_xy_to_map(x: float, y: float, transform: object | None) -> dict[str, float]:
    if transform is None:
        return {"x": x, "y": y}
    translation = getattr(transform, "translation", None)
    rotation = getattr(transform, "rotation", None)
    rotated_x, rotated_y = rotate_point(x, y, quaternion_to_yaw(rotation))
    return {
        "x": rotated_x + float(getattr(translation, "x", 0.0)),
        "y": rotated_y + float(getattr(translation, "y", 0.0)),
    }


def costmap_cell_center_to_world(
    cell: dict[str, int],
    origin: dict[str, float],
    resolution: float,
) -> tuple[float, float]:
    local_x = (float(cell["x"]) + 0.5) * resolution
    local_y = (float(cell["y"]) + 0.5) * resolution
    rotated_x, rotated_y = rotate_point(local_x, local_y, origin["yaw"])
    return origin["x"] + rotated_x, origin["y"] + rotated_y


def costmap_cells_to_points(
    cells: Iterable[dict[str, int]],
    *,
    origin: dict[str, float],
    resolution: float,
    transform: object | None = None,
) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for cell in cells:
        x, y = costmap_cell_center_to_world(cell, origin, resolution)
        point = transform_xy_to_map(x, y, transform)
        point["value"] = float(cell["value"])
        points.append(point)
    return points


def transform_to_pose_payload(
    transform: object,
    *,
    frame: str = DEFAULT_MAP_FRAME,
    source_frame: str | None = None,
) -> dict[str, float | str]:
    translation = getattr(transform, "translation", None)
    rotation = getattr(transform, "rotation", None)
    payload: dict[str, float | str] = {
        "type": "robot_pose",
        "x": float(getattr(translation, "x", 0.0)),
        "y": float(getattr(translation, "y", 0.0)),
        "yaw": quaternion_to_yaw(rotation),
        "frame": frame,
    }
    if source_frame:
        payload["sourceFrame"] = source_frame
    return payload


def encode_websocket_text_frame(message: str) -> bytes:
    payload = message.encode("utf-8")
    length = len(payload)
    if length < 126:
        header = bytes([0x81, length])
    elif length <= 0xFFFF:
        header = bytes([0x81, 126]) + length.to_bytes(2, "big")
    else:
        header = bytes([0x81, 127]) + length.to_bytes(8, "big")
    return header + payload


def encode_masked_client_text_frame(message: str, mask: bytes = b"\x11\x22\x33\x44") -> bytes:
    payload = message.encode("utf-8")
    length = len(payload)
    if len(mask) != 4:
        raise ValueError("mask must be 4 bytes")
    if length >= 126:
        raise ValueError("test helper only supports short frames")
    masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return bytes([0x81, 0x80 | length]) + mask + masked


def decode_websocket_text_messages(buffer: bytes) -> tuple[list[str], bytes]:
    messages: list[str] = []
    offset = 0

    while len(buffer) - offset >= 2:
        first = buffer[offset]
        second = buffer[offset + 1]
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        header_length = 2

        if length == 126:
            if len(buffer) - offset < 4:
                break
            length = int.from_bytes(buffer[offset + 2 : offset + 4], "big")
            header_length = 4
        elif length == 127:
            if len(buffer) - offset < 10:
                break
            length = int.from_bytes(buffer[offset + 2 : offset + 10], "big")
            header_length = 10

        mask_start = offset + header_length
        payload_start = mask_start + (4 if masked else 0)
        payload_end = payload_start + length
        if len(buffer) < payload_end:
            break

        payload = buffer[payload_start:payload_end]
        if masked:
            mask = buffer[mask_start:payload_start]
            payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))

        if opcode == 0x1:
            messages.append(payload.decode("utf-8"))
        elif opcode == 0x8:
            offset = payload_end
            break

        offset = payload_end

    return messages, buffer[offset:]


class WebBridgeServer:
    def __init__(
        self,
        host: str,
        port: int,
        static_dir: Path,
        ws_path: str,
        nav_ws_path: str = DEFAULT_NAV_WS_PATH,
        nav_command_handler: Callable[[str], None] | None = None,
        control_service: ControlService | None = None,
        additional_ws_paths: Iterable[str] = (),
    ) -> None:
        self.host = host
        self.port = port
        self.static_dir = static_dir.resolve()
        self.ws_path = ws_path
        self.nav_ws_path = nav_ws_path
        self.nav_command_handler = nav_command_handler
        self.control_service = control_service or ControlService()
        channels = dict.fromkeys([self.ws_path, self.nav_ws_path, *additional_ws_paths])
        self.clients: dict[str, set[asyncio.StreamWriter]] = {
            channel: set() for channel in channels
        }
        self.clients_lock = asyncio.Lock()
        self.locks: dict[str, asyncio.Lock] = {
            channel: asyncio.Lock() for channel in channels
        }
        self.retained_payloads: dict[str, dict[str, str]] = {
            channel: {} for channel in channels
        }
        self.connection_tasks: set[asyncio.Task[None]] = set()
        self.server: asyncio.AbstractServer | None = None

    async def serve_forever(self) -> None:
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        addresses = ", ".join(str(sock.getsockname()) for sock in self.server.sockets or [])
        print(f"[nav-ui-bridge] 正在监听 HTTP/WebSocket：{addresses}", flush=True)
        try:
            async with self.server:
                await self.server.serve_forever()
        finally:
            await self.close_connections()

    def has_clients(self, ws_path: str) -> bool:
        return bool(self.clients.get(ws_path))

    async def close_connections(self) -> None:
        async with self.clients_lock:
            writers = [writer for clients in self.clients.values() for writer in clients]
            for clients in self.clients.values():
                clients.clear()
        for writer in writers:
            close_writer(writer)

        current = asyncio.current_task()
        tasks = [task for task in self.connection_tasks if task is not current]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def broadcast(
        self,
        payload: str,
        ws_path: str | None = None,
        *,
        retain_key: str | None = None,
    ) -> None:
        channel = ws_path or self.ws_path
        if retain_key is not None:
            self.retained_payloads.setdefault(channel, {})[retain_key] = payload
        async with self.clients_lock:
            writers = list(self.clients.setdefault(channel, set()))
            if not writers:
                return

        frame = encode_websocket_text_frame(payload)
        stale: list[asyncio.StreamWriter] = []
        lock = self.locks.setdefault(channel, asyncio.Lock())
        async with lock:
            for writer in writers:
                try:
                    writer.write(frame)
                    await writer.drain()
                except (ConnectionError, RuntimeError, OSError):
                    stale.append(writer)
        if stale:
            async with self.clients_lock:
                clients = self.clients.setdefault(channel, set())
                for writer in stale:
                    clients.discard(writer)
                    close_writer(writer)

    async def handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        task = asyncio.current_task()
        if task is not None:
            self.connection_tasks.add(task)
        try:
            await self._handle_client(reader, writer)
        finally:
            close_writer(writer)
            if task is not None:
                self.connection_tasks.discard(task)

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            raw = await reader.readuntil(b"\r\n\r\n")
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError):
            return

        request = parse_http_request(raw)
        if request is None:
            await send_response(writer, 400, b"Bad Request", "text/plain")
            return

        method, target, headers = request
        parsed_target = urlparse(target)
        path = parsed_target.path
        if is_websocket_request(path, headers) and path in self.clients:
            await self.accept_websocket(reader, writer, headers, path)
            return

        if path == "/console.html" and method == "GET":
            await send_response(
                writer,
                302,
                b"Redirecting to the task panel",
                "text/plain",
                {"Location": "/?panel=tasks"},
            )
            return

        if path.startswith("/api/"):
            await self.handle_api(method, parsed_target, headers, reader, writer)
            return

        if method != "GET":
            await send_response(writer, 405, b"Method Not Allowed", "text/plain")
            return

        await self.serve_static(path, writer)

    async def handle_api(
        self,
        method: str,
        target: ParseResult,
        headers: dict[str, str],
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        path = target.path
        try:
            if method == "GET" and path == "/api/state":
                payload = await asyncio.to_thread(self.control_service.snapshot)
                await send_json_response(writer, 200, payload)
                return

            if method == "GET" and path == "/api/log":
                params = parse_qs(target.query, keep_blank_values=True)
                run_id = params.get("runId", [None])[-1]
                raw_lines = params.get("lines", ["220"])[-1]
                try:
                    lines = int(raw_lines)
                except (TypeError, ValueError) as exc:
                    raise ControlError(400, "invalid lines") from exc
                payload = await asyncio.to_thread(self.control_service.read_log, run_id, lines)
                await send_json_response(writer, 200, payload)
                return

            if method == "POST" and path == "/api/run":
                body = await read_json_body(reader, headers)
                payload = await asyncio.to_thread(
                    self.control_service.run_flow,
                    body.get("flowId"),
                    body.get("args", []),
                )
                await send_json_response(writer, 202, payload)
                return

            if method == "POST" and path == "/api/stop":
                body = await read_json_body(reader, headers)
                payload = await asyncio.to_thread(self.control_service.stop, body.get("runId"))
                await send_json_response(writer, 200, payload)
                return

            known_path = path in {"/api/state", "/api/log", "/api/run", "/api/stop"}
            if known_path:
                await send_json_response(writer, 405, {"error": "method not allowed"})
            else:
                await send_json_response(writer, 404, {"error": "unknown endpoint"})
        except ControlError as exc:
            await send_json_response(writer, exc.status, {"error": exc.message})
        except (asyncio.IncompleteReadError, UnicodeDecodeError, json.JSONDecodeError, ValueError):
            await send_json_response(writer, 400, {"error": "invalid JSON body"})
        except Exception as exc:  # noqa: BLE001 - one request must not stop the bridge.
            print(f"[nav-ui-bridge] 控制 API 请求失败：{exc}", file=sys.stderr, flush=True)
            await send_json_response(writer, 500, {"error": "internal server error"})

    async def accept_websocket(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        headers: dict[str, str],
        path: str,
    ) -> None:
        key = headers.get("sec-websocket-key")
        if not key:
            await send_response(writer, 400, b"Missing Sec-WebSocket-Key", "text/plain")
            return

        accept = base64.b64encode(
            hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
        ).decode("ascii")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        )
        writer.write(response.encode("ascii"))
        await writer.drain()

        async with self.clients_lock:
            self.clients.setdefault(path, set()).add(writer)
            retained_payloads = list(self.retained_payloads.setdefault(path, {}).values())
        if retained_payloads:
            lock = self.locks.setdefault(path, asyncio.Lock())
            async with lock:
                for payload in retained_payloads:
                    writer.write(encode_websocket_text_frame(payload))
                await writer.drain()
        print(
            f"[nav-ui-bridge] WebSocket 客户端已连接 {path} ({len(self.clients[path])})",
            flush=True,
        )

        pending = b""
        try:
            while not reader.at_eof():
                data = await reader.read(1024)
                if not data:
                    break
                if path == self.nav_ws_path and self.nav_command_handler is not None:
                    pending += data
                    messages, pending = decode_websocket_text_messages(pending)
                    for message in messages:
                        self.nav_command_handler(message)
        finally:
            async with self.clients_lock:
                self.clients.setdefault(path, set()).discard(writer)
            close_writer(writer)
            print(
                f"[nav-ui-bridge] WebSocket 客户端已断开 {path} ({len(self.clients[path])})",
                flush=True,
            )

    async def serve_static(self, path: str, writer: asyncio.StreamWriter) -> None:
        if path in ("", "/"):
            requested = self.static_dir / "index.html"
        else:
            requested = (self.static_dir / unquote(path).lstrip("/")).resolve()

        if not is_relative_to(requested, self.static_dir):
            await send_response(writer, 403, b"Forbidden", "text/plain")
            return

        if not requested.is_file():
            requested = self.static_dir / "index.html"

        if not requested.is_file():
            await send_response(writer, 404, b"Frontend dist/index.html not found", "text/plain")
            return

        content = requested.read_bytes()
        mime_type = mimetypes.guess_type(str(requested))[0] or "application/octet-stream"
        await send_response(writer, 200, content, mime_type)


class RosImagePublisher:
    def __init__(
        self,
        topic: str,
        quality: int,
        max_fps: float,
        server: WebBridgeServer,
        ws_path: str | None = None,
    ) -> None:
        self.topic = topic
        self.quality = quality
        self.min_interval = 1.0 / max_fps if max_fps > 0 else 0.0
        self.server = server
        self.ws_path = ws_path or server.ws_path
        self.last_sent_at = 0.0
        self.last_frame_at: float | None = None

    def start(self, loop: asyncio.AbstractEventLoop, node: object) -> None:
        import cv2
        import numpy as np
        import rclpy.callback_groups
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self.cv2 = cv2
        self.np = np
        self.loop = loop

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        node.create_subscription(
            Image,
            self.topic,
            self.on_image,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        print(f"[nav-ui-bridge] 已订阅 {self.topic} -> {self.ws_path}", flush=True)

    def convert_image(self, msg: object) -> object:
        return image_message_to_bgr(msg, self.np, self.cv2)

    def on_image(self, msg: object) -> None:
        if not self.server.has_clients(self.ws_path):
            return
        now = time.time()
        if self.min_interval and now - self.last_sent_at < self.min_interval:
            return
        self.last_sent_at = now

        try:
            cv_image = self.convert_image(msg)
            ok, jpeg = self.cv2.imencode(
                ".jpg",
                cv_image,
                [int(self.cv2.IMWRITE_JPEG_QUALITY), self.quality],
            )
            if not ok:
                print("[nav-ui-bridge] JPEG 编码失败", file=sys.stderr, flush=True)
                return

            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            timestamp = None
            if stamp is not None and (stamp.sec or stamp.nanosec):
                timestamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000
            latency_ms = calculate_frame_latency_ms(now, timestamp)
            fps = None
            if self.last_frame_at is not None and now > self.last_frame_at:
                fps = 1.0 / (now - self.last_frame_at)
            self.last_frame_at = now

            payload = build_frame_payload(
                image_base64=base64.b64encode(jpeg.tobytes()).decode("ascii"),
                width=int(getattr(msg, "width", cv_image.shape[1])),
                height=int(getattr(msg, "height", cv_image.shape[0])),
                timestamp=timestamp,
                fps=fps,
                latency_ms=latency_ms,
                topic=self.topic,
            )
            asyncio.run_coroutine_threadsafe(
                self.server.broadcast(payload, self.ws_path),
                self.loop,
            )
        except Exception as exc:  # noqa: BLE001 - 保持 ROS 回调持续运行。
            print(
                f"[nav-ui-bridge] 图像帧转换失败 ({self.topic})：{exc}",
                file=sys.stderr,
                flush=True,
            )


class RosDepthPublisher(RosImagePublisher):
    def __init__(
        self,
        topic: str,
        quality: int,
        max_fps: float,
        server: WebBridgeServer,
        ws_path: str,
        min_depth_m: float,
        max_depth_m: float,
    ) -> None:
        super().__init__(topic, quality, max_fps, server, ws_path)
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m

    def convert_image(self, msg: object) -> object:
        return depth_message_to_bgr(
            msg,
            self.np,
            self.cv2,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
        )


class RosNavigationPublisher:
    def __init__(
        self,
        *,
        server: WebBridgeServer,
        loop: asyncio.AbstractEventLoop,
        nav_ws_path: str,
        map_topic: str,
        global_costmap_topic: str,
        local_costmap_topic: str,
        global_plan_topic: str,
        local_plan_topic: str,
        navigation_ready_topic: str,
        lethal_threshold: int,
        map_frame: str,
        robot_frames: list[str],
    ) -> None:
        self.server = server
        self.loop = loop
        self.nav_ws_path = nav_ws_path
        self.map_topic = map_topic
        self.global_costmap_topic = global_costmap_topic
        self.local_costmap_topic = local_costmap_topic
        self.global_plan_topic = global_plan_topic
        self.local_plan_topic = local_plan_topic
        self.navigation_ready_topic = navigation_ready_topic
        self.lethal_threshold = lethal_threshold
        self.map_frame = map_frame
        self.robot_frames = robot_frames
        self.current_goal_handle: object | None = None
        self.navigation_ready: bool | None = None
        self.last_pose_warning_at = 0.0

    def start(self, node: object) -> None:
        import rclpy.callback_groups
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Path as NavPath
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformException, TransformListener

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.PoseStamped = PoseStamped
        self.NavigateToPose = NavigateToPose
        self.NavigateThroughPoses = NavigateThroughPoses
        self.TransformException = TransformException
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.navigate_to_pose_client = self._create_action_client(NavigateToPose, "navigate_to_pose")
        self.navigate_through_poses_client = self._create_action_client(
            NavigateThroughPoses,
            "navigate_through_poses",
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        live_qos = QoSProfile(depth=1)

        node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.on_map,
            map_qos,
            callback_group=self.callback_group,
        )
        node.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            lambda msg: self.on_costmap(msg, "global", self.global_costmap_topic),
            live_qos,
            callback_group=self.callback_group,
        )
        node.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            lambda msg: self.on_costmap(msg, "local", self.local_costmap_topic),
            live_qos,
            callback_group=self.callback_group,
        )
        node.create_subscription(
            NavPath,
            self.global_plan_topic,
            lambda msg: self.on_path(msg, "global", self.global_plan_topic),
            live_qos,
            callback_group=self.callback_group,
        )
        node.create_subscription(
            NavPath,
            self.local_plan_topic,
            lambda msg: self.on_path(msg, "local", self.local_plan_topic),
            live_qos,
            callback_group=self.callback_group,
        )
        node.create_subscription(
            Bool,
            self.navigation_ready_topic,
            self.on_navigation_ready,
            map_qos,
            callback_group=self.callback_group,
        )
        node.create_timer(0.1, self.publish_robot_pose, callback_group=self.callback_group)
        self.publish_status(
            "ready",
            detail=f"导航桥已连接 ROS2，机器人 TF: {','.join(self.robot_frames)} -> {self.map_frame}",
        )
        print("[nav-ui-bridge] 导航 WebSocket 桥已启动", flush=True)

    def handle_command(self, message: str) -> None:
        try:
            payload = json.loads(message)
        except json.JSONDecodeError as exc:
            self.publish_status("error", detail=f"导航命令不是 JSON: {exc}")
            return

        command_type = payload.get("type")
        if command_type == "ping":
            self.publish_status("ready", detail="pong")
            return
        if command_type == "cancel_navigation":
            self.cancel_navigation()
            return
        if command_type != "navigate":
            self.publish_status("error", detail=f"不支持的导航命令: {command_type}")
            return

        if self.navigation_ready is not True:
            detail = (
                f"导航尚未就绪，等待 {self.navigation_ready_topic}=true"
                if self.navigation_ready is False
                else f"尚未收到 {self.navigation_ready_topic}"
            )
            self.publish_status("waiting", detail=detail)
            return

        waypoints = payload.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            self.publish_status("error", detail="导航命令缺少航点")
            return
        normalized = [self._normalize_waypoint(waypoint) for waypoint in waypoints]
        if any(waypoint is None for waypoint in normalized):
            self.publish_status("error", detail="航点格式错误")
            return

        self.send_navigation_goal([waypoint for waypoint in normalized if waypoint is not None])

    def on_navigation_ready(self, msg: object) -> None:
        self.navigation_ready = bool(getattr(msg, "data", False))
        payload = build_navigation_ready_payload(
            self.navigation_ready,
            self.navigation_ready_topic,
        )
        asyncio.run_coroutine_threadsafe(
            self.server.broadcast(
                payload,
                self.nav_ws_path,
                retain_key="navigation_ready",
            ),
            self.loop,
        )

    def on_map(self, msg: object) -> None:
        info = msg.info
        frame = message_frame_id(msg)
        payload = {
            "type": "map",
            "width": int(info.width),
            "height": int(info.height),
            "resolution": float(info.resolution),
            "origin": pose_origin_payload(info.origin),
            "data": list(msg.data),
            "frame": frame,
            "topic": self.map_topic,
        }
        self.broadcast(payload)

    def on_costmap(self, msg: object, scope: str, topic: str) -> None:
        info = msg.info
        source_frame = message_frame_id(msg)
        origin = pose_origin_payload(info.origin)
        cells = filter_costmap_obstacles(
            msg.data,
            width=int(info.width),
            lethal_threshold=self.lethal_threshold,
        )
        transform = self._lookup_map_transform(source_frame)
        frame = self.map_frame if transform is not False else source_frame
        payload = {
            "type": "costmap",
            "scope": scope,
            "width": int(info.width),
            "height": int(info.height),
            "resolution": float(info.resolution),
            "origin": origin,
            "cells": cells,
            "points": costmap_cells_to_points(
                cells,
                origin=origin,
                resolution=float(info.resolution),
                transform=None if transform is False else transform,
            )
            if transform is not False
            else [],
            "frame": frame,
            "sourceFrame": source_frame,
            "topic": topic,
        }
        self.broadcast(payload)

    def on_path(self, msg: object, scope: str, topic: str) -> None:
        points, frame = self._path_points_in_map(msg)
        payload = {
            "type": "path",
            "scope": scope,
            "points": points,
            "frame": frame,
            "sourceFrame": message_frame_id(msg),
            "topic": topic,
        }
        self.broadcast(payload)

    def publish_robot_pose(self) -> None:
        errors: list[str] = []
        for robot_frame in self.robot_frames:
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame, robot_frame, self._time_zero())
            except self.TransformException as exc:
                errors.append(f"{robot_frame}: {exc}")
                continue
            self.broadcast(
                transform_to_pose_payload(
                    transform.transform,
                    frame=self.map_frame,
                    source_frame=robot_frame,
                ),
            )
            return

        now = time.time()
        if now - self.last_pose_warning_at > 5.0:
            self.last_pose_warning_at = now
            print(
                f"[nav-ui-bridge] 机器人位姿 TF 不可用 ({' | '.join(errors)})",
                file=sys.stderr,
                flush=True,
            )

    def send_navigation_goal(self, waypoints: list[dict[str, float]]) -> None:
        action = select_navigation_action(waypoints)
        client = (
            self.navigate_through_poses_client
            if action == "NavigateThroughPoses"
            else self.navigate_to_pose_client
        )
        if not client.wait_for_server(timeout_sec=0.2):
            self.publish_status("error", detail=f"{action} action server 未就绪", action=action)
            return

        if action == "NavigateThroughPoses":
            goal = self.NavigateThroughPoses.Goal()
            goal.poses = [self._make_pose_stamped(waypoint) for waypoint in waypoints]
        else:
            goal = self.NavigateToPose.Goal()
            goal.pose = self._make_pose_stamped(waypoints[0])

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._on_goal_response(result, action))
        self.publish_status("sent", detail=f"已发送 {len(waypoints)} 个航点", action=action)

    def cancel_navigation(self) -> None:
        if self.current_goal_handle is None:
            self.publish_status("canceled", detail="没有正在执行的导航")
            return
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _result: self.publish_status("canceled", detail="已请求取消当前导航"),
        )

    def publish_status(
        self,
        state: str,
        *,
        detail: str | None = None,
        action: str | None = None,
    ) -> None:
        payload = build_nav_status_payload(state, detail=detail, action=action)
        asyncio.run_coroutine_threadsafe(self.server.broadcast(payload, self.nav_ws_path), self.loop)

    def broadcast(self, payload: dict[str, object]) -> None:
        asyncio.run_coroutine_threadsafe(
            self.server.broadcast(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), self.nav_ws_path),
            self.loop,
        )

    def _create_action_client(self, action_type: object, action_name: str) -> object:
        import rclpy.action

        return rclpy.action.ActionClient(self.node, action_type, action_name)

    def _make_pose_stamped(self, waypoint: dict[str, float]) -> object:
        pose = self.PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.position.z = 0.0
        quaternion = yaw_to_quaternion(waypoint["yaw"])
        pose.pose.orientation.x = quaternion["x"]
        pose.pose.orientation.y = quaternion["y"]
        pose.pose.orientation.z = quaternion["z"]
        pose.pose.orientation.w = quaternion["w"]
        return pose

    def _normalize_waypoint(self, waypoint: object) -> dict[str, float] | None:
        if not isinstance(waypoint, dict):
            return None
        try:
            return {
                "x": float(waypoint["x"]),
                "y": float(waypoint["y"]),
                "yaw": float(waypoint.get("yaw", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            return None

    def _lookup_map_transform(self, source_frame: str) -> object | None | bool:
        if source_frame == self.map_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, source_frame, self._time_zero()).transform
        except self.TransformException as exc:
            print(
                f"[nav-ui-bridge] 无法将 {source_frame} 转换到 map：{exc}",
                file=sys.stderr,
                flush=True,
            )
            return False

    def _path_points_in_map(self, msg: object) -> tuple[list[dict[str, float]], str]:
        path_frame = message_frame_id(msg)
        points: list[dict[str, float]] = []
        for pose_stamped in msg.poses:
            pose_frame = message_frame_id(pose_stamped, path_frame)
            transform = self._lookup_map_transform(pose_frame)
            if transform is False:
                return [], pose_frame
            position = pose_stamped.pose.position
            points.append(transform_xy_to_map(float(position.x), float(position.y), transform))
        return points, "map"

    def _on_goal_response(self, future: object, action: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - 单次失败不能中断后续命令。
            self.publish_status("error", detail=f"发送导航目标失败: {exc}", action=action)
            return
        if not goal_handle.accepted:
            self.publish_status("failed", detail="Nav2 拒绝目标", action=action)
            return
        self.current_goal_handle = goal_handle
        self.publish_status("executing", detail="Nav2 已接受目标", action=action)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_goal_result(result, action))

    def _on_goal_result(self, future: object, action: str) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - 单次失败不能中断后续命令。
            self.publish_status("error", detail=f"导航结果异常: {exc}", action=action)
            return
        status = int(getattr(result, "status", 0))
        if status == 4:
            state = "succeeded"
            detail = "导航完成"
        elif status == 5:
            state = "canceled"
            detail = "导航已取消"
        else:
            state = "failed"
            detail = f"导航失败，状态码 {status}"
        self.current_goal_handle = None
        self.publish_status(state, detail=detail, action=action)

    def _time_zero(self) -> object:
        import rclpy.time

        return rclpy.time.Time()


def parse_http_request(
    raw: bytes,
) -> tuple[str, str, dict[str, str]] | None:
    try:
        text = raw.decode("iso-8859-1")
        lines = text.split("\r\n")
        method, target, _version = lines[0].split(" ", 2)
    except ValueError:
        return None

    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line or ":" not in line:
            continue
        name, value = line.split(":", 1)
        headers[name.strip().lower()] = value.strip()
    return method, target, headers


def is_websocket_request(path: str, headers: dict[str, str]) -> bool:
    return (
        bool(path)
        and headers.get("upgrade", "").lower() == "websocket"
        and "upgrade" in headers.get("connection", "").lower()
    )


async def read_json_body(
    reader: asyncio.StreamReader,
    headers: Mapping[str, str],
) -> dict[str, Any]:
    content_type = headers.get("content-type", "").split(";", 1)[0].strip().lower()
    if content_type != "application/json":
        raise ControlError(415, "Content-Type must be application/json")
    raw_length = headers.get("content-length", "0")
    try:
        length = int(raw_length)
    except ValueError as exc:
        raise ControlError(400, "invalid Content-Length") from exc
    if length < 0:
        raise ControlError(400, "invalid Content-Length")
    if length > MAX_HTTP_BODY_BYTES:
        raise ControlError(413, "request body too large")
    if length == 0:
        return {}
    raw = await reader.readexactly(length)
    payload = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ControlError(400, "JSON body must be an object")
    return payload


async def send_json_response(
    writer: asyncio.StreamWriter,
    status: int,
    payload: Mapping[str, Any],
) -> None:
    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    await send_response(writer, status, body, "application/json")


async def send_response(
    writer: asyncio.StreamWriter,
    status: int,
    body: bytes,
    content_type: str,
    extra_headers: Mapping[str, str] | None = None,
) -> None:
    reason = {
        200: "OK",
        202: "Accepted",
        302: "Found",
        400: "Bad Request",
        403: "Forbidden",
        404: "Not Found",
        405: "Method Not Allowed",
        409: "Conflict",
        413: "Payload Too Large",
        415: "Unsupported Media Type",
        500: "Internal Server Error",
        503: "Service Unavailable",
    }.get(status, "OK")
    additional = "".join(f"{name}: {value}\r\n" for name, value in (extra_headers or {}).items())
    headers = (
        f"HTTP/1.1 {status} {reason}\r\n"
        f"Content-Length: {len(body)}\r\n"
        f"Content-Type: {content_type}; charset=utf-8\r\n"
        f"{additional}"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    )
    writer.write(headers.encode("ascii") + body)
    await writer.drain()
    close_writer(writer)


def close_writer(writer: asyncio.StreamWriter) -> None:
    try:
        writer.close()
    except RuntimeError:
        pass


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    root_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=os.getenv("SLAM_NAV_CAMERA_TOPIC", DEFAULT_TOPIC))
    parser.add_argument(
        "--nav-depth-topic",
        default=os.getenv("SLAM_NAV_DEPTH_TOPIC", DEFAULT_NAV_DEPTH_TOPIC),
    )
    parser.add_argument(
        "--piper-color-topic",
        default=os.getenv("SLAM_NAV_PIPER_COLOR_TOPIC", DEFAULT_PIPER_COLOR_TOPIC),
    )
    parser.add_argument(
        "--piper-depth-topic",
        default=os.getenv("SLAM_NAV_PIPER_DEPTH_TOPIC", DEFAULT_PIPER_DEPTH_TOPIC),
    )
    parser.add_argument(
        "--navigation-ready-topic",
        default=os.getenv("SLAM_NAV_NAVIGATION_READY_TOPIC", DEFAULT_NAVIGATION_READY_TOPIC),
    )
    parser.add_argument(
        "--host",
        default=os.getenv(
            "SLAM_NAV_UI_HOST",
            os.getenv("SLAM_NAV_NAV_UI_HOST", DEFAULT_HOST),
        ),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(
            os.getenv(
                "SLAM_NAV_UI_PORT",
                os.getenv("SLAM_NAV_NAV_UI_PORT", str(DEFAULT_PORT)),
            )
        ),
    )
    parser.add_argument("--ws-path", default="/ws/rgb")
    parser.add_argument("--nav-ws-path", default=os.getenv("SLAM_NAV_NAV_WS_PATH", DEFAULT_NAV_WS_PATH))
    parser.add_argument(
        "--nav-depth-ws-path",
        default=os.getenv("SLAM_NAV_NAV_DEPTH_WS_PATH", DEFAULT_NAV_DEPTH_WS_PATH),
    )
    parser.add_argument(
        "--piper-rgb-ws-path",
        default=os.getenv("SLAM_NAV_PIPER_RGB_WS_PATH", DEFAULT_PIPER_RGB_WS_PATH),
    )
    parser.add_argument(
        "--piper-depth-ws-path",
        default=os.getenv("SLAM_NAV_PIPER_DEPTH_WS_PATH", DEFAULT_PIPER_DEPTH_WS_PATH),
    )
    parser.add_argument("--map-topic", default=os.getenv("SLAM_NAV_MAP_TOPIC", DEFAULT_MAP_TOPIC))
    parser.add_argument(
        "--global-costmap-topic",
        default=os.getenv("SLAM_NAV_GLOBAL_COSTMAP_TOPIC", DEFAULT_GLOBAL_COSTMAP_TOPIC),
    )
    parser.add_argument(
        "--local-costmap-topic",
        default=os.getenv("SLAM_NAV_LOCAL_COSTMAP_TOPIC", DEFAULT_LOCAL_COSTMAP_TOPIC),
    )
    parser.add_argument(
        "--global-plan-topic",
        default=os.getenv("SLAM_NAV_GLOBAL_PLAN_TOPIC", DEFAULT_GLOBAL_PLAN_TOPIC),
    )
    parser.add_argument(
        "--local-plan-topic",
        default=os.getenv("SLAM_NAV_LOCAL_PLAN_TOPIC", DEFAULT_LOCAL_PLAN_TOPIC),
    )
    parser.add_argument(
        "--lethal-cost-threshold",
        type=int,
        default=int(os.getenv("SLAM_NAV_LETHAL_COST_THRESHOLD", DEFAULT_LETHAL_COST_THRESHOLD)),
    )
    parser.add_argument("--map-frame", default=os.getenv("SLAM_NAV_MAP_FRAME", DEFAULT_MAP_FRAME))
    parser.add_argument(
        "--robot-frames",
        default=os.getenv("SLAM_NAV_ROBOT_FRAMES", DEFAULT_ROBOT_FRAMES),
        help="用于查询地图位姿的机器人基座 frame，使用逗号分隔。",
    )
    parser.add_argument("--static-dir", type=Path, default=root_dir / "dist")
    parser.add_argument("--quality", type=int, default=85)
    parser.add_argument("--max-fps", type=float, default=15.0)
    parser.add_argument(
        "--piper-quality",
        type=int,
        default=int(os.getenv("SLAM_NAV_PIPER_CAMERA_QUALITY", "80")),
    )
    parser.add_argument(
        "--piper-max-fps",
        type=float,
        default=float(os.getenv("SLAM_NAV_PIPER_CAMERA_MAX_FPS", "10")),
    )
    parser.add_argument(
        "--piper-depth-min",
        type=float,
        default=float(os.getenv("SLAM_NAV_PIPER_DEPTH_MIN_M", "0.15")),
    )
    parser.add_argument(
        "--piper-depth-max",
        type=float,
        default=float(os.getenv("SLAM_NAV_PIPER_DEPTH_MAX_M", "2.5")),
    )
    parser.add_argument(
        "--nav-depth-min",
        type=float,
        default=float(os.getenv("SLAM_NAV_DEPTH_MIN_M", "0.25")),
    )
    parser.add_argument(
        "--nav-depth-max",
        type=float,
        default=float(os.getenv("SLAM_NAV_DEPTH_MAX_M", "5.0")),
    )
    parser.add_argument(
        "--disable-camera",
        action="store_true",
        default=os.getenv(
            "SLAM_NAV_UI_DISABLE_CAMERA",
            os.getenv("SLAM_NAV_NAV_UI_DISABLE_CAMERA", "0"),
        ) == "1",
        help="关闭导航 RGB-D 图像订阅。",
    )
    parser.add_argument(
        "--disable-piper-camera",
        action="store_true",
        default=os.getenv("SLAM_NAV_UI_DISABLE_PIPER_CAMERA", "0") == "1",
        help="关闭 Piper 腕部 RGB-D 图像订阅。",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    depth_ranges = (
        ("导航", args.nav_depth_min, args.nav_depth_max),
        ("Piper", args.piper_depth_min, args.piper_depth_max),
    )
    invalid_depth = next(
        (label for label, minimum, maximum in depth_ranges if minimum < 0.0 or minimum >= maximum),
        None,
    )
    if invalid_depth is not None:
        print(f"[nav-ui-bridge] {invalid_depth}深度显示范围无效。", file=sys.stderr, flush=True)
        return 2
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    server = WebBridgeServer(
        args.host,
        args.port,
        args.static_dir,
        args.ws_path,
        nav_ws_path=args.nav_ws_path,
        additional_ws_paths=(
            args.nav_depth_ws_path,
            args.piper_rgb_ws_path,
            args.piper_depth_ws_path,
        ),
    )
    publisher = RosImagePublisher(args.topic, args.quality, args.max_fps, server)
    nav_depth_publisher = RosDepthPublisher(
        args.nav_depth_topic,
        args.quality,
        args.max_fps,
        server,
        args.nav_depth_ws_path,
        args.nav_depth_min,
        args.nav_depth_max,
    )
    piper_rgb_publisher = RosImagePublisher(
        args.piper_color_topic,
        args.piper_quality,
        args.piper_max_fps,
        server,
        args.piper_rgb_ws_path,
    )
    piper_depth_publisher = RosDepthPublisher(
        args.piper_depth_topic,
        args.piper_quality,
        args.piper_max_fps,
        server,
        args.piper_depth_ws_path,
        args.piper_depth_min,
        args.piper_depth_max,
    )
    navigation = RosNavigationPublisher(
        server=server,
        loop=loop,
        nav_ws_path=args.nav_ws_path,
        map_topic=args.map_topic,
        global_costmap_topic=args.global_costmap_topic,
        local_costmap_topic=args.local_costmap_topic,
        global_plan_topic=args.global_plan_topic,
        local_plan_topic=args.local_plan_topic,
        navigation_ready_topic=args.navigation_ready_topic,
        lethal_threshold=args.lethal_cost_threshold,
        map_frame=args.map_frame,
        robot_frames=parse_frame_list(args.robot_frames),
    )
    server.nav_command_handler = navigation.handle_command

    try:
        import rclpy

        rclpy.init(args=None)
        node = rclpy.create_node("slam_nav_ui_bridge")
        if not args.disable_camera:
            publisher.start(loop, node)
            nav_depth_publisher.start(loop, node)
        else:
            print("[nav-ui-bridge] 已关闭导航相机订阅。", flush=True)
        if not args.disable_piper_camera:
            piper_rgb_publisher.start(loop, node)
            piper_depth_publisher.start(loop, node)
        else:
            print("[nav-ui-bridge] 已关闭 Piper 腕部相机订阅。", flush=True)
        import rclpy.executors

        navigation.start(node)
    except ImportError as exc:
        print(
            "[nav-ui-bridge] 缺少 ROS2 依赖。请先加载 ROS2 Humble 与当前工作区环境，"
            "并确认 NumPy、OpenCV、Nav2 和 tf2_ros 已安装。",
            file=sys.stderr,
            flush=True,
        )
        print(f"[nav-ui-bridge] 导入失败: {exc}", file=sys.stderr, flush=True)
        return 2

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    server_task = loop.create_task(server.serve_forever())
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, server_task.cancel)
        except NotImplementedError:
            pass

    try:
        loop.run_until_complete(server_task)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
