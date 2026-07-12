import asyncio
import json
import os
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from control_api import ControlError, ControlService, OPERATOR_EXECUTABLE, RunState
from ros2_nav_ws_bridge import WebBridgeServer


TEST_FLOWS = {
    "check": {
        "label": "Check",
        "command": ["task1-status"],
        "group": "Checks",
        "level": "check",
        "summary": "Run a check.",
    },
    "operator": {
        "label": "Qt/RViz Operator",
        "command": ["operator"],
        "group": "Tools",
        "level": "primary",
        "summary": "Open the professional UI.",
    },
}


class FakeProcess:
    def __init__(self, pid: int = 1234, return_code: int | None = None) -> None:
        self.pid = pid
        self.return_code = return_code

    def poll(self) -> int | None:
        return self.return_code

    def wait(self, timeout: float | None = None) -> int:
        if self.return_code is None:
            raise RuntimeError("test process is still running")
        return self.return_code


class ControlServiceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        (self.root / "run.sh").touch()
        self.service = ControlService(self.root, self.root / "logs", TEST_FLOWS)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def make_operator_executable(self) -> None:
        executable = self.root / OPERATOR_EXECUTABLE
        executable.parent.mkdir(parents=True, exist_ok=True)
        executable.touch()
        executable.chmod(0o755)

    def test_operator_status_requires_display_and_built_executable(self):
        with patch.dict(os.environ, {}, clear=True):
            status = self.service.operator_status(running=False)
        self.assertEqual(
            status,
            {
                "available": False,
                "running": False,
                "reason": "未检测到 DISPLAY 或 WAYLAND_DISPLAY",
            },
        )

        self.make_operator_executable()
        with patch.dict(os.environ, {"DISPLAY": ":0"}, clear=True):
            status = self.service.operator_status(running=True)
        self.assertEqual(status, {"available": True, "running": True, "reason": None})

    def test_run_flow_starts_a_detached_process_and_tracks_it(self):
        process = FakeProcess()
        with (
            patch("control_api.subprocess.Popen", return_value=process) as popen,
            patch("control_api.threading.Thread") as thread,
        ):
            result = self.service.run_flow("check", ["--verbose"])

        self.assertTrue(result["ok"])
        self.assertEqual(result["current"]["flowId"], "check")
        self.assertEqual(len(self.service.active), 1)
        self.assertTrue(result["current"]["logPath"].startswith("logs/"))
        self.assertEqual(
            popen.call_args.args[0],
            ["bash", str(self.root / "run.sh"), "task1-status", "--verbose"],
        )
        self.assertTrue(popen.call_args.kwargs["start_new_session"])
        thread.return_value.start.assert_called_once_with()

    def test_operator_is_single_instance(self):
        self.make_operator_executable()
        with (
            patch.dict(os.environ, {"DISPLAY": ":0"}, clear=True),
            patch("control_api.process_online", return_value=False),
            patch("control_api.subprocess.Popen", return_value=FakeProcess()),
            patch("control_api.threading.Thread"),
        ):
            self.service.run_flow("operator")
            with self.assertRaises(ControlError) as context:
                self.service.run_flow("operator")

        self.assertEqual(context.exception.status, 409)
        self.assertEqual(context.exception.message, "operator already running")

    def test_operator_rejects_missing_graphical_environment(self):
        self.make_operator_executable()
        with (
            patch.dict(os.environ, {}, clear=True),
            patch("control_api.process_online", return_value=False),
        ):
            with self.assertRaises(ControlError) as context:
                self.service.run_flow("operator")
        self.assertEqual(context.exception.status, 503)

    def test_run_flow_validates_flow_and_args(self):
        for flow_id, args in (("missing", []), ("check", "bad"), ("check", [1])):
            with self.subTest(flow_id=flow_id, args=args):
                with self.assertRaises(ControlError) as context:
                    self.service.run_flow(flow_id, args)
                self.assertEqual(context.exception.status, 400)

    def test_stop_can_target_one_run_or_all_runs(self):
        first = RunState(run_id="one", flow_id="check", process=FakeProcess(1))
        second = RunState(run_id="two", flow_id="check", process=FakeProcess(2))
        self.service.active = [first, second]
        with patch("control_api.stop_process") as stop:
            result = self.service.stop("one")
            self.assertEqual(result, {"ok": True, "stopped": 1})
            stop.assert_called_once_with(first.process)
            self.assertTrue(first.stop_requested)
            self.assertFalse(second.stop_requested)

            stop.reset_mock()
            result = self.service.stop()
            self.assertEqual(result, {"ok": True, "stopped": 2})
            self.assertEqual(stop.call_count, 2)
            self.assertTrue(second.stop_requested)

    def test_stop_rejects_unknown_run_id(self):
        with self.assertRaises(ControlError) as context:
            self.service.stop("missing")
        self.assertEqual(context.exception.status, 404)

    def test_log_can_select_current_or_historical_run(self):
        current_log = self.root / "logs" / "current.log"
        history_log = self.root / "logs" / "history.log"
        current_log.parent.mkdir(parents=True)
        current_log.write_text("one\ntwo\nthree\n", encoding="utf-8")
        history_log.write_text("old log\n", encoding="utf-8")
        self.service.current = RunState(run_id="current", log_path=current_log)
        self.service.history = [
            RunState(run_id="old", log_path=history_log, finished_at=1.0, return_code=0),
        ]

        self.assertEqual(self.service.read_log(None, 2), {"runId": "current", "text": "two\nthree"})
        self.assertEqual(self.service.read_log("old", 10), {"runId": "old", "text": "old log"})
        with self.assertRaises(ControlError) as context:
            self.service.read_log("missing", 10)
        self.assertEqual(context.exception.status, 404)

    def test_snapshot_exposes_operator_wire_shape(self):
        self.make_operator_executable()
        self.service.active = [
            RunState(run_id="operator-run", flow_id="operator", process=FakeProcess()),
        ]
        runtime = {
            "processes": {"operator": False},
            "ros": {"ok": False, "count": 0, "items": []},
            "display": {"DISPLAY": ":0", "WAYLAND_DISPLAY": "", "ok": True},
        }
        with (
            patch.dict(os.environ, {"DISPLAY": ":0"}, clear=True),
            patch("control_api.collect_runtime", return_value=runtime),
            patch("control_api.collect_health", return_value={}),
        ):
            snapshot = self.service.snapshot()

        self.assertEqual(snapshot["operator"], {"available": True, "running": True, "reason": None})
        self.assertFalse(snapshot["runtime"]["processes"]["operator"])


class StubControlService:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.snapshot_thread: int | None = None

    def snapshot(self) -> dict[str, object]:
        self.snapshot_thread = threading.get_ident()
        return {"running": False, "operator": {"available": True, "running": False, "reason": None}}

    def read_log(self, run_id: str | None, lines: int) -> dict[str, object]:
        self.calls.append(("log", run_id, lines))
        return {"runId": run_id, "text": "test log"}

    def run_flow(self, flow_id: object, args: object) -> dict[str, object]:
        self.calls.append(("run", flow_id, args))
        return {"ok": True, "current": {"flowId": flow_id}}

    def stop(self, run_id: object = None) -> dict[str, object]:
        self.calls.append(("stop", run_id))
        return {"ok": True, "stopped": 1}


class WebBridgeControlApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        static_dir = Path(self.temp_dir.name)
        (static_dir / "index.html").write_text("SLAM Nav UI", encoding="utf-8")
        self.control = StubControlService()
        self.bridge = WebBridgeServer(
            "127.0.0.1",
            0,
            static_dir,
            "/ws/rgb",
            control_service=self.control,
            additional_ws_paths=("/ws/nav/depth", "/ws/piper/rgb", "/ws/piper/depth"),
        )
        self.server = await asyncio.start_server(self.bridge.handle_client, "127.0.0.1", 0)
        self.port = self.server.sockets[0].getsockname()[1]

    async def asyncTearDown(self) -> None:
        self.server.close()
        await self.server.wait_closed()
        self.temp_dir.cleanup()

    async def request(
        self,
        method: str,
        target: str,
        body: bytes = b"",
        content_length: int | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
        length = len(body) if content_length is None else content_length
        content_type_header = "Content-Type: application/json\r\n" if method == "POST" else ""
        request = (
            f"{method} {target} HTTP/1.1\r\n"
            "Host: localhost\r\n"
            f"{content_type_header}"
            f"Content-Length: {length}\r\n"
            "Connection: close\r\n\r\n"
        ).encode("ascii") + body
        writer.write(request)
        await writer.drain()
        response = await reader.read()
        writer.close()
        await writer.wait_closed()

        header_bytes, response_body = response.split(b"\r\n\r\n", 1)
        header_lines = header_bytes.decode("iso-8859-1").split("\r\n")
        status = int(header_lines[0].split(" ")[1])
        headers = {
            name.lower(): value.strip()
            for name, value in (line.split(":", 1) for line in header_lines[1:] if ":" in line)
        }
        return status, headers, response_body

    async def test_state_runs_off_the_event_loop(self):
        loop_thread = threading.get_ident()
        status, _headers, body = await self.request("GET", "/api/state")

        self.assertEqual(status, 200)
        self.assertFalse(json.loads(body)["running"])
        self.assertNotEqual(self.control.snapshot_thread, loop_thread)

    async def test_log_query_and_post_bodies_are_forwarded(self):
        status, _headers, body = await self.request("GET", "/api/log?runId=abc&lines=40")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["text"], "test log")

        run_body = json.dumps({"flowId": "check", "args": ["--fast"]}).encode("utf-8")
        status, _headers, _body = await self.request("POST", "/api/run", run_body)
        self.assertEqual(status, 202)

        stop_body = json.dumps({"runId": "abc"}).encode("utf-8")
        status, _headers, _body = await self.request("POST", "/api/stop", stop_body)
        self.assertEqual(status, 200)
        self.assertEqual(
            self.control.calls,
            [("log", "abc", 40), ("run", "check", ["--fast"]), ("stop", "abc")],
        )

    async def test_empty_stop_body_stops_all(self):
        status, _headers, _body = await self.request("POST", "/api/stop")
        self.assertEqual(status, 200)
        self.assertEqual(self.control.calls, [("stop", None)])

    async def test_invalid_json_and_query_are_rejected(self):
        status, _headers, _body = await self.request("POST", "/api/run", b"{")
        self.assertEqual(status, 400)
        status, _headers, _body = await self.request("GET", "/api/log?lines=bad")
        self.assertEqual(status, 400)

    async def test_post_requires_json_content_type(self):
        reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
        writer.write(
            b"POST /api/stop HTTP/1.1\r\n"
            b"Host: localhost\r\n"
            b"Content-Length: 0\r\n"
            b"Connection: close\r\n\r\n"
        )
        await writer.drain()
        response = await reader.read()
        writer.close()
        await writer.wait_closed()

        self.assertTrue(response.startswith(b"HTTP/1.1 415 Unsupported Media Type"))
        self.assertEqual(self.control.calls, [])

    async def test_expected_control_errors_keep_their_http_status(self):
        body = json.dumps({"flowId": "operator"}).encode("utf-8")
        with patch.object(
            self.control,
            "run_flow",
            side_effect=ControlError(409, "operator already running"),
        ):
            status, _headers, response_body = await self.request("POST", "/api/run", body)
        self.assertEqual(status, 409)
        self.assertEqual(json.loads(response_body), {"error": "operator already running"})

    async def test_console_redirect_and_static_site_share_the_api_port(self):
        status, headers, _body = await self.request("GET", "/console.html")
        self.assertEqual(status, 302)
        self.assertEqual(headers["location"], "/?panel=tasks")

        status, _headers, body = await self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertEqual(body, b"SLAM Nav UI")

    async def test_existing_websocket_upgrades_are_preserved(self):
        for path in ("/ws/rgb", "/ws/nav", "/ws/nav/depth", "/ws/piper/rgb", "/ws/piper/depth"):
            with self.subTest(path=path):
                reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
                writer.write(
                    f"GET {path} HTTP/1.1\r\n".encode("ascii")
                    + b"Host: localhost\r\n"
                    + b"Upgrade: websocket\r\n"
                    + b"Connection: Upgrade\r\n"
                    + b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n"
                )
                await writer.drain()
                response = await reader.readuntil(b"\r\n\r\n")
                self.assertTrue(response.startswith(b"HTTP/1.1 101 Switching Protocols"))
                writer.close()
                await writer.wait_closed()

    async def test_navigation_ready_payload_is_replayed_to_late_websocket_clients(self):
        payload = '{"type":"navigation_ready","ready":true}'
        await self.bridge.broadcast(
            payload,
            "/ws/nav",
            retain_key="navigation_ready",
        )

        reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
        writer.write(
            b"GET /ws/nav HTTP/1.1\r\n"
            b"Host: localhost\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n"
        )
        await writer.drain()
        await reader.readuntil(b"\r\n\r\n")
        frame = await reader.readexactly(2 + len(payload.encode("utf-8")))

        self.assertEqual(frame[:2], bytes((0x81, len(payload.encode("utf-8")))))
        self.assertEqual(frame[2:].decode("utf-8"), payload)
        writer.close()
        await writer.wait_closed()

    async def test_serve_forever_cancellation_closes_connected_clients(self):
        static_dir = Path(self.temp_dir.name)
        bridge = WebBridgeServer("127.0.0.1", 0, static_dir, "/ws/rgb")
        server_task = asyncio.create_task(bridge.serve_forever())
        while bridge.server is None:
            await asyncio.sleep(0)
        port = bridge.server.sockets[0].getsockname()[1]

        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        writer.write(
            b"GET /ws/rgb HTTP/1.1\r\n"
            b"Host: localhost\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n"
        )
        await writer.drain()
        await reader.readuntil(b"\r\n\r\n")

        server_task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await server_task

        self.assertFalse(bridge.connection_tasks)
        writer.close()
        await writer.wait_closed()


if __name__ == "__main__":
    unittest.main()
