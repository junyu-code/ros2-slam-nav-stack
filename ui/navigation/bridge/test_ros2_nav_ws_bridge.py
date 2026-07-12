import json
import os
import struct
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from ros2_nav_ws_bridge import (
    DEFAULT_NAV_WS_PATH,
    build_frame_payload,
    build_nav_status_payload,
    build_navigation_ready_payload,
    calculate_frame_latency_ms,
    costmap_cells_to_points,
    decode_websocket_text_messages,
    depth_message_to_bgr,
    encode_masked_client_text_frame,
    encode_websocket_text_frame,
    filter_costmap_obstacles,
    image_message_to_bgr,
    message_frame_id,
    parse_args,
    parse_frame_list,
    RosImagePublisher,
    RosNavigationPublisher,
    select_navigation_action,
    transform_xy_to_map,
    WebBridgeServer,
)


class BridgeHelpersTest(unittest.TestCase):
    def test_rgb_image_conversion_handles_row_padding_without_cv_bridge(self):
        import cv2
        import numpy as np

        message = SimpleNamespace(
            width=2,
            height=1,
            encoding="rgb8",
            step=8,
            data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
        )

        image = image_message_to_bgr(message, np, cv2)

        self.assertEqual(image.tolist(), [[[3, 2, 1], [6, 5, 4]]])

    def test_image_conversion_rejects_unsupported_encoding(self):
        import cv2
        import numpy as np

        message = SimpleNamespace(width=1, height=1, encoding="16UC1", step=2, data=b"\x00\x00")

        with self.assertRaisesRegex(ValueError, "暂不支持图像编码"):
            image_message_to_bgr(message, np, cv2)

    def test_depth_image_conversion_colorizes_16uc1_and_masks_invalid_pixels(self):
        import cv2
        import numpy as np

        message = SimpleNamespace(
            width=2,
            height=1,
            encoding="16UC1",
            is_bigendian=0,
            step=6,
            data=struct.pack("<HH", 900, 0) + b"\xff\xff",
        )

        image = depth_message_to_bgr(
            message,
            np,
            cv2,
            min_depth_m=0.15,
            max_depth_m=2.5,
        )

        self.assertEqual(image.shape, (1, 2, 3))
        self.assertNotEqual(image[0, 0].tolist(), [0, 0, 0])
        self.assertEqual(image[0, 1].tolist(), [0, 0, 0])

    def test_depth_image_conversion_supports_32fc1(self):
        import cv2
        import numpy as np

        message = SimpleNamespace(
            width=1,
            height=1,
            encoding="32FC1",
            is_bigendian=0,
            step=4,
            data=struct.pack("<f", 1.2),
        )

        image = depth_message_to_bgr(
            message,
            np,
            cv2,
            min_depth_m=0.15,
            max_depth_m=2.5,
        )

        self.assertEqual(image.shape, (1, 1, 3))
        self.assertNotEqual(image[0, 0].tolist(), [0, 0, 0])

    def test_build_frame_payload_uses_frontend_wire_shape(self):
        payload = build_frame_payload(
            image_base64="abc123",
            width=640,
            height=480,
            timestamp=12.5,
            fps=29.9,
            latency_ms=18.2,
            topic="/nav_camera/color/image_raw",
        )

        data = json.loads(payload)

        self.assertEqual(data["image"], "data:image/jpeg;base64,abc123")
        self.assertEqual(data["width"], 640)
        self.assertEqual(data["height"], 480)
        self.assertEqual(data["timestamp"], 12.5)
        self.assertEqual(data["fps"], 29.9)
        self.assertEqual(data["latencyMs"], 18.2)
        self.assertEqual(data["topic"], "/nav_camera/color/image_raw")

    def test_frame_latency_ignores_sim_time_and_future_timestamps(self):
        self.assertAlmostEqual(calculate_frame_latency_ms(100.0, 99.975), 25.0)
        self.assertIsNone(calculate_frame_latency_ms(1_800_000_000.0, 42.0))
        self.assertIsNone(calculate_frame_latency_ms(100.0, 101.0))

    def test_encode_websocket_text_frame_creates_unmasked_server_frame(self):
        frame = encode_websocket_text_frame("ok")

        self.assertEqual(frame, b"\x81\x02ok")

    def test_decode_websocket_text_messages_reads_masked_browser_frames(self):
        frame = encode_masked_client_text_frame('{"type":"ping"}', mask=b"\x01\x02\x03\x04")

        messages, remainder = decode_websocket_text_messages(frame)

        self.assertEqual(messages, ['{"type":"ping"}'])
        self.assertEqual(remainder, b"")

    def test_filter_costmap_obstacles_keeps_only_lethal_cells(self):
        cells = filter_costmap_obstacles([0, 100, 252, 253, 254, 255, -1], width=7, lethal_threshold=253)

        self.assertEqual(
            cells,
            [
                {"x": 3, "y": 0, "value": 253},
                {"x": 4, "y": 0, "value": 254},
                {"x": 5, "y": 0, "value": 255},
            ],
        )

    def test_costmap_cells_to_points_applies_origin_and_map_transform(self):
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=10.0, y=-1.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        points = costmap_cells_to_points(
            [{"x": 1, "y": 2, "value": 254}],
            origin={"x": 1.0, "y": 2.0, "yaw": 0.0},
            resolution=0.5,
            transform=transform,
        )

        self.assertEqual(points, [{"x": 11.75, "y": 2.25, "value": 254.0}])

    def test_transform_xy_to_map_uses_transform_yaw_and_translation(self):
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=2.0, y=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=2**0.5 / 2, w=2**0.5 / 2),
        )

        point = transform_xy_to_map(1.0, 0.0, transform)

        self.assertAlmostEqual(point["x"], 2.0)
        self.assertAlmostEqual(point["y"], 4.0)

    def test_message_frame_id_defaults_to_map(self):
        self.assertEqual(message_frame_id(SimpleNamespace()), "map")
        self.assertEqual(
            message_frame_id(SimpleNamespace(header=SimpleNamespace(frame_id="odom"))),
            "odom",
        )

    def test_parse_frame_list_keeps_order_and_drops_blanks(self):
        self.assertEqual(parse_frame_list("base_link, base_footprint,,body"), ["base_link", "base_footprint", "body"])
        self.assertEqual(parse_frame_list(" , "), ["base_link"])

    def test_select_navigation_action_switches_on_waypoint_count(self):
        self.assertEqual(select_navigation_action([{"x": 1, "y": 2, "yaw": 0}]), "NavigateToPose")
        self.assertEqual(
            select_navigation_action([
                {"x": 1, "y": 2, "yaw": 0},
                {"x": 3, "y": 4, "yaw": 0},
            ]),
            "NavigateThroughPoses",
        )

    def test_build_nav_status_payload_uses_frontend_wire_shape(self):
        payload = json.loads(build_nav_status_payload("executing", detail="moving", action="NavigateToPose"))

        self.assertEqual(payload["type"], "nav_status")
        self.assertEqual(payload["state"], "executing")
        self.assertEqual(payload["detail"], "moving")
        self.assertEqual(payload["action"], "NavigateToPose")

    def test_build_navigation_ready_payload_uses_frontend_wire_shape(self):
        payload = json.loads(build_navigation_ready_payload(True, "/navigation_ready"))

        self.assertEqual(
            payload,
            {
                "type": "navigation_ready",
                "ready": True,
                "topic": "/navigation_ready",
            },
        )

    def test_navigation_command_waits_for_ready_signal(self):
        navigation = RosNavigationPublisher(
            server=SimpleNamespace(),
            loop=SimpleNamespace(),
            nav_ws_path="/ws/nav",
            map_topic="/map",
            global_costmap_topic="/global_costmap/costmap",
            local_costmap_topic="/local_costmap/costmap",
            global_plan_topic="/plan",
            local_plan_topic="/local_plan",
            navigation_ready_topic="/navigation_ready",
            lethal_threshold=253,
            map_frame="map",
            robot_frames=["base_link"],
        )
        navigation.navigation_ready = False
        navigation.publish_status = Mock()

        navigation.handle_command(
            json.dumps(
                {
                    "type": "navigate",
                    "waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0}],
                },
            ),
        )

        navigation.publish_status.assert_called_once_with(
            "waiting",
            detail="导航尚未就绪，等待 /navigation_ready=true",
        )

    def test_rgb_and_navigation_websockets_use_independent_send_locks(self):
        server = WebBridgeServer(
            "127.0.0.1",
            8765,
            Path("."),
            "/ws/rgb",
            additional_ws_paths=("/ws/nav/depth", "/ws/piper/rgb", "/ws/piper/depth"),
        )

        self.assertIsNot(server.locks["/ws/rgb"], server.locks[DEFAULT_NAV_WS_PATH])
        self.assertIn("/ws/nav/depth", server.clients)
        self.assertIn("/ws/piper/rgb", server.clients)
        self.assertIn("/ws/piper/depth", server.clients)

    def test_image_subscription_uses_sensor_data_qos(self):
        qos_marker = object()
        image_type = object()
        callback_group = object()

        rclpy_module = ModuleType("rclpy")
        rclpy_module.__path__ = []
        callback_groups_module = ModuleType("rclpy.callback_groups")
        callback_groups_module.ReentrantCallbackGroup = Mock(return_value=callback_group)
        rclpy_module.callback_groups = callback_groups_module
        qos_module = ModuleType("rclpy.qos")
        qos_module.qos_profile_sensor_data = qos_marker

        sensor_msgs_module = ModuleType("sensor_msgs")
        sensor_msgs_module.__path__ = []
        sensor_msgs_msg_module = ModuleType("sensor_msgs.msg")
        sensor_msgs_msg_module.Image = image_type

        node = SimpleNamespace(create_subscription=Mock())
        server = SimpleNamespace(ws_path="/ws/rgb")
        publisher = RosImagePublisher("/camera", 85, 15.0, server)

        with patch.dict(
            sys.modules,
            {
                "rclpy": rclpy_module,
                "rclpy.callback_groups": callback_groups_module,
                "rclpy.qos": qos_module,
                "sensor_msgs": sensor_msgs_module,
                "sensor_msgs.msg": sensor_msgs_msg_module,
            },
        ):
            publisher.start(SimpleNamespace(), node)

        node.create_subscription.assert_called_once_with(
            image_type,
            "/camera",
            publisher.on_image,
            qos_marker,
            callback_group=callback_group,
        )

    def test_default_camera_fps_is_limited_to_reduce_navigation_backlog(self):
        with patch.dict(os.environ, {}, clear=True):
            args = parse_args([])

        self.assertEqual(args.max_fps, 15.0)
        self.assertEqual(args.topic, "/nav_camera/color/image_raw")
        self.assertEqual(args.nav_depth_topic, "/nav_camera/depth/image_raw")
        self.assertEqual(args.nav_depth_min, 0.25)
        self.assertEqual(args.nav_depth_max, 5.0)
        self.assertEqual(args.piper_color_topic, "/piper/arm_camera/color/image_raw")
        self.assertEqual(args.piper_depth_topic, "/piper/arm_camera/depth/image_raw")
        self.assertEqual(args.navigation_ready_topic, "/navigation_ready")
        self.assertEqual(args.piper_max_fps, 10.0)
        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 8765)

    def test_new_ui_environment_names_override_legacy_names(self):
        with patch.dict(
            os.environ,
            {
                "SLAM_NAV_UI_HOST": "0.0.0.0",
                "SLAM_NAV_UI_PORT": "9000",
                "SLAM_NAV_UI_DISABLE_CAMERA": "1",
                "SLAM_NAV_NAV_UI_HOST": "127.0.0.2",
                "SLAM_NAV_NAV_UI_PORT": "9001",
                "SLAM_NAV_NAV_UI_DISABLE_CAMERA": "0",
            },
            clear=True,
        ):
            args = parse_args([])

        self.assertEqual(args.host, "0.0.0.0")
        self.assertEqual(args.port, 9000)
        self.assertTrue(args.disable_camera)

    def test_camera_subscription_can_be_disabled(self):
        args = parse_args(["--disable-camera", "--disable-piper-camera"])

        self.assertTrue(args.disable_camera)
        self.assertTrue(args.disable_piper_camera)


if __name__ == "__main__":
    unittest.main()
