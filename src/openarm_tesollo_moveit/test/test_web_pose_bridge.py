import importlib.util
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.time import Time
import xacro
from ament_index_python.packages import get_package_share_directory

spec = importlib.util.spec_from_file_location(
    "web_pose_bridge", Path(__file__).parents[1] / "scripts/web_pose_bridge.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WebPoseBridgeTest(unittest.TestCase):
    def target(self):
        return {"side": "left", "position": [0.2, 0.25, 0.4], "orientation": [0, 0, 0, 1]}

    def test_valid_pose_uses_world_frame(self):
        side, pose = module.validate_target(self.target())
        self.assertEqual(side, "left")
        self.assertEqual(pose.header.frame_id, "world")
        self.assertEqual(pose.pose.position.y, 0.25)
        self.assertEqual(pose.pose.orientation.w, 1.0)

    def test_invalid_numeric_and_quaternion_inputs_are_rejected(self):
        for field, value in (("side", "other"), ("position", [True, 0, 0]),
                             ("position", [float("nan"), 0, 0]), ("position", [4, 0, 0]),
                             ("orientation", [0, 0, 0, 0]), ("orientation", [0, 0, 1])):
            with self.subTest(field=field, value=value):
                data = self.target()
                data[field] = value
                with self.assertRaises(ValueError):
                    module.validate_target(data)

    def test_expanded_position_and_half_turn_are_accepted(self):
        data = self.target()
        data["position"] = [1.2, -1.2, 1.8]
        data["orientation"] = [1, 0, 0, 0]
        _, pose = module.validate_target(data)
        self.assertEqual(pose.pose.position.z, 1.8)
        self.assertEqual(pose.pose.orientation.x, 1.0)

    def test_state_reports_hand_back_tcp_pose(self):
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            rotation=SimpleNamespace(x=0, y=0, z=0, w=1),
        )
        node = SimpleNamespace(
            ready={"left", "right"}, pending=set(), lock=threading.Lock(),
            buffer=Mock(), state={"arms": {}},
        )
        node.buffer.lookup_transform.return_value.transform = transform
        module.WebPoseBridge.update_state(node)
        for side, call in zip(("left", "right"), node.buffer.lookup_transform.call_args_list):
            self.assertEqual(call.args[:2], ("world", f"{side}_hand_back_tcp"))
            self.assertEqual(node.state["arms"][side]["tip_frame"], f"{side}_hand_back_tcp")
            self.assertEqual(node.state["arms"][side]["position"], [0.1, 0.2, 0.3])

    def test_expired_input_stops_and_discards_target(self):
        node = SimpleNamespace(
            lock=threading.Lock(), target=module.validate_target(self.target()),
            received=time.monotonic(), state={}, publishers_by_side={"left": Mock()},
            get_clock=Mock(return_value=Mock()),
        )
        node.get_clock().now.return_value = Time(seconds=5)
        module.WebPoseBridge.publish_target(node)
        node.publishers_by_side["left"].publish.assert_called_once()
        self.assertTrue(node.state["active"])
        node.received = time.monotonic() - 1
        module.WebPoseBridge.publish_target(node)
        self.assertFalse(node.state["active"])
        self.assertIsNone(node.target)
        node.publishers_by_side["left"].publish.assert_called_once()

    def test_camera_scene_uses_only_registered_local_meshes(self):
        path = Path(get_package_share_directory("openarm_tesollo_moveit")) / "urdf/robot.urdf.xacro"
        description = xacro.process_file(str(path)).toxml()
        scene, assets = module.browser_scene(description)
        root = ET.fromstring(scene["urdf"])
        self.assertIsNotNone(root.find(f"link[@name='{scene['camera_frame']}']"))
        self.assertTrue(assets)
        for mesh in root.findall(".//visual/geometry/mesh"):
            self.assertIn(mesh.get("filename"), assets)
            self.assertTrue(assets[mesh.get("filename")].is_file())

    def test_scene_rejects_non_package_mesh_paths(self):
        description = '<robot><link name="openarm_body_zed_left_camera_frame_optical"><visual><geometry><mesh filename="/etc/passwd"/></geometry></visual></link></robot>'
        with self.assertRaises(ValueError):
            module.browser_scene(description)


if __name__ == "__main__":
    unittest.main()
