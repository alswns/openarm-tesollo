import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


class ModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        share = Path(get_package_share_directory("openarm_tesollo_moveit"))
        cls.urdf = xacro.process_file(str(share / "urdf/robot.urdf.xacro")).toxml()
        cls.robot = ET.fromstring(cls.urdf)
        cls.controllers = yaml.safe_load((share / "config/controllers.yaml").read_text())
        spec = importlib.util.spec_from_file_location("practice_launch", share / "launch/practice.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.srdf = ET.fromstring(module.semantic_description(cls.urdf))

    def test_graph_and_controller_coverage(self):
        links = [link.get("name") for link in self.robot.findall("link")]
        joints = self.robot.findall("joint")
        names = [joint.get("name") for joint in joints]
        self.assertEqual(len(links), len(set(links)))
        self.assertEqual(len(names), len(set(names)))
        children = [joint.find("child").get("link") for joint in joints]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(set(links) - set(children), {"world"})
        controlled = {joint.get("name") for joint in self.robot.find("ros2_control").findall("joint")}
        configured = []
        for side in ("left", "right"):
            for kind, expected in (("arm", 7), ("hand", 20)):
                names = self.controllers[f"{side}_{kind}_controller"]["ros__parameters"]["joints"]
                self.assertEqual(len(names), expected)
                configured.extend(names)
        self.assertEqual(len(set(configured)), 54)
        self.assertEqual(set(configured), controlled)

    def test_servo_chains_and_initial_positions(self):
        joints = self.robot.findall("joint")
        for side in ("left", "right"):
            chain = self.srdf.find(f"group[@name='{side}_arm']/chain")
            self.assertEqual(chain.get("tip_link"), f"{side}_hand_back_tcp")
            current = chain.get("tip_link")
            count = 0
            while current != chain.get("base_link"):
                joint = next(joint for joint in joints if joint.find("child").get("link") == current)
                count += joint.get("type") != "fixed"
                current = joint.find("parent").get("link")
            self.assertEqual(count, 7)
        by_name = {joint.get("name"): joint for joint in joints}
        for control in self.robot.find("ros2_control").findall("joint"):
            initial = float(control.find("state_interface/param").text)
            limit = by_name[control.get("name")].find("limit")
            self.assertLessEqual(float(limit.get("lower")), initial)
            self.assertLessEqual(initial, float(limit.get("upper")))

    def test_hand_back_tcp_does_not_change_hand_mount(self):
        for side in ("left", "right"):
            mount = self.robot.find(f"joint[@name='{side}_hand_mount']")
            self.assertEqual(mount.find("origin").get("xyz"), "0 0 -0.03")
            self.assertEqual(mount.find("origin").get("rpy"), "3.14 0 0")
            tcp = self.robot.find(f"joint[@name='{side}_hand_back_tcp_joint']")
            self.assertEqual(tcp.get("type"), "fixed")
            self.assertEqual(tcp.find("parent").get("link"), f"{side}_hand_link_base")
            self.assertEqual(tcp.find("origin").get("xyz"), "-0.01804 0 0.05")
            self.assertEqual(tcp.find("origin").get("rpy"), "0 0 0")
            self.assertIsNone(self.robot.find(f"link[@name='{side}_hand_back_tcp']/collision"))

    def test_web_start_is_separate_and_within_joint_limits(self):
        share = Path(get_package_share_directory("openarm_tesollo_moveit"))
        web = ET.fromstring(xacro.process_file(
            str(share / "urdf/robot.urdf.xacro"), mappings={"initial_pose": "web"}).toxml())
        by_name = {joint.get("name"): joint for joint in web.findall("joint")}
        for control in self.robot.find("ros2_control").findall("joint"):
            self.assertEqual(float(control.find("state_interface/param").text), 0.0)
        for control in web.find("ros2_control").findall("joint"):
            initial = float(control.find("state_interface/param").text)
            limit = by_name[control.get("name")].find("limit")
            self.assertLessEqual(float(limit.get("lower")), initial)
            self.assertLessEqual(initial, float(limit.get("upper")))
            if "hand_joint" in control.get("name"):
                self.assertEqual(initial, 0.0)

    def test_camera_is_attached_with_optical_frame(self):
        mount = self.robot.find("joint[@name='openarm_body_zed_mount_joint']")
        self.assertEqual(mount.get("type"), "fixed")
        self.assertEqual(mount.find("parent").get("link"), "openarm_body_zed_support_camera_mount_point")
        self.assertIsNotNone(self.robot.find("link[@name='openarm_body_zed_left_camera_frame_optical']"))


if __name__ == "__main__":
    unittest.main()
