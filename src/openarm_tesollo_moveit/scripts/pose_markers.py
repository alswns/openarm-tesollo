#!/usr/bin/env python3
"""Forward RViz marker targets to each arm's MoveIt Servo pose input."""
import copy
import math

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class PoseMarkers(Node):
    def __init__(self):
        super().__init__("pose_markers")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.server = InteractiveMarkerServer(self, "pose_markers")
        self.pose_publishers = {
            side: self.create_publisher(PoseStamped, f"/{side}_servo/pose_target_cmds", 10)
            for side in ("left", "right")
        }
        self.jog_publishers = {
            side: self.create_publisher(JointJog, f"/{side}_servo/delta_joint_cmds", 10)
            for side in ("left", "right")
        }
        self.servo_clients = {
            side: self.create_client(ServoCommandType, f"/{side}_servo/switch_command_type")
            for side in ("left", "right")
        }
        self.ready = set()
        self.pending = {}
        self.markers = set()
        self.targets = {}
        self.modes = {}
        self.desired_modes = dict.fromkeys(("left", "right"), ServoCommandType.Request.POSE)
        self.jogs = {}
        self.menus = {}
        self.reset_markers = set()
        self.create_timer(0.05, self.tick)

    def switch_done(self, side, mode, future):
        self.pending.pop(side, None)
        try:
            if future.result().success:
                self.ready.add(side)
                self.modes[side] = mode
            else:
                self.get_logger().error(f"{side}: Servo rejected command mode {mode}; retrying")
        except Exception as exc:
            self.get_logger().error(f"{side}: Servo mode switch failed: {exc}")

    def add_marker(self, side, transform):
        marker = InteractiveMarker()
        marker.header.frame_id = "world"
        marker.name = side
        marker.description = f"{side} hand-back TCP target"
        marker.scale = 0.22
        marker.pose.position.x = transform.translation.x
        marker.pose.position.y = transform.translation.y
        marker.pose.position.z = transform.translation.z
        marker.pose.orientation = transform.rotation
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
        sphere.color.a = 0.8
        sphere.color.r = 0.2 if side == "left" else 0.9
        sphere.color.g = 0.7
        sphere.color.b = 0.9 if side == "left" else 0.2
        center = InteractiveMarkerControl()
        center.always_visible = True
        center.markers.append(sphere)
        marker.controls.append(center)
        for axis in ("x", "y", "z"):
            for mode in (InteractiveMarkerControl.MOVE_AXIS, InteractiveMarkerControl.ROTATE_AXIS):
                control = InteractiveMarkerControl()
                control.name = f"{axis}_{mode}"
                control.orientation.w = math.sqrt(0.5)
                setattr(control.orientation, axis, math.sqrt(0.5))
                control.interaction_mode = mode
                marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self.feedback)
        menu = MenuHandler()
        root = menu.insert("Joint jog (escape singularity)")
        for joint in range(1, 8):
            parent = menu.insert(f"Joint {joint}", parent=root)
            for direction, label in ((1, "+"), (-1, "-")):
                menu.insert(
                    f"{label} 0.1 rad (nominal)", parent=parent,
                    callback=lambda feedback, joint=joint, direction=direction:
                        self.start_jog(feedback.marker_name, joint, direction),
                )
        menu.apply(self.server, side)
        self.menus[side] = menu
        self.server.applyChanges()
        self.markers.add(side)

    def start_jog(self, side, joint, direction):
        if side not in self.ready or side in self.pending or side in self.jogs or side in self.reset_markers:
            self.get_logger().warning(f"{side}: Wait for the previous command to finish")
            return
        self.targets.pop(side, None)
        self.jogs[side] = (joint, direction, None)
        self.desired_modes[side] = ServoCommandType.Request.JOINT_JOG
        self.get_logger().info(f"{side}: Jogging joint {joint}, direction {direction:+d}")

    def jog_tick(self, side):
        joint, direction, start = self.jogs[side]
        now = self.get_clock().now()
        if start is None:
            start = now
            self.jogs[side] = (joint, direction, start)
        elapsed = (now - start).nanoseconds / 1e9
        command = JointJog()
        command.header.stamp = now.to_msg()
        command.joint_names = [f"openarm_{side}_joint{joint}"]
        command.velocities = [direction * 0.2 if elapsed < 0.5 else 0.0]
        self.jog_publishers[side].publish(command)
        if elapsed >= 0.75:
            self.jogs.pop(side)
            self.desired_modes[side] = ServoCommandType.Request.POSE
            self.reset_markers.add(side)

    def reset_marker(self, side):
        try:
            transform = self.buffer.lookup_transform("world", f"{side}_hand_back_tcp", Time()).transform
        except TransformException as exc:
            self.get_logger().warning(f"{side}: Cannot reset marker yet: {exc}", throttle_duration_sec=2.0)
            return
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        self.server.setPose(side, pose)
        self.server.applyChanges()
        self.reset_markers.remove(side)

    def feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        if feedback.marker_name not in self.ready:
            self.get_logger().warning(f"{feedback.marker_name}: Servo is not ready")
            return
        if feedback.marker_name in self.jogs or feedback.marker_name in self.reset_markers:
            return
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose = copy.deepcopy(feedback.pose)
        self.targets[feedback.marker_name] = target

    def tick(self):
        for side in ("left", "right"):
            if self.modes.get(side) != self.desired_modes[side] and side not in self.pending and self.servo_clients[side].service_is_ready():
                request = ServoCommandType.Request()
                request.command_type = self.desired_modes[side]
                future = self.servo_clients[side].call_async(request)
                self.pending[side] = future
                future.add_done_callback(lambda result, side=side, mode=request.command_type:
                                         self.switch_done(side, mode, result))
            if side in self.ready and side not in self.markers:
                try:
                    transform = self.buffer.lookup_transform("world", f"{side}_hand_back_tcp", Time())
                    self.add_marker(side, transform.transform)
                except TransformException:
                    continue
            if side in self.jogs and self.modes.get(side) == ServoCommandType.Request.JOINT_JOG:
                self.jog_tick(side)
            if side in self.reset_markers and self.modes.get(side) == ServoCommandType.Request.POSE:
                self.reset_marker(side)
            if side in self.targets and self.modes.get(side) == ServoCommandType.Request.POSE:
                target = self.targets[side]
                target.header.stamp = self.get_clock().now().to_msg()
                self.pose_publishers[side].publish(target)


def main():
    rclpy.init()
    node = PoseMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
