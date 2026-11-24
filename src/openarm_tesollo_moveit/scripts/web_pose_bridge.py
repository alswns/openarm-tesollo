#!/usr/bin/env python3
"""Loopback-only web pose input for RViz mock MoveIt Servo."""
import copy
import json
import math
import mimetypes
import secrets
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, urljoin

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def validate_target(data):
    if not isinstance(data, dict) or data.get("side") not in ("left", "right"):
        raise ValueError("Select left or right arm")
    values = []
    for field, size in (("position", 3), ("orientation", 4)):
        vector = data.get(field)
        if not isinstance(vector, list) or len(vector) != size:
            raise ValueError(f"{field} must contain {size} numbers")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in vector):
            raise ValueError(f"{field} must contain finite numbers")
        values.append([float(v) for v in vector])
    position, orientation = values
    if any(abs(v) > 3.0 for v in position):
        raise ValueError("Position must be within +/-3 m of world origin")
    if abs(sum(v * v for v in orientation) - 1.0) > 0.01:
        raise ValueError("Orientation must be a unit quaternion in x,y,z,w order")
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = orientation
    return data["side"], pose


def browser_scene(description):
    root = ET.fromstring(description)
    camera_frame = "openarm_body_zed_left_camera_frame_optical"
    if root.find(f"link[@name='{camera_frame}']") is None:
        raise ValueError("Robot description must include the OpenArm ZED camera")
    assets = {}
    urls = {}
    for mesh in root.findall(".//visual/geometry/mesh"):
        uri = mesh.get("filename")
        if not uri or not uri.startswith("package://"):
            raise ValueError(f"Unsupported mesh URI: {uri}")
        package, relative = uri[len("package://"):].split("/", 1)
        share = Path(get_package_share_directory(package)).resolve()
        resource = Path(relative)
        if resource.is_absolute() or ".." in resource.parts:
            raise ValueError(f"Mesh path escapes its package: {uri}")
        path = (share / resource).resolve()
        if path.suffix.lower() not in (".dae", ".stl") or not path.is_file():
            raise ValueError(f"Missing or unsupported package mesh: {uri}")
        if path not in urls:
            url = f"/assets/{len(urls)}/{quote(path.name)}"
            urls[path] = url
            assets[url] = path
            if path.suffix.lower() == ".dae":
                dae = ET.parse(path)
                for image in dae.findall(".//{*}image/{*}init_from"):
                    name = (image.text or "").strip()
                    texture = (path.parent / name).resolve()
                    if not name or "://" in name or not texture.is_relative_to(path.parent) or not texture.is_file():
                        raise ValueError(f"Unsupported or missing Collada texture: {name}")
                    assets[urljoin(url, quote(name, safe="/"))] = texture
        mesh.set("filename", urls[path])
    return {
        "urdf": ET.tostring(root, encoding="unicode"),
        "camera_frame": camera_frame,
        "camera_link": "openarm_body_zed_camera_link",
    }, assets


class WebPoseBridge(Node):
    def __init__(self):
        super().__init__("web_pose_bridge")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.token = secrets.token_urlsafe(32)
        self.target = None
        self.received = 0.0
        self.ready = set()
        self.pending = set()
        self.state = {"arms": {}, "active": False, "joints": {}}
        description = self.declare_parameter("robot_description", "").value
        if not description:
            raise ValueError("web_pose_bridge requires robot_description")
        scene, assets = browser_scene(description)
        self.create_subscription(JointState, "/joint_states", self.joint_state, 10)
        self.publishers_by_side = {
            side: self.create_publisher(PoseStamped, f"/{side}_servo/pose_target_cmds", 10)
            for side in ("left", "right")
        }
        self.clients_by_side = {
            side: self.create_client(ServoCommandType, f"/{side}_servo/switch_command_type")
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.create_subscription(
                ServoStatus, f"/{side}_servo/status",
                lambda msg, side=side: self.status(side, msg), 10,
            )
        web = Path(get_package_share_directory("openarm_tesollo_moveit")) / "web"
        html = (web / "index.html").read_bytes()
        camera_script = (web / "dist/camera-view.js").read_bytes()
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def respond(self, code, data, content_type="application/json"):
                body = json.dumps(data, allow_nan=False).encode() if content_type == "application/json" else data
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    bridge.get_logger().warning("Browser disconnected during HTTP response", throttle_duration_sec=2.0)

            def valid_host(self):
                return self.headers.get("Host") in ("127.0.0.1:8765", "localhost:8765")

            def do_GET(self):
                if not self.valid_host():
                    self.respond(403, {"error": "Loopback host required"})
                elif self.path == "/":
                    self.respond(200, html, "text/html; charset=utf-8")
                elif self.path == "/api/state":
                    with bridge.lock:
                        snapshot = copy.deepcopy(bridge.state)
                    snapshot["token"] = bridge.token
                    self.respond(200, snapshot)
                elif self.path == "/api/scene":
                    self.respond(200, scene)
                elif self.path == "/camera-view.js":
                    self.respond(200, camera_script, "text/javascript; charset=utf-8")
                elif self.path in assets:
                    path = assets[self.path]
                    content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
                    self.respond(200, path.read_bytes(), content_type)
                else:
                    self.respond(404, {"error": "Not found"})

            def do_POST(self):
                origin = self.headers.get("Origin")
                if (not self.valid_host() or self.headers.get("X-Control-Token") != bridge.token
                        or origin not in (None, "http://127.0.0.1:8765", "http://localhost:8765")):
                    self.respond(403, {"error": "Control token and same-origin request required"})
                    return
                if self.path == "/api/stop":
                    with bridge.lock:
                        bridge.target = None
                        bridge.state["active"] = False
                    self.respond(200, {"ok": True})
                    return
                if self.path != "/api/target":
                    self.respond(404, {"error": "Not found"})
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 2048:
                        raise ValueError("Expected a JSON body up to 2048 bytes")
                    data = json.loads(self.rfile.read(size))
                    side, pose = validate_target(data)
                    with bridge.lock:
                        actual = bridge.state["arms"].get(side, {})
                        if not actual.get("ready") or "position" not in actual:
                            raise ValueError("Servo or robot state is not ready")
                        position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                        if math.dist(position, actual["position"]) > 3.5:
                            raise ValueError("Target is more than 3.5 m from the current hand-back TCP")
                        bridge.target = (side, pose)
                        bridge.received = time.monotonic()
                except (ValueError, TypeError, UnicodeDecodeError) as exc:
                    bridge.get_logger().warning(f"Rejected web target: {exc}", throttle_duration_sec=2.0)
                    self.respond(400, {"error": str(exc)})
                    return
                self.respond(200, {"ok": True})

            def log_message(self, format, *args):
                pass

        self.http = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
        self.http.daemon_threads = True
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        self.create_timer(0.02, self.publish_target)
        self.create_timer(0.1, self.update_state)
        self.get_logger().info("Mock-only web pose control: http://127.0.0.1:8765")

    def status(self, side, message):
        with self.lock:
            arm = self.state["arms"].setdefault(side, {})
            arm["status"] = {"code": message.code, "message": message.message}

    def joint_state(self, message):
        if len(message.name) != len(message.position) or any(not math.isfinite(v) for v in message.position):
            self.get_logger().error("Invalid joint state received for camera view")
            return
        with self.lock:
            self.state["joints"].update(zip(message.name, message.position))

    def mode_done(self, side, future):
        self.pending.discard(side)
        try:
            if future.result().success:
                self.ready.add(side)
            else:
                self.get_logger().error(f"{side}: Servo rejected POSE mode")
        except Exception as exc:
            self.get_logger().error(f"{side}: Servo mode switch failed: {exc}")

    def update_state(self):
        for side in ("left", "right"):
            if side not in self.ready and side not in self.pending and self.clients_by_side[side].service_is_ready():
                request = ServoCommandType.Request()
                request.command_type = ServoCommandType.Request.POSE
                self.pending.add(side)
                future = self.clients_by_side[side].call_async(request)
                future.add_done_callback(lambda result, side=side: self.mode_done(side, result))
            try:
                transform = self.buffer.lookup_transform("world", f"{side}_hand_back_tcp", Time()).transform
            except TransformException:
                continue
            with self.lock:
                arm = self.state["arms"].setdefault(side, {})
                arm["position"] = [transform.translation.x, transform.translation.y, transform.translation.z]
                arm["orientation"] = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
                arm["tip_frame"] = f"{side}_hand_back_tcp"
                arm["ready"] = side in self.ready

    def publish_target(self):
        with self.lock:
            active = self.target is not None and time.monotonic() - self.received < 0.25
            self.state["active"] = active
            target = copy.deepcopy(self.target) if active else None
            if not active:
                self.target = None
        if target is not None:
            side, pose = target
            pose.header.stamp = self.get_clock().now().to_msg()
            self.publishers_by_side[side].publish(pose)

    def close(self):
        self.http.shutdown()
        self.http.server_close()
        self.thread.join(timeout=2)


def main():
    rclpy.init()
    node = WebPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
