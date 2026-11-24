import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def semantic_description(urdf):
    robot = ET.fromstring(urdf)
    srdf = ET.Element("robot", name="openarm_tesollo")
    for side in ("left", "right"):
        group = ET.SubElement(srdf, "group", name=f"{side}_arm")
        ET.SubElement(group, "chain", base_link=f"openarm_{side}_base_link",
                      tip_link=f"{side}_hand_back_tcp")
        for finger in range(1, 6):
            group = ET.SubElement(srdf, "group", name=f"{side}_finger_{finger}")
            for joint in range(1, 5):
                ET.SubElement(group, "joint", name=f"{side}_hand_joint_{finger}_{joint}")
    for joint in robot.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is not None and child is not None:
            ET.SubElement(srdf, "disable_collisions", link1=parent.attrib["link"],
                          link2=child.attrib["link"], reason="Adjacent")
    group = ET.SubElement(srdf, "group", name="bimanual")
    for side in ("left", "right"):
        ET.SubElement(group, "group", name=f"{side}_arm")
    return ET.tostring(srdf, encoding="unicode")


def setup(context):
    share = Path(get_package_share_directory("openarm_tesollo_moveit"))
    mode = LaunchConfiguration("control_mode").perform(context)
    if mode not in ("planning", "servo", "web"):
        raise ValueError("control_mode must be planning, servo or web")
    description = xacro.process_file(
        str(share / "urdf/robot.urdf.xacro"),
        mappings={"initial_pose": "web" if mode == "web" else "zero"},
    ).toxml()
    with (share / "config/kinematics.yaml").open() as stream:
        kinematics = yaml.safe_load(stream)
    robot_parameters = {
        "robot_description": description,
        "robot_description_semantic": semantic_description(description),
        "robot_description_kinematics": kinematics,
        "robot_description_planning": {"joint_limits": {
            joint.get("name"): {
                "has_velocity_limits": True,
                "max_velocity": float(joint.find("limit").get("velocity")),
                "has_acceleration_limits": True,
                "max_acceleration": 1.0,
            }
            for joint in ET.fromstring(description).findall("joint") if joint.get("type") != "fixed"
        }},
    }
    rviz_config = {"planning": "planning.rviz", "servo": "rviz.rviz", "web": "web.rviz"}[mode]
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[str(share / "config/controllers.yaml")], output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(share / "config" / rviz_config)],
             parameters=[robot_parameters], output="screen"),
    ]
    for controller in ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                       "left_hand_controller", "right_hand_controller"):
        nodes.append(Node(package="controller_manager", executable="spawner",
                          arguments=[controller, "--controller-manager", "/controller_manager",
                                     "--controller-manager-timeout", "120"], output="screen"))
    if mode == "planning":
        with (share / "config/planning.yaml").open() as stream:
            planning = yaml.safe_load(stream)
        nodes.append(Node(package="moveit_ros_move_group", executable="move_group",
                          parameters=[robot_parameters, planning], output="screen"))
        return nodes
    nodes.append(Node(package="openarm_tesollo_moveit",
                      executable="web_pose_bridge.py" if mode == "web" else "pose_markers.py",
                      parameters=[{"robot_description": description}] if mode == "web" else [],
                      output="screen"))
    for side in ("left", "right"):
        servo = {
            "moveit_servo": {
                "move_group_name": f"{side}_arm",
                "command_in_type": "speed_units",
                "command_out_topic": f"/{side}_arm_controller/joint_trajectory",
                "is_primary_planning_scene_monitor": side == "left",
                "scale": {"linear": 1.2, "rotational": 1.2} if mode == "web" else {"linear": 0.4, "rotational": 0.8},
                "joint_limit_margins": [0.01],
                "incoming_command_timeout": 0.25,
                "check_collisions": False,
                "publish_joint_positions": True,
                "publish_joint_velocities": False,
            }
        }
        nodes.append(Node(package="moveit_servo", executable="servo_node",
                          name=f"{side}_servo", parameters=[robot_parameters, servo], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="planning"),
        OpaqueFunction(function=setup),
    ])
