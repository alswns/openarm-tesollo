import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def semantic_description(urdf, upstream_srdf):
    robot = ET.fromstring(urdf)
    srdf = ET.fromstring(upstream_srdf)
    srdf.set("name", robot.get("name"))
    links = {link.get("name") for link in robot.findall("link")}
    for element in list(srdf):
        if (element.tag == "group" and element.get("name") in ("left_gripper", "right_gripper")
                or element.tag in ("end_effector", "passive_joint")
                or element.tag == "disable_collisions"
                and (element.get("link1") not in links or element.get("link2") not in links)):
            srdf.remove(element)
    for side in ("left", "right"):
        group = srdf.find(f"group[@name='{side}_arm']")
        if group is None:
            raise ValueError(f"Official SRDF is missing {side}_arm")
        group.clear()
        group.set("name", f"{side}_arm")
        ET.SubElement(group, "chain", base_link=f"openarm_{side}_base_link",
                      tip_link=f"{side}_hand_back_tcp")
        for finger in range(1, 6):
            group = ET.SubElement(srdf, "group", name=f"{side}_finger_{finger}")
            for joint in range(1, 5):
                ET.SubElement(group, "joint", name=f"{side}_hand_joint_{finger}_{joint}")
    disabled = {frozenset((item.get("link1"), item.get("link2")))
                for item in srdf.findall("disable_collisions")}
    for joint in robot.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is not None and child is not None and frozenset((parent.get("link"), child.get("link"))) not in disabled:
            ET.SubElement(srdf, "disable_collisions", link1=parent.attrib["link"],
                          link2=child.attrib["link"], reason="Adjacent")
    group = ET.SubElement(srdf, "group", name="bimanual")
    for side in ("left", "right"):
        ET.SubElement(group, "group", name=f"{side}_arm")
    return ET.tostring(srdf, encoding="unicode")


def moveit_parameters(share, initial_pose):
    config_dir = "config/openarm_v2.0"
    config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(file_path=str(share / "urdf/robot.urdf.xacro"),
                           mappings={"initial_pose": initial_pose})
        .robot_description_semantic(file_path=f"{config_dir}/openarm_bimanual.srdf")
        .robot_description_kinematics(file_path=f"{config_dir}/kinematics.yaml")
        .joint_limits(file_path=f"{config_dir}/joint_limits.yaml")
        .trajectory_execution(file_path=f"{config_dir}/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    description = config.robot_description["robot_description"]
    config.robot_description_semantic["robot_description_semantic"] = semantic_description(
        description, config.robot_description_semantic["robot_description_semantic"])
    for side in ("left", "right"):
        config.robot_description_kinematics["robot_description_kinematics"][f"{side}_arm"]["kinematics_solver_timeout"] = 0.05

    limits = config.joint_limits["robot_description_planning"]["joint_limits"]
    robot_joints = {joint.get("name"): joint for joint in ET.fromstring(description).findall("joint")
                    if joint.get("type") != "fixed"}
    for name in list(limits):
        if name not in robot_joints:
            del limits[name]
    for name, joint in robot_joints.items():
        limit = limits.setdefault(name, {})
        velocity = float(joint.find("limit").get("velocity"))
        limit.update(has_velocity_limits=True, max_velocity=min(limit.get("max_velocity", velocity), velocity),
                     has_acceleration_limits=True, max_acceleration=min(limit.get("max_acceleration", 1.0), 1.0))

    controllers = config.trajectory_execution["moveit_simple_controller_manager"]
    for side in ("left", "right"):
        official = controllers.pop(f"{side}_joint_trajectory_controller")
        controllers[f"{side}_arm_controller"] = official
        controllers.pop(f"{side}_gripper_controller")
    controllers["controller_names"] = ["left_arm_controller", "right_arm_controller"]
    parameters = config.to_dict()
    parameters["trajectory_execution"] = {
        "allowed_execution_duration_scaling": 2.0,
        "allowed_goal_duration_margin": 1.0,
        "allowed_start_tolerance": 0.01,
    }
    return parameters


def setup(context):
    share = Path(get_package_share_directory("openarm_tesollo_moveit"))
    mode = LaunchConfiguration("control_mode").perform(context)
    if mode not in ("planning", "servo", "web"):
        raise ValueError("control_mode must be planning, servo or web")
    robot_parameters = moveit_parameters(share, "web" if mode == "web" else "zero")
    description = robot_parameters["robot_description"]
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
        nodes.append(Node(package="moveit_ros_move_group", executable="move_group",
                          parameters=[robot_parameters], output="screen"))
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
