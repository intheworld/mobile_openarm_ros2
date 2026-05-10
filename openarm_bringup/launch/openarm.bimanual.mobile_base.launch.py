# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip("/")
    return None


def _add_inertial(link, mass, ixx, iyy, izz):
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(
        inertial,
        "inertia",
        ixx=str(ixx),
        ixy="0.0",
        ixz="0.0",
        iyy=str(iyy),
        iyz="0.0",
        izz=str(izz),
    )


def _add_box_link(root, name, size, color, mass):
    link = ET.SubElement(root, "link", name=name)
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", size=size)
    material = ET.SubElement(visual, "material", name=f"{name}_material")
    ET.SubElement(material, "color", rgba=color)

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", size=size)
    _add_inertial(link, mass, mass * 0.01, mass * 0.01, mass * 0.01)
    return link


def _add_cylinder_link(root, name, radius, length, color, mass):
    link = ET.SubElement(root, "link", name=name)
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
    material = ET.SubElement(visual, "material", name=f"{name}_material")
    ET.SubElement(material, "color", rgba=color)

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "cylinder", radius=str(radius), length=str(length))
    _add_inertial(
        link,
        mass,
        mass * radius * radius / 4.0,
        mass * radius * radius / 4.0,
        mass * radius * radius / 2.0,
    )
    return link


def _add_offset_cylinder_link(root, name, radius, length, xyz, color, mass):
    link = ET.SubElement(root, "link", name=name)
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz=xyz, rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
    material = ET.SubElement(visual, "material", name=f"{name}_material")
    ET.SubElement(material, "color", rgba=color)

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz=xyz, rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "cylinder", radius=str(radius), length=str(length))
    _add_inertial(
        link,
        mass,
        mass * radius * radius / 4.0,
        mass * radius * radius / 4.0,
        mass * radius * radius / 2.0,
    )
    return link


def _add_support_column(root, base_mount_z):
    mount_z = float(base_mount_z)
    base_height = 0.16
    top_plate_height = 0.05
    lower_column_length = max(mount_z - base_height / 2.0 - top_plate_height / 2.0, 0.05)
    lower_column_center_z = base_height / 2.0 + lower_column_length / 2.0
    upper_column_length = 0.24
    upper_column_center_z = -(top_plate_height / 2.0 + upper_column_length / 2.0)

    _add_cylinder_link(
        root,
        "mobile_base_lower_support_column_link",
        0.085,
        lower_column_length,
        "0.22 0.24 0.26 1.0",
        2.2,
    )
    _add_fixed_joint(
        root,
        "mobile_base_lower_support_column_joint",
        "mobile_base_link",
        "mobile_base_lower_support_column_link",
        f"0 0 {lower_column_center_z}",
    )

    _add_offset_cylinder_link(
        root,
        "mobile_base_upper_support_column_link",
        0.055,
        upper_column_length,
        f"0 0 {upper_column_center_z}",
        "0.38 0.40 0.42 1.0",
        1.4,
    )

    _add_box_link(
        root,
        "mobile_base_mount_link",
        "0.30 0.22 0.05",
        "0.32 0.34 0.36 1.0",
        2.6,
    )
    _add_fixed_joint(
        root,
        "mobile_base_upper_support_column_joint",
        "mobile_base_mount_link",
        "mobile_base_upper_support_column_link",
        "0 0 0",
    )
    _add_prismatic_joint(
        root,
        "mobile_base_lift_joint",
        "mobile_base_link",
        "mobile_base_mount_link",
        f"0 0 {mount_z}",
        "0 0 1",
        "0.0",
        "0.30",
        "1000.0",
        "0.05",
    )


def _add_prismatic_joint(
    root, name, parent, child, xyz, axis, lower, upper, effort, velocity
):
    joint = ET.SubElement(root, "joint", name=name, type="prismatic")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=xyz, rpy="0 0 0")
    ET.SubElement(joint, "axis", xyz=axis)
    ET.SubElement(
        joint,
        "limit",
        lower=lower,
        upper=upper,
        effort=effort,
        velocity=velocity,
    )
    return joint


def _add_fixed_joint(root, name, parent, child, xyz, rpy="0 0 0"):
    joint = ET.SubElement(root, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
    return joint


def _add_continuous_joint(root, name, parent, child, xyz, rpy="0 0 0"):
    joint = ET.SubElement(root, "joint", name=name, type="continuous")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
    ET.SubElement(joint, "axis", xyz="0 0 1")
    return joint


def _add_mobile_base_ros2_control(root, wheel_joint_names):
    ros2_control = ET.SubElement(
        root, "ros2_control", name="mobile_base_hardware", type="system"
    )
    hardware = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    ET.SubElement(hardware, "param", name="fake_sensor_commands").text = "true"
    ET.SubElement(hardware, "param", name="state_following_offset").text = "0.0"

    lift_joint = ET.SubElement(ros2_control, "joint", name="mobile_base_lift_joint")
    ET.SubElement(lift_joint, "command_interface", name="position")
    lift_position = ET.SubElement(lift_joint, "state_interface", name="position")
    ET.SubElement(lift_position, "param", name="initial_value").text = "0.0"
    lift_velocity = ET.SubElement(lift_joint, "state_interface", name="velocity")
    ET.SubElement(lift_velocity, "param", name="initial_value").text = "0.0"

    for joint_name in wheel_joint_names:
        joint = ET.SubElement(ros2_control, "joint", name=joint_name)
        ET.SubElement(joint, "command_interface", name="velocity")
        state_position = ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(state_position, "param", name="initial_value").text = "0.0"
        state_velocity = ET.SubElement(joint, "state_interface", name="velocity")
        ET.SubElement(state_velocity, "param", name="initial_value").text = "0.0"


def add_omni_base(robot_description, base_mount_z):
    root = ET.fromstring(robot_description)

    # Reuse the bimanual model's base_link and body joint, but attach that chain
    # to the mobile base instead of the fixed world frame.
    pedestal_parent = root.find("./joint[@name='world_to_pedestal']/parent")
    if pedestal_parent is not None:
        pedestal_parent.set("link", "mobile_base_mount_link")

    body_parent = root.find("./joint[@name='openarm_body_world_joint']/parent")
    if body_parent is not None:
        body_parent.set("link", "base_link")

    for link in list(root.findall("./link")):
        if link.get("name") == "world":
            root.remove(link)

    _add_box_link(
        root, "mobile_base_link", "0.72 0.52 0.16", "0.12 0.14 0.16 1.0", 18.0
    )
    _add_support_column(root, base_mount_z)

    wheel_positions = {
        "front_left": "0.26 0.22 -0.04",
        "front_right": "0.26 -0.22 -0.04",
        "rear_left": "-0.26 0.22 -0.04",
        "rear_right": "-0.26 -0.22 -0.04",
    }
    roller_rpy = {
        "front_left": "0.7854 0 0",
        "front_right": "-0.7854 0 0",
        "rear_left": "-0.7854 0 0",
        "rear_right": "0.7854 0 0",
    }

    for wheel_name, xyz in wheel_positions.items():
        link_name = f"mobile_base_{wheel_name}_wheel_link"
        joint_name = f"mobile_base_{wheel_name}_wheel_joint"
        _add_cylinder_link(
            root, link_name, 0.075, 0.055, "0.02 0.02 0.02 1.0", 0.8
        )
        _add_continuous_joint(
            root,
            joint_name,
            "mobile_base_link",
            link_name,
            xyz,
            "1.5708 0 0",
        )

        roller_link = f"mobile_base_{wheel_name}_roller_link"
        _add_cylinder_link(
            root, roller_link, 0.018, 0.08, "0.78 0.78 0.72 1.0", 0.15
        )
        _add_fixed_joint(
            root,
            f"mobile_base_{wheel_name}_roller_joint",
            link_name,
            roller_link,
            "0 0 0",
            roller_rpy[wheel_name],
        )

    _add_mobile_base_ros2_control(
        root,
        [
            "mobile_base_front_left_wheel_joint",
            "mobile_base_front_right_wheel_joint",
            "mobile_base_rear_left_wheel_joint",
            "mobile_base_rear_right_wheel_joint",
        ],
    )

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    right_can_interface,
    left_can_interface,
    base_mount_z,
):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    base_mount_z_str = context.perform_substitution(base_mount_z)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        description_file_str,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
        },
    ).toprettyxml(indent="  ")

    return add_omni_base(robot_description, base_mount_z_str)


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    use_fake_hardware,
    controllers_file,
    right_can_interface,
    left_can_interface,
    arm_prefix,
    base_mount_z,
):
    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context,
        description_package,
        description_file,
        arm_type,
        use_fake_hardware,
        right_can_interface,
        left_can_interface,
        base_mount_z,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    if namespace:
        controllers_file_str = controllers_file_str.replace(
            "openarm_v10_bimanual_controllers.yaml",
            "openarm_v10_bimanual_controllers_namespaced.yaml",
        )

    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )

    robot_controller_str = context.perform_substitution(robot_controller)
    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            robot_controller_left,
            robot_controller_right,
            "-c",
            controller_manager_ref,
        ],
    )

    return [robot_controller_spawner]


def controller_manager_ref(context, arm_prefix):
    namespace = namespace_from_context(context, arm_prefix)
    return f"/{namespace}/controller_manager" if namespace else "/controller_manager"


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_bimanual_moveit_config",
            description="Package with the bimanual robot URDF/xacro file.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="config/openarm_bimanual.urdf.xacro",
            description="Bimanual URDF/XACRO description file to extend.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with controller configuration files.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_bimanual_mobile_base_controllers.yaml",
            description="Controllers file to use.",
        ),
        DeclareLaunchArgument(
            "base_mount_z",
            default_value="0.22",
            description="Z offset from mobile_base_link to the OpenArm body mount.",
        ),
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="openarm_description",
            description="Package with RViz configuration.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="rviz/bimanual.rviz",
            description="RViz configuration file relative to rviz_config_package.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    base_mount_z = LaunchConfiguration("base_mount_z")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "v10_controllers",
            controllers_file,
        ]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package,
            description_file,
            arm_type,
            use_fake_hardware,
            controllers_file,
            right_can_interface,
            left_can_interface,
            arm_prefix,
            base_mount_z,
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package), rviz_config_file]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    world_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
    )

    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace_from_context(context, arm_prefix),
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    controller_manager_ref(context, arm_prefix),
                ],
            )
        ]
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller, arm_prefix]
    )

    gripper_controller_spawner = OpaqueFunction(
        function=lambda context: [
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace_from_context(context, arm_prefix),
                arguments=[
                    "left_gripper_controller",
                    "right_gripper_controller",
                    "-c",
                    controller_manager_ref(context, arm_prefix),
                ],
            )
        ]
    )

    mobile_base_controller_spawner = OpaqueFunction(
        function=lambda context: [
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace_from_context(context, arm_prefix),
                arguments=[
                    "mobile_base_controller",
                    "mobile_base_lift_controller",
                    "-c",
                    controller_manager_ref(context, arm_prefix),
                ],
            )
        ]
    )

    launch_delay_seconds = 1.0
    delayed_joint_state_broadcaster = TimerAction(
        period=launch_delay_seconds,
        actions=[joint_state_broadcaster_spawner],
    )
    delayed_robot_controller = TimerAction(
        period=launch_delay_seconds,
        actions=[controller_spawner_func],
    )
    delayed_gripper_controller = TimerAction(
        period=launch_delay_seconds,
        actions=[gripper_controller_spawner],
    )
    delayed_mobile_base_controller = TimerAction(
        period=launch_delay_seconds,
        actions=[mobile_base_controller_spawner],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            rviz_node,
            world_to_odom_tf_node,
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            delayed_gripper_controller,
            delayed_mobile_base_controller,
        ]
    )
