#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""Brings up a single Franka arm (no teleop pairing) running crisp_controllers'
CartesianController, patched with the proposal §4.3 log-space stiffness rate limiter +
energy tank -- see franka_ros2_teleop/README.md ("Variable-impedance controller") and
tasks.md Day 5.

Defaults to `use_fake_hardware:=true` (no physical robot needed, via ros2_control's
mock_components/GenericSystem): this validates the wiring -- controller activation, topic
names, parameter nesting, the stiffness-shaping pipeline's own numerics -- but NOT real
physical dynamics, since fake hardware does not integrate torque commands into motion.
Pass `use_fake_hardware:=false robot_ip:=<ip>` for a real-hardware run once one is available;
nothing else in this launch file or the controller config needs to change.

Run this first, then in a second terminal run the probe script:
    ros2 run franka_ros2_teleop probe_variable_impedance_sinusoid.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    controllers_yaml = LaunchConfiguration("controllers_yaml")

    single_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("franka_ros2_teleop"), "launch",
                "teleop_single_robot.launch.py",
            ])
        ),
        launch_arguments={
            "arm_id": LaunchConfiguration("arm_id"),
            "arm_prefix": LaunchConfiguration("arm_prefix"),
            "namespace": namespace,
            "urdf_file": LaunchConfiguration("urdf_file"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "load_gripper": "false",  # end_effector_frame is fr3_link8; no gripper needed
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "fake_sensor_commands": LaunchConfiguration("use_fake_hardware"),
            "controllers_yaml": controllers_yaml,
        }.items(),
    )

    # NOT spawning joint_state_broadcaster here: teleop_single_robot.launch.py's own
    # generate_robot_nodes() already spawns it unconditionally (and, on real hardware,
    # franka_robot_state_broadcaster too -- see variable_impedance_validation_controllers.yaml
    # for why that controller is defined in this package's own config despite crisp_controllers
    # not needing it). A second spawn attempt here raced with that one and failed outright on
    # a real-hardware run (see tasks.md Day 5) -- confirmed empirically, not just reasoned
    # about after the fact.

    pose_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["pose_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )

    variable_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "variable_impedance_controller", "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("arm_id", default_value="", description="ID of the type of arm"),
        DeclareLaunchArgument("arm_prefix", default_value="", description="Prefix for arm topics"),
        DeclareLaunchArgument(
            "namespace", default_value="follower",
            description="Namespace for the arm under validation"
        ),
        DeclareLaunchArgument(
            "urdf_file", default_value="fr3/fr3.urdf.xacro", description="Path to URDF file"
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.101.2",
            description="Hostname or IP address of the robot (ignored when use_fake_hardware)"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description=(
                "Use fake hardware (mock_components/GenericSystem) -- no physical "
                "robot needed, but no real dynamics either. Set false for a real run."
            ),
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=PathJoinSubstitution([
                FindPackageShare("franka_ros2_teleop"), "config",
                "variable_impedance_validation_controllers.yaml",
            ]),
            description=(
                "Override for debugging (e.g. a scratch copy with different "
                "stiffness_shaping params); defaults to this package's own config."
            ),
        ),
        single_robot,
        pose_broadcaster_spawner,
        variable_impedance_controller_spawner,
    ])
