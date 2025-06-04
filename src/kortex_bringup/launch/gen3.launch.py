# Copyright (c) 2021 PickNik, Inc.
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
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="gen3",
            description="Type/series of robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("dof", default_value="6", description="DoF of robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_group_velocity_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="view_single_robot.rviz",
            description="Rviz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description="Name of the gripper attached to the arm",
            choices=["", "robotiq_2f_85", "robotiq_2f_140"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="robotiq_85_left_knuckle_joint",
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="false",
            description="Use internal bus for gripper communication?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description", # Match default in kortex_control.launch.py
            description="Package containing the robot description (URDF/XACRO files).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the robot.",
        )
    )
    # Initial Pose Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_x", default_value="0.0", description="Initial X position of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_y", default_value="0.0", description="Initial Y position of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_z", default_value="0.0", description="Initial Z position of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_roll", default_value="0.0", description="Initial Roll orientation of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_pitch", default_value="0.0", description="Initial Pitch orientation of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_yaw", default_value="0.0", description="Initial Yaw orientation of the robot base."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo", default_value="false", description="Use Gazebo simulation."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_gazebo_server_client", default_value="false", description="Launch Gazebo server and client."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false", # Or "true" if you want it to be the default
            description="Use Ignition Gazebo (Fortress) simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="",
            description="Path to the Gazebo world file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="false",
            description="Run Gazebo in headless mode (no GUI).",
        )
    )


    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    dof = LaunchConfiguration("dof")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    gripper = LaunchConfiguration("gripper")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_file = LaunchConfiguration("controllers_file")
    rviz_file = LaunchConfiguration("rviz_file")
    description_package = LaunchConfiguration("description_package")
    namespace = LaunchConfiguration("namespace")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    launch_gazebo_server_client = LaunchConfiguration("launch_gazebo_server_client")
    sim_ignition = LaunchConfiguration("sim_ignition")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Initial Pose Arguments
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_roll = LaunchConfiguration("initial_pose_roll")
    initial_pose_pitch = LaunchConfiguration("initial_pose_pitch")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    # Gazebo specific arguments (newly added for sim_gazebo)
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")
    headless_rendering = LaunchConfiguration("headless_rendering")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/kortex_control.launch.py"]),
        launch_arguments={
            "robot_type": robot_type,
            "robot_ip": robot_ip,
            "dof": dof,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "robot_controller": robot_controller,
            "gripper": gripper,
            "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
            "gripper_max_velocity": gripper_max_velocity,
            "gripper_max_force": gripper_max_force,
            "gripper_joint_name": gripper_joint_name,
            "launch_rviz": launch_rviz,
            "controllers_file": controllers_file,
            "description_file": "gen3.xacro",
            "rviz_file": rviz_file,
            "description_package": description_package,
            "namespace": namespace,
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_z": initial_pose_z,
            "initial_pose_roll": initial_pose_roll,
            "initial_pose_pitch": initial_pose_pitch,
            "initial_pose_yaw": initial_pose_yaw,
            "sim_gazebo": sim_gazebo,
            "sim_ignition": sim_ignition, 
            "launch_gazebo_server_client": launch_gazebo_server_client,
            "use_sim_time": use_sim_time,
            "gazebo_world_file": gazebo_world_file,
            "headless_rendering": headless_rendering,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])
