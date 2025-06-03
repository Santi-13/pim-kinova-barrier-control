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
# Authors: Marq Rasmussen, Denis Stogl

import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")
    gripper = LaunchConfiguration("gripper")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    fault_controller = LaunchConfiguration("fault_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
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

    # if we are using fake hardware then we can't use the internal gripper communications of the hardware
    use_fake_hardware_value = use_fake_hardware.perform(context)
    if use_fake_hardware_value == "true":
        use_internal_bus_gripper_comm = "false"

    # Evaluate controllers_file to check if it's an absolute path
    controllers_file_str = controllers_file.perform(context)
    # Use a consistent variable name for the path to the controllers file
    robot_controllers_path: str
    if os.path.isabs(controllers_file_str):
        robot_controllers_path = controllers_file_str
    else:
        # If not absolute, construct path as before.
        # controllers_file is a LaunchConfiguration, PathJoinSubstitution handles it.
        robot_controllers_path = PathJoinSubstitution(
            [
                FindPackageShare(description_package.perform(context)),
                "arms/" + robot_type.perform(context) + "/" + dof.perform(context) + "dof/config",
                controllers_file, # Pass the LaunchConfiguration object
            ]
        ).perform(context) # Perform substitution to get string path for Node parameter

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", description_file]
            ),
            " ",
            "rviz_file:=",
            rviz_file,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            robot_name,
            " ",
            "namespace:=",
            namespace,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "gripper:=",
            gripper,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "gripper_max_velocity:=",
            gripper_max_velocity,
            " ",
            "gripper_max_force:=",
            gripper_max_force,
            " ",
            "gripper_joint_name:=",
            gripper_joint_name,
            " ",
            "initial_pose_x:=",
            initial_pose_x,
            " ",
            "initial_pose_y:=",
            initial_pose_y,
            " ",
            "initial_pose_z:=",
            initial_pose_z,
            " ",
            "initial_pose_roll:=",
            initial_pose_roll,
            " ",
            "initial_pose_pitch:=",
            initial_pose_pitch,
            " ",
            "initial_pose_yaw:=",
            initial_pose_yaw,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=", 
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers_path,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package.perform(context)), "rviz", rviz_file.perform(context)]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="THISNODE",  # Explicitly name the node
        parameters=[
            robot_controllers_path,
            {"use_sim_time": use_sim_time},
            ], # Use the determined path
            
        remappings=[
            ("~/robot_description", f"{namespace.perform(context)}/robot_description"),
        ],
        # arguments=['--ros-args', '--log-level', 'info'],  
        arguments=['--ros-args', '--log-level', 'KortexMultiInterfaceHardware:=INFO'],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            ],
        remappings=[ # Add remapping for namespaced robot_description
            ("robot_description", PathJoinSubstitution([namespace, "robot_description"])),
        ],
    )

    rviz_node_inst = Node(
        package="rviz2",
        condition=IfCondition(PythonExpression(["'", launch_rviz.perform(context), "' == 'true'"])),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            PathJoinSubstitution([namespace, "controller_manager"]), # Use namespaced controller_manager
        ],
    )   

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, # This spawner is conditional
            on_exit=[rviz_node_inst], # Launch the rviz_node_inst
        ),
        # This event handler is active if:
        # 1. RViz is to be launched
        condition=IfCondition(PythonExpression([
            "'", launch_rviz, "' == 'true'"
        ]))
    )



    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "controller_manager"]
    )

    # robot_pos_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[robot_pos_controller, "--inactive", "-c", "controller_manager"],
    # )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "controller_manager"],
        condition=IfCondition(PythonExpression([
            "'", gripper, "' != ''", # Check gripper is not empty
        ])),
    )


    robotiq_description_prefix = get_package_prefix("robotiq_description")
    gz_robotiq_env_var_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(robotiq_description_prefix, "share")
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 
                   f"{namespace.perform(context)}/robot_description",
                   '-name',
                   robot_name, 
                   '-allow_renaming', 
                   'true'
                   # Adding initial pose arguments for Ignition as well for consistency
                   '-x', initial_pose_x,
                   '-y', initial_pose_y,
                   '-z', initial_pose_z,
                   '-R', initial_pose_roll,
                   '-P', initial_pose_pitch,
                   '-Y', initial_pose_yaw,
                   ],

        condition=IfCondition(sim_ignition),
    )



    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # only start the fault controller if we are using hardware
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[fault_controller, "-c", "controller_manager"],
        condition=IfCondition(use_internal_bus_gripper_comm),

    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        
        # robot_pos_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner, # Handles delayed RViz

    ]
    
    if use_fake_hardware.perform(context) == "false":
        nodes_to_start.append(joint_state_broadcaster_spawner)
        nodes_to_start.append(robot_traj_controller_spawner)
    nodes_to_start.append(robot_hand_controller_spawner) # It has its own IfCondition
    nodes_to_start.append(fault_controller_spawner)      # It has its own IfCondition
    

    sim_gazebo_value = sim_gazebo.perform(context)
    sim_ignition_value = sim_ignition.perform(context)

    if sim_ignition_value == "true":
        pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

        # Construct a valid Python expression string that, when evaluated,
        # gives the desired gz_args string.
        gz_args_expression = PythonExpression([
            "'-r -v 1 '",                                      # Python string literal for Gazebo args part 1
            " + ' ' + str('", gazebo_world_file, "')",         # Add space and the world file (ensuring it's a string in Python)
            " + (' --headless-rendering' if str('", headless_rendering, "').lower() == 'true' else '')" # Conditional headless part
        ])


        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]
            ),
            launch_arguments=[('gz_args', [' -r -v 1 ', gazebo_world_file])],
        )  
        nodes_to_start.append(gazebo_launch)
        nodes_to_start.append(gz_spawn_entity)

        nodes_to_start.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner], # JSB is already conditional
            )
        ))


        # Determine actions to run after joint_state_broadcaster_spawner exits
        on_exit_actions_after_jsb = [robot_traj_controller_spawner] # Already conditional
        # robot_hand_controller_spawner is also already conditional
        # We only add it to on_exit if its other primary condition (gripper exists) is also met.
        if gripper.perform(context) != '': 
                on_exit_actions_after_jsb.append(robot_hand_controller_spawner)

        nodes_to_start.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner, # JSB is already conditional
                on_exit=on_exit_actions_after_jsb,
            )
        ))

        nodes_to_start.append(gz_robotiq_env_var_resource_path)
        nodes_to_start.append(bridge)

    # start_robot_hand_controller = gripper.perform(context) != ""
    # # Conditionally add robot_hand_controller_spawner
    # if start_robot_hand_controller:
    #     nodes_to_start.append(robot_hand_controller_spawner)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type", description="Type/series of robot.", choices=["gen3", "gen3_lite"]
        )
    )
    declared_arguments.append(DeclareLaunchArgument("dof", description="DoF of robot."))
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "username", description="Robot session username.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "password", description="Robot session password.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port", description="Robot port for tcp connection.", default_value="10000"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_realtime",
            description="Robot port for udp realtime control.",
            default_value="10001",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "session_inactivity_timeout_ms",
            description="Robot session inactivity timeout in milliseconds.",
            default_value="60000",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "connection_inactivity_timeout_ms",
            description="Robot connection inactivity timeout in milliseconds.",
            default_value="2000",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kinova.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
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
            "robot_name",
            default_value="arm",
            description="Name of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value='""',
            description="namespace for the robot controller manager",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description="Name of the gripper attached to the arm",
        )
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

    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_controller",
    #         default_value="joint_trajectory_controller",
    #         description="Robot controller to start.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_pos_controller",
    #         default_value="twist_controller",
    #         description="Robot controller to start.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_hand_controller",
    #         default_value="robotiq_gripper_controller",
    #         description="Robot hand controller to start.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fault_controller",
            default_value="fault_controller",
            description="Name of the 'fault controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
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
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="finger_joint",
            description="Max force for gripper commands",
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
            "gazebo_world_file",
            default_value="empty.world", 
            description="Path to the Gazebo world file to load (e.g., 'src/my_package/worlds/my_world.sdf'). Relative paths are resolved from where launch is run.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="false",
            description="Enable headless rendering for Gazebo simulation (gz_args '--headless-rendering').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition", default_value="false", description="Use Ignition Gazebo (Fortress) simulation."
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
            "vision",
            default_value="false",
            description="Use vision.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
                             )
