from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- Common Paths ---
    kortex_bringup_pkg_path = get_package_share_directory('kortex_bringup')
    gen3_launch_file = os.path.join(kortex_bringup_pkg_path, 'launch', 'gen3.launch.py')
    barrier_control_pkg_name = 'barrier_control' # Your package name

    # --- Robot 1 Configuration ---
    robot1_namespace = 'robot1'
    robot1_controller_name = 'robot1_controller'
    robot1_target_pose_topic = 'target_pose' # Relative topic name within namespace
    robot1_joint_state_topic = 'joint_states' # Relative topic name within namespace
    robot1_velocity_command_topic = 'joint_group_velocity_controller/commands' # Relative topic name

    # --- Robot 2 Configuration ---
    robot2_namespace = 'robot2'
    robot2_controller_name = 'robot2_controller'
    robot2_target_pose_topic = 'target_pose' # Can be the same relative name
    robot2_joint_state_topic = 'joint_states' # Can be the same relative name
    robot2_velocity_command_topic = 'joint_group_velocity_controller/commands' # Can be the same relative name

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # --- Robot 1 Group ---
    robot1_group = GroupAction(
        actions=[
            # Push the namespace for all nodes/includes within this group
            PushRosNamespace(robot1_namespace),

            # Include kortex_bringup for Robot 1
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gen3_launch_file),
                launch_arguments={
                    'dof': '6',
                    'use_fake_hardware': 'true',
                    'robot_ip': 'dummy1' # Use unique dummy IPs if needed, though often not critical for fake_hardware
                }.items()
            ),

            # Launch Controller for Robot 1
            Node(
                package=barrier_control_pkg_name,
                executable='rigid_body_dynamics_controller',
                name=robot1_controller_name, # Unique node name
                output='screen',
                emulate_tty=True,
                parameters=[{
                    # --- Frame names might need prefixing if not handled by bringup ---
                    'base_frame': 'base_link',
                    'tool_frame': 'end_effector_link',
                    # --- Gains, Limits, etc. (can be customized per robot) ---
                    'kp_gains': [5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
                    'kd_gains': [0.1, 0.1, 0.1, 0.05, 0.05, 0.05],
                    'max_joint_velocities': [1.5, 1.5, 1.5, 2.0, 2.0, 2.0],
                    'control_frequency': 50.0,
                    'error_tolerance': [0.01, 0.01, 0.01, 0.03, 0.03, 0.03],
                    # --- Remapped Topics (relative to namespace) ---
                    'joint_state_topic': robot1_joint_state_topic,
                    'target_pose_topic': robot1_target_pose_topic,
                    'velocity_command_topic': robot1_velocity_command_topic,
                    # --- Robot Description (likely same for both) ---
                    'robot_description_package': 'kortex_description',
                    'robot_description_xacro_path': 'robots/gen3.xacro',
                    'xacro_args': 'dof:=6 use_fake_hardware:=true robot_ip:=dummy1', # Match bringup args
                }],
                # arguments=['--ros-args', '--log-level', f'{robot1_controller_name}:=debug'] # Optional debug
            )
        ]
    )
    ld.add_action(robot1_group)

    # --- Robot 2 Group (Similar structure to Robot 1) ---
    # You would create a similar GroupAction for robot2, changing the namespace,
    # controller name, dummy IP, and potentially any specific parameters if needed.
    # Ensure the topic parameter values point to the correct relative topic names
    # for robot 2 (e.g., robot2_target_pose_topic).
    # Example (add this to ld):
    # robot2_group = GroupAction(...)
    # ld.add_action(robot2_group)

    return ld