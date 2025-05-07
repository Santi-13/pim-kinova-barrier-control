from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Path to the launch file to include
    kortex_bringup_pkg_path = get_package_share_directory('kortex_bringup')
    gen3_launch_file = os.path.join(kortex_bringup_pkg_path, 'launch', 'gen3.launch.py')

    return LaunchDescription([
        # --- Include the kortex_bringup launch file ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gen3_launch_file),
            launch_arguments={
                'dof': '6',
                'use_fake_hardware': 'true',
                'robot_ip': 'dummy'
            }.items() # Pass arguments as key-value pairs
        ),
        Node(
            package='barrier_control',
            executable='rigid_body_dynamics_controller',
            name='rigid_body_dynamics_controller',
            output='screen',
            emulate_tty=True, 
            parameters=[{
                # --- Frame Names ---
                'base_frame': 'base_link',
                'tool_frame': 'end_effector_link', # Adjust if your tool frame is different
                # --- Controller Gains & Settings ---
                # Kp gains [x, y, z, rx, ry, rz] - Tune these values!
                'kp_gains': [0.1] * 6, # Reduced for slower movement
                'kd_gains': [0.005]*3 + [0.005] * 3,
                'jacobian_damping': 0.05, # Damping for pseudo-inverse singularity robustness (increased slightly)
                'euler_input_convention': 'quat', # Convention for interpreting target orientation if given as Euler: 'xyz', 'zyx', etc.
                # --- Joint Specific Settings ---
                # IMPORTANT: List joint names in the order your robot model (and RTB) expects them.
                'controlled_joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                # Max joint velocities (rad/s) - Set based on your robot's limits!
                # Example for a 6DOF arm, replace with actual values
                'max_joint_velocities': [0.1] * 6, 
                # --- Control Settings & Topic Names ---
                'joint_limit_buffer': 0.01, # Small buffer (rad or m) for joint position limits
                'control_frequency': 20.0, # Control loop frequency in Hz
                'error_tolerance': [0.05] * 3 + [0.1]*3, # Pos [m], Orient [rad]
                'joint_state_topic': '/joint_states',
                'target_pose_topic': '/target_pose',
                'velocity_command_topic': '/joint_group_velocity_controller/commands', # Check this matches your ros2_control setup
                # --- Robot Description ---
                'robot_description_package': 'kortex_description', # Your robot description package
                'robot_description_xacro_path': 'robots/gen3.xacro', # Relative path within the package
                # Arguments passed to xacro - adjust as needed for your URDF/XACRO
                'xacro_args': 'dof:=6 use_fake_hardware:=true robot_ip:=dummy',
            }],
            # --- Optional: Set Log Level ---
            arguments=['--ros-args', '--log-level', 'rigid_body_dynamics_controller:=debug']
        )
    ])
