from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Declare arguments for this top-level launch file ---
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz_robot1",
            default_value="true", # Set to "false" if you don't want RViz by default for robot1
            description="Launch RViz for Robot 1."
        )
    )
    # If you add robot2, declare a similar argument for it:
    # declared_arguments.append(
    #     DeclareLaunchArgument("launch_rviz_robot2", default_value="true", description="Launch RViz for Robot 2.")
    # )
    # --- Common Paths ---
    kortex_bringup_pkg_path = get_package_share_directory('kortex_bringup')
    gen3_launch_file = os.path.join(kortex_bringup_pkg_path, 'launch', 'gen3.launch.py')
    barrier_control_pkg_name = 'barrier_control'

    # --- Robot 1 Configuration ---
    robot1_namespace = 'robot1'
    robot1_controller_name = 'robot1_controller'
    robot1_controller_file = 'robot1_controllers.yaml'
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
    ld = LaunchDescription(declared_arguments)

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
                    'robot_ip': 'dummy1', # Use unique dummy IPs if needed, though often not critical for fake_hardware
                    'prefix': f"{robot1_namespace}/", # Pass the prefix argument with a trailing slash
                    'namespace': f"/{robot1_namespace}", # Namespace for the robot
                    # Initial pose for Robot 1 (e.g., 0.5m to the left on Y-axis)
                    'initial_pose_x': '0.0',
                    'initial_pose_y': '-0.5',
                    'initial_pose_z': '0.0',
                    'initial_pose_roll': '0.0',
                    'initial_pose_pitch': '0.0',
                    'initial_pose_yaw': '0.0',
                    'controllers_file': robot1_controller_file,
                    'robot_controller': 'joint_group_velocity_controller', # Specify this controller
                    
                    'launch_rviz': LaunchConfiguration("launch_rviz_robot1"), # Pass the declared argument

                }.items()
            ),

            # Launch Controller for Robot 1
            # Node(
            #     package=barrier_control_pkg_name,
            #     executable='rigid_body_dynamics_controller',
            #     name=robot1_controller_name, # Unique node name
            #     output='screen',
            #     emulate_tty=True,
            #     parameters=[{
            #         # --- Frame names might need prefixing if not handled by bringup ---
            #         'base_frame': f"{robot1_namespace}/base_link", # Add prefix to frame names
            #         'tool_frame': f"{robot1_namespace}/end_effector_link", # Add prefix to frame names
            #         # --- Gains, Limits, etc. (can be customized per robot) ---
            #         'kp_gains': [5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            #         'kd_gains': [0.1, 0.1, 0.1, 0.05, 0.05, 0.05],
            #         'max_joint_velocities': [1.5, 1.5, 1.5, 2.0, 2.0, 2.0],
            #         'control_frequency': 50.0,
            #         'error_tolerance': [0.01, 0.01, 0.01, 0.03, 0.03, 0.03],
            #         # --- Remapped Topics (relative to namespace) ---
            #         'joint_state_topic': robot1_joint_state_topic,
            #         'target_pose_topic': robot1_target_pose_topic,
            #         'velocity_command_topic': robot1_velocity_command_topic,
            #         # --- Robot Description (likely same for both) ---
            #         'robot_description_package': 'kortex_description',
            #         'robot_description_xacro_path': 'robots/gen3.xacro', # Relative path within the package
            #         'xacro_args': f"dof:=6 use_fake_hardware:=true robot_ip:=dummy1 prefix:={robot1_namespace}/",
            #     }],
            #     arguments=['--ros-args', '--log-level', f'{robot1_namespace}.{robot1_controller_name}:=debug'] # Optional debug with namespace
            # )
            Node(
                package=barrier_control_pkg_name,
                executable='rigid_body_dynamics_controller',
                name=robot1_controller_name,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    # --- Frame Names ---
                    'base_frame': f"{robot1_namespace}/base_link",
                    'tool_frame': f"{robot1_namespace}/end_effector_link",
                    # --- Controller Gains & Settings ---
                    # Kp gains [x, y, z, rx, ry, rz] - Tune these values!
                    'kp_gains': [0.1] * 6, # Reduced for slower movement
                    'kd_gains': [0.005]*3 + [0.005] * 3,
                    'jacobian_damping': 0.05, # Damping for pseudo-inverse singularity robustness (increased slightly)
                    'euler_input_convention': 'quat', # Convention for interpreting target orientation if given as Euler: 'xyz', 'zyx', etc.
                    # --- Joint Specific Settings ---
                    # IMPORTANT: List joint names in the order your robot model (and RTB) expects them.
                    'controlled_joint_names': [f'{robot1_namespace}/joint_1', f'{robot1_namespace}/joint_2', f'{robot1_namespace}/joint_3', f'{robot1_namespace}/joint_4', f'{robot1_namespace}/joint_5', f'{robot1_namespace}/joint_6'],
                    # Max joint velocities (rad/s) - Set based on your robot's limits!
                    # Example for a 6DOF arm, replace with actual values
                    'max_joint_velocities': [0.1] * 6, 
                    # --- Control Settings & Topic Names ---
                    'joint_limit_buffer': 0.01, # Small buffer (rad or m) for joint position limits
                    'control_frequency': 20.0, # Control loop frequency in Hz
                    'error_tolerance': [0.05] * 3 + [0.1]*3, # Pos [m], Orient [rad]
                    'joint_state_topic': robot1_joint_state_topic,
                    'target_pose_topic': robot1_target_pose_topic,
                    'velocity_command_topic': robot1_velocity_command_topic,
                    # --- Robot Description ---
                    'robot_description_package': 'kortex_description', # Your robot description package
                    'robot_description_xacro_path': 'robots/gen3.xacro', # Relative path within the package
                    # Arguments passed to xacro - adjust as needed for your URDF/XACRO
                    'xacro_args': f"dof:=6 use_fake_hardware:=true robot_ip:=dummy1 prefix:={robot1_namespace}/ initial_pose_y:=-0.5", # Also pass to controller if it loads its own model
                }],
                arguments=['--ros-args', '--log-level', 'rigid_body_dynamics_controller:=debug']
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