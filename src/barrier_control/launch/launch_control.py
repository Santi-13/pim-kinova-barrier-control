from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # --- Declare the package name ---
    pkg_name = 'barrier_control'
    pkg_path = get_package_share_directory(pkg_name)

    # --- Declare arguments for this launch file ---
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false", # Set to "true" to run with Gazebo
            description="Launch in Gazebo simulation."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=f"{pkg_path}/worlds/my.world", # Path to a specific world file, e.g., $(find my_pkg)/worlds/my.world
            description="Path to the Gazebo world file. If empty, Gazebo's default empty world is used."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_x", default_value="0.0", description="Initial X position for the robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_y", default_value="0.0", description="Initial Y position for the robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_z", default_value="0.0", description="Initial Z position for the robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_roll", default_value="0.0", description="Initial Roll orientation for the robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_pitch", default_value="0.0", description="Initial Pitch orientation for the robot.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0", description="Initial Yaw orientation for the robot.")
    )

    # --- Launch Configurations ---
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    launch_rviz = LaunchConfiguration('launch_rviz')
    world = LaunchConfiguration('world')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_roll = LaunchConfiguration('initial_pose_roll')
    initial_pose_pitch = LaunchConfiguration('initial_pose_pitch')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # Path to the launch file to include
    kortex_bringup_pkg_path = get_package_share_directory('kortex_bringup')
    gen3_launch_file = os.path.join(kortex_bringup_pkg_path, 'launch', 'gen3.launch.py')

    ld = LaunchDescription(declared_arguments)

    # --- Include the kortex_bringup launch file ---
    # It will set up robot_state_publisher, ros2_control_node, spawners, etc.
    kortex_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gen3_launch_file),
        launch_arguments={
            'dof': '6',
            'use_fake_hardware': 'true', # Keep true for sim; Gazebo uses gazebo_ros2_control plugin
            'robot_ip': 'dummy',
            'robot_controller': 'joint_group_velocity_controller',
            'launch_rviz': launch_rviz,
            'sim_gazebo': sim_gazebo,
            # Tell gen3.launch.py not to start Gazebo if we are starting it here
            'launch_gazebo_server_client': PythonExpression(["'false' if '", sim_gazebo, "' == 'true' else 'true'"]),
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_z': initial_pose_z,
            'initial_pose_roll': initial_pose_roll,
            'initial_pose_pitch': initial_pose_pitch,
            'initial_pose_yaw': initial_pose_yaw,
        }.items()
    )
    ld.add_action(kortex_bringup_include)

    # --- Rigid Body Dynamics Controller Node ---
    # This is your custom controller
    rigid_body_controller_node = Node(
        package='barrier_control',
        executable='rigid_body_dynamics_controller',
        name='rigid_body_dynamics_controller', # Node name
        output='screen',
        emulate_tty=True,
        parameters=[{
            # --- Frame Names ---
            'base_frame': 'base_link', # Assuming no prefix for a single robot
            'tool_frame': 'end_effector_link',
            # --- Controller Gains & Settings ---
            'kp_gains': [0.1] * 6,
            'kd_gains': [0.005]*3 + [0.005] * 3,
            'jacobian_damping': 0.05,
            'euler_input_convention': 'quat',
            # --- Joint Specific Settings ---
            'controlled_joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            'max_joint_velocities': [0.1] * 6,
            # --- Control Settings & Topic Names ---
            'joint_limit_buffer': 0.01,
            'control_frequency': 20.0,
            'error_tolerance': [0.05] * 3 + [0.1]*3,
            'joint_state_topic': '/joint_states', # Global topic
            'target_pose_topic': '/target_pose',  # Global topic
            'velocity_command_topic': '/joint_group_velocity_controller/commands', # Global topic
            # --- Robot Description (for this node's internal model) ---
            'robot_description_package': 'kortex_description',
            'robot_description_xacro_path': 'robots/gen3.xacro',
            # Arguments passed to xacro - ensure consistency with Gazebo's URDF
            'xacro_args': PythonExpression([
                "'dof:=6 use_fake_hardware:=true robot_ip:=dummy sim_gazebo:=", sim_gazebo,
                " initial_pose_x:=", initial_pose_x,
                " initial_pose_y:=", initial_pose_y,
                " initial_pose_z:=", initial_pose_z,
                " initial_pose_roll:=", initial_pose_roll,
                " initial_pose_pitch:=", initial_pose_pitch,
                " initial_pose_yaw:=", initial_pose_yaw,
                "'"
            ]),
        }],
        arguments=['--ros-args', '--log-level', 'rigid_body_dynamics_controller:=debug']
    )
    ld.add_action(rigid_body_controller_node)

    # --- Gazebo Simulation ---
    # Conditionally launch Gazebo server and client
    def include_gazebo_if_enabled(context):
        if LaunchConfiguration("sim_gazebo").perform(context) == "true":
            LogInfo(msg="sim_gazebo is true, including Gazebo using gazebo_ros/launch/gazebo.launch.py.")
            gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
            gazebo_launch_file = os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')

            world_path_str = LaunchConfiguration("world").perform(context)
            gazebo_launch_args = {'verbose': 'true'}
            if world_path_str: # Only add 'world' argument if a non-empty path is provided
                # Check if world_path_str is a package-relative path or absolute
                if not os.path.isabs(world_path_str) and '/' in world_path_str:
                    # Assume it's 'package_name/path/to/world'
                    try:
                        pkg_name, rel_path = world_path_str.split('/', 1)
                        abs_world_path = os.path.join(get_package_share_directory(pkg_name), rel_path)
                        gazebo_launch_args['world'] = abs_world_path
                    except Exception as e:
                        LogInfo(msg=f"Could not resolve world path '{world_path_str}' relative to a package: {e}. Trying as absolute.")
                        if os.path.isabs(world_path_str): # If it was already absolute or became one
                             gazebo_launch_args['world'] = world_path_str
                elif os.path.isabs(world_path_str):
                     gazebo_launch_args['world'] = world_path_str
                # If it's just a filename, Gazebo will search its resource paths.
                # If it's empty, Gazebo loads its default empty world.
                elif world_path_str: # It's just a filename
                    gazebo_launch_args['world'] = world_path_str

            gazebo_include = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file),
                launch_arguments=gazebo_launch_args.items(),
            )
            return [gazebo_include]
        LogInfo(msg="sim_gazebo is false, not launching Gazebo from launch_control.py.")
        return []

    ld.add_action(OpaqueFunction(function=include_gazebo_if_enabled))

    return ld
       