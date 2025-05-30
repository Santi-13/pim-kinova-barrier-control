from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz_robot2", default_value="false", description="Launch RViz for Robot 2.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_robot1", # Unique name if you want per-robot override
            default_value="kortex_description",
            description="Description package for Robot 1."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_robot2", # Unique name if you want per-robot override
            default_value="kortex_description",
            description="Description package for Robot 2."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_x_robot1", default_value="0.0", description="Initial X for Robot 1"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_y_robot1", default_value="-0.5", description="Initial Y for Robot 1" # Default from your existing file
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_z_robot1", default_value="0.0", description="Initial Z for Robot 1"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_roll_robot1", default_value="0.0", description="Initial Roll for Robot 1"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_pitch_robot1", default_value="0.0", description="Initial Pitch for Robot 1"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_yaw_robot1", default_value="0.0", description="Initial Yaw for Robot 1"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_x_robot2", default_value="0.0", description="Initial X for Robot 2"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_y_robot2", default_value="0.5", description="Initial Y for Robot 2" # Default from your existing file
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_z_robot2", default_value="0.0", description="Initial Z for Robot 2"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_roll_robot2", default_value="0.0", description="Initial Roll for Robot 2"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_pitch_robot2", default_value="0.0", description="Initial Pitch for Robot 2"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_pose_yaw_robot2", default_value="0.0", description="Initial Yaw for Robot 2"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false", description="Use fake hardware for both robots."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo", default_value="false", description="Use Gazebo simulation."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false", # Set to true if this is your new default
            description="Use Ignition Gazebo (Fortress) simulation for both robots."
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
            default_value=f"{get_package_share_directory('barrier_control')}/worlds/my_world.sdf", 
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_ip_real", default_value="192.168.1.10", description="Real IP for Robot 1 (if use_fake_hardware is false)."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot2_ip_real", default_value="192.168.1.11", description="Real IP for Robot 2 (if use_fake_hardware is false)."
        )
    )


    # --- Common Paths ---
    kortex_bringup_pkg_path = get_package_share_directory('kortex_bringup')
    gen3_launch_file = os.path.join(kortex_bringup_pkg_path, 'launch', 'gen3.launch.py')
    barrier_control_pkg_name = 'barrier_control'
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    sim_ignition = LaunchConfiguration('sim_ignition')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Robot 1 Configuration ---
    robot1_namespace = 'robot1'
    robot1_controller_name = 'robot1_controller'
    robot1_controller_file = 'robot1_controllers.yaml'
    robot1_target_pose_topic = 'target_pose' # Relative topic name within namespace
    robot1_joint_state_topic = 'joint_states' # Relative topic name within namespace
    robot1_velocity_command_topic = 'robot1_controllers/commands' # Relative topic name
    
    # Logic for use_fake_hardware passed to gen3.launch.py
    # If main use_fake_hardware is false (real robot) AND not Gazebo/Ignition, gen3.launch uses its fake_hardware.
    # Otherwise, gen3.launch uses the main use_fake_hardware setting.
    gen3_use_fake_hardware_setting = PythonExpression([
        "'true' if ('", use_fake_hardware, "' == 'false' and '", sim_gazebo, "' == 'false' and '", sim_ignition, "' == 'false') else '", use_fake_hardware, "'"
    ])
    # If main use_fake_hardware is false (real robot mode), then gen3.launch.py is told it's managed externally.
    gen3_managed_by_external_controller_setting = PythonExpression([
        "'true' if '", use_fake_hardware, "' == 'false' else 'false'"
    ])
    LogInfo(msg=f"========================\ngen3_use_fake_hardware_setting: {gen3_use_fake_hardware_setting}")

    

    # --- Robot 2 Configuration ---
    robot2_namespace = 'robot2'
    robot2_controller_name = 'robot2_controller'
    robot2_controller_file = 'robot2_controllers.yaml'
    robot2_target_pose_topic = 'target_pose' # Can be the same relative name
    robot2_joint_state_topic = 'joint_states' # Can be the same relative name
    robot2_velocity_command_topic = 'robot2_controllers/commands' # Can be the same relative name

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
                    'use_fake_hardware': gen3_use_fake_hardware_setting,
                    'sim_gazebo': sim_gazebo,
                    'sim_ignition': sim_ignition,
                    'use_sim_time': use_sim_time,
                    'robot_ip': 'dummy1', # Use unique dummy IPs if needed, though often not critical for fake_hardware
                    'prefix': f"{robot1_namespace}/", # Pass the prefix argument with a trailing slash
                    'namespace': f"/{robot1_namespace}", # Namespace for the robot
                    # Initial pose for Robot 1
                    'initial_pose_x': LaunchConfiguration("initial_pose_x_robot1"),
                    'initial_pose_y': LaunchConfiguration("initial_pose_y_robot1"),
                    'initial_pose_z': LaunchConfiguration("initial_pose_z_robot1"),
                    'initial_pose_roll': LaunchConfiguration("initial_pose_roll_robot1"),
                    'initial_pose_pitch': LaunchConfiguration("initial_pose_pitch_robot1"),
                    'initial_pose_yaw': LaunchConfiguration("initial_pose_yaw_robot1"),
                    'controllers_file': robot1_controller_file,
                    'robot_controller': 'robot1_controllers', # Specify this controller
                    'rviz_file': 'view_robots.rviz', # Specify the RViz file for Robots
                    'description_package': LaunchConfiguration("description_package_robot1"),
                    'launch_rviz': LaunchConfiguration("launch_rviz_robot1"), # Pass the declared argument
                    # gen3.launch should not start Gazebo if dual_launch is managing it (when sim_gazebo or sim_ignition is true for dual_launch)
                    'launch_gazebo_server_client': PythonExpression(
                        ["'false' if ('", sim_gazebo, "' == 'true' or '", sim_ignition, "' == 'true') else 'true'"]
                    ),
                    'gazebo_world_file': LaunchConfiguration("gazebo_world_file"),
                    'headless_rendering': LaunchConfiguration("headless_rendering"),
                    'managed_by_external_controller': gen3_managed_by_external_controller_setting,
                }.items()
            ),
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
                    'error_tolerance': [0.02] * 3 + [0.1]*3, # Pos [m], Orient [rad]
                    'joint_state_topic': robot1_joint_state_topic,
                    'target_pose_topic': robot1_target_pose_topic,
                    'velocity_command_topic': robot1_velocity_command_topic,
                    # --- Velocity Command Service Parameters ---
                    'use_service_velocity_command': PythonExpression(["'true' if '", use_fake_hardware, "' == 'false' else 'false'"]),
                    'arm_index': 0,
                    'velocity_command_service': '/send_joint_speeds', # Default, but can be explicit
                    # --- Robot Description ---
                    'robot_description_package': 'kortex_description', # Your robot description package
                    'robot_description_xacro_path': 'robots/gen3.xacro', # Relative path within the package
                    # Arguments passed to xacro - adjust as needed for your URDF/XACRO
                    'xacro_args': PythonExpression([
                        "'dof:=6 use_fake_hardware:=",
                        use_fake_hardware,
                        " robot_ip:=dummy1 prefix:=", # dummy1 is fine for xacro context if not used for real connection by xacro
                        robot1_namespace, "/", # Prefix for namespacing in URDF
                        " initial_pose_x:=", LaunchConfiguration("initial_pose_x_robot1"),
                        " initial_pose_y:=", LaunchConfiguration("initial_pose_y_robot1"),
                        " initial_pose_z:=", LaunchConfiguration("initial_pose_z_robot1"),
                        " initial_pose_roll:=", LaunchConfiguration("initial_pose_roll_robot1"),
                        " initial_pose_pitch:=", LaunchConfiguration("initial_pose_pitch_robot1"),
                        " initial_pose_yaw:=", LaunchConfiguration("initial_pose_yaw_robot1"),
                        " sim_gazebo:=",
                        sim_gazebo, " sim_ignition:=",
                        sim_ignition, " robot_controller:=",
                        robot1_controller_name, "'"
                    ]),
                }],
                arguments=['--ros-args', '--log-level', 'rigid_body_dynamics_controller:=debug']
            )
        ]
    )
    ld.add_action(robot1_group)
    
    # --- Robot 2 Group ---
    robot2_group = GroupAction(
        actions=[
            # Push the namespace for all nodes/includes within this group
            PushRosNamespace(robot2_namespace),

            # Include kortex_bringup for Robot 1
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gen3_launch_file),
                launch_arguments={
                    'dof': '6',
                    'use_fake_hardware': gen3_use_fake_hardware_setting,
                    'sim_gazebo': 'false', # Pass the main sim_gazebo flag
                    'sim_ignition': 'false', # Pass the main sim_ignition flag
                    'use_sim_time': 'false',
                    'robot_ip': 'dummy2', # Use unique dummy IPs if needed, though often not critical for fake_hardware
                    'prefix': f"{robot2_namespace}/", # Pass the prefix argument with a trailing slash
                    'namespace': f"/{robot2_namespace}", # Namespace for the robot
                    # Initial pose for Robot 1
                    'initial_pose_x': LaunchConfiguration("initial_pose_x_robot2"),
                    'initial_pose_y': LaunchConfiguration("initial_pose_y_robot2"),
                    'initial_pose_z': LaunchConfiguration("initial_pose_z_robot2"),
                    'initial_pose_roll': LaunchConfiguration("initial_pose_roll_robot2"),
                    'initial_pose_pitch': LaunchConfiguration("initial_pose_pitch_robot2"),
                    'initial_pose_yaw': LaunchConfiguration("initial_pose_yaw_robot2"),
                    'controllers_file': robot2_controller_file,
                    # 'rviz_file': 'view_robots.rviz', # Not necessary as view_rviz is set to false
                    'robot_controller': 'robot2_controllers',
                    
                    'description_package': LaunchConfiguration("description_package_robot2"),
                    'launch_rviz': LaunchConfiguration("launch_rviz_robot2"), # Pass the declared argument
                    # gen3.launch should not start Gazebo if dual_launch is managing it
                    'launch_gazebo_server_client': PythonExpression(
                        ["'false' if ('", sim_gazebo, "' == 'true' or '", sim_ignition, "' == 'true') else 'true'"]
                    ),
                    'gazebo_world_file': LaunchConfiguration("gazebo_world_file"),
                    'headless_rendering': LaunchConfiguration("headless_rendering"),
                    'managed_by_external_controller': gen3_managed_by_external_controller_setting,
                }.items()
            ),
            Node(
                package=barrier_control_pkg_name,
                executable='rigid_body_dynamics_controller',
                name=robot2_controller_name,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    # --- Frame Names ---
                    'base_frame': f"{robot2_namespace}/base_link",
                    'tool_frame': f"{robot2_namespace}/end_effector_link",
                    # --- Controller Gains & Settings ---
                    # Kp gains [x, y, z, rx, ry, rz] - Tune these values!
                    'kp_gains': [0.1] * 6, # Reduced for slower movement
                    'kd_gains': [0.005]*3 + [0.005] * 3,
                    'jacobian_damping': 0.05, # Damping for pseudo-inverse singularity robustness (increased slightly)
                    'euler_input_convention': 'quat', # Convention for interpreting target orientation if given as Euler: 'xyz', 'zyx', etc.
                    # --- Joint Specific Settings ---
                    # IMPORTANT: List joint names in the order your robot model (and RTB) expects them.
                    'controlled_joint_names': [f'{robot2_namespace}/joint_1', f'{robot2_namespace}/joint_2', f'{robot2_namespace}/joint_3', f'{robot2_namespace}/joint_4', f'{robot2_namespace}/joint_5', f'{robot2_namespace}/joint_6'],
                    # Max joint velocities (rad/s) - Set based on your robot's limits!
                    # Example for a 6DOF arm, replace with actual values
                    'max_joint_velocities': [0.1] * 6, 
                    # --- Control Settings & Topic Names ---
                    'joint_limit_buffer': 0.01, # Small buffer (rad or m) for joint position limits
                    'control_frequency': 20.0, # Control loop frequency in Hz
                    'error_tolerance': [0.02] * 3 + [0.1]*3, # Pos [m], Orient [rad]
                    'joint_state_topic': robot2_joint_state_topic,
                    'target_pose_topic': robot2_target_pose_topic,
                    'velocity_command_topic': robot2_velocity_command_topic,
                    # --- Velocity Command Service Parameters ---
                    'use_service_velocity_command': PythonExpression(["'true' if '", use_fake_hardware, "' == 'false' else 'false'"]),
                    'arm_index': 1,
                    'velocity_command_service': '/send_joint_speeds', # Default, but can be explicit
                    # --- Robot Description ---
                    'robot_description_package': 'kortex_description', # Your robot description package
                    'robot_description_xacro_path': 'robots/gen3.xacro', # Relative path within the package
                    # Arguments passed to xacro - adjust as needed for your URDF/XACRO

                    'xacro_args': PythonExpression([
                        "'dof:=6 use_fake_hardware:=",
                        use_fake_hardware,
                        " robot_ip:=dummy2 prefix:=", # dummy2 is fine for xacro context
                        robot2_namespace, "/", # Prefix for namespacing in URDF
                        " initial_pose_x:=", LaunchConfiguration("initial_pose_x_robot2"),
                        " initial_pose_y:=", LaunchConfiguration("initial_pose_y_robot2"),
                        " initial_pose_z:=", LaunchConfiguration("initial_pose_z_robot2"),
                        " initial_pose_roll:=", LaunchConfiguration("initial_pose_roll_robot2"),
                        " initial_pose_pitch:=", LaunchConfiguration("initial_pose_pitch_robot2"),
                        " initial_pose_yaw:=", LaunchConfiguration("initial_pose_yaw_robot2"),
                        " sim_gazebo:=",
                        sim_gazebo, " sim_ignition:=",
                        sim_ignition, " robot_controller:=",
                        robot2_controller_name, "'" # Make sure spacing is correct for the final xacro string
                    ]),
                }],
                arguments=['--ros-args', '--log-level', 'rigid_body_dynamics_controller:=debug']
            )
        ]
    )
    ld.add_action(robot2_group)

    # --- Conditionally launch Kortex Dual Arm Node for real robot control ---
    def launch_kortex_node_conditionally(context):
        if LaunchConfiguration('use_fake_hardware').perform(context) == 'false':
            LogInfo(msg="use_fake_hardware is false. Launching Kortex Dual Arm Node for real robot control.")
            kortex_dual_arm_node = Node(
                package=barrier_control_pkg_name,
                executable='kortex_dual_arm_node', # Ensure this is the correct executable name from setup.py
                name='kortex_dual_arm_manager',
                output='screen',
                parameters=[
                    {'robot_ips': [
                        LaunchConfiguration('robot1_ip_real').perform(context), 
                        LaunchConfiguration('robot2_ip_real').perform(context)
                    ]},
                    {'robot_ports': [10000, 10000]}, # Default Kortex ports, can be parameterized if needed
                    {'robot_0_username': 'admin'}, # Can be parameterized
                    {'robot_0_password': 'admin'}, # Can be parameterized
                    {'robot_1_username': 'admin'}, # Can be parameterized
                    {'robot_1_password': 'admin'}, # Can be parameterized
                    {'robot1_joint_names': [f'{robot1_namespace}/joint_{i+1}' for i in range(6)]},
                    {'robot2_joint_names': [f'{robot2_namespace}/joint_{i+1}' for i in range(6)]},
                    {'joint_state_publish_rate': 50.0} # Can be parameterized
                ]
            )
            return [kortex_dual_arm_node]
        else:
            LogInfo(msg="use_fake_hardware is true. Skipping Kortex Dual Arm Node launch.")
            return []

    ld.add_action(OpaqueFunction(function=launch_kortex_node_conditionally))

    # --- Gazebo Simulation ---
    # This OpaqueFunction allows us to conditionally add Gazebo-related nodes
    # based on the runtime value of sim_gazebo
    # def include_simulation_environment(context):
    #     nodes = []
    #     sim_ignition_str = sim_ignition.perform(context)
    #     sim_gazebo_str = sim_gazebo.perform(context)

    #     if sim_ignition_str == "true":
    #         LogInfo(msg="sim_ignition is true, launching Ignition Gazebo (Fortress).")
    #         pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #         ignition_launch_file = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #         # Use -r for GUI, -s for server-only. Add your world file.
    #         # Example: 'gz_args': '-r -v 4 your_world.sdf' or 'gz_args': '-r -v 4 empty.sdf'
    #         gazebo_sim_launch = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(ignition_launch_file),
    #             launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    #         )
    #         nodes.append(gazebo_sim_launch)

    #     elif sim_gazebo_str == "true":
    #         LogInfo(msg="sim_gazebo is true, launching Gazebo Classic.")
    #         gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
    #         gazebo_classic_launch_file = os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')
    #         gazebo_classic_launch = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(gazebo_classic_launch_file),
    #             launch_arguments={'verbose': 'true'}.items(), # Example args for Gazebo Classic
    #         )
    #         nodes.append(gazebo_classic_launch)
    #     else:
    #         LogInfo(msg="Neither sim_ignition nor sim_gazebo is true. No simulation environment launched by dual_gen3_control_launch.")
    #     return nodes

    # ld.add_action(OpaqueFunction(function=include_simulation_environment))
    return ld