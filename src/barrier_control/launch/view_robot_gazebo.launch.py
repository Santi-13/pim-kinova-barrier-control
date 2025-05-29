import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Constants
    package_name = 'barrier_control' # Change to your package name
    xacro_file_name = 'rrbot.xacro' # Change to your xacro file name
    world_file_name = 'my_world.sdf' # Optional: your world file
    robot_controller_config_file = 'rrbot_controllers.yaml'

    # Paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_file_path = PathJoinSubstitution([pkg_share, 'urdf', xacro_file_name])
    world_file_path = PathJoinSubstitution([pkg_share, 'worlds', world_file_name]) # Optional

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_args = LaunchConfiguration('gz_args', default='')
    gz_verbosity = LaunchConfiguration('gz_verbosity', default='1') # 0=Quiet, 1=Err, 2=Warn, 3=Info, 4=Debug

    # 1. Process the XACRO file to get URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file_path,
        # You can pass xacro arguments here, e.g.:
        # ' sim_mode:=true',
        # ' use_ros2_control:=true',
        # ' gazebo_control_config_package:=my_robot_description',
        # ' gazebo_control_config_file:=my_controllers.yaml'
    ])

    # 2. Controller Configuration
    robot_controllers = PathJoinSubstitution(
        [
            pkg_share,
            'config',
            robot_controller_config_file,
        ]
    )

    # 3. Robot State Publisher Node
    # Takes the URDF and publishes transforms (/tf) and the robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # 4. Joint State 
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )


    # 5. Spawn Robot Entity in Gazebo
    # Uses the `create` service of Gazebo to spawn the robot.
    # The `ros_gz_sim.create` node makes a service call to `/world/{world_name}/create`
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', # The topic where the robot description is published
            '-string', robot_description_content, # Pass the URDF string
            '-name', 'my_robot',           # Name of the entity in Gazebo
            '-allow_renaming', 'true',     # Allow Gazebo to rename if the name conflicts
            '-x', '0.0',                   # Initial X position
            '-y', '0.0',                   # Initial Y position
            '-z', '0.5',                   # Initial Z position
            # '-R', '0.0',                 # Initial Roll
            # '-P', '0.0',                 # Initial Pitch
            # '-Y', '0.0'                  # Initial Yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. Bridge ROS 2 topics to Gazebo (Ignition) Transport topics
    # This is essential for sensors, clock, etc.
    # Example: Bridge the clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_ros_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], # Bidirectional
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            }]
    )

    # Example: Bridge joint states from Gazebo to ROS 2 (if your robot has sensors publishing in Gazebo)
    # bridge_joint_states = Node(
    #    package='ros_gz_bridge',
    #    executable='parameter_bridge',
    #    name='bridge_gz_ros_joint_states',
    #    arguments=['/world/default/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
    #    # The Gz topic name might vary depending on your Gazebo setup and plugins
    #    # Use `gz topic -l` to find the correct Gazebo topic for joint states
    #    remappings=[('/world/default/model/my_robot/joint_state', '/joint_states')], # Remap to /joint_states for RSP
    #    output='screen',
    #    parameters=[{'use_sim_time': use_sim_time}]
    # )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world', default_value=world_file_path,
                              description='SDF world file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo Ignition in headless mode.'),

        # Start Gazebo
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v ', gz_verbosity, ' empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner,
                         imu_sensor_broadcaster_spawner],
            )
        ),

        # Nodes
        robot_state_publisher_node,
        spawn_entity_node,
        # Bridges
        bridge_clock,
    ])