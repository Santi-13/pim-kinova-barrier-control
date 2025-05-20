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
    xacro_file_name = 'my_robot.urdf.xacro' # Change to your xacro file name
    world_file_name = 'my_world.sdf' # Optional: your world file

    # Paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_file_path = PathJoinSubstitution([pkg_share, 'urdf', xacro_file_name])
    world_file_path = PathJoinSubstitution([pkg_share, 'worlds', world_file_name]) # Optional

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_verbosity = LaunchConfiguration('gz_verbosity', default='3') # 0=Quiet, 1=Err, 2=Warn, 3=Info, 4=Debug
    headless = LaunchConfiguration('headless', default='false') # Set to 'true' for headless Gazebo

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



    # 2. Robot State Publisher Node
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

    # 3. Joint State Publisher GUI Node (for testing without actual robot hardware)
    # Publishes dummy joint states. You can move the sliders in the GUI.
    # For a real robot, you would have a hardware interface node publishing actual joint states.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui', # Optional: good practice to name nodes
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # condition=IfCondition(LaunchConfiguration('start_jsp_gui')) # Example for conditional launch
    )
    # Or, without GUI:
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )


    # 4. Launch Gazebo Sim
    # The `gz sim` executable (formerly `ign gazebo`)
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={
    #         'gz_args': ['-r -v 4 ', world_file_path], # -r: run on startup, -v 4: verbosity
    #         # 'gz_args': ['-r -v 4 ', LaunchConfiguration('world')], # If world is a launch argument
    #         # 'on_exit_shutdown': 'true', # Shutdown gz sim if this launch file is terminated
    #     }.items()
    # )
    # OR: Launch Gazebo using ExecuteProcess for more control if needed, or if you have a specific world
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', # '-s', '--headless-rendering', # Uncomment for server-only or headless
             world_file_path, # Optional: if you have a world file
             '-v', gz_verbosity, # Control Gazebo's verbosity
            ],
        output='screen'
    )


    # 5. Spawn Robot Entity in Gazebo
    # Uses the `create` service of Gazebo to spawn the robot.
    # The `ros_gz_sim.create` node makes a service call to `/world/{world_name}/create`
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
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
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
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

    # Example: Bridge a Lidar sensor
    # bridge_lidar = Node(
    #    package='ros_gz_bridge',
    #    executable='parameter_bridge',
    #    name='bridge_gz_ros_lidar',
    #    # Assuming your Lidar in Gazebo publishes to /lidar and its Gz type is gz.msgs.LaserScan
    #    arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
    #    remappings=[('/lidar', '/scan')], # Remap to /scan or your preferred ROS topic
    #    output='screen',
    #    parameters=[{'use_sim_time': use_sim_time}]
    # )


    # Optional: Delay start of spawn_entity_node until Gazebo is ready
    # This can be tricky. A common approach is to wait for a specific Gazebo service or topic.
    # For simplicity, a fixed delay or manual trigger might be used for basic setups.
    # A more robust way: use an event handler to trigger spawning after Gazebo starts.
    # For example, wait for the `/world/{world_name}/create` service to be available.

    # Ensure Gazebo server is launched before trying to spawn entities or bridges
    # One way to do this: use an event handler on the Gazebo process.
    # However, just because the process started doesn't mean its services are ready.
    # A more robust solution involves checking for specific Gazebo services.
    # For this example, we'll launch them sequentially, which often works but isn't guaranteed.

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world', default_value=world_file_path,
                              description='SDF world file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo Ignition in headless mode.'),

        # Start Gazebo
        # gz_sim, # Using IncludeLaunchDescription
        gazebo_process, # Using ExecuteProcess

        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # joint_state_publisher_node, # If not using GUI

        # Spawn robot after Gazebo is up (simple sequential, might need better sync for complex setups)
        # A better way is to use an EventHandler to trigger spawn_entity_node
        # once a Gazebo service like `/world/default/create` is available.
        # For now, we'll launch it directly. If it fails, Gazebo might not have been ready.
        spawn_entity_node,

        # Bridges
        bridge_clock,
        # bridge_joint_states, # Uncomment if you have joint states published from Gazebo
        # bridge_lidar,      # Uncomment if you have a Lidar in Gazebo

        # Example: Ensure spawn_entity runs after Gazebo starts (basic event handling)
        # This is a very basic way to try and sequence. A service check is more robust.
        # RegisterEventHandler(
        #     event_handler=OnProcessStart( # This event might not exist, OnProcessExit is common
        #         target_action=gazebo_process, # Or gz_sim if using IncludeLaunchDescription
        #         on_start=[spawn_entity_node]
        #     )
        # ),
    ])