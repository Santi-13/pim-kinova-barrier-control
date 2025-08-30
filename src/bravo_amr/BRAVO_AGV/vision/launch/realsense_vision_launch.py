from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_camera.color_profile', default_value='1280x720x10'),
        DeclareLaunchArgument('depth_module.depth_profile', default_value='640x480x10'),
        DeclareLaunchArgument('depth_module.infra_profile', default_value='640x480x10'),
        DeclareLaunchArgument('align_depth.enable', default_value='true'),
        Node(
            package='realsense2_camera',
            namespace='bravo',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'rgb_camera.color_profile' : LaunchConfiguration('rgb_camera.color_profile'),
                'depth_module.depth_profile' : LaunchConfiguration('depth_module.depth_profile'),
                'depth_module.infra_profile' : LaunchConfiguration('depth_module.infra_profile'),
                'align_depth.enable' : LaunchConfiguration('align_depth.enable'),
            }]
        )
    ])