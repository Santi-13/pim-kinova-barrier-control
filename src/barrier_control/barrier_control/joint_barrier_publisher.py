# joint_barrier_publisher.py

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration as RclpyDuration

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import roboticstoolbox as rtb
import os
import subprocess
import tempfile
import math

class JointBarrierPublisher(Node):
    def __init__(self):
        super().__init__('joint_barrier_publisher')

        # --- Parameters ---
        self.declare_parameter('update_frequency', 10.0)
        self.declare_parameter('robot_description_package', 'kortex_description')
        self.declare_parameter('robot_description_xacro_path', 'robots/gen3.xacro')
        self.declare_parameter('xacro_args', 'dof:=6 use_fake_hardware:=true robot_ip:=dummy')
        self.declare_parameter('joint_names_for_barriers', [
            'robot1/joint_1', 'robot1/joint_2', 'robot1/joint_3',
            'robot1/joint_4', 'robot1/joint_5', 'robot1/joint_6'
        ])
        self.declare_parameter('link_names_for_barriers', [
            'robot1/link_1', 'robot1/link_2', 'robot1/link_3',
            'robot1/link_4', 'robot1/link_5', 'robot1/link_6'
        ])
        self.declare_parameter('joint_barrier_radii', [
            0.15, 0.15, 0.15, 0.13, 0.1, 0.12 # Radii in meters for each joint barrier
        ])
        self.declare_parameter('joint_state_topic', '/robot1/joint_states')
        self.declare_parameter('joint_barriers_markers_topic', '/dynamic_joint_barriers_markers')
        self.declare_parameter('joint_barriers_topic', '/dynamic_joint_barriers_data')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('barrier_color', [0.0, 1.0, 1.0, 0.5]) # RGBA for cyan
        
        # --- NEW PARAMETERS for intermediate barriers ---
        self.declare_parameter('num_intermediate_barriers', 3) # n points between start and end
        self.declare_parameter('intermediate_barrier_start_link', 'robot1/link_3')
        self.declare_parameter('intermediate_barrier_end_link', 'robot1/link_4')


        # --- Get Parameters ---
        update_frequency = self.get_parameter('update_frequency').value
        self.joint_names_for_barriers = self.get_parameter('joint_names_for_barriers').value
        self.link_names_for_barriers = self.get_parameter('link_names_for_barriers').value
        self.barrier_radii = self.get_parameter('joint_barrier_radii').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        
        joint_barriers_topic = self.get_parameter('joint_barriers_topic').value
        joint_barriers_markers_topic = self.get_parameter('joint_barriers_markers_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.barrier_color = self.get_parameter('barrier_color').value

        # --- Get NEW PARAMETERS ---
        self.num_intermediate_barriers = self.get_parameter('num_intermediate_barriers').value
        self.intermediate_start_link = self.get_parameter('intermediate_barrier_start_link').value
        self.intermediate_end_link = self.get_parameter('intermediate_barrier_end_link').value

        # --- State Variables ---
        self.robot = None
        self.current_joint_positions = None
        self.joint_states_received_once = False
        self.joint_name_to_idx_map = {} # For quick lookups

        # --- ROS2 Communications ---
        self.joint_state_sub = self.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10
        )
        self.get_logger().info(f"Subscribed to joint states on: {joint_state_topic}")
        self.marker_pub = self.create_publisher(
            MarkerArray, joint_barriers_markers_topic, 10
        )
        self.get_logger().info(f"Publishing markers to: {joint_barriers_markers_topic}")
        self.barrier_pub = self.create_publisher(
            Float64MultiArray, joint_barriers_topic, 10
        )
        
        self.get_logger().info(f"Publishing dynamic joint barriers to: {joint_barriers_topic}")

        # --- Load Robot Model ---
        self.load_robot()

        # --- Main Execution Timer ---
        self.timer = self.create_timer(1.0 / update_frequency, self.publish_barriers_callback)

        self.get_logger().info(f"Joint Barrier Publisher node has been initialized for joints: {self.joint_names_for_barriers}")
        if self.num_intermediate_barriers > 0:
            self.get_logger().info(f"Publishing {self.num_intermediate_barriers} intermediate barriers between '{self.intermediate_start_link}' and '{self.intermediate_end_link}'.")


    def load_robot(self):
        """Loads the robot model from URDF/XACRO, identical to the controller's method."""
        tmp_file_path = None
        try:
            package_name = self.get_parameter('robot_description_package').value
            xacro_r_path = self.get_parameter('robot_description_xacro_path').value
            xacro_args = self.get_parameter('xacro_args').value.split()
            pkg_share_dir = get_package_share_directory(package_name)
            xacro_abs_path = os.path.join(pkg_share_dir, xacro_r_path)

            self.get_logger().info(f"Loading robot model from: {xacro_abs_path} with args: {xacro_args}")
            cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_abs_path] + xacro_args
            process = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding='utf-8')
            urdf_str = process.stdout

            with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.urdf', encoding='utf-8') as tmp_file:
                tmp_file.write(urdf_str)
                tmp_file_path = tmp_file.name

            self.robot = rtb.ERobot.URDF(tmp_file_path)

            if self.robot:
                self.get_logger().info(f"Robot model loaded: {self.robot.name} with {self.robot.n} DoF.")
            else:
                self.get_logger().error("Failed to load robot model with ERobot.URDF.")
        except Exception as e:
            self.get_logger().error(f"Exception during robot model loading: {e}")
            self.robot = None
        finally:
            if tmp_file_path and os.path.exists(tmp_file_path):
                os.remove(tmp_file_path)

    def joint_state_callback(self, msg: JointState):
        """Processes incoming joint states and maps them to the correct order."""
        if not self.joint_names_for_barriers:
            self.get_logger().warn("No joint names defined for barriers. Skipping joint state processing.")
            return

        # Build the name->index map once for efficiency
        if not self.joint_name_to_idx_map:
            for i, name in enumerate(msg.name):
                self.joint_name_to_idx_map[name] = i

        num_joints = len(self.robot.links) if self.robot else 0
        if num_joints == 0: 
            self.get_logger().warn("Robot model not loaded or has no joints. Skipping joint state processing.")
            return
        
        try:
            num_controlled = len(self.joint_names_for_barriers)
            new_positions = np.zeros(num_controlled)
            new_velocities = np.zeros(num_controlled)
            all_controlled_joints_found = True

            for i, name in enumerate(self.joint_names_for_barriers):
                try:
                    idx_in_msg = msg.name.index(name)
                    new_positions[i] = msg.position[idx_in_msg]
                    if msg.velocity and len(msg.velocity) > idx_in_msg:
                        new_velocities[i] = msg.velocity[idx_in_msg]
                    else:
                        new_velocities[i] = 0.0
                except ValueError:
                    # self.get_logger().warn(f"Controlled joint '{name}' not found in JointState. Available: {msg.name}", throttle_duration_sec=10)
                    all_controlled_joints_found = False
                    break

            if all_controlled_joints_found:
                self.current_joint_positions = new_positions
                self.current_joint_velocities = new_velocities

                if not self.joint_states_received_once:
                    self.get_logger().info("First joint states received.")
                    self.joint_states_received_once = True

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None
            self.current_joint_velocities = None


    def publish_barriers_callback(self):
        """
        The main loop: performs FK for all joints and publishes their positions as markers.
        """
        if self.robot is None:
            self.get_logger().warn("Robot model not loaded. Skipping barrier publication.", throttle_duration_sec=5)
            return

        if self.current_joint_positions is None:
            self.get_logger().warn("No joint positions available. Skipping barrier publication.", throttle_duration_sec=5)
            return
        
        try:
            all_link_poses = self.robot.fkine_all(self.current_joint_positions)
        except Exception as e:
            self.get_logger().error(f"Error during fkine_all: {e}")
            return

        marker_array = MarkerArray()
        barrier_data_list = [] # To store [x, y, z, radius] for each barrier
        
        # For efficiency, create a map of link names to their poses
        link_poses_map = {
            link.name: all_link_poses[i] for i, link in enumerate(self.robot.links)
            }

        # --- 1. Publish barriers for the specified joints ---
        for i, link_name in enumerate(self.link_names_for_barriers):
            if link_name in link_poses_map:
                link_pose = link_poses_map[link_name]
                link_position = link_pose.t  # Translation vector [x, y, z]
                barrier_radius = self.barrier_radii[i]

                marker = self.create_barrier_marker(
                    marker_id=i,
                    position=link_position,
                    radius=barrier_radius,
                    ns="dynamic_joint_barriers"
                )
                marker_array.markers.append(marker)
                barrier_data_list.extend([float(pos) for pos in link_position] + [float(barrier_radius)])
            else:
                self.get_logger().warn(f"Could not find link '{link_name}' in robot model.", throttle_duration_sec=10)

        # --- 2. Publish intermediate barriers for the long link ---
        if self.num_intermediate_barriers > 0:
            if self.intermediate_start_link in link_poses_map and self.intermediate_end_link in link_poses_map:
                p_start = link_poses_map[self.intermediate_start_link].t
                p_end = link_poses_map[self.intermediate_end_link].t
                
                # Find the radius for the start link (e.g., joint3's radius)
                try:
                    start_link_index = self.link_names_for_barriers.index(self.intermediate_start_link)
                    radius = self.barrier_radii[start_link_index]
                except ValueError:
                    self.get_logger().error(f"Start link '{self.intermediate_start_link}' for interpolation not found in 'link_names_for_barriers'. Cannot determine radius.")
                    radius = 0.1 # Fallback radius


                for i in range(self.num_intermediate_barriers):
                    # Interpolate position along the line from p_start to p_end
                    # The fraction is (i+1) / (n+1) to create n evenly spaced points
                    fraction = (i + 1.0) / (self.num_intermediate_barriers + 1.0)
                    interp_position = p_start + (p_end - p_start) * fraction

                    # Create a unique ID for this intermediate marker
                    marker_id = len(self.link_names_for_barriers) + i
                    
                    marker = self.create_barrier_marker(
                        marker_id=marker_id,
                        position=interp_position,
                        radius=radius,
                        ns="intermediate_link_barriers" # Use a different namespace for clarity in RViz
                    )
                    marker_array.markers.append(marker)
                    barrier_data_list.extend([float(pos) for pos in interp_position] + [float(radius)])

            else:
                self.get_logger().warn(f"Could not find start ('{self.intermediate_start_link}') or end ('{self.intermediate_end_link}') link for intermediate barriers.", throttle_duration_sec=10)

        # Publish the complete array of markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
        
        # Publish the barrier data as Float64MultiArray
        if barrier_data_list:
            barrier_msg = Float64MultiArray()
            barrier_msg.data = barrier_data_list
            self.barrier_pub.publish(barrier_msg)

    def create_barrier_marker(self, marker_id, position, radius, ns):
        """Helper function to create a single SPHERE marker."""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0

        marker.color.r = self.barrier_color[0]
        marker.color.g = self.barrier_color[1]
        marker.color.b = self.barrier_color[2]
        marker.color.a = self.barrier_color[3]

        marker.lifetime = RclpyDuration(seconds=(1.0 / self.get_parameter('update_frequency').value) * 2.0).to_msg()
        return marker


def main(args=None):
    rclpy.init(args=args)
    joint_barrier_publisher = JointBarrierPublisher()
    rclpy.spin(joint_barrier_publisher)
    joint_barrier_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()