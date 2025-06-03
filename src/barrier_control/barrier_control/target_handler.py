import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

from scipy.spatial.transform import Rotation

class TargetHandler(Node):
    def __init__(self):
        super().__init__('target_handler')

        # Declare parameters for robot base frames
        self.declare_parameter('robot1_base_frame', 'robot1/base_link')
        self.declare_parameter('robot2_base_frame', 'robot2/base_link')
        self.robot1_base_frame = self.get_parameter('robot1_base_frame').value
        self.robot2_base_frame = self.get_parameter('robot2_base_frame').value

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_coordinates',
            self.listener_callback,
            10)
        self.publisher_robot1 = self.create_publisher(PoseStamped, '/robot1/target_pose', 10)
        self.publisher_robot2 = self.create_publisher(PoseStamped, '/robot2/target_pose', 10)

        self.robot1_offset = np.array([0.0, -0.15, 0.0])  # Example offset for robot 1
        self.robot2_offset = np.array([0.0, 0.15, 0.0]) # Example offset for robot 2

        self.get_logger().info("TargetHandler node started.")
        self.get_logger().info(f"Robot 1 base frame: {self.robot1_base_frame}")
        self.get_logger().info(f"Robot 2 base frame: {self.robot2_base_frame}")

    def listener_callback(self, msg):
        target_coordinates_world = np.array(msg.data)

        if len(target_coordinates_world) != 3:
            self.get_logger().warn('Received invalid target coordinates. Expected 3 values (x, y, z).')
            return

        # Decide which robot to send the target to based on Y coordinate in world
        if target_coordinates_world[1] < 0:  # Example: if target x-coordinate is positive, send to robot 1
            robot_number = 1
            robot_base_offset_in_world = self.robot1_offset
            robot_target_base_frame_id = self.robot1_base_frame
            publisher = self.publisher_robot1
        else:
            robot_number = 2
            robot_base_offset_in_world = self.robot2_offset
            robot_target_base_frame_id = self.robot2_base_frame
            publisher = self.publisher_robot2
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        # The target pose is defined in the robot's own base frame
        pose_msg.header.frame_id = robot_target_base_frame_id
        
        # Calculate the target end-effector position *relative to the robot's base frame*
        target_ee_position_in_base_frame = target_coordinates_world - robot_base_offset_in_world

        # Target EE position in the robot's base frame
        target_x_in_base = target_ee_position_in_base_frame[0]
        target_y_in_base = target_ee_position_in_base_frame[1]
        target_z_in_base = target_ee_position_in_base_frame[2] # Z position for EE

        pose_msg.pose.position.x = float(target_x_in_base)
        pose_msg.pose.position.y = float(target_y_in_base)
        pose_msg.pose.position.z = float(target_z_in_base)

        # Calculate the desired orientation
        # 1. Tool's Z-axis ($Z_{tool}$): Points "away" from the robot base in the base's XY plane,
        #    towards the target's XY projection. This is the primary pointing axis.
        vec_xy_for_tool_z = np.array([target_x_in_base, target_y_in_base, 0.0])
        norm_vec_xy_for_tool_z = np.linalg.norm(vec_xy_for_tool_z[:2]) # Norm of XY components

        if norm_vec_xy_for_tool_z < 1e-6:
            # Target's XY projection is at the base origin.
            # Default $Z_{tool}$ to align with base's X-axis (an arbitrary horizontal direction).
            z_axis_tool = np.array([1.0, 0.0, 0.0])
        else:
            z_axis_tool = vec_xy_for_tool_z / norm_vec_xy_for_tool_z
            z_axis_tool[2] = 0.0 # Ensure it's purely in the XY plane of the base

        # 2. Tool's Y-axis ($Y_{tool}$): Aligns with the base's Z-axis (typically "up").
        #    This defines the "roll" of the tool around its new Z-axis (pointing axis).
        y_axis_tool = np.array([0.0, 0.0, 1.0]) # Tool's "up" is base's "up"

        # 3. Tool's X-axis ($X_{tool}$): $X_{tool} = Y_{tool} \times Z_{tool}$.
        #    This ensures a right-handed coordinate system.
        #    Since Y_tool is along base Z (unit vector) and Z_tool is in base XY (unit vector and orthogonal to Y_tool),
        #    X_tool will be a unit vector and also in the base XY plane.
        x_axis_tool = np.cross(y_axis_tool, z_axis_tool)
        # No explicit normalization needed for x_axis_tool here if y_axis_tool and z_axis_tool are
        # correctly formed unit orthogonal vectors.

        # 4. Form the rotation matrix: R = [X_tool, Y_tool, Z_tool] (columns)
        #    The columns are the axes of the tool's coordinate frame, expressed in base_link's coordinates.
        rotation_matrix = np.column_stack((x_axis_tool, y_axis_tool, z_axis_tool))

        # 5. Convert rotation matrix to quaternion
        try:
            r = Rotation.from_matrix(rotation_matrix)
            quaternion = r.as_quat()  # Returns (x, y, z, w)
        except Exception as e:
            self.get_logger().error(f"Error converting rotation matrix to quaternion: {e}. Using identity orientation as fallback.")
            quaternion = np.array([0.0, 0.0, 0.0, 1.0]) # Default to identity

        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])

        # Publish the message to the appropriate robot
        publisher.publish(pose_msg)
        self.get_logger().info(
            f'Publishing to Robot {robot_number} (frame: {pose_msg.header.frame_id}):\n'
            f'  Position: [x: {pose_msg.pose.position.x:.3f}, y: {pose_msg.pose.position.y:.3f}, z: {pose_msg.pose.position.z:.3f}]\n'
            f'  Tool X-axis (in base): [{x_axis_tool[0]:.3f}, {x_axis_tool[1]:.3f}, {x_axis_tool[2]:.3f}]\n'
            f'  Tool Y-axis (in base): [{y_axis_tool[0]:.3f}, {y_axis_tool[1]:.3f}, {y_axis_tool[2]:.3f}]\n'
            f'  Tool Z-axis (in base): [{z_axis_tool[0]:.3f}, {z_axis_tool[1]:.3f}, {z_axis_tool[2]:.3f}]\n'
            f'  Orientation (Quaternion): [x: {pose_msg.pose.orientation.x:.3f}, y: {pose_msg.pose.orientation.y:.3f}, z: {pose_msg.pose.orientation.z:.3f}, w: {pose_msg.pose.orientation.w:.3f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    target_handler = TargetHandler()
    rclpy.spin(target_handler)
    target_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()