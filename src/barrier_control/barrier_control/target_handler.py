import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class TargetHandler(Node):
    def __init__(self):
        super().__init__('target_handler')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_coordinates',
            self.listener_callback,
            10)
        self.publisher_robot1 = self.create_publisher(PoseStamped, '/robot1/target_pose', 10)
        self.publisher_robot2 = self.create_publisher(PoseStamped, '/robot2/target_pose', 10)
        self.robot1_offset = np.array([0.0, -0.15, 0.0])  # Example offset for robot 1
        self.robot2_offset = np.array([0.0, 0.15, 0.0]) # Example offset for robot 2

    def listener_callback(self, msg):
        target_coordinates = np.array(msg.data)
        if len(target_coordinates) != 3:
            self.get_logger().warn('Received invalid target coordinates. Expected 3 values (x, y, z).')
            return

        # Simple logic to decide which robot to send the target to.
        # This can be replaced with more sophisticated logic based on workspace, etc.
        if target_coordinates[1] < 0:  # Example: if target x-coordinate is positive, send to robot 1
            robot_number = 1
            robot_offset = self.robot1_offset
        else:
            robot_number = 2
            robot_offset = self.robot2_offset
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"  # Assuming world frame

        # Apply offset to target coordinates
        target_with_offset = target_coordinates - robot_offset

        pose_msg.pose.position.x = target_with_offset[0]
        pose_msg.pose.position.y = target_with_offset[1]
        pose_msg.pose.position.z = target_with_offset[2]

        # Assuming no rotation for now, set orientation to identity quaternion
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # Publish the message to the appropriate robot
        if robot_number == 1:
            self.publisher_robot1.publish(pose_msg)
            self.get_logger().info(f'Publishing to Robot 1: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}, z={pose_msg.pose.position.z}')
        else:
            self.publisher_robot2.publish(pose_msg)
            self.get_logger().info(f'Publishing to Robot 2: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}, z={pose_msg.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    target_handler = TargetHandler()
    rclpy.spin(target_handler)
    target_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()