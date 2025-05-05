import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs # Required for transforming geometry_msgs types if needed
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.duration import Duration

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link' # Example base frame
        self.tool_frame = 'end_effector_link'     # Example tool frame
        self.timer = self.create_timer(1.0, self.get_current_pose) # Example timer

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            # Wait up to 1 second for transform to become available
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                now,
                timeout=Duration(seconds=1.0))

            # Extract pose information
            current_pose = PoseStamped()
            current_pose.header.stamp = trans.header.stamp
            current_pose.header.frame_id = self.base_frame
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation

            self.get_logger().info(f'Current Pose ({self.tool_frame} in {self.base_frame}): \n{current_pose.pose}')
            return current_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {self.tool_frame} to {self.base_frame}: {e}')
            return None
        
def main():
    rclpy.init()

    inverse_kinematics_server = PoseListener()

    rclpy.spin(inverse_kinematics_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()