import roboticstoolbox as rtb
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs # Required for transforming geometry_msgs types if needed
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

class RigidBodyDynamicsController(Node):
    def __init__(self):
        super().__init__('rigid_body_dynamics_controller')

        ### Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link' 
        self.tool_frame = 'end_effector_link'     
        self.timer = self.create_timer(1.0, self.get_current_pose) 
        self.now = None

        ### State
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.joint_names = None

    
    def get_current_pose(self):
        try:
            # now = rclpy.time.Time()
            # Wait up to 1 second for transform to become available
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                self.now,
                timeout=Duration(seconds=10.0))

            # Extract pose information
            current_pose = PoseStamped()
            current_pose.header.stamp = trans.header.stamp
            current_pose.header.frame_id = self.base_frame
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation

            self.get_logger().info(f'Current Pose ({self.tool_frame} in {self.base_frame}): {current_pose.pose}')
            return current_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {self.tool_frame} to {self.base_frame}: {e}')
            return None
        

    def joint_state_callback(self, msg: JointState):
        # Ensure msg.name aligns with expected joint order if necessary
        if not self.joint_names:
             self.joint_names = msg.name # Store the order on first message
        # Store positions in the correct order
        
        try:
            self.now = msg.header.stamp
            ordered_positions = [msg.position[msg.name.index(j_name)] for j_name in self.joint_names]
            ordered_velocities = [msg.velocity[msg.name.index(j_name)] for j_name in self.joint_names]
            self.current_joint_positions = ordered_positions
            self.current_joint_velocities = ordered_velocities
            self.get_logger().info(f'Received joint states: {self.current_joint_positions}\n{self.current_joint_velocities}')
        except ValueError as e:
            self.get_logger().error(f"Error processing joint states: {e}")

def main():
    rclpy.init()

    inverse_kinematics_server = RigidBodyDynamicsController()

    rclpy.spin(inverse_kinematics_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # Assume 'urdf_string' contains the URDF loaded from the parameter server
# # Or load directly from file if parameter server is not used in this node context
# robot = rtb.ERobot.URDF(urdf_string) # Pseudocode for loading from string
# # Example loading from file (adjust path as needed):
# robot = rtb.models.URDF.UR3() # Replace with loading Kinova URDF

# # --- Inside the control loop ---
# current_joint_positions = self.current_joint_positions # From subscriber (Section 3)
# #Ensure current_joint_positions is a numpy array with the correct shape
# q_np = np.array(current_joint_positions)


# #Calculate Jacobian in the base frame (Jacobian has 6 rows)
# try:
#     J_base = robot.jacob0(q_np) # Returns a 6xN numpy array
#     # Ensure J_base is 6x6 for the 6DOF arm
#     if J_base.shape == (6, 6):
#         # Proceed with using J_base
#         pass
#     else:
#         # Handle unexpected shape
#         self.get_logger().error(f"Jacobian shape mismatch: {J_base.shape}")
# except Exception as e: # Catch potential errors during calculation
#     self.get_logger().error(f"Error calculating Jacobian: {e}")
#     # Handle error, e.g., skip control cycle or command zero velocity