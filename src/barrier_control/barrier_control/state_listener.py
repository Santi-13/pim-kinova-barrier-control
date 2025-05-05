import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StateListener(Node):
    def __init__(self):
        super().__init__('state_listener')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.joint_names = None

    def joint_state_callback(self, msg):
        # Ensure msg.name aligns with expected joint order if necessary
        if not self.joint_names:
             self.joint_names = msg.name # Store the order on first message
        # Store positions in the correct order
        try:
            ordered_positions = [msg.position[msg.name.index(j_name)] for j_name in self.joint_names]
            ordered_velocities = [msg.velocity[msg.name.index(j_name)] for j_name in self.joint_names]
            self.current_joint_positions = ordered_positions
            self.current_joint_velocities = ordered_velocities
            self.get_logger().info(f'Received joint states: {self.current_joint_positions}\n{self.current_joint_velocities}')
        except ValueError as e:
            self.get_logger().error(f"Error processing joint states: {e}")

def main():
    rclpy.init()

    inverse_kinematics_server = StateListener()

    rclpy.spin(inverse_kinematics_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()