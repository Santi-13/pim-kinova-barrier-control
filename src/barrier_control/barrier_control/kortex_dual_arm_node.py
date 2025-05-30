#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from .dual_robot_connection import KinovaDualArmController 

from service_interface.srv import SendJointSpeeds
from sensor_msgs.msg import JointState

import math

class KortexDualArmNode(Node):
    def __init__(self):
        super().__init__('kortex_dual_arm_node')
        self.get_logger().info("Kortex Dual Arm Node starting...")

        # Declare parameters for robot IPs, ports, and credentials
        self.declare_parameter('robot_ips', ['192.168.1.10', '192.168.1.11'])
        self.declare_parameter('robot_ports', [10000, 10000]) # Default Kortex TCP port for Base

        self.declare_parameter('robot_0_username', 'admin')
        self.declare_parameter('robot_0_password', 'admin')
        self.declare_parameter('robot_1_username', 'admin')
        self.declare_parameter('robot_1_password', 'admin')

        # Joint names for each arm ( crucial for sensor_msgs/JointState)
        default_robot1_joint_names = ['robot1/joint_1', 'robot1/joint_2', 'robot1/joint_3', 'robot1/joint_4', 'robot1/joint_5', 'robot1/joint_6']
        default_robot2_joint_names = ['robot2/joint_1', 'robot2/joint_2', 'robot2/joint_3', 'robot2/joint_4', 'robot2/joint_5', 'robot2/joint_6']

        self.declare_parameter('robot1_joint_names', default_robot1_joint_names)
        self.declare_parameter('robot2_joint_names', default_robot2_joint_names)
        self.declare_parameter('joint_state_publish_rate', 50.0) # Hz

        # Get parameter values
        ips = self.get_parameter('robot_ips').get_parameter_value().string_array_value
        ports = self.get_parameter('robot_ports').get_parameter_value().integer_array_value
        
        creds = [
            (self.get_parameter('robot_0_username').get_parameter_value().string_value,
             self.get_parameter('robot_0_password').get_parameter_value().string_value),
            (self.get_parameter('robot_1_username').get_parameter_value().string_value,
             self.get_parameter('robot_1_password').get_parameter_value().string_value)
        ]

        self.arm_joint_names = [
            self.get_parameter('robot1_joint_names').get_parameter_value().string_array_value,
            self.get_parameter('robot2_joint_names').get_parameter_value().string_array_value
        ]

        self.publish_rate = self.get_parameter('joint_state_publish_rate').get_parameter_value().double_value
        self.publish_timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.02 # seconds

        self.dual_arm_controller = None # Initialize to None

        if len(ips) != 2 or len(ports) != 2:
            self.get_logger().fatal("Configuration error: Requires exactly 2 IP addresses and ports.")
            # In a real node, you might want to prevent it from spinning or cleanly exit
            # For simplicity here, we'll let it proceed but it won't connect.
            self.dual_arm_controller = None
            return 

        self.get_logger().info(f"Attempting to connect to robots at IPs: {ips} on ports: {ports}")

        try:
            # Pass the logger to your controller if you modified it to accept one
            self.dual_arm_controller = KinovaDualArmController(ips=list(ips), ports=list(ports), credentials=creds)
            self.get_logger().info("KinovaDualArmController initialized and arms should be homed.")
            # Register shutdown hook
            rclpy.get_default_context().on_shutdown(self._node_shutdown)
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize KinovaDualArmController: {str(e)}")
            self.dual_arm_controller = None # Ensure it's None if init failed
            if hasattr(self, 'dual_arm_controller') and self.dual_arm_controller:
                 self.dual_arm_controller._safe_teardown() # Call the teardown method
            raise e 
    
        # Register shutdown hook only if controller initialized successfully
        if self.dual_arm_controller:
            rclpy.get_default_context().on_shutdown(self._node_shutdown)

            # Create Service Server for SendJointSpeeds
            try:
                self.send_speeds_service = self.create_service(
                    SendJointSpeeds,
                    'send_joint_speeds',
                    self.send_joint_speeds_callback
                )
                self.get_logger().info("Service '/send_joint_speeds' is ready.")
            except Exception as e: # More specific exceptions could be useful here
                self.get_logger().error(f"Failed to create service 'send_joint_speeds': {e}")

            # Create Publishers for Joint States
            self.joint_state_pub_robot1 = self.create_publisher(JointState, 'robot1/joint_states', 10)
            self.joint_state_pub_robot2 = self.create_publisher(JointState, 'robot2/joint_states', 10)
            self.joint_state_pubs = [self.joint_state_pub_robot1, self.joint_state_pub_robot2]
            self.get_logger().info(f"Publishing joint states to '/robot1/joint_states' and '/robot2/joint_states' at {self.publish_rate} Hz.")

            # Create Timer for publishing joint states
            self.joint_state_timer = self.create_timer(self.publish_timer_period, self.publish_joint_states_callback)
        else:
            self.get_logger().error("Dual arm controller not initialized. Service '/send_joint_speeds' will not be available.")


    def _node_shutdown(self):
        """Callback for when the node is shutting down."""
        self.get_logger().info("Node is shutting down. Tearing down Kortex connections.")
        if hasattr(self, 'dual_arm_controller') and self.dual_arm_controller:
            self.dual_arm_controller._safe_teardown()
        self.get_logger().info("Kortex connections closed.")

    def send_joint_speeds_callback(self, request, response):
        """
        ROS 2 Service callback to handle requests for sending joint speeds.
        'request' and 'response' types depend on your .srv definition.
        Assuming SendJointSpeeds.srv with fields:
        request: arm_index, joint_speeds, duration, control_frequency
        response: success, message
        """
        try:
            self.get_logger().info(
                f"Received request to send joint speeds to arm {request.arm_index}: "
                f"Speeds: {list(request.joint_speeds)}, Duration: {request.duration}s, "
                f"Frequency: {request.control_frequency}Hz"
            )

            if not (0 <= request.arm_index < len(self.dual_arm_controller.arms)):
                response.success = False
                response.message = f"Invalid arm_index: {request.arm_index}. Must be 0 or 1."
                self.get_logger().error(response.message)
                return response

            if len(request.joint_speeds) != 6:
                response.success = False
                response.message = f"Invalid joint_speeds length: {len(request.joint_speeds)}. Must be 6."
                self.get_logger().error(response.message)
                return response

            # Call the method from your KinovaDualArmController
            self.dual_arm_controller.send_joint_speeds(
                arm_index=request.arm_index,
                joint_speeds=list(request.joint_speeds), # Ensure it's a list if the srv field is an array
                duration=request.duration,
                control_frequency=request.control_frequency
            )
            
            response.success = True
            response.message = f"Successfully initiated joint speeds for arm {request.arm_index}."
            self.get_logger().info(response.message)

        except ValueError as ve: # Catch specific errors from your controller
            response.success = False
            response.message = f"ValueError: {str(ve)}"
            self.get_logger().error(response.message)
        except TimeoutError as te:
            response.success = False
            response.message = f"TimeoutError: {str(te)}"
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"An unexpected error occurred: {str(e)}"
            self.get_logger().error(response.message, exc_info=True) # Log traceback
        
        return response

    def publish_joint_states_callback(self):
        """
        Called by a timer to fetch and publish real-time joint states for both arms.
        """
        if not self.dual_arm_controller:
            # self.get_logger().warn("Dual arm controller not initialized. Cannot publish joint states.", throttle_duration_sec=5)
            return

        for arm_idx in range(len(self.dual_arm_controller.arms)):
            try:
                arm_base_cyclic_client = self.dual_arm_controller.arms[arm_idx]['base_cyclic']
                feedback = arm_base_cyclic_client.RefreshFeedback()

                if not feedback.actuators:
                    # self.get_logger().warn(f"No actuator data in feedback for arm {arm_idx}.", throttle_duration_sec=5)
                    continue

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                
                num_joints_from_param = len(self.arm_joint_names[arm_idx])
                num_actuators_feedback = len(feedback.actuators)

                # Ensure we have names for the joints we get feedback for, or truncate
                max_joints_to_publish = min(num_joints_from_param, num_actuators_feedback)
                
                joint_state_msg.name = self.arm_joint_names[arm_idx][:max_joints_to_publish]
                joint_state_msg.position = []
                joint_state_msg.velocity = []
                joint_state_msg.effort = [] # Kortex calls it torque

                for i in range(max_joints_to_publish):
                    actuator = feedback.actuators[i]
                    # Assuming actuator_id corresponds to index i+1, and names are ordered 1 to N
                    joint_state_msg.position.append(math.radians(actuator.position)) # Convert deg to rad
                    joint_state_msg.velocity.append(math.radians(actuator.velocity)) # Convert deg/s to rad/s
                    joint_state_msg.effort.append(actuator.torque) # Nm

                if max_joints_to_publish < num_actuators_feedback and arm_idx == 0: # Log only once per type of message
                     self.get_logger().warn(
                        f"Arm {arm_idx}: Feedback has {num_actuators_feedback} actuators, "
                        f"but only {max_joints_to_publish} joint names are configured. Truncating.",
                        throttle_duration_sec=10)
                elif max_joints_to_publish < num_joints_from_param and arm_idx == 0:
                     self.get_logger().warn(
                        f"Arm {arm_idx}: {num_joints_from_param} joint names configured, "
                        f"but only received feedback for {max_joints_to_publish} actuators. Publishing available.",
                        throttle_duration_sec=10)


                self.joint_state_pubs[arm_idx].publish(joint_state_msg)

            except Exception as e:
                self.get_logger().error(f"Error publishing joint states for arm {arm_idx}: {e}", throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    kortex_node = None
    try:
        kortex_node = KortexDualArmNode()
        if hasattr(kortex_node, 'dual_arm_controller') and kortex_node.dual_arm_controller is not None:
            rclpy.spin(kortex_node)
        else:
            kortex_node.get_logger().fatal("Dual arm controller not initialized. Shutting down.")
    except Exception as e:
        if kortex_node:
            kortex_node.get_logger().fatal(f"Crashed with exception: {e}", exc_info=True)
        else:
            print(f"Failed to initialize KortexDualArmNode: {e}") # rclpy logger not available yet
    finally:
        if kortex_node:
            # The shutdown hook should handle teardown, but an explicit destroy is good practice
            kortex_node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()