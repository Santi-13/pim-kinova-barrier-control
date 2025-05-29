#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import time # Keep time for direct Kortex API usage if needed

from dual_robot_connection import KinovaDualArmController 

from srv import SendJointSpeeds
# For now, let's define a placeholder if the custom service isn't built yet
# To make this runnable without building the custom service immediately,
# we'll describe its structure and how it would be used.
# When you build the srv, uncomment the import above and use SendJointSpeeds.Request and SendJointSpeeds.Response


class KortexDualArmNode(Node):
    def __init__(self):
        super().__init__('kortex_dual_arm_node')
        self.get_logger().info("Kortex Dual Arm Node starting...")

        # Declare parameters for robot IPs, ports, and credentials
        self.declare_parameter('robot_ips', ['192.168.1.10', '192.168.1.11'])
        self.declare_parameter('robot_ports', [10000, 10000]) # Default Kortex TCP port for Base
        # For credentials, it's a bit trickier with lists of tuples in ROS params.
        # Often, you'd flatten this or read from a YAML file structured appropriately.
        # For simplicity, let's assume two sets of username/password params.
        self.declare_parameter('robot_0_username', 'admin')
        self.declare_parameter('robot_0_password', 'admin')
        self.declare_parameter('robot_1_username', 'admin')
        self.declare_parameter('robot_1_password', 'admin')

        # Get parameter values
        ips = self.get_parameter('robot_ips').get_parameter_value().string_array_value
        ports = self.get_parameter('robot_ports').get_parameter_value().integer_array_value
        
        creds = [
            (self.get_parameter('robot_0_username').get_parameter_value().string_value,
             self.get_parameter('robot_0_password').get_parameter_value().string_value),
            (self.get_parameter('robot_1_username').get_parameter_value().string_value,
             self.get_parameter('robot_1_password').get_parameter_value().string_value)
        ]

        if len(ips) != 2 or len(ports) != 2:
            self.get_logger().fatal("Configuration error: Requires exactly 2 IP addresses and ports.")
            # Consider raising an exception or sys.exit(1) if rclpy allows clean exit from constructor
            return 

        self.get_logger().info(f"Attempting to connect to robots at IPs: {ips} on ports: {ports}")

        try:
            # Instantiate your controller
            # The KinovaDualArmController uses 'with' statement, so we can replicate that
            # or handle __enter__ and __exit__ manually if we store it as a member.
            # For a long-running node, we initialize and store it.
            self.dual_arm_controller = KinovaDualArmController(ips=list(ips), ports=list(ports), credentials=creds)
            # The __init__ of KinovaDualArmController calls _setup_connection which includes homing.
            self.get_logger().info("KinovaDualArmController initialized and arms should be homed.")
            
            # Register shutdown hook
            rclpy.get_default_context().on_shutdown(self._node_shutdown)

        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize KinovaDualArmController: {str(e)}")
            # Clean up if partially initialized dual_arm_controller, though its __init__ should handle it
            if hasattr(self, 'dual_arm_controller') and self.dual_arm_controller:
                 self.dual_arm_controller._safe_teardown() # Call the teardown method
            # Make sure rclpy can exit cleanly. Raising an exception here might be okay.
            raise e # Re-raise to stop node creation if critical

        # Create the service server for sending joint speeds
        # Replace 'PlaceholderServiceType' with your actual 'SendJointSpeeds' when defined and built
        # self.send_speeds_service = self.create_service(
        #     SendJointSpeeds, # Your custom service type
        #     'send_joint_speeds',
        #     self.send_joint_speeds_callback
        # )
        # self.get_logger().info("Service '/send_joint_speeds' is ready.")
        
        # --- Placeholder for demonstrating without custom service ---
        # As a temporary measure if SendJointSpeeds.srv is not yet built,
        # you could use a simple subscriber for testing, e.g., to a Float64MultiArray
        # But a service is the correct approach. For now, I'll comment out the service creation
        # and you can uncomment and use it once your .srv file is processed by your build system.
        self.get_logger().warn("Custom service 'SendJointSpeeds' is not implemented in this snippet.")
        self.get_logger().warn("Uncomment and use 'self.create_service(SendJointSpeeds, ...)' after building the .srv")
        # Example of how the service structure might look (replace with your actual .srv import):
        # Example Request fields: arm_index, joint_speeds, duration, control_frequency
        # Example Response fields: success, message
        # For now, we will define a dummy service callback to show structure
        # You would need to create the SendJointSpeeds.srv file and build it:
        #
        # kortex_commander/srv/SendJointSpeeds.srv
        # ```
        # int32 arm_index
        # float64[6] joint_speeds
        # float64 duration
        # int32 control_frequency
        # ---
        # bool success
        # string message
        # ```
        # Then in CMakeLists.txt:
        # find_package(rosidl_default_generators REQUIRED)
        # rosidl_generate_interfaces(${PROJECT_NAME} "srv/SendJointSpeeds.srv")
        #
        # In package.xml:
        # <build_depend>rosidl_default_generators</build_depend>
        # <exec_depend>rosidl_default_runtime</exec_depend>
        # <member_of_group>rosidl_interface_packages</member_of_group>

        # Assuming the srv is built, it would be:
        # try:
        #     from srv import SendJointSpeeds # Adjust if your package name for srv is different
        #     self.send_speeds_service = self.create_service(
        #         SendJointSpeeds,
        #         'send_joint_speeds',
        #         self.send_joint_speeds_callback
        #     )
        #     self.get_logger().info("Service '/send_joint_speeds' is ready.")
        # except ImportError:
        #     self.get_logger().error("Service 'SendJointSpeeds' not found. Please build the interface package.")
        #     self.get_logger().error("Functionality to send joint speeds via ROS service will be unavailable.")


    def _node_shutdown(self):
        """Callback for when the node is shutting down."""
        self.get_logger().info("Node is shutting down. Tearing down Kortex connections.")
        if hasattr(self, 'dual_arm_controller') and self.dual_arm_controller:
            self.dual_arm_controller._safe_teardown()
        self.get_logger().info("Kortex connections closed.")

    # def send_joint_speeds_callback(self, request, response):
    #     """
    #     ROS 2 Service callback to handle requests for sending joint speeds.
    #     'request' and 'response' types depend on your .srv definition.
    #     Assuming SendJointSpeeds.srv with fields:
    #     request: arm_index, joint_speeds, duration, control_frequency
    #     response: success, message
    #     """
    #     try:
    #         self.get_logger().info(
    #             f"Received request to send joint speeds to arm {request.arm_index}: "
    #             f"Speeds: {list(request.joint_speeds)}, Duration: {request.duration}s, "
    #             f"Frequency: {request.control_frequency}Hz"
    #         )

    #         if not (0 <= request.arm_index < len(self.dual_arm_controller.arms)):
    #             response.success = False
    #             response.message = f"Invalid arm_index: {request.arm_index}. Must be 0 or 1."
    #             self.get_logger().error(response.message)
    #             return response

    #         if len(request.joint_speeds) != 6:
    #             response.success = False
    #             response.message = f"Invalid joint_speeds length: {len(request.joint_speeds)}. Must be 6."
    #             self.get_logger().error(response.message)
    #             return response

    #         # Call the method from your KinovaDualArmController
    #         self.dual_arm_controller.send_joint_speeds(
    #             arm_index=request.arm_index,
    #             joint_speeds=list(request.joint_speeds), # Ensure it's a list if the srv field is an array
    #             duration=request.duration,
    #             control_frequency=request.control_frequency
    #         )
            
    #         response.success = True
    #         response.message = f"Successfully initiated joint speeds for arm {request.arm_index}."
    #         self.get_logger().info(response.message)

    #     except ValueError as ve: # Catch specific errors from your controller
    #         response.success = False
    #         response.message = f"ValueError: {str(ve)}"
    #         self.get_logger().error(response.message)
    #     except TimeoutError as te:
    #         response.success = False
    #         response.message = f"TimeoutError: {str(te)}"
    #         self.get_logger().error(response.message)
    #     except Exception as e:
    #         response.success = False
    #         response.message = f"An unexpected error occurred: {str(e)}"
    #         self.get_logger().error(response.message, exc_info=True) # Log traceback
        
    #     return response


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