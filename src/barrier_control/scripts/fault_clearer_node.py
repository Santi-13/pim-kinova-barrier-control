#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Common service type for such actions
import sys

class FaultClearer(Node):
    def __init__(self, robot_namespace_param=""):
        # Ensure unique node name, especially if launching multiple
        node_name = f'fault_clearer_{robot_namespace_param.replace("/", "_")}'
        if not robot_namespace_param: # Handle empty case for node name
            node_name = 'fault_clearer_default'
        super().__init__(node_name)

        self.robot_namespace = robot_namespace_param # Store the passed namespace
        self.cli = None

        # Construct service name based on the provided namespace parameter
        if self.robot_namespace:
            # Ensure leading slash if namespace doesn't have it, but typical ROS ns does.
            # If robot_namespace is like "robot1", service becomes "/robot1/fault_controller/reset_faults"
            ns_prefix = self.robot_namespace if self.robot_namespace.startswith('/') else f'/{self.robot_namespace}'
            self.service_name = f'{ns_prefix}/fault_controller/reset_faults'
        else:
            self.service_name = '/fault_controller/reset_faults' # Global if no namespace

        self.get_logger().info(f"FaultClearer initialized for namespace '{self.robot_namespace}'. "
                               f"Will target service: '{self.service_name}' on shutdown.")

        # Register the shutdown hook
        rclpy.get_default_context().on_shutdown(self.on_node_shutdown)

    def on_node_shutdown(self):
        self.get_logger().info(
            f"Shutdown initiated for FaultClearer (ns: '{self.robot_namespace}'). "
            f"Attempting to clear faults via service: {self.service_name}")

        if self.cli is None: # Create client if not already created
            self.cli = self.create_client(Trigger, self.service_name)

        if not self.cli.service_is_ready():
            # Attempt to wait for a short period, as other nodes might be shutting down too.
            if not self.cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"Service '{self.service_name}' not available during shutdown. Cannot clear faults.")
                return

        req = Trigger.Request()
        try:
            # Asynchronous call with a spin to allow completion during shutdown
            future = self.cli.call_async(req)

            # We need to spin briefly for the call to be processed.
            # This is a delicate part of shutdown hooks.
            # Create a temporary minimal event loop for the future.
            timeout_sec = 3.0 
            start_time = self.get_clock().now()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1) # Spin self briefly
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                    self.get_logger().error(f"Service call to '{self.service_name}' timed out during shutdown.")
                    break

            if future.done() and future.result() is not None:
                if future.result().success:
                    self.get_logger().info(
                        f"Fault clear service '{self.service_name}' called successfully: {future.result().message}")
                else:
                    self.get_logger().warn(
                        f"Fault clear service '{self.service_name}' call failed: {future.result().message}")
            elif not future.done(): # Only log if it didn't complete (wasn't just a timeout logged above)
                 self.get_logger().error(f"Service call future for '{self.service_name}' did not complete.")

        except Exception as e:
            self.get_logger().error(f"Exception while calling fault clear service '{self.service_name}': {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create a temporary node to easily read the parameter passed from the launch file
    temp_node = rclpy.create_node('fault_clearer_param_reader')
    temp_node.declare_parameter('robot_namespace_for_fault_clearer', '')
    robot_ns = temp_node.get_parameter('robot_namespace_for_fault_clearer').get_parameter_value().string_value
    temp_node.destroy_node()

    fault_clearer_node = FaultClearer(robot_namespace_param=robot_ns)

    try:
        rclpy.spin(fault_clearer_node)
    except KeyboardInterrupt:
        fault_clearer_node.get_logger().info("FaultClearer spinning interrupted.")
    except Exception as e:
        fault_clearer_node.get_logger().error(f"FaultClearer spinning loop error: {e}")
    finally:
        # The on_shutdown hook is the primary mechanism for action.
        # Node destruction and rclpy.shutdown() will trigger it.
        fault_clearer_node.get_logger().info("Spin complete, FaultClearer node is shutting down.")
        # rclpy.shutdown() will be called by the top-level execution if this is the main script.
        # If called from a launch file, node destruction is handled by launch system.

if __name__ == '__main__':
    main(sys.argv)