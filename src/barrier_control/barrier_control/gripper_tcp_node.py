#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

# Configuration for the TCP Server
HOST = "192.168.0.100"  # IP address of the TCP server
PORT = 4001             # Port number of the TCP server

class GripperTCPControllerNode(Node):
    """
    A ROS 2 node that controls a gripper via TCP commands.
    It subscribes to a topic to receive 'open' or 'close' commands
    and sends them to a specified TCP server.
    """
    def __init__(self):
        super().__init__('gripper_tcp_controller_node')
        
        # Declare parameters for HOST and PORT for better flexibility
        self.declare_parameter('tcp_host', HOST)
        self.declare_parameter('tcp_port', PORT)
        
        # Get the parameters
        self.host = self.get_parameter('tcp_host').get_parameter_value().string_value
        self.port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        
        self.get_logger().info(f"Attempting to connect to gripper server at {self.host}:{self.port}")

        # Create a subscriber to the '/gripper_command' topic.
        # The message type is std_msgs.msg.String.
        # The queue size is 10.
        self.subscription = self.create_subscription(
            String,
            'gripper_command',  # Topic name
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Gripper TCP Controller Node has started.")
        self.get_logger().info("Listening for commands on /gripper_command topic.")
        self.get_logger().info("Valid commands are 'a' = 'open' or 'c' = 'close'.")

    def command_callback(self, msg):
        """
        Callback function that is executed when a message is received on the '/gripper_command' topic.
        """
        command = msg.data.lower()  # Convert command to lowercase, e.g., "OPEN" -> "open"
        self.get_logger().info(f'Received command: "{command}"')

        if command not in ["a", "c"]:
            self.get_logger().warn(f'Invalid command: "{command}". Only "a" = "open" or "c" = "close" are accepted.')
            return

        server_command = command

        try:
            # Create a new socket connection for each command, as in the original script.
            # AF_INET for IPv4, SOCK_STREAM for TCP.
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                self.get_logger().info(f'Connecting to {self.host}:{self.port}...')
                s.connect((self.host, self.port))
                self.get_logger().info(f'Connected. Sending command: "{server_command}"')
                
                # Send the command, encoded to bytes (UTF-8 is common).
                s.sendall(server_command.encode())
                self.get_logger().info(f'Command "{server_command}" sent successfully.')
                
                # You might want to receive a response from the server here if it sends one.
                # For example:
                # response = s.recv(1024)
                # self.get_logger().info(f'Received response: {response.decode("utf-8")}')

        except socket.timeout:
            self.get_logger().error(f'Connection to {self.host}:{self.port} timed out.')
        except socket.error as e:
            self.get_logger().error(f'Socket error when trying to send command: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    gripper_controller_node = GripperTCPControllerNode()

    try:
        rclpy.spin(gripper_controller_node)  # Keep the node alive to process callbacks
    except KeyboardInterrupt:
        gripper_controller_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Clean up and shutdown the node
        gripper_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()