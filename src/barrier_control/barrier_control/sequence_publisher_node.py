import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class SequencePublisherNode(Node):
    def __init__(self):
        super().__init__('sequence_publisher_node')

        # Declare parameters
        self.declare_parameter('target_topic', '/target_coordinates')
        self.declare_parameter('point_sequence', [
            [0.4, -0.15, 0.4],
            [0.4, -0.15, 0.5],
            [0.4, -0.15, 0.6]
        ]) # Default sequence
        self.declare_parameter('time_between_points_s', 5.0) # Seconds
        self.declare_parameter('loop_sequence', False)

        # Get parameters
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.point_sequence = self.get_parameter('point_sequence').get_parameter_value().double_array_array_value
        # Convert list of lists of doubles (from parameter) to list of lists of floats
        self.point_sequence = [[float(coord) for coord in point] for point in self.point_sequence]

        self.time_between_points_s = self.get_parameter('time_between_points_s').get_parameter_value().double_value
        self.loop_sequence = self.get_parameter('loop_sequence').get_parameter_value().bool_value

        # Validate points
        for i, point in enumerate(self.point_sequence):
            if len(point) != 3:
                self.get_logger().error(
                    f"Point {i} in sequence has {len(point)} coordinates, expected 3 (x,y,z). Shutting down."
                )
                rclpy.try_shutdown()
                return

        # Publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, target_topic, 10)

        # Sequence state
        self.current_point_index = 0

        if not self.point_sequence:
            self.get_logger().warn("Point sequence is empty. Node will not publish anything.")
            return

        self.get_logger().info(f"Starting sequence publisher. Target topic: '{target_topic}'")
        self.get_logger().info(f"Point sequence: {self.point_sequence}")
        self.get_logger().info(f"Time between points: {self.time_between_points_s}s, Loop: {self.loop_sequence}")

        # Timer to send points
        self.timer = self.create_timer(self.time_between_points_s, self.send_next_point)
        
        # Send the first point immediately if time_between_points_s > 0,
        # otherwise the timer will handle it.
        # Or, to be simpler, let the timer handle the first point too after the initial delay.
        # If an immediate first point is desired, call self.send_next_point() here and adjust timer logic.
        self.get_logger().info(f"Waiting {self.time_between_points_s}s to send the first point...")


    def send_next_point(self):
        if not self.point_sequence:
            return

        if self.current_point_index >= len(self.point_sequence):
            if self.loop_sequence:
                self.get_logger().info("End of sequence reached. Looping back to the start.")
                self.current_point_index = 0
            else:
                self.get_logger().info("End of sequence reached. Stopping publisher.")
                self.timer.cancel() # Stop the timer
                # Optionally, shutdown the node:
                # self.get_logger().info("Shutting down sequence publisher node.")
                # rclpy.try_shutdown()
                return

        point_to_send = self.point_sequence[self.current_point_index]

        msg = Float64MultiArray()
        msg.data = point_to_send # msg.data expects a list of floats

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent point {self.current_point_index + 1}/{len(self.point_sequence)}: {point_to_send}')

        self.current_point_index += 1

def main(args=None):
    rclpy.init(args=args)
    sequence_publisher_node = SequencePublisherNode()
    rclpy.spin(sequence_publisher_node)
    sequence_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()