import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SequencePublisherNode(Node):
    def __init__(self):
        super().__init__('sequence_publisher_node')

        # Declare parameters
        self.declare_parameter('target_topic', '/target_coordinates')
        self.declare_parameter('sequence_number', 0) 

        self.declare_parameter('time_between_points_s', 10.0) # Seconds
        self.declare_parameter('loop_sequence', True)

        # Get parameters
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        sequence_number = self.get_parameter('sequence_number').get_parameter_value().integer_value
        self.get_logger().info(f"Executing sequence number: {sequence_number}")

        # Setting up the point sequence based on the sequence number
        if sequence_number == 0:
            raw_sequence_from_param = [ # Default value
                    [[0.4, -0.2, 0.7], [0.4, 0.2, 0.3]],  # Pair 1
                    [[0.5, -0.2, 0.5], [0.5, 0.2, 0.5]],  # Pair 2
                    [[0.6, -0.2, 0.3], [0.6, 0.2, 0.7]]   # Pair 3
                ]
        elif sequence_number == 1:
            raw_sequence_from_param = [
                    [[0.4, -0.15, 0.7], [0.2, 0.65, 0.1]],  # Pair 1
                    [[0.5, -0.15, 0.5], [0.0, 0.65, 0.7]],  # Pair 2
                    [[0.6, -0.15, 0.3], [-.5, 0.45, 0.75]]   # Pair 3
                    
                ]

        try:
            # Convert list of lists of lists of doubles (from parameter) to list of lists of lists of floats
            self.point_sequence = [
                [
                    [float(coord) for coord in point]
                    for point in pair
                ]
                for pair in raw_sequence_from_param
            ]
        except (TypeError, ValueError) as e:
            self.get_logger().error(
                f"Failed to parse point_sequence. Ensure it's a list of pairs (list of 2) of 3D points: {e}. Shutting down."
            )
            rclpy.try_shutdown()
            return
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error while parsing point_sequence: {e}. Shutting down."
            )
            rclpy.try_shutdown()
            return
        
        self.time_between_points_s = self.get_parameter('time_between_points_s').get_parameter_value().double_value
        self.loop_sequence = self.get_parameter('loop_sequence').get_parameter_value().bool_value

        # Validate sequence structure (list of pairs, each pair has 2 points, each point has 3 coords)
        for i, pair in enumerate(self.point_sequence):
            if not isinstance(pair, list) or len(pair) != 2:
                self.get_logger().error(
                    f"Item {i} in sequence is not a pair of 2 points. Expected list of 2 points, found: {pair}. Shutting down."
                )
                rclpy.try_shutdown()
                return
            for j, point in enumerate(pair):
                if not isinstance(point, list) or len(point) != 3:
                    self.get_logger().error(
                        f"Point {j} in pair {i} does not have 3 coordinates. Expected list of 3 floats, found: {point}. Shutting down."
                    )
                    rclpy.try_shutdown()
                    return

        # Publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, target_topic, 10)

        # Sequence state
        self.current_pair_index = 0

        if not self.point_sequence:
            self.get_logger().warn("Point sequence (list of pairs) is empty. Node will not publish anything.")
            return

        self.get_logger().info(f"Starting sequence publisher. Target topic: '{target_topic}'")
        self.get_logger().info(f"Sequence of pairs: {self.point_sequence}")
        self.get_logger().info(f"Time between sending pairs: {self.time_between_points_s}s, Loop: {self.loop_sequence}")

        # Timer to send points
        self.timer = self.create_timer(self.time_between_points_s, self.send_next_point)
        
        # Send the first point immediately if time_between_points_s > 0,
        # otherwise the timer will handle it.
        # Or, to be simpler, let the timer handle the first point too after the initial delay.
        # If an immediate first point is desired, call self.send_next_point() here and adjust timer logic.
        self.get_logger().info(f"Waiting {self.time_between_points_s}s to send the first pair...")


    def send_next_point(self):
        if not self.point_sequence:
            return

        if self.current_pair_index >= len(self.point_sequence):
            if self.loop_sequence:
                self.get_logger().info("End of sequence reached. Looping back to the start.")
                self.current_pair_index = 0
            else:
                self.get_logger().info("End of sequence reached and not looping. Stopping publisher.")
                self.timer.cancel() # Stop the timer
                # Optionally, shutdown the node:
                # self.get_logger().info("Shutting down sequence publisher node.")
                # rclpy.try_shutdown()
                return

        current_pair = self.point_sequence[self.current_pair_index]
        point1_coords = current_pair[0]
        point2_coords = current_pair[1]

        # Publish first point of the pair
        msg1 = Float64MultiArray()
        msg1.data = point1_coords
        self.publisher_.publish(msg1)
        self.get_logger().info(f'Sent point 1 of pair {self.current_pair_index + 1}/{len(self.point_sequence)}: {point1_coords}')

        # Publish second point of the pair
        msg2 = Float64MultiArray()
        msg2.data = point2_coords
        self.publisher_.publish(msg2)
        self.get_logger().info(f'Sent point 2 of pair {self.current_pair_index + 1}/{len(self.point_sequence)}: {point2_coords}')

        self.current_pair_index += 1

def main(args=None):
    rclpy.init(args=args)
    sequence_publisher_node = SequencePublisherNode()
    rclpy.spin(sequence_publisher_node)
    sequence_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()