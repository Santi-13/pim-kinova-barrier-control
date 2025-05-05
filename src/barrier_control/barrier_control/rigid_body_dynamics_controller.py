import roboticstoolbox as rtb
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros
import subprocess # For running xacro command
import os # For path joining
import quaternion
import tf2_geometry_msgs # Required for transforming geometry_msgs types if needed
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import tempfile # For creating temporary files



class RigidBodyDynamicsController(Node):
    """
    A ROS 2 node for controlling a robot arm's joint velocities based on
    a target end-effector pose using Jacobian-based control.
    """
    def __init__(self):
        super().__init__('rigid_body_dynamics_controller')

        # --- Parameters ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_frame', 'end_effector_link')
        self.declare_parameter('kp_gains', [1.0]*6) # Diagonal Kp gains [x,y,z,rx,ry,rz]
        self.declare_parameter('kd_gains', [0.0]*6) # Diagonal Kd gains [x,y,z,rx,ry,rz] - Start with zeros or small values
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('velocity_command_topic', '/joint_group_velocity_controller/commands')
        self.declare_parameter('robot_description_package', 'kortex_description')
        self.declare_parameter('robot_description_xacro_path', 'robots/gen3.xacro')
        self.declare_parameter('xacro_args', 'dof:=6 use_fake_hardware:=true robot_ip:=dummy')
        self.declare_parameter('jacobian_damping', 0.01) # Damping factor for DLS
        self.declare_parameter('max_joint_velocities', [0.87]*6)
        self.declare_parameter('control_frequency', 50.0) # Hz
        self.declare_parameter('error_tolerance', [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]) # [m, m, m, rad, rad, rad]

        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        kp_diag = self.get_parameter('kp_gains').value
        self.kp = np.diag(kp_diag) # Create diagonal matrix from list
        kd_diag = self.get_parameter('kd_gains').value
        self.kd = np.diag(kd_diag) # Create diagonal matrix from list
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        velocity_command_topic = self.get_parameter('velocity_command_topic').value
        self.jacobian_damping = self.get_parameter('jacobian_damping').value
        self.max_joint_velocities_np = np.array(self.get_parameter('max_joint_velocities').value)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.error_tolerance_np = np.array(self.get_parameter('error_tolerance').value)
        
        # --- ---

        ### Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)    
        self.robot = None

        ### State
        self.joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10)
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.joint_names = None

        ### Target Pose
        self.pose_target_subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.pose_target_callback,
            10
        )
        self.active_target_pose: PoseStamped | None = None # Store the active goal
        self.pose_error = np.zeros(6)

        ### Joint Velocity Publisher
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            velocity_command_topic, # Target controller topic
            10)

        ### Load robot model at initialization
        self.load_robot()

        ### Control Loop Timer
        if self.control_frequency > 0:
            timer_period = 1.0 / self.control_frequency
            self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
            self.get_logger().info(f"Control loop running at {self.control_frequency} Hz.")
    
    def get_current_pose_from_tf(self)-> PoseStamped | None:
        """Fetches the current pose of the tool_frame relative to the base_frame."""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0))

            # Extract pose information
            current_pose = PoseStamped()
            current_pose.header.stamp = trans.header.stamp
            current_pose.header.frame_id = self.base_frame
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation
            self.get_logger().debug(f'Current Pose ({self.tool_frame} in {self.base_frame}): \n{current_pose.pose}')
            
            return current_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {self.tool_frame} to {self.base_frame}: {e}')
            return None

    def joint_state_callback(self, msg: JointState):        
        try:
            # Store the order on first message, ensure it matches the robot model later if needed
            if self.joint_names is None:
                 self.joint_names = msg.name
                 # Optional: Add check here to ensure self.joint_names matches self.robot.joint_names if robot is loaded

            ordered_positions = [msg.position[msg.name.index(j_name)] for j_name in self.joint_names]
            ordered_velocities = np.array([msg.velocity[msg.name.index(j_name)] for j_name in self.joint_names]) # Store as numpy array
            self.current_joint_positions = ordered_positions
            self.current_joint_velocities = ordered_velocities
            # self.get_logger().debug(f'Received joint states: {self.current_joint_positions}\n{self.current_joint_velocities}')
        except ValueError as e:
            self.get_logger().error(f"Error processing joint states: {e}")

    def pose_target_callback(self, msg: PoseStamped):
        """Callback to receive and store the desired target pose."""
        self.get_logger().info(f'Received new target pose: {msg.pose}')
        self.active_target_pose = msg # Store the new target
        # Reset pose error calculation (will be recalculated in control loop)
        self.pose_error = np.zeros(6)

    def calculate_pose_error(self, current_pose: PoseStamped, target_pose: PoseStamped) -> np.ndarray:
        """Calculates the 6D pose error vector between current and target poses."""
        # --- Pose Error ---
        # Position error
        position_error = np.array([
            target_pose.pose.position.x - current_pose.pose.position.x,
            target_pose.pose.position.y - current_pose.pose.position.y,
            target_pose.pose.position.z - current_pose.pose.position.z
        ])
        
        
        # Orientation Error Calculation
        # Convert ROS quaternions (x,y,z,w) to numpy-quaternion objects (w,x,y,z)
        
        q_target_npq = np.quaternion(
            target_pose.pose.orientation.w,
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z
        )
        
        q_current_npq = np.quaternion(
            current_pose.pose.orientation.w, # Use the passed current_pose
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z
        )
        
        # Calculate error quaternion: rotation from current to target
        # q_target = q_error * q_current => q_error = q_target * conjugate(q_current)
        q_error = q_target_npq * q_current_npq.conjugate()

        # Calculate the 3D angular error vector (axis * angle)
        # Use the vector part of the error quaternion.
        # Multiply by 2 to get the correct magnitude for small angles.
        # Ensure shortest path rotation by checking the scalar part's sign.
        angular_error = 2.0 * q_error.vec # q_error.vec is [x, y, z]
        if q_error.w < 0:
                angular_error *= -1.0

        # Combine into a 6D error vector [dx, dy, dz, ex, ey, ez]
        return np.concatenate((position_error, angular_error))

    def load_robot(self):
        """Loads the robot model from XACRO by converting it to URDF."""
        tmp_file_path = None
        try:
            # --- Package and Xacro file ---
            package_name = self.get_parameter('robot_description_package').value
            xacro_file_relative_path = self.get_parameter('robot_description_xacro_path').value
            xacro_args_str = self.get_parameter('xacro_args').value
            xacro_args = xacro_args_str.split() # Split args string into a list
            # --- ---

            package_share_directory = get_package_share_directory(package_name)
            xacro_file_path = os.path.join(package_share_directory, xacro_file_relative_path)

            self.get_logger().info(f"Attempting to load robot model from: {xacro_file_path} with args: {xacro_args}")

            # Run the xacro command
            command = ['ros2', 'run', 'xacro', 'xacro', xacro_file_path] + xacro_args
            process = subprocess.run(command,
                capture_output=True, text=True, check=True, encoding='utf-8'
            )
            urdf_string = process.stdout

            # --- Workaround: Save URDF string to a temporary file ---
            with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.urdf', encoding='utf-8') as tmp_file:
                tmp_file.write(urdf_string)
                tmp_file_path = tmp_file.name # Store the path
                self.get_logger().debug(f"Saved temporary URDF to: {tmp_file_path}")
            # --- ---

            # Load the URDF string into roboticstoolbox
            self.robot = rtb.ERobot.URDF(tmp_file_path)
            self.get_logger().info(f"Successfully loaded robot model:\n{self.robot}")

        except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e:
            self.get_logger().error(f"Failed to load robot model: {e}")

        finally:
            # Clean up the temporary file if it was created
            if tmp_file_path and os.path.exists(tmp_file_path):
                os.remove(tmp_file_path)
                self.get_logger().debug(f"Removed temporary URDF file: {tmp_file_path}")

    def calculate_jacobian(self):
        try:
            # --- Add Pre-checks ---
            if self.robot is None:
                self.get_logger().warn("Robot model not loaded yet. Skipping Jacobian calculation.")
                return None
            if self.current_joint_positions is None:
                self.get_logger().warn("Joint positions not yet received. Skipping Jacobian calculation.")
                return None
            # --- ---

            # Ensure current_joint_positions is a numpy array with the correct shape
            q_np = np.array(self.current_joint_positions)

            # Calculate Jacobian in the base frame (Jacobian has 6 rows)
            J_base = self.robot.jacob0(q_np) # Returns a 6xN numpy array
            

            # Ensure J_base is 6xN for the 6DOF arm
            if J_base.shape[0] == 6 and J_base.shape[1] == len(q_np):
                self.get_logger().debug(f"Jacobian calculated successfully:\n{J_base}")
                return J_base
            else:
                self.get_logger().error(f"Jacobian shape mismatch: {J_base.shape}")
                return None
            
        except Exception as e:
            self.get_logger().error(f"Error calculating Jacobian: {e}")
            return None
        
    def calculate_and_limit_joint_velocities(self, jacobian: np.ndarray, pose_error: np.ndarray) -> np.ndarray | None:
        """Calculates and limits joint velocities based on Jacobian, pose error, and current joint velocities (for Kd term)."""
        try:
            # --- Pre-checks ---
            if jacobian is None:
                self.get_logger().warn("Jacobian not available for velocity calculation.")
                return None
            if pose_error is None: # Should generally not happen if called correctly
                self.get_logger().warn("Pose Error not available for velocity calculation.")
                return None
            if self.current_joint_velocities is None:
                self.get_logger().warn("Current joint velocities not available for velocity calculation (Kd term).")
                return None

            # --- Calculate desired end-effector velocity (proportional control) ---
            pose_error_np = pose_error.reshape(-1, 1)

            # --- Calculate joint velocities using Damped Least Squares (DLS) ---
            # J_dls = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + self.jacobian_damping**2 * np.eye(6)) # DLS formula
            # joint_velocities_raw = J_dls @ v_desired

            # --- Calculate current end-effector velocity (for derivative term) ---
            q_dot_current = self.current_joint_velocities.reshape(-1, 1)
            v_current = jacobian @ q_dot_current

            # --- Combine P and D terms for desired EE velocity ---
            v_desired_pd = self.kp @ pose_error_np - self.kd @ v_current

            # --- Or using standard Pseudo-Inverse ---
            J_pseudo_inverse = np.linalg.pinv(jacobian)
            joint_velocities_raw = J_pseudo_inverse @ v_desired_pd # Use PD desired velocity

            # --- Velocity Limiting ---
            joint_velocities_limited = np.clip(joint_velocities_raw.flatten(), -self.max_joint_velocities_np, self.max_joint_velocities_np)

            self.get_logger().debug(f"Raw Vels: {joint_velocities_raw.flatten()}, Limited Vels: {joint_velocities_limited}")
            return joint_velocities_limited

        except Exception as e:
            self.get_logger().error(f"Error calculating/limiting joint velocities: {e}")
            return None

    def publish_velocities(self, velocities: np.ndarray | None):
        """Publishes the calculated joint velocities."""
        if velocities is None:
            # If calculation failed, publish zeros as a safety measure
            num_joints = len(self.max_joint_velocities_np)
            velocities = np.zeros(num_joints)

        msg = Float64MultiArray()
        msg.data = velocities.tolist()
        self.velocity_pub.publish(msg)

    def control_loop_callback(self):
        """Main control loop executed at a fixed frequency."""
        if self.active_target_pose is None:
            # No active target, ensure robot stops
            self.publish_velocities(None)
            return

        # Check prerequisites
        if self.robot is None or self.current_joint_positions is None or self.current_joint_velocities is None:
            self.get_logger().warn("Robot model or joint states not ready. Skipping control loop iteration.", throttle_duration_sec=5)
            self.publish_velocities(None) # Publish zeros
            return

        current_pose = self.get_current_pose_from_tf()
        if current_pose is None:
            self.get_logger().warn("Could not get current pose. Skipping control loop iteration.", throttle_duration_sec=5)
            self.publish_velocities(None) # Publish zeros
            return
        
        # Calculate error
        self.pose_error = self.calculate_pose_error(current_pose, self.active_target_pose)
        self.get_logger().debug(f'6D Pose error: {self.pose_error}')

        

        # Check if target is reached
        if np.all(np.abs(self.pose_error) < self.error_tolerance_np):
            self.get_logger().info("Target reached within tolerance.")
            self.publish_velocities(np.zeros(len(self.max_joint_velocities_np))) # Publish zero velocity
            self.active_target_pose = None # Deactivate target until a new one arrives
            return # Exit the control loop for this cycle

        # --- If target not reached, calculate and publish velocities ---
        jacobian = self.calculate_jacobian()
        joint_velocities = self.calculate_and_limit_joint_velocities(jacobian, self.pose_error)
        self.publish_velocities(joint_velocities)

def main():
    rclpy.init()

    inverse_kinematics_server = RigidBodyDynamicsController()    

    rclpy.spin(inverse_kinematics_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
