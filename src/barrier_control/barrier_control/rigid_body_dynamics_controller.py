import roboticstoolbox as rtb
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf2_ros
import subprocess # For running xacro command
import os # For path joining
import math

from geometry_msgs.msg import TransformStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # New import
from rclpy.duration import Duration as RclpyDuration 
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import tempfile # For creating temporary files
import quaternion 

class RigidBodyDynamicsController(Node):
    """
    A ROS 2 node for controlling a robot arm's joint positions based on
    a target end-effector pose using Jacobian-based control, publishing
    to a joint_trajectory_controller.
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

        self.declare_parameter('joint_trajectory_topic', '/joint_trajectory_controller/joint_trajectory') # New
        self.declare_parameter('arm_index', 0)
        self.declare_parameter('robot_description_package', 'kortex_description')
        self.declare_parameter('robot_description_xacro_path', 'robots/gen3.xacro')
        self.declare_parameter('xacro_args', 'dof:=6 use_fake_hardware:=true robot_ip:=dummy')
        self.declare_parameter('jacobian_damping', 0.01) # Damping factor for DLS
        self.declare_parameter('max_joint_velocities', [0.87]*6)
        self.declare_parameter('control_frequency', 50.0) # Hz
        self.declare_parameter('controlled_joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']) # List of joint names to control
        self.declare_parameter('joint_limit_buffer', 0.01) # Buffer for joint limits
        self.declare_parameter('error_tolerance', [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]) # [m, m, m, rad, rad, rad]
        self.declare_parameter('euler_input_convention', 'xyz') # e.g., 'xyz', 'zyx'. For interpreting incoming target orientation.


        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        kp_diag = self.get_parameter('kp_gains').value
        self.kp = np.diag(kp_diag) # Create diagonal matrix from list
        kd_diag = self.get_parameter('kd_gains').value
        self.kd = np.diag(kd_diag) # Create diagonal matrix from list
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value

        self.jacobian_damping = self.get_parameter('jacobian_damping').value
        self.max_joint_velocities_np = np.array(self.get_parameter('max_joint_velocities').value)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.joint_limit_buffer_fval = self.get_parameter('joint_limit_buffer').value # Renamed for clarity
        self.error_tolerance_np = np.array(self.get_parameter('error_tolerance').value)
        self.euler_input_convention = self.get_parameter('euler_input_convention').value.lower()     

        self.arm_index = self.get_parameter('arm_index').value

        # --- ---

        ### Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)    
        
        ### Robot Model & Joint Info
        min_limits = [
            -2 * math.pi,  # joint_1 min
            -2.2,         # joint_2 min
            -2.5,         # joint_3 min
            -2 * math.pi,  # joint_4 min
            -2.08,         # joint_5 min
            -2 * math.pi   # joint_6 min
        ]

        max_limits = [
            2 * math.pi,   # joint_1 max
            2.2,          # joint_2 max
            2.5,          # joint_3 max
            2 * math.pi,   # joint_4 max
            2.08,          # joint_5 max
            2 * math.pi    # joint_6 max
        ]

        self.robot = None
        self.joint_position_limits = np.array([min_limits, max_limits]) # Will be (2, N) numpy array [min_limits; max_limits]
        self.num_model_joints = 0

        ### State
        self.joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10)
        self.current_joint_positions = None
        self.current_joint_velocities = None

        ### Target Pose
        self.pose_target_subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.pose_target_callback,
            10
        )
        self.active_target_pose: PoseStamped | None = None # Store the active goal
        self.pose_error = np.zeros(6)

        ### Joint Trajectory Publisher (New)
        joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            joint_trajectory_topic,
            10
        )
        self.get_logger().info(f"Publishing joint trajectories to: {joint_trajectory_topic}")

        ### Load robot model at initialization
        self.load_robot()

        ### Control Loop Timer
        if self.control_frequency > 0:
            timer_period = 1.0 / self.control_frequency
            self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
            self.get_logger().info(f"Control loop running at {self.control_frequency} Hz.")

        ### Send Home Position on Startup
        if self.current_joint_positions is None:
            self.get_logger().info("Current joint positions not set. Sending home position on startup.")
            # Send home position (zeros) if no joint states received yet
            home_position = [0.0, 15.0, -130.0, 0.0, 55.0, 90.0] # home position in degrees
            # Convert to radians
            home_position_rad = np.radians(home_position)
            self.get_logger().info(f"Home position (rad): {home_position_rad}, Home position (deg): {home_position}")
            self.publish_target_joint_positions(home_position_rad, time_from_start=5.0)
        else:
            self.get_logger().info(f"Current joint positions already set: {self.current_joint_positions}. Not sending home position on startup.")
    
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
        if not self.controlled_joint_names:
            self.get_logger().warn("Controlled joint names not set. Cannot process joint states.", throttle_duration_sec=5)
            return
        if self.num_model_joints == 0: # Robot model not ready yet
            self.get_logger().debug("Robot model not loaded, skipping joint state processing.", throttle_duration_sec=5)
            return
        if len(self.controlled_joint_names) != self.num_model_joints:
            self.get_logger().error(f"Mismatch: controlled_joint_names ({len(self.controlled_joint_names)}) vs num_model_joints ({self.num_model_joints}). Cannot reliably process joint states.", throttle_duration_sec=5)
            return

        try:
            new_positions = np.zeros(self.num_model_joints)
            new_velocities = np.zeros(self.num_model_joints)
            
            all_controlled_joints_found = True
            for i, name in enumerate(self.controlled_joint_names):
                try:
                    idx_in_msg = msg.name.index(name)
                    new_positions[i] = msg.position[idx_in_msg]
                    if msg.velocity and len(msg.velocity) > idx_in_msg:
                        new_velocities[i] = msg.velocity[idx_in_msg]
                    else:
                        new_velocities[i] = 0.0 # Default to 0 if not available
                except ValueError:
                    self.get_logger().warn(f"Controlled joint '{name}' not found in received JointState message. Available: {msg.name}", throttle_duration_sec=5)
                    all_controlled_joints_found = False
                    break # Stop processing this message
            
            if all_controlled_joints_found:
                self.current_joint_positions = new_positions
                self.current_joint_velocities = new_velocities
                # self.get_logger().debug(f'Processed joint states for {self.controlled_joint_names}: P={self.current_joint_positions}, V={self.current_joint_velocities}')
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None # Invalidate on error
            self.current_joint_velocities = None

    def pose_target_callback(self, msg: PoseStamped):
        """Callback to receive and store the desired target pose."""
        self.get_logger().info(f'Received new target pose message. Position: {msg.pose.position}, Orientation (interpreted as Euler {self.euler_input_convention}): [x(roll):{msg.pose.orientation.x}, y(pitch):{msg.pose.orientation.y}, z(yaw):{msg.pose.orientation.z}]')

        # Create a new PoseStamped object to store the processed target
        # The internal active_target_pose will always use quaternions.
        processed_target_pose = PoseStamped()
        processed_target_pose.header = msg.header
        processed_target_pose.pose.position = msg.pose.position

        # Interpret incoming orientation.x, .y, .z as Euler angles
        roll = msg.pose.orientation.x
        pitch = msg.pose.orientation.y
        yaw = msg.pose.orientation.z

        try:
            if self.euler_input_convention == 'xyz':
                # Convert Euler angles (roll, pitch, yaw for 'xyz' intrinsic) to quaternion [x, y, z, w]
                # Half angles
                hr = roll * 0.5
                hp = pitch * 0.5
                hy = yaw * 0.5

                # Sines and cosines of half angles
                sr, cr = np.sin(hr), np.cos(hr)
                sp, cp = np.sin(hp), np.cos(hp)
                sy, cy = np.sin(hy), np.cos(hy)

                # Quaternion components for 'xyz' intrinsic rotation order
                # q = q_yaw * q_pitch * q_roll (if thinking about coordinate transformations)
                # or q_final = q_roll_body * q_pitch_body * q_yaw_body
                # For 'xyz' intrinsic:
                # w = cr*cp*cy + sr*sp*sy
                # x = sr*cp*cy - cr*sp*sy
                # y = cr*sp*cy + sr*cp*sy
                # z = cr*cp*sy - sr*sp*cy
                # Note: ROS quaternion is [x, y, z, w]
                processed_target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
                processed_target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
                processed_target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
                processed_target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
                
                self.get_logger().info(f"Converted Euler ('xyz') to Quaternion: {processed_target_pose.pose.orientation}")
                self.active_target_pose = processed_target_pose # Store the new target with converted quaternion
            elif self.euler_input_convention == 'quat':
                # If the input is already a quaternion, just copy it
                processed_target_pose.pose.orientation = msg.pose.orientation
                self.active_target_pose = processed_target_pose
            else:
                self.get_logger().error(f"Euler input convention '{self.euler_input_convention}' is not supported by the current numpy implementation. Only 'xyz' or 'quat' is supported. Target not updated.")
                return # Do not update target if convention is not supported
        except ValueError as e:
             self.get_logger().error(f"Invalid Euler angle convention '{self.euler_input_convention}' or angles: {e}. Target not updated.")
        
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
        
        q_target_npq = quaternion.as_quat_array([
            # current_pose.pose.orientation.w, # Use the passed current_pose
            # current_pose.pose.orientation.x,
            # current_pose.pose.orientation.y,
            # current_pose.pose.orientation.z
            target_pose.pose.orientation.w,
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z
        ])
        
        q_current_npq = quaternion.as_quat_array([
            current_pose.pose.orientation.w, # Use the passed current_pose
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z
        ])
        
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

            if self.robot:
                self.num_model_joints = self.robot.n # Number of (movable) joints in RTB model
                self.get_logger().info(f"Successfully loaded robot model with {self.num_model_joints} DoF: {self.robot.name}")

                if not self.controlled_joint_names:
                    self.get_logger().warn("'controlled_joint_names' parameter is empty. Joint limit checks and state processing might be unreliable.")
                elif len(self.controlled_joint_names) != self.num_model_joints:
                    self.get_logger().error(
                        f"Mismatch: 'controlled_joint_names' parameter has {len(self.controlled_joint_names)} names ({self.controlled_joint_names}), "
                        f"but the loaded RTB model has {self.num_model_joints} DoF (model joints: {self.robot.links_str(q='qlim')}). " # Added more detail
                        f"Controller may not function correctly. Check URDF and 'controlled_joint_names'."
                    )
                    self.joint_position_limits = None # Disable limits if counts don't match
                    self.get_logger().warn("Disabling joint position limit checks due to DoF mismatch.")
                elif self.robot.qlim is not None:
                    if self.robot.qlim.shape[0] == 2 and self.robot.qlim.shape[1] == self.num_model_joints:
                        self.joint_position_limits = self.robot.qlim
                        self.get_logger().info(f"Loaded joint position limits (min/max) for {self.num_model_joints} joints:\n{self.joint_position_limits}")
                    else:
                        self.get_logger().error(f"Joint limits shape mismatch. Expected (2, {self.num_model_joints}), got {self.robot.qlim.shape}. Disabling joint limit checks.")
                        self.joint_position_limits = None
                else:
                    self.get_logger().warn("Robot model (qlim) does not define joint limits. Position limits will not be enforced by this controller.")
                    self.joint_position_limits = None
                    
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
            if not self.controlled_joint_names or len(self.controlled_joint_names) != self.num_model_joints :
                self.get_logger().warn("Controlled joint names misconfigured or mismatch with model DoF. Skipping Jacobian calculation.", throttle_duration_sec=5)
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
            if J_base.shape[0] == 6 and J_base.shape[1] == self.num_model_joints:
                self.get_logger().debug(f"Jacobian calculated successfully:\n{J_base}")
                return J_base
            else:
                self.get_logger().error(f"Jacobian shape mismatch: {J_base.shape}, expected (6, {self.num_model_joints}) for q={q_np}")
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

            # --- Calculate current end-effector velocity (for derivative term) ---
            q_dot_current = self.current_joint_velocities.reshape(-1, 1)
            v_current = jacobian @ q_dot_current

            # --- Combine P and D terms for desired EE velocity ---
            # v_desired_pd = self.kp @ pose_error_np - self.kd @ v_current
            v_desired_pd = self.kp @ pose_error_np.reshape(6,1) - self.kd @ v_current.reshape(6,1)

            self.get_logger().debug(f"v_desired: {v_desired_pd.flatten()}")

            # --- Calculate joint velocities using Damped Least Squares (DLS) ---
            J_dls = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + self.jacobian_damping**2 * np.eye(6)) # DLS formula
            joint_velocities_raw = J_dls @ v_desired_pd

            # --- Or using standard Pseudo-Inverse ---
            # J_pseudo_inverse = np.linalg.pinv(jacobian)
            # joint_velocities_raw = J_pseudo_inverse @ v_desired_pd # Use PD desired velocity

            # --- Velocity Limiting ---
            joint_velocities_limited = np.clip(joint_velocities_raw.flatten(), -self.max_joint_velocities_np, self.max_joint_velocities_np)

            # --- Joint Position Limit Handling ---
            if self.joint_position_limits is not None and \
               self.current_joint_positions is not None and \
               len(joint_velocities_limited) == self.num_model_joints: # Ensure consistent length
                
                # Make a copy to modify based on position limits
                velocities_after_pos_limits = np.copy(joint_velocities_limited) 

                for i in range(self.num_model_joints):
                    q_i = self.current_joint_positions[i]
                    v_i = velocities_after_pos_limits[i] 

                    q_min_i = self.joint_position_limits[0, i]
                    q_max_i = self.joint_position_limits[1, i]
                    
                    if q_i <= (q_min_i + self.joint_limit_buffer_fval) and v_i < 0:
                        self.get_logger().debug(f"Joint {self.controlled_joint_names[i]} ({i}) near lower limit (q={q_i:.3f} vs lim={q_min_i:.3f}, buff={self.joint_limit_buffer_fval:.3f}) with v={v_i:.3f}. Clamping v to 0.")
                        velocities_after_pos_limits[i] = 0.0
                    elif q_i >= (q_max_i - self.joint_limit_buffer_fval) and v_i > 0:
                        self.get_logger().debug(f"Joint {self.controlled_joint_names[i]} ({i}) near upper limit (q={q_i:.3f} vs lim={q_max_i:.3f}, buff={self.joint_limit_buffer_fval:.3f}) with v={v_i:.3f}. Clamping v to 0.")
                        velocities_after_pos_limits[i] = 0.0
                joint_velocities_limited = velocities_after_pos_limits # Assign back the potentially modified velocities
            self.get_logger().debug(f"Raw Vels: {joint_velocities_raw.flatten()}, Final Limited Vels: {joint_velocities_limited}")
            return joint_velocities_limited # Return the final limited velocities
        
        except Exception as e:
            self.get_logger().error(f"Error calculating/limiting joint velocities: {e}")
            return None


    def publish_target_joint_positions(self, target_positions: np.ndarray | None, time_from_start: float = 0.1):
        """Publishes the calculated target joint positions as a JointTrajectory."""
        if target_positions is None:
            # self.get_logger().warn("Target positions are None. Not publishing.")
            # In a real scenario, you might want to publish the current positions to "hold"
            # For now, if None, we assume the JTC will hold its last valid trajectory point.
            return

        if not self.controlled_joint_names:
            self.get_logger().error("Controlled joint names not set. Cannot publish trajectory.")
            return

        if len(target_positions) != len(self.controlled_joint_names):
            self.get_logger().error(
                f"Mismatch between target_positions length ({len(target_positions)}) "
                f"and controlled_joint_names length ({len(self.controlled_joint_names)}). "
                f"Cannot publish trajectory."
            )
            return

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        # Ensure joint names here match what joint_trajectory_controller expects.
        # These should be the namespaced joint names if your JTC is configured for them.
        traj_msg.joint_names = self.controlled_joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions.tolist()
        
        # time_from_start is crucial. It tells the JTC how quickly to reach this point.
        # For streaming targets, this is typically the control period.
        point.time_from_start = RclpyDuration(seconds=time_from_start).to_msg()
        
        # Optional: You can also set velocities if your JTC uses them and you have good estimates.
        # If self.current_joint_velocities is reliable and represents the velocities for the *target* point:
        # point.velocities = self.current_joint_velocities.tolist() # Or the calculated limited joint_velocities
        # Otherwise, leave it empty and let JTC handle velocity profiling.

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().debug(f"Arm {self.arm_index} publishing trajectory: names {traj_msg.joint_names}, pos {point.positions}, time {point.time_from_start.sec}.{point.time_from_start.nanosec}")

    def control_loop_callback(self):
        """Main control loop executed at a fixed frequency."""
        if self.active_target_pose is None:
            # No active target, do not send any commands.
            # self.get_logger().debug("No active target pose. Controller is idle.", throttle_duration_sec=5)
            return

        # Check prerequisites
        if self.robot is None or self.current_joint_positions is None or self.current_joint_velocities is None:
            self.get_logger().warn("Robot model or joint states not ready. Skipping control loop iteration.", throttle_duration_sec=5)
            return

        current_pose = self.get_current_pose_from_tf()
        if current_pose is None:
            self.get_logger().warn("Could not get current pose. Skipping control loop iteration.", throttle_duration_sec=5)
            return
        
        # Calculate error
        self.pose_error = self.calculate_pose_error(current_pose, self.active_target_pose)
        self.get_logger().debug(f'6D Pose error: {self.pose_error}')

        # Check if target is reached within tolerance
        if np.all(np.abs(self.pose_error) < self.error_tolerance_np):
            self.get_logger().info(f"Target reached within tolerance. Pose error: {self.pose_error}. Controller is now idle.")
            time_from_start = (1.0 / self.control_frequency)
            self.publish_target_joint_positions(self.current_joint_positions, time_from_start) # Publish current positions to hold
            self.active_target_pose = None # Deactivate target until a new one arrives
            return 

        # --- If target not reached, calculate and publish pose ---
        jacobian = self.calculate_jacobian()
        if jacobian is None:
            self.get_logger().warn("Jacobian calculation failed. Skipping velocity command.", throttle_duration_sec=5)
            return
            
        joint_velocities = self.calculate_and_limit_joint_velocities(jacobian, self.pose_error)
        if joint_velocities is None:
            self.get_logger().warn("Joint velocity calculation failed. Skipping velocity command.", throttle_duration_sec=5)
            return
            
        # --- Integrate velocities to get target joint positions ---
        dt = 1.0 / self.control_frequency
        delta_joint_positions = joint_velocities * dt

        # Ensure current_joint_positions is not None before operation
        if self.current_joint_positions is None:
            self.get_logger().warn(f"Arm {self.arm_index} - Current joint positions are None. Cannot calculate target positions.")
            return
            
        target_joint_positions = self.current_joint_positions + delta_joint_positions

        # --- Apply joint position limits to the target_joint_positions before sending ---
        # This is a safeguard; the joint_trajectory_controller will also respect limits.
        if self.joint_position_limits is not None:
            min_limits = self.joint_position_limits[0, :]
            max_limits = self.joint_position_limits[1, :]
            # Apply buffer to avoid commanding exactly to the limit
            buffered_min_limits = min_limits + self.joint_limit_buffer_fval
            buffered_max_limits = max_limits - self.joint_limit_buffer_fval
            
            # Ensure target_joint_positions and limits have compatible shapes
            if target_joint_positions.shape == buffered_min_limits.shape and \
               target_joint_positions.shape == buffered_max_limits.shape:
                target_joint_positions = np.clip(target_joint_positions, buffered_min_limits, buffered_max_limits)
            else:
                self.get_logger().warn(f"Arm {self.arm_index} - Shape mismatch for joint limit clipping. Skipping clip.")

        time_from_start = (1.0 / self.control_frequency)
        self.publish_target_joint_positions(target_joint_positions, time_from_start)

def main():
    rclpy.init()

    inverse_kinematics_server = RigidBodyDynamicsController()    

    rclpy.spin(inverse_kinematics_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
