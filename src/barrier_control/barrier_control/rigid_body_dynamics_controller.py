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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration as RclpyDuration
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import tempfile # For creating temporary files
import quaternion

class RigidBodyDynamicsController(Node):
    def __init__(self):
        super().__init__('rigid_body_dynamics_controller')

        # --- Parameters ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_frame', 'end_effector_link')
        self.declare_parameter('kp_gains', [1.0]*6)
        self.declare_parameter('kd_gains', [0.0]*6)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_trajectory_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('arm_index', 0)
        self.declare_parameter('robot_description_package', 'kortex_description')
        self.declare_parameter('robot_description_xacro_path', 'robots/gen3.xacro')
        self.declare_parameter('xacro_args', 'dof:=6 use_fake_hardware:=true robot_ip:=dummy')
        self.declare_parameter('jacobian_damping', 0.01)
        self.declare_parameter('max_joint_velocities', [0.87]*6) # Radians/s
        self.declare_parameter('control_frequency', 50.0) # Hz
        self.declare_parameter('controlled_joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('joint_limit_buffer', 0.01) # Radians
        self.declare_parameter('error_tolerance', [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]) # [m, m, m, rad, rad, rad]
        self.declare_parameter('euler_input_convention', 'xyz')
        self.declare_parameter('j6_offset_time_s', 3.0) # Time for J6 to perform its offset move
        self.declare_parameter('j6_offset_degrees', 35.0) # Offset for J6 in degrees for arm_index 1

        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.kp = np.diag(self.get_parameter('kp_gains').value)
        self.kd = np.diag(self.get_parameter('kd_gains').value)
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value

        self.jacobian_damping = self.get_parameter('jacobian_damping').value
        self.max_joint_velocities_np = np.array(self.get_parameter('max_joint_velocities').value)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.joint_limit_buffer_fval = self.get_parameter('joint_limit_buffer').value
        self.error_tolerance_np = np.array(self.get_parameter('error_tolerance').value)
        self.euler_input_convention = self.get_parameter('euler_input_convention').value.lower()

        self.arm_index = self.get_parameter('arm_index').value
        self.j6_offset_time_s = self.get_parameter('j6_offset_time_s').value
        j6_offset_degrees_param = self.get_parameter('j6_offset_degrees').value

        self.active_joint6_offset_rad = 0.0
        if self.arm_index == 1: # Assuming arm_index 1 is robot2
            self.active_joint6_offset_rad = math.radians(j6_offset_degrees_param)
            self.get_logger().info(
                f"Arm {self.arm_index} (robot2) configured with a joint_6 offset of {j6_offset_degrees_param} degrees.")

        # Flags and state for sequential motion
        self.executing_cartesian_phase = False
        self.executing_j6_offset_phase = False
        self.j6_target_q_during_offset = None # Stores the [q1..q6] target during J6 offset
        self.joint_space_error_tolerance_rad = math.radians(1.0) # e.g., 1 degree tolerance for J6 movement

        self.joint_states_received_once = False # For robust startup

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot = None
        # Default limits, will be overwritten by URDF if available
        min_limits_default = [-2 * math.pi, -2.2, -2.5, -2 * math.pi, -2.08, -2 * math.pi]
        max_limits_default = [ 2 * math.pi,  2.2,  2.5,  2 * math.pi,  2.08,  2 * math.pi]
        self.joint_position_limits = np.array([min_limits_default, max_limits_default])
        self.num_model_joints = len(self.controlled_joint_names) # Initialize based on controller config

        self.joint_state_subscription = self.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10)
        self.current_joint_positions = None
        self.current_joint_velocities = None

        self.pose_target_subscription = self.create_subscription(
            PoseStamped, target_pose_topic, self.pose_target_callback, 10)
        self.active_target_pose: PoseStamped | None = None
        self.pose_error = np.zeros(6)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.joint_trajectory_topic, 10)
        self.get_logger().info(f"Publishing joint trajectories to: {self.joint_trajectory_topic}")

        self.load_robot() # num_model_joints might be updated here

        if self.control_frequency > 0:
            timer_period = 1.0 / self.control_frequency
            self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
            self.get_logger().info(f"Control loop running at {self.control_frequency} Hz.")
        else:
            self.get_logger().warn("Control frequency is <= 0. Control loop will not run.")

        # Simplified startup homing (consider a more robust homing state if needed)
        # This initial homing attempt relies on joint states not being received yet.
        # A better approach would use self.joint_states_received_once.
        # For now, keeping your original logic structure for this part.
        if self.current_joint_positions is None: # This check might be too early
             self.get_logger().info("Attempting to send home position on startup as current_joint_positions is None.")
             home_position_deg = [0.0, 15.0, -130.0, 0.0, 55.0, 90.0]
             home_position_rad = np.radians(home_position_deg)
             if len(home_position_rad) == len(self.controlled_joint_names):
                 self.publish_target_joint_positions(home_position_rad, time_from_start=5.0)
             else:
                 self.get_logger().error(f"Home position length mismatch with controlled_joint_names. Skipping startup home.")
        else:
             self.get_logger().info("Current joint positions seem available. Skipping sending home position from __init__.")


    def joint_state_callback(self, msg: JointState):
        if not self.controlled_joint_names: # Should be set by parameters
            # self.get_logger().warn("Controlled joint names not set. Cannot process joint states.", throttle_duration_sec=5)
            return
        if self.num_model_joints == 0: # Robot model not ready yet or error
            # self.get_logger().debug("Robot model not loaded or num_model_joints is zero, skipping joint state processing.", throttle_duration_sec=5)
            return
        # Basic check, num_model_joints should align with controlled_joint_names length after load_robot
        if len(self.controlled_joint_names) != self.num_model_joints:
            # This warning might be frequent if num_model_joints is based on rtb model which might differ initially
            # self.get_logger().warn(
            #     f"Mismatch: controlled_joint_names ({len(self.controlled_joint_names)}) vs num_model_joints ({self.num_model_joints}).",
            #     throttle_duration_sec=5)
            # Consider if processing should stop or adapt if this is a persistent issue.
            # For now, we assume controlled_joint_names is the source of truth for array sizes here.
            pass


        try:
            num_controlled = len(self.controlled_joint_names)
            new_positions = np.zeros(num_controlled)
            new_velocities = np.zeros(num_controlled)
            all_controlled_joints_found = True

            for i, name in enumerate(self.controlled_joint_names):
                try:
                    idx_in_msg = msg.name.index(name)
                    new_positions[i] = msg.position[idx_in_msg]
                    if msg.velocity and len(msg.velocity) > idx_in_msg:
                        new_velocities[i] = msg.velocity[idx_in_msg]
                    else:
                        new_velocities[i] = 0.0
                except ValueError:
                    # self.get_logger().warn(f"Controlled joint '{name}' not found in JointState. Available: {msg.name}", throttle_duration_sec=10)
                    all_controlled_joints_found = False
                    break

            if all_controlled_joints_found:
                self.current_joint_positions = new_positions
                self.current_joint_velocities = new_velocities
                if not self.joint_states_received_once:
                    self.get_logger().info("First joint states received.")
                    self.joint_states_received_once = True
            # else:
                # self.get_logger().warn("Not all controlled joints found in JointState message. Current state not updated.", throttle_duration_sec=5)

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None
            self.current_joint_velocities = None


    def pose_target_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received new target pose. Position: {msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}')

        processed_target_pose = PoseStamped()
        processed_target_pose.header = msg.header
        processed_target_pose.pose.position = msg.pose.position

        roll = msg.pose.orientation.x
        pitch = msg.pose.orientation.y
        yaw = msg.pose.orientation.z

        try:
            if self.euler_input_convention == 'xyz':
                hr, hp, hy = roll * 0.5, pitch * 0.5, yaw * 0.5
                sr, cr = np.sin(hr), np.cos(hr)
                sp, cp = np.sin(hp), np.cos(hp)
                sy, cy = np.sin(hy), np.cos(hy)
                processed_target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
                processed_target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
                processed_target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
                processed_target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
            elif self.euler_input_convention == 'quat':
                processed_target_pose.pose.orientation = msg.pose.orientation
            else:
                self.get_logger().error(f"Euler input convention '{self.euler_input_convention}' not supported. Target not updated.")
                return

            self.active_target_pose = processed_target_pose
            self.executing_cartesian_phase = True  # Start/restart Cartesian movement
            self.executing_j6_offset_phase = False # Reset any J6 phase
            self.j6_target_q_during_offset = None  # Clear previous J6 specific target
            self.pose_error = np.zeros(6)      # Reset pose error
            self.get_logger().info(f"Arm {self.arm_index} - New target set. Engaging Cartesian phase.")

        except ValueError as e:
             self.get_logger().error(f"Invalid Euler angle convention '{self.euler_input_convention}' or angles: {e}. Target not updated.")


    def control_loop_callback(self):
        if not self.joint_states_received_once:
            self.get_logger().info("Waiting for initial joint states...", throttle_duration_sec=5)
            return

        # --- Phase 1: J6 Offset (if arm_index == 1 and this phase is active) ---
        if self.executing_j6_offset_phase:
            if self.arm_index != 1 or self.j6_target_q_during_offset is None or self.current_joint_positions is None:
                self.get_logger().error(f"Arm {self.arm_index} - In J6 offset phase with inconsistent state. Resetting to idle.")
                self.executing_j6_offset_phase = False
                self.active_target_pose = None # Ensure idle
                return

            current_j6_pos = self.current_joint_positions[5] # Assuming 6th joint is index 5
            target_j6_pos_abs = self.j6_target_q_during_offset[5]
            j6_error_rad = abs(target_j6_pos_abs - current_j6_pos)

            if j6_error_rad < self.joint_space_error_tolerance_rad:
                self.get_logger().info(f"Arm {self.arm_index} - Joint 6 offset completed. Final J6: {math.degrees(current_j6_pos):.2f} deg. Holding final position.")
                self.executing_j6_offset_phase = False
                # Publish final hold command with the target J6 position
                # Use a short time_from_start to ensure JTC holds this exact pose
                time_to_command_hold = 2.0 / self.control_frequency # Give it a couple of cycles
                self.publish_target_joint_positions(self.j6_target_q_during_offset, time_to_command_hold)
                # self.active_target_pose is already None from Cartesian phase completion
            else:
                self.get_logger().debug(f"Arm {self.arm_index} - Waiting for J6. Current: {math.degrees(current_j6_pos):.2f}, Target: {math.degrees(target_j6_pos_abs):.2f}, Error: {math.degrees(j6_error_rad):.2f} deg")
                # JTC is handling the move. No continuous re-publishing needed here typically.
            return # Dedicate this cycle to J6 offset monitoring, then exit

        # --- Phase 2: Cartesian Movement (if active and J6 offset is not) ---
        if self.executing_cartesian_phase:
            if self.active_target_pose is None: # This should be set if executing_cartesian_phase is true
                self.get_logger().warn(f"Arm {self.arm_index} - In Cartesian phase but no active_target_pose. Resetting to idle.", throttle_duration_sec=5)
                self.executing_cartesian_phase = False
                return

            if self.robot is None or self.current_joint_positions is None or self.current_joint_velocities is None:
                self.get_logger().warn(f"Arm {self.arm_index} - Robot model or joint states not ready for Cartesian control. Skipping.", throttle_duration_sec=5)
                return

            current_ee_pose = self.get_current_pose_from_tf()
            if current_ee_pose is None:
                self.get_logger().warn(f"Arm {self.arm_index} - Could not get current EE pose. Skipping Cartesian control.", throttle_duration_sec=1)
                return

            self.pose_error = self.calculate_pose_error(current_ee_pose, self.active_target_pose)
            # self.get_logger().debug(f'Arm {self.arm_index} - 6D Pose error: {["{:.3f}".format(e) for e in self.pose_error]}')

            # Check if Cartesian target is reached
            if np.all(np.abs(self.pose_error) < self.error_tolerance_np):
                self.get_logger().info(f"Arm {self.arm_index} - Cartesian target reached. Pose error norm: {np.linalg.norm(self.pose_error):.4f}.")
                self.executing_cartesian_phase = False # Cartesian servoing is done

                if self.arm_index == 1 and self.active_joint6_offset_rad != 0.0:
                    self.get_logger().info(f"Arm {self.arm_index} - Proceeding to Joint 6 offset of {math.degrees(self.active_joint6_offset_rad):.1f} deg.")
                    self.executing_j6_offset_phase = True

                    if self.current_joint_positions is None or len(self.current_joint_positions) != self.num_model_joints:
                        self.get_logger().error(f"Arm {self.arm_index} - Current joint positions invalid for J6 offset. Aborting offset.")
                        self.executing_j6_offset_phase = False
                        self.active_target_pose = None # Go idle
                        return

                    q_at_cartesian_target = np.copy(self.current_joint_positions)
                    j6_target_absolute = q_at_cartesian_target[5] + self.active_joint6_offset_rad

                    # Clip J6 target to its limits
                    if self.joint_position_limits is not None and self.joint_position_limits.shape[0] == 2 and self.joint_position_limits.shape[1] >= 6:
                        min_limit_j6 = self.joint_position_limits[0, 5]
                        max_limit_j6 = self.joint_position_limits[1, 5]
                        j6_target_absolute_clipped = np.clip(j6_target_absolute, min_limit_j6, max_limit_j6)
                        if abs(j6_target_absolute_clipped - j6_target_absolute) > 1e-6 : # If clipping occurred
                            self.get_logger().warn(f"Arm {self.arm_index} - J6 offset target {math.degrees(j6_target_absolute):.2f} deg was clipped to {math.degrees(j6_target_absolute_clipped):.2f} deg.")
                        j6_target_absolute = j6_target_absolute_clipped
                    else:
                        self.get_logger().warn(f"Arm {self.arm_index} - Could not clip J6 offset target; joint limits not available or improperly shaped for J6.")

                    self.j6_target_q_during_offset = np.copy(q_at_cartesian_target)
                    self.j6_target_q_during_offset[5] = j6_target_absolute

                    self.get_logger().info(f"Arm {self.arm_index} - Commanding J6 to {math.degrees(j6_target_absolute):.2f} deg (others holding). Move time: {self.j6_offset_time_s}s.")
                    self.publish_target_joint_positions(self.j6_target_q_during_offset, time_from_start=self.j6_offset_time_s)
                else:
                    # Not arm 1 or no offset, so movement is complete
                    self.get_logger().info(f"Arm {self.arm_index} - Target reached. Holding current position.")
                    if self.current_joint_positions is not None:
                         time_to_command_hold = 2.0 / self.control_frequency
                         self.publish_target_joint_positions(self.current_joint_positions, time_to_command_hold)

                self.active_target_pose = None # Deactivate Cartesian target; next phase (J6 or idle) takes over.
                return # End of this cycle after handling target reached

            # --- If Cartesian target not reached, continue servoing ---
            jacobian = self.calculate_jacobian()
            if jacobian is None: return # Error already logged

            joint_velocities = self.calculate_and_limit_joint_velocities(jacobian, self.pose_error)
            if joint_velocities is None: return # Error already logged

            dt = 1.0 / self.control_frequency
            delta_joint_positions = joint_velocities * dt

            if self.current_joint_positions is None: # Should be caught by prerequisite check
                self.get_logger().error(f"Arm {self.arm_index} - Current joint positions became None during servoing. Cannot calculate target. Critical error.")
                self.executing_cartesian_phase = False # Stop servoing
                self.active_target_pose = None
                return

            target_joint_positions = self.current_joint_positions + delta_joint_positions

            # Apply joint position limits with buffer before sending
            if self.joint_position_limits is not None and \
               self.joint_position_limits.shape[0] == 2 and \
               target_joint_positions.shape[0] == self.joint_position_limits.shape[1]:
                min_limits_b = self.joint_position_limits[0, :] + self.joint_limit_buffer_fval
                max_limits_b = self.joint_position_limits[1, :] - self.joint_limit_buffer_fval
                target_joint_positions = np.clip(target_joint_positions, min_limits_b, max_limits_b)
            # else:
                # self.get_logger().warn(f"Arm {self.arm_index} - Skipping joint limit clipping for servo target due to shape mismatch or no limits.", throttle_duration_sec=5)

            # Publish servoing command
            time_to_reach_next_point = 2.0 / self.control_frequency # Give JTC 2 control cycles
            self.publish_target_joint_positions(target_joint_positions, time_to_reach_next_point)
            return # End of Cartesian servoing cycle for this iteration

        # --- If neither Cartesian phase nor J6 offset phase is active ---
        if not self.executing_cartesian_phase and not self.executing_j6_offset_phase:
            # self.get_logger().debug(f"Arm {self.arm_index} - Controller is idle or holding last position.", throttle_duration_sec=5)
            # JTC will maintain the last commanded position.
            # If active "servo-to-hold" is desired, you could periodically re-publish the last known good q.
            pass

    # --- Helper Functions (get_current_pose_from_tf, calculate_pose_error, load_robot, calculate_jacobian, etc.) ---
    # These are assumed to be the same as in your provided code, so they are omitted here for brevity
    # but should be included in your actual file. Ensure they are efficient.
    def get_current_pose_from_tf(self)-> PoseStamped | None:
        """Fetches the current pose of the tool_frame relative to the base_frame."""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(), # Get latest available transform
                timeout=Duration(seconds=0.1)) # Shorter timeout for control loop

            current_pose = PoseStamped()
            current_pose.header.stamp = trans.header.stamp
            current_pose.header.frame_id = self.base_frame
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation
            # self.get_logger().debug(f'Current Pose ({self.tool_frame} in {self.base_frame}): \n{current_pose.pose}')
            return current_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {self.tool_frame} to {self.base_frame} for current pose: {e}', throttle_duration_sec=1.0)
            return None

    def calculate_pose_error(self, current_pose: PoseStamped, target_pose: PoseStamped) -> np.ndarray:
        position_error = np.array([
            target_pose.pose.position.x - current_pose.pose.position.x,
            target_pose.pose.position.y - current_pose.pose.position.y,
            target_pose.pose.position.z - current_pose.pose.position.z
        ])
        q_target_npq = quaternion.as_quat_array([
            target_pose.pose.orientation.w, target_pose.pose.orientation.x,
            target_pose.pose.orientation.y, target_pose.pose.orientation.z
        ])
        q_current_npq = quaternion.as_quat_array([
            current_pose.pose.orientation.w, current_pose.pose.orientation.x,
            current_pose.pose.orientation.y, current_pose.pose.orientation.z
        ])
        q_error = q_target_npq * q_current_npq.conjugate()
        angular_error = 2.0 * q_error.vec
        if q_error.w < 0:
            angular_error *= -1.0
        return np.concatenate((position_error, angular_error))

    def load_robot(self):
        tmp_file_path = None
        try:
            package_name = self.get_parameter('robot_description_package').value
            xacro_r_path = self.get_parameter('robot_description_xacro_path').value
            xacro_args_str = self.get_parameter('xacro_args').value
            xacro_args = xacro_args_str.split()
            pkg_share_dir = get_package_share_directory(package_name)
            xacro_abs_path = os.path.join(pkg_share_dir, xacro_r_path)

            self.get_logger().info(f"Loading robot model from: {xacro_abs_path} with args: {xacro_args}")
            cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_abs_path] + xacro_args
            process = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding='utf-8')
            urdf_str = process.stdout

            with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.urdf', encoding='utf-8') as tmp_file:
                tmp_file.write(urdf_str)
                tmp_file_path = tmp_file.name
            
            self.robot = rtb.ERobot.URDF(tmp_file_path) # Load from file path

            if self.robot:
                self.num_model_joints = self.robot.n
                self.get_logger().info(f"Robot model loaded: {self.robot.name} with {self.num_model_joints} DoF.")
                if len(self.controlled_joint_names) != self.num_model_joints:
                    self.get_logger().error(
                        f"CRITICAL MISMATCH: 'controlled_joint_names' has {len(self.controlled_joint_names)} names, "
                        f"but loaded RTB model has {self.num_model_joints} DoF. Controller will likely fail.")
                    # Potentially raise an error or prevent control loop from running
                if self.robot.qlim is not None and self.robot.qlim.shape == (2, self.num_model_joints):
                    self.joint_position_limits = self.robot.qlim
                    self.get_logger().info(f"Loaded joint limits from URDF for {self.num_model_joints} joints.")
                else:
                    self.get_logger().warn(f"Joint limits from URDF are missing or improperly shaped. Using default/parameterized limits.")
            else:
                self.get_logger().error("Failed to load robot model using roboticstoolbox.")

        except Exception as e:
            self.get_logger().error(f"Failed to load robot model: {e}")
        finally:
            if tmp_file_path and os.path.exists(tmp_file_path):
                os.remove(tmp_file_path)

    def calculate_jacobian(self):
        try:
            if self.robot is None or self.current_joint_positions is None:
                # self.get_logger().warn("Jacobian: Robot model or joint positions not ready.", throttle_duration_sec=2)
                return None
            if len(self.current_joint_positions) != self.num_model_joints:
                # self.get_logger().warn(f"Jacobian: current_joint_positions length ({len(self.current_joint_positions)}) "
                                    #  f"mismatches num_model_joints ({self.num_model_joints}).", throttle_duration_sec=2)
                return None

            q_np = np.array(self.current_joint_positions)
            J_base = self.robot.jacob0(q_np) # Geometric Jacobian in base frame

            if J_base.shape[0] == 6 and J_base.shape[1] == self.num_model_joints:
                # self.get_logger().debug("Jacobian calculated successfully.")
                return J_base
            else:
                self.get_logger().error(f"Jacobian shape mismatch: {J_base.shape}, expected (6, {self.num_model_joints}) for q={q_np}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error calculating Jacobian: {e}")
            return None

    def calculate_and_limit_joint_velocities(self, jacobian: np.ndarray, pose_error: np.ndarray) -> np.ndarray | None:
        try:
            if self.current_joint_velocities is None:
                # self.get_logger().warn("Vel calc: Current joint velocities not available for Kd term.", throttle_duration_sec=2)
                # Optionally, proceed without Kd or use zero velocities for Kd term
                q_dot_current = np.zeros((self.num_model_joints,1))
            else:
                 q_dot_current = self.current_joint_velocities.reshape(-1, 1)

            v_current = jacobian @ q_dot_current
            v_desired_pd = self.kp @ pose_error.reshape(-1, 1) - self.kd @ v_current

            # Damped Least Squares
            J_JT = jacobian @ jacobian.T
            lambda_sq_I = (self.jacobian_damping**2) * np.eye(6)
            J_pseudo_inv_dls = jacobian.T @ np.linalg.inv(J_JT + lambda_sq_I)
            joint_velocities_raw = J_pseudo_inv_dls @ v_desired_pd

            joint_velocities_limited = np.clip(joint_velocities_raw.flatten(),
                                               -self.max_joint_velocities_np,
                                               self.max_joint_velocities_np)

            # Joint position limit avoidance (simple clamping of velocity)
            if self.joint_position_limits is not None and self.current_joint_positions is not None and \
               len(joint_velocities_limited) == self.num_model_joints:
                v_after_pos_limits = np.copy(joint_velocities_limited)
                for i in range(self.num_model_joints):
                    q_i = self.current_joint_positions[i]
                    v_i = v_after_pos_limits[i]
                    q_min_i = self.joint_position_limits[0, i]
                    q_max_i = self.joint_position_limits[1, i]
                    if (q_i <= (q_min_i + self.joint_limit_buffer_fval) and v_i < 0) or \
                       (q_i >= (q_max_i - self.joint_limit_buffer_fval) and v_i > 0):
                        # self.get_logger().debug(f"Joint {self.controlled_joint_names[i]} near limit, clamping v from {v_i:.3f} to 0.")
                        v_after_pos_limits[i] = 0.0
                joint_velocities_limited = v_after_pos_limits
            # self.get_logger().debug(f"Final Limited Vels: {['{:.3f}'.format(v) for v in joint_velocities_limited]}")
            return joint_velocities_limited
        except Exception as e:
            self.get_logger().error(f"Error calculating/limiting joint velocities: {e}")
            return None

    def publish_target_joint_positions(self, target_positions: np.ndarray | None, time_from_start: float = 0.1):
        if target_positions is None:
            return
        if not self.controlled_joint_names or len(target_positions) != len(self.controlled_joint_names):
            self.get_logger().error(f"Arm {self.arm_index} - Cannot publish trajectory due to name/length mismatch. Target len: {len(target_positions) if target_positions is not None else 'None'}, Controlled names: {len(self.controlled_joint_names)}")
            return

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.controlled_joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions.tolist()
        point.time_from_start = RclpyDuration(seconds=time_from_start).to_msg()
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)
        # self.get_logger().debug(f"Arm {self.arm_index} publishing trajectory: pos {['{:.3f}'.format(p) for p in point.positions]}, time {time_from_start:.3f}s")

def main():
    rclpy.init()
    try:
        controller_node = RigidBodyDynamicsController()
        rclpy.spin(controller_node)
    except Exception as e:
        if 'controller_node' in locals():
            controller_node.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before node initialization: {e}")
    finally:
        if 'controller_node' in locals() and rclpy.ok():
            controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()