import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration as RclpyDuration
import tf2_ros


from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray 


import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import tempfile
import os
import subprocess
import math
import quaternion 

class BarrierDynamicsController(Node):
    def __init__(self):
        super().__init__('barrier_dynamics_controller')


        # Initialize parameters
        self.declare_parameter('controller_frequency', 10.0) # Hz
        self.declare_parameter('robot_description_package', 'kortex_description')
        self.declare_parameter('robot_description_xacro_path', 'robots/gen3.xacro')
        self.declare_parameter('xacro_args', 'dof:=6 use_fake_hardware:=true robot_ip:=dummy')
        self.declare_parameter('controlled_joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('euler_input_convention', 'xyz')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_trajectory_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('arm_index', 0) 
        self.declare_parameter('joint_limit_buffer', 0.01) # Radians
        self.declare_parameter('cartesian_pos_tolerance', 0.01)  # meters
        self.declare_parameter('cartesian_rot_tolerance', 0.05) # radians (approx 2.8 degrees)   
        self.declare_parameter('robot_base_frame', 'base_link') # Default   
        
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.num_model_joints = len(self.controlled_joint_names) # Initialize based on controller config
        self.euler_input_convention = self.get_parameter('euler_input_convention').value.lower()
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.arm_index = self.get_parameter('arm_index').value
        self.joint_limit_buffer_fval = self.get_parameter('joint_limit_buffer').value
        self.cartesian_pos_tolerance = self.get_parameter('cartesian_pos_tolerance').get_parameter_value().double_value
        self.cartesian_rot_tolerance = self.get_parameter('cartesian_rot_tolerance').get_parameter_value().double_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value


        # Initialize dynamic parameters
        self.declare_parameter('P', [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]*2)        
        self.declare_parameter('lambda_1_0', 10.0) # Initial value for lambda_1
        self.declare_parameter('lambda_2_0', 0.1)
        self.declare_parameter('r_1', 1.4) # Exponent for adaptive terms
        self.declare_parameter('r_2', 2.0) # Exponent for adaptive terms
        self.declare_parameter('K_P_initial_diag', [1.0]*6) # Initial diagonal values for K_P
        self.declare_parameter('K_D_initial_diag', [1.0]*6)  # Initial diagonal values for K_D
        self.declare_parameter('barrier_gain', 20.0)  # Gain for the q_dot_avoid term
        self.declare_parameter('h_denominator_offset', 0.01) # Small constant for h(x) denominator
    

        self.P = np.diag(self.get_parameter('P').get_parameter_value().double_array_value)        
        self.lambda_1_0 = self.get_parameter('lambda_1_0').value
        self.lambda_2_0 = self.get_parameter('lambda_2_0').value
        self.r_1 = self.get_parameter('r_1').value
        self.r_2 = self.get_parameter('r_2').value
        self.K_P = np.diag(self.get_parameter('K_P_initial_diag').value)
        self.K_D = np.diag(self.get_parameter('K_D_initial_diag').value)
        self.barrier_gain_param = self.get_parameter('barrier_gain').get_parameter_value().double_value
        self.h_denominator_offset_param = self.get_parameter('h_denominator_offset').get_parameter_value().double_value
        

        # Calculation parameters
        self.lambda_1 = 0.0
        self.lambda_2 = 0.0
        self.epsilon = 0.007

        
        # --- TF2 Listener Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Default limits, will be overwritten by URDF if available
        min_position_limits_default = [-1.0 * math.pi,    -2.2,   -2.58,   -1.0 * math.pi,   -2.099, -1.0 * math.pi]
        # min_position_limits_default = [0.0] * 6
        max_position_limits_default = [ 1.0 * math.pi,     2.2,    2.58,    1.0 * math.pi,    2.099,  1.0 * math.pi]
        self.joint_position_limits = np.array((min_position_limits_default, max_position_limits_default))

        min_velocity_limits_default = [-0.2]*6
        max_velocity_limits_default = [ 0.2]*6
        self.joint_velocity_limits = np.array([min_velocity_limits_default, max_velocity_limits_default])

        
        # Update joint limits boundary condition
        self.default_x_plus_boundary = self.norm_squared_P(np.concatenate((self.joint_position_limits[0], self.joint_velocity_limits[0])), self.P)
        self.current_dynamic_x_plus = self.default_x_plus_boundary # Initialize with the default/max
        self.get_logger().info(f"Default x_plus boundary condition set to: {self.default_x_plus_boundary:.4f}")

        # Subscribers 
        self.joint_state_subscription = self.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10)
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.joint_states_received_once = False # For robust startup

        self.pose_target_subscription = self.create_subscription(
            PoseStamped, target_pose_topic, self.pose_target_callback, 10)
        self.active_target_pose: PoseStamped | None = None
        self.target_joint_positions = None
        self.target_joint_velocities = None
        self.pose_error = np.zeros(6)


        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.joint_trajectory_topic, 10)
        self.get_logger().info(f"Publishing joint trajectories to: {self.joint_trajectory_topic}")

        # --- Marker Publisher for Visualizations ---
        self.marker_pub = self.create_publisher(MarkerArray, f'arm_{self.arm_index}/barrier_visuals', 10)
        self.get_logger().info(f"Publishing barrier visualizations to: arm_{self.arm_index}/barrier_visuals")


        # Flags and state for sequential motion
        self.executing_cartesian_phase = False
        self.executing_j6_offset_phase = False
        self.target_reached_for_current_pose = False # --- New flag for error tolerance ---
        self.j6_target_q_during_offset = None # Stores the [q1..q6] target during J6 offset
        self.joint_space_error_tolerance_rad = math.radians(1.0) # e.g., 1 degree tolerance for J6 movement


        # --- Homing related members ---
        self.home_position_rad = None # Will be set if homing is initiated
        self.homing_in_progress = False
        self.homing_duration_s = 5.0 # Duration for the homing trajectory
        self.homing_check_tolerance_rad = math.radians(2.0) # Tolerance to consider homing complete (e.g., 2 degrees)


        # After all parameters are set, initialize the robot model
        self.robot = None
        self.load_robot()

        # --- Startup Homing Sequence ---
        if self.current_joint_positions is None: # This check might be too early
             self.get_logger().info("Attempting to send home position on startup as current_joint_positions is None.")
             home_position_deg = [0.0, 15.0, -130.0, 0.0, 55.0, 90.0]
             home_position_rad = np.radians(home_position_deg)
             if len(home_position_rad) == len(self.controlled_joint_names):
                 self.publish_target_joint_positions(home_position_rad, time_from_start=2.0)
             else:
                 self.get_logger().error(f"Home position length mismatch with controlled_joint_names. Skipping startup home.")
        else:
             self.get_logger().info("Current joint positions seem available. Skipping sending home position from __init__.")



        timer_period = 1.0 / self.controller_frequency
        self.control_timer = self.create_timer(timer_period, self.control_step_callback)

    ### UTILITY FUNCTIONS ###
    def norm_squared_P(self, x, P):
        """
        Calculates the squared norm of a vector x with respect to a symmetric positive definite matrix P.
        """
        # If P is symmetric and positive definite, we can use the quadratic form.
        if not isinstance(P, np.ndarray):
            raise TypeError("P debe ser un array de numpy.")
        if P.ndim != 2 or P.shape[0] != P.shape[1]:
            raise ValueError("P debe ser una matriz cuadrada.")
        if not np.allclose(P, P.T):
            raise ValueError("P debe ser simétrica.")
        
        return float(x.T @ P @ x)
          
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

    def get_current_ee_pose_in_base(self) -> SE3 | None:
        """Helper to get current EE pose in robot's base frame using FK."""
        if self.robot and self.current_joint_positions is not None:
            try:
                return self.robot.fkine(self.current_joint_positions)
            except Exception as e:
                self.get_logger().error(f"Error during FK: {e}")
                return None
        return None
    
    def calculate_cartesian_error_to_target(self, target_pose_ros: PoseStamped, current_pose_rtb: SE3) -> tuple[float, float]:
        """
        Calculates the norm of position error and the angle of orientation error.
        target_pose_ros: The desired pose as a PoseStamped message.
        current_pose_rtb: The current pose as an SE3 object from roboticstoolbox.
        Returns: tuple (position_error_norm, orientation_error_angle)
        """
        # Position error (remains the same)
        target_pos_vec = np.array([
            target_pose_ros.pose.position.x,
            target_pose_ros.pose.position.y,
            target_pose_ros.pose.position.z
        ])
        current_pos_vec = current_pose_rtb.t #
        pos_error_norm = np.linalg.norm(target_pos_vec - current_pos_vec)

        # Orientation error using numpy-quaternion
        q_target_msg = target_pose_ros.pose.orientation #
        # numpy-quaternion.quaternion constructor takes (w, x, y, z)
        q_target_npq = quaternion.as_quat_array([
            q_target_msg.w, q_target_msg.x,
            q_target_msg.y, q_target_msg.z
        ])
            
        # Convert current SE3 orientation (from roboticstoolbox FK) to numpy-quaternion
        # First, get spatialmath UnitQuaternion from the rotation matrix
        sm_q_current = UnitQuaternion(current_pose_rtb.R) #
        # Then convert to numpy-quaternion format (w, x, y, z)
        q_current_npq = quaternion.as_quat_array([
            sm_q_current.s, sm_q_current.v[0],
            sm_q_current.v[1], sm_q_current.v[2]
        ])

        # Error quaternion: q_err = q_target * q_current_conjugate
        # This represents the rotation needed to go from current orientation to target orientation.
        q_err_npq = q_target_npq * q_current_npq.conjugate()

        # Ensure the error quaternion represents the shortest rotation path (w >= 0).
        # If w is negative, the rotation is > 180 degrees around some axis.
        # Negating the quaternion (all components w,x,y,z) gives an equivalent rotation
        # but with a positive w and an angle <= 180 degrees.
        if q_err_npq.w < 0:
            q_err_npq = -q_err_npq

        # The angle of rotation theta for a quaternion (w, x, y, z) is 2 * acos(w).
        # We clip w to the range [-1.0, 1.0] to prevent math errors from floating point inaccuracies.
        w_error_clipped = np.clip(q_err_npq.w, -1.0, 1.0)
        orientation_error_angle = 2 * np.arccos(w_error_clipped)

        return pos_error_norm, orientation_error_angle
    
    def calculate_joint_space_barrier_gradient(self) -> np.ndarray:
        """
        Calculates the gradient of the barrier function h(q) w.r.t. joint angles q
        using numerical finite differences. This gradient vector represents the direction
        of "steepest ascent" away from the barrier in joint space.
        """
        if self.current_joint_positions is None or self.current_joint_velocities is None:
            return np.zeros(self.num_model_joints)

        # The state vector x is [q, q_dot]
        x_current = np.concatenate((self.current_joint_positions, self.current_joint_velocities))
        
        # Calculate h(q) at the current position
        norm_x_current_sq = self.norm_squared_P(x_current, self.P)
        h_current = self.current_dynamic_x_plus - norm_x_current_sq

        # The gradient vector to be populated
        grad_h = np.zeros(self.num_model_joints)
        
        # A small perturbation for finite differences
        delta = 0.001  # radians

        for i in range(self.num_model_joints):
            # Create a perturbed joint position vector
            q_perturbed = np.copy(self.current_joint_positions)
            q_perturbed[i] += delta

            # Create the corresponding perturbed state vector x
            x_perturbed = np.concatenate((q_perturbed, self.current_joint_velocities)) # Assume velocities are constant for this small change

            # Calculate h(q) at the perturbed position
            norm_x_perturbed_sq = self.norm_squared_P(x_perturbed, self.P)
            h_perturbed = self.current_dynamic_x_plus - norm_x_perturbed_sq

            # Calculate the i-th component of the gradient
            grad_h[i] = (h_perturbed - h_current) / delta
            
        # Normalize the gradient to get a direction vector (optional but good practice)
        norm_grad = np.linalg.norm(grad_h)
        if norm_grad > 1e-6:
            return grad_h / norm_grad
        else:
            return np.zeros(self.num_model_joints)
        
    ### PRIMARY FUNCTIONS ###
    def load_robot(self):
        tmp_file_path = None
        try:
            package_name = self.get_parameter('robot_description_package').value
            xacro_r_path = self.get_parameter('robot_description_xacro_path').value
            xacro_args = self.get_parameter('xacro_args').value.split()
            pkg_share_dir = get_package_share_directory(package_name)
            xacro_abs_path = os.path.join(pkg_share_dir, xacro_r_path)

            self.get_logger().info(f"Loading robot model from: {xacro_abs_path} with args: {xacro_args}")
            cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_abs_path] + xacro_args
            process = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding='utf-8')
            urdf_str = process.stdout

            with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.urdf', encoding='utf-8') as tmp_file:
                tmp_file.write(urdf_str)
                tmp_file_path = tmp_file.name
            
            self.robot = rtb.ERobot.URDF(tmp_file_path)

            if self.robot:
                self.num_model_joints = self.robot.n
                self.get_logger().info(f"Robot model loaded: {self.robot.name} with {self.num_model_joints} DoF.")
                
                # --- Centralized qlim cleaning and setting ---
                default_qlim_min = np.array([-np.pi] * self.num_model_joints, dtype=np.float64)
                default_qlim_max = np.array([np.pi] * self.num_model_joints, dtype=np.float64)
                default_full_qlim = np.vstack([default_qlim_min, default_qlim_max])


                if self.robot.qlim is not None and self.robot.qlim.shape == (2, self.num_model_joints):
                    self.get_logger().info(f"Original robot qlim from URDF:\n{self.robot.qlim}\ndtype: {self.robot.qlim.dtype}")
                    cleaned_qlim = np.array(self.robot.qlim, dtype=np.float64)
                    
                    nan_mask_lower = np.isnan(cleaned_qlim[0,:])
                    nan_mask_upper = np.isnan(cleaned_qlim[1,:])
                    if nan_mask_lower.any() or nan_mask_upper.any():
                        self.get_logger().warn("NaNs found in robot.qlim from URDF, replacing with default +/-pi for affected joints.")
                        cleaned_qlim[0, nan_mask_lower] = default_qlim_min[nan_mask_lower]
                        cleaned_qlim[1, nan_mask_upper] = default_qlim_max[nan_mask_upper]

                    inf_mask_lower = np.isinf(cleaned_qlim[0,:])
                    inf_mask_upper = np.isinf(cleaned_qlim[1,:])
                    if inf_mask_lower.any() or inf_mask_upper.any():
                        self.get_logger().warn("Infinities found in robot.qlim from URDF, replacing with default +/-4*pi for affected joints.")
                        default_finite_val = 2 * np.pi 
                        cleaned_qlim[0, inf_mask_lower] = -default_finite_val
                        cleaned_qlim[1, inf_mask_upper] = default_finite_val
                    
                    self.robot.qlim = cleaned_qlim # Update robot object's qlim
                    self.get_logger().info(f"Using cleaned qlim from URDF for robot object:\n{self.robot.qlim}")
                else:
                    self.get_logger().warn("robot.qlim from URDF is None or improperly shaped. Setting default +/-pi limits on robot object.")
                    self.robot.qlim = default_full_qlim
                
                # Store a copy for controller's direct use if needed elsewhere, already cleaned
                self.joint_position_limits = np.copy(self.robot.qlim) 
                # --- End of qlim cleaning ---

                try: # ETS jindices logging
                    robot_ets = self.robot.ets()
                    if hasattr(robot_ets, 'jindices'):
                        j_indices = robot_ets.jindices
                        j_indices_dtype = j_indices.dtype if isinstance(j_indices, np.ndarray) else type(j_indices)
                        self.get_logger().info(f"Robot ETS jindices: {j_indices}, dtype: {j_indices_dtype}")
                        if isinstance(j_indices, np.ndarray) and not np.issubdtype(j_indices.dtype, np.integer):
                             self.get_logger().warn(f"CRITICAL WARNING: jindices dtype is {j_indices_dtype}, NOT INTEGER!")
                    else: self.get_logger().warn("Robot ETS does not have 'jindices' attribute.")
                except Exception as e: self.get_logger().error(f"Error getting/inspecting ETS jindices: {e}")
            else:
                self.get_logger().error("Failed to load robot model with ERobot.URDF.")
        except Exception as e:
            self.get_logger().error(f"Exception during robot model loading: {e}")
            self.robot = None
        finally:
            if tmp_file_path and os.path.exists(tmp_file_path): os.remove(tmp_file_path)

    def calculate_adaptive_terms(self):

        if not hasattr(self, 'x') or self.x is None:
            self.get_logger().warn("State self.x not available for adaptive term calculation.", throttle_duration_sec=5)
            self.lambda_1 = 0.0 # Default to no adaptation if state is missing
            self.lambda_2 = 0.0
            return

        norm_x = self.norm_squared_P(self.x, self.P)
        self.get_logger().warn(f"The current state of the arm {self.arm_index} is {self.x.tolist()}")
        self.get_logger().warn(f"The current norm_x (for x_plus comparison) is {norm_x:.4f}")

        # Use self.current_dynamic_x_plus which is updated by update_current_dynamic_x_plus()
        active_x_plus_boundary = self.current_dynamic_x_plus 

        if active_x_plus_boundary <= 0.0 or norm_x >= active_x_plus_boundary:
            # self.get_logger().warn(f"Arm {self.arm_index} - Barrier issue for lambda calc: x_plus={active_x_plus_boundary:.4g}, norm_x^2_P={norm_x:.4g}. Clamping lambdas.")
            self.lambda_1 = 0.0
            self.lambda_2 = 0.0
        else:
            lambda_factor_1 = (active_x_plus_boundary - norm_x) / active_x_plus_boundary
            lambda_factor_2 = active_x_plus_boundary / (active_x_plus_boundary - norm_x)
            # Ensure lambda_factor is not negative due to numerical precision if norm_x is extremely close to active_x_plus_boundary
            lambda_factor_1 = max(0.0, lambda_factor_1) 
            lambda_factor_2 = max(0.0, lambda_factor_2) 
            self.lambda_1 = self.lambda_1_0 * (lambda_factor_1)**self.r_1
            self.lambda_2 = self.lambda_2_0 * math.log(lambda_factor_2)**self.r_2
        # self.get_logger().info(f"DEBUG: norm_x={norm_x}, active_x_plus={active_x_plus_boundary}, factor={lambda_factor}, lambda1={self.lambda_1}")

    def calculate_x_plus_for_spherical_obstacle(self, obstacle_center_base: np.ndarray, obstacle_radius: float) -> float | None:
        """
        Calculates the x_plus value if the EE were at the closest point on the given spherical obstacle.
        Assumes obstacle_center_base is in the robot's base frame.
        """
        if self.robot is None or self.current_joint_positions is None:
            self.get_logger().warn("Robot model or q_current not ready for sphere barrier x_plus calc.", throttle_duration_sec=5)
            return None

        current_ee_pose_base: SE3 = self.get_current_ee_pose_in_base()
        if current_ee_pose_base is None:
            return None # Error already logged by get_current_ee_pose_in_base
        
        ee_pos_base = current_ee_pose_base.t

        vec_obs_to_ee = ee_pos_base - obstacle_center_base
        dist_obs_to_ee = np.linalg.norm(vec_obs_to_ee)

        # If EE is already inside or at the center, return a very small x_plus to activate barrier strongly
        if dist_obs_to_ee < obstacle_radius:
            self.get_logger().warn(f"Arm {self.arm_index} - EE is INSIDE spherical obstacle (dist: {dist_obs_to_ee:.3f} < R: {obstacle_radius:.3f}). Returning highly restrictive x_plus.")
            return 1e-9 
        if dist_obs_to_ee < 1e-6 : # Effectively at the center
            self.get_logger().warn(f"Arm {self.arm_index} - EE at center of spherical obstacle. Returning highly restrictive x_plus.")
            return 1e-9
        
        # If EE is double the radius away, return a very large x_plus to deactivate barrier
        if dist_obs_to_ee > (2 * obstacle_radius):
            self.get_logger().info(f"Arm {self.arm_index} - EE is far from spherical obstacle (dist: {dist_obs_to_ee:.3f} > 2*R: {2*obstacle_radius:.3f}). Returning large x_plus.")
            return 1e6

        closest_point_on_sphere_surface_to_ee = obstacle_center_base + obstacle_radius * (vec_obs_to_ee / dist_obs_to_ee)

        # self.get_logger().info(f"Arm {self.arm_index} - Closest point on sphere surface to EE (P_surf): {closest_point_on_sphere_surface_to_ee.tolist()}")

        current_ee_R_matrix = current_ee_pose_base.R

        target_barrier_pose_rtb = None

        try:
            # Use SE3.Rt(Rotation, translation) for direct and unambiguous construction
            target_barrier_pose_rtb = SE3.Rt(current_ee_R_matrix, closest_point_on_sphere_surface_to_ee)
        except Exception as e_se3:
            self.get_logger().error(f"Arm {self.arm_index} - Error constructing target_barrier_pose_rtb with SE3.Rt: {e_se3}")
            self.get_logger().error(f"    Input R shape: {current_ee_R_matrix.shape if isinstance(current_ee_R_matrix, np.ndarray) else type(current_ee_R_matrix)}, dtype: {current_ee_R_matrix.dtype if isinstance(current_ee_R_matrix, np.ndarray) else 'N/A'}")
            self.get_logger().error(f"    Input t shape: {closest_point_on_sphere_surface_to_ee.shape if isinstance(closest_point_on_sphere_surface_to_ee, np.ndarray) else type(closest_point_on_sphere_surface_to_ee)}, dtype: {closest_point_on_sphere_surface_to_ee.dtype if isinstance(closest_point_on_sphere_surface_to_ee, np.ndarray) else 'N/A'}")
            return None

        # --- Explicitly log the components of the single target_barrier_pose_rtb ---
        # self.get_logger().info(f"Arm {self.arm_index} - Target Barrier IK - Calculated Position (Tep.t): {target_barrier_pose_rtb.t.tolist()}")
        # self.get_logger().info(f"Arm {self.arm_index} - Target Barrier IK - Calculated Rotation (Tep.R):\n{target_barrier_pose_rtb.R}")
        # The following log is good for one last sanity check if Tep.R still looks wrong
        # self.get_logger().info(f"Arm {self.arm_index} - Current EE Rotation used for Tep.R (for reference):\n{current_ee_R_matrix}")

        
        q_ik_guess_for_barrier = np.copy(self.current_joint_positions) 

        try:
            sol = self.robot.ikine_NR(Tep=target_barrier_pose_rtb, q0=q_ik_guess_for_barrier, joint_limits=True, slimit=100, ilimit=50)
        except Exception as e:
            self.get_logger().error(f"Arm {self.arm_index} - Exception during IK for spherical barrier point: {e}")
            return None


        if not sol.success:
            self.get_logger().warn(f"Arm {self.arm_index} - IK for spherical barrier point NOT found. Reason: {sol.reason}. Sphere x_plus not calculated.")
            return None 
        
        q_at_barrier = sol.q
        q_dot_at_barrier = np.zeros_like(q_at_barrier) 
        # q_dot_at_barrier = self.joint_velocity_limits[1,:] * 1 # Use max velocity limit as a placeholder for q_dot at barrier point
        x_at_barrier = np.concatenate((q_at_barrier, q_dot_at_barrier))
        
        if not (isinstance(self.P, np.ndarray) and self.P.shape == (len(x_at_barrier), len(x_at_barrier))):
            self.get_logger().error(f"P matrix shape {self.P.shape if isinstance(self.P, np.ndarray) else 'Invalid'} not compatible with state vector length {len(x_at_barrier)} for sphere x_plus.")
            return None
        
        self.get_logger().info(f"x state at barrier point: {x_at_barrier.tolist()}")
        x_plus_for_this_sphere = self.norm_squared_P(x_at_barrier, self.P)
        self.get_logger().info(f"Arm {self.arm_index} - Calculated x_plus for spherical obstacle: {x_plus_for_this_sphere:.4f} (from norm_P(x_at_barrier))")
        return x_plus_for_this_sphere + self.epsilon
        

    def update_current_dynamic_x_plus(self):
        """
        Calculates x_plus based on all relevant barriers (e.g., default, obstacles)
        and updates self.current_dynamic_x_plus to the most restrictive (smallest) one.
        This should be called in your control_step_callback before calculate_adaptive_terms.
        """
        candidate_x_plus_values = [self.default_x_plus_boundary] # Start with the most permissive boundary

        marker_array_msg = MarkerArray()
        marker_id_counter = 0 # To give unique IDs to markers

        # --- Example: Spherical Obstacle (coordinates in base frame) ---
        obs1_center = np.array([0.5, 0.15, 0.7]) # Example obstacle
        obs1_radius = 0.1
        x_plus_obs1 = self.calculate_x_plus_for_spherical_obstacle(obs1_center, obs1_radius)
        if x_plus_obs1 is not None and x_plus_obs1 > 0: # Ensure positive
            candidate_x_plus_values.append(x_plus_obs1)
        
        # Add a marker for this sphere
        sphere_marker_1 = Marker()
        sphere_marker_1.header.frame_id = 'world' # IMPORTANT: Frame for the obstacle
        sphere_marker_1.header.stamp = self.get_clock().now().to_msg()
        sphere_marker_1.ns = f"spherical_obstacles_arm_{self.arm_index}_{marker_id_counter}"
        sphere_marker_1.id = marker_id_counter
        marker_id_counter += 1
        sphere_marker_1.type = Marker.SPHERE
        sphere_marker_1.action = Marker.ADD

        sphere_marker_1.pose.position.x = obs1_center[0]
        sphere_marker_1.pose.position.y = obs1_center[1]
        sphere_marker_1.pose.position.z = obs1_center[2]
        sphere_marker_1.pose.orientation.w = 1.0 # Identity quaternion for a sphere

        sphere_marker_1.scale.x = obs1_radius * 2.0 # Diameter
        sphere_marker_1.scale.y = obs1_radius * 2.0
        sphere_marker_1.scale.z = obs1_radius * 2.0

        sphere_marker_1.color.r = 1.0  # Red
        sphere_marker_1.color.g = 0.0
        sphere_marker_1.color.b = 0.0
        sphere_marker_1.color.a = 0.3  # Semi-transparent

        # sphere_marker_1.lifetime = RclpyDuration(seconds=0.0).to_msg() # 0 = infinite/persistent until deleted
        # For dynamic obstacles, you might set a short lifetime or manage ADD/DELETE actions.
        # For static ones, a longer lifetime or just publishing once might be okay,
        # but periodic publishing ensures it reappears if RViz is restarted.
        # Let's publish periodically by giving it a lifetime slightly longer than the update rate.
        sphere_marker_1.lifetime = RclpyDuration(seconds=(1.0 / self.controller_frequency) * 10.0).to_msg()


        marker_array_msg.markers.append(sphere_marker_1)

        # --- Example: Spherical Obstacle 2 ---
        obs2_center = np.array([0.0, 0.75, 0.4]) # Example obstacle
        obs2_radius = 0.1
        x_plus_obs2 = self.calculate_x_plus_for_spherical_obstacle(obs2_center, obs2_radius)
        if x_plus_obs2 is not None and x_plus_obs2 > 0: # Ensure positive
            candidate_x_plus_values.append(x_plus_obs2)
        
        # Add a marker for this sphere
        sphere_marker_2 = Marker()
        sphere_marker_2.header.frame_id = 'world' # IMPORTANT: Frame for the obstacle
        sphere_marker_2.header.stamp = self.get_clock().now().to_msg()
        sphere_marker_2.ns = f"spherical_obstacles_arm_{self.arm_index}_{marker_id_counter}"
        sphere_marker_2.id = marker_id_counter
        marker_id_counter += 1
        sphere_marker_2.type = Marker.SPHERE
        sphere_marker_2.action = Marker.ADD

        sphere_marker_2.pose.position.x = obs2_center[0]
        sphere_marker_2.pose.position.y = obs2_center[1]
        sphere_marker_2.pose.position.z = obs2_center[2]
        sphere_marker_2.pose.orientation.w = 1.0 # Identity quaternion for a sphere

        sphere_marker_2.scale.x = obs2_radius * 2.0 # Diameter
        sphere_marker_2.scale.y = obs2_radius * 2.0
        sphere_marker_2.scale.z = obs2_radius * 2.0

        sphere_marker_2.color.r = 0.0  # Red
        sphere_marker_2.color.g = 0.0
        sphere_marker_2.color.b = 1.0
        sphere_marker_2.color.a = 0.3  # Semi-transparent

        sphere_marker_2.lifetime = RclpyDuration(seconds=(1.0 / self.controller_frequency) * 10.0).to_msg()

        marker_array_msg.markers.append(sphere_marker_2)
        
        # --- Robot-Robot Avoidance (Conceptual) ---
        # If you have the state x_other_robot of the other robot, your markdown says:
        # x_plus_inter_robot = some_function(x_other_robot) 
        # You'd calculate this x_plus_inter_robot here and add to candidates.
        # This x_plus_inter_robot would be the x_plus_current_dynamic value for one robot based on the other's state.
        # Note that your BLF formulation implies x_plus is a function of the OTHER robot's state,
        # so for robot i, x_plus_i = f(x_j). This x_plus_i is what robot i uses.


        # Publish the markers
        if marker_array_msg.markers:
            self.marker_pub.publish(marker_array_msg)


        positive_candidates = [c for c in candidate_x_plus_values if c > 0] # Ensure we only consider positive x_plus
        self.get_logger().info(f"Arm {self.arm_index} - Candidate x_plus values: {positive_candidates}")
        if not positive_candidates:
            self.get_logger().warn(f"Arm {self.arm_index} - No positive candidate x_plus values. Using small default fallback.")
            self.current_dynamic_x_plus = 1e-6 
        else:
            # This is the most restrictive (smallest) potential boundary from all sources
            potential_next_x_plus = min(positive_candidates) 

            # # Get the norm of the current state right now.
            # current_norm_x = self.norm_squared_P(self.x, self.P) 

            # # THE "DESCENDING CEILING" LOGIC:
            # # If the new potential boundary is already being violated by our current state...
            # if potential_next_x_plus < current_norm_x:
            #     # ...don't just slam the boundary down. 
            #     # Instead, set the boundary to be our current position plus a small buffer.
            #     # This gives the robot a chance to move away from a slowly descending ceiling,
            #     # rather than having a wall appear on top of it.
            #     # The avoidance term (lambda_2) will activate and push it away from this ceiling.
            #     new_dynamic_x_plus = current_norm_x * 1.05 # e.g., 5% buffer
            #     self.get_logger().warn(
            #         f"Arm {self.arm_index} - Potential x_plus ({potential_next_x_plus:.4f}) is too restrictive for current norm_x ({current_norm_x:.4f}). "
            #         f"Applying descending ceiling: new x_plus = {new_dynamic_x_plus:.4f}",
            #         throttle_duration_sec=1.0
            #     )
            # else:
            #     # It's safe to adopt the new, more restrictive boundary.
            new_dynamic_x_plus = potential_next_x_plus 

            # Only log if it changes significantly to reduce noise
            if not np.isclose(new_dynamic_x_plus, self.current_dynamic_x_plus): #
                 self.get_logger().info(f"Arm {self.arm_index} - Updated current_dynamic_x_plus from {self.current_dynamic_x_plus:.4f} to {new_dynamic_x_plus:.4f}") #
            
            self.current_dynamic_x_plus = new_dynamic_x_plus

    
    ### CALLBACKS ###
    def joint_state_callback(self, msg: JointState):
        if not self.controlled_joint_names: # Should be set by parameters
            self.get_logger().warn("Controlled joint names not set. Cannot process joint states.", throttle_duration_sec=5)
            return
        if self.num_model_joints == 0: 
            self.get_logger().debug("Robot model not loaded or num_model_joints is zero, skipping joint state processing.", throttle_duration_sec=5)
            return
        
        # self.get_logger().error(f"Received JointState msg.name: {msg.name}", throttle_duration_sec=5) # ADD THIS LINE FOR DEBUGGING

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
                # self.get_logger().error(f"Arm {self.arm_index} - Joint states updated: Pos={self.current_joint_positions}, Vel={self.current_joint_velocities}")

                self.x = np.concatenate((self.current_joint_positions, self.current_joint_velocities))
                if not self.joint_states_received_once:
                    self.get_logger().info("First joint states received.")
                    self.joint_states_received_once = True

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None
            self.current_joint_velocities = None
    
    def pose_target_callback(self, msg: PoseStamped): #
        if self.homing_in_progress:
            self.get_logger().info(f"Arm {self.arm_index} - Homing in progress. New target pose ignored until homing is complete.", throttle_duration_sec=5.0)
            return # Ignore new targets while homing
        
        self.get_logger().info(f'Arm {self.arm_index} - Received new target pose in frame "{msg.header.frame_id}": Pos(x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f})') #

        # --- Prepare the pose that will be transformed ---
        # This pose_for_transform initially holds the received pose data.
        # If Euler conversion is needed, it's applied here first.
        pose_for_transform = PoseStamped()
        pose_for_transform.header = msg.header # Keep original header for TF lookup
        pose_for_transform.pose.position = msg.pose.position #
        
        # Handle Euler input convention before transformation
        if self.euler_input_convention == 'xyz': #
            ros_q_x = msg.pose.orientation.x # Euler roll from message #
            ros_q_y = msg.pose.orientation.y # Euler pitch from message #
            ros_q_z = msg.pose.orientation.z # Euler yaw from message #
            try:
                hr, hp, hy = ros_q_x * 0.5, ros_q_y * 0.5, ros_q_z * 0.5 #
                sr, cr = np.sin(hr), np.cos(hr) #
                sp, cp = np.sin(hp), np.cos(hp) #
                sy, cy = np.sin(hy), np.cos(hy) #
                pose_for_transform.pose.orientation.w = cr * cp * cy + sr * sp * sy #
                pose_for_transform.pose.orientation.x = sr * cp * cy - cr * sp * sy #
                pose_for_transform.pose.orientation.y = cr * sp * cy + sr * cp * sy #
                pose_for_transform.pose.orientation.z = cr * cp * sy - sr * sp * cy #
            except Exception as e:
                self.get_logger().error(f"Error converting Euler to Quaternion: {e}") #
                return #
        elif self.euler_input_convention == 'quat': #
            pose_for_transform.pose.orientation = msg.pose.orientation # Assumes valid quaternion in msg #
        else:
            self.get_logger().error(f"Euler input convention '{self.euler_input_convention}' not supported. Target not updated.") #
            return #

        # --- Transform the target pose to the robot's base frame ---
        transformed_pose_stamped = None
        if pose_for_transform.header.frame_id == self.robot_base_frame:
            self.get_logger().warn(f"Target pose is already in robot base frame: {self.robot_base_frame}")
            transformed_pose_stamped = pose_for_transform
        else:
            try:
                self.get_logger().warn(f"Attempting to transform target pose from {pose_for_transform.header.frame_id} to {self.robot_base_frame}")
                # It's good to wait for the transform to be available, especially at startup
                # Timeout for waiting for the transform (e.g., 1 second)
                # when = self.get_clock().now() - rclpy.time.Duration(seconds=0.1) # Transform slightly in the past
                when = rclpy.time.Time() # Latest available
                transformed_pose_stamped = self.tf_buffer.transform(
                    pose_for_transform,
                    self.robot_base_frame,
                    timeout=RclpyDuration(seconds=1.0) # Wait up to 1 sec for the transform
                )
                self.get_logger().info(f"Successfully transformed target pose to {transformed_pose_stamped.header.frame_id}.")
            except Exception as e: # Catch more specific TF exceptions
                self.get_logger().error(
                    f"Could not transform target pose from '{pose_for_transform.header.frame_id}' "
                    f"to '{self.robot_base_frame}': {e}"
                )
                return # Don't process this target if transformation fails

        # --- Use the transformed_pose_stamped for IK ---
        self.active_target_pose = transformed_pose_stamped # This is now in robot_base_frame
        self.executing_cartesian_phase = True #
        self.target_reached_for_current_pose = False # Reset for new target
        self.executing_j6_offset_phase = False  #
        self.j6_target_q_during_offset = None   #
        self.pose_error = np.zeros(6)       #
        self.get_logger().info(f"Arm {self.arm_index} - New target set (in {self.robot_base_frame}). Engaging Cartesian phase.") #

        # 2. Prepare inputs for IK (using self.active_target_pose)
        if not (self.active_target_pose and self.robot): #
            self.get_logger().warn(f"Arm {self.arm_index} - Cannot proceed with IK: No active target (post-transform) or robot model not loaded.") #
            self.executing_cartesian_phase = False #
            self.target_joint_positions = None #
            return #

        pos = self.active_target_pose.pose.position # This is now from the transformed pose #
        orient_q_ros = self.active_target_pose.pose.orientation # Also from transformed pose #

        try:
            sm_quat = UnitQuaternion(s=orient_q_ros.w, v=[orient_q_ros.x, orient_q_ros.y, orient_q_ros.z]) #
            target_pose_rtb = SE3(pos.x, pos.y, pos.z) * sm_quat.SE3() #
        except Exception as e:
            self.get_logger().error(f"Arm {self.arm_index} - Error creating SE3 for IK from transformed pose: {e}") #
            self.target_joint_positions = None #
            self.executing_cartesian_phase = False  #
            return #

        # ... (rest of your IK logic: q_initial_guess, self.robot.ikine_NR call, processing sol) ...
        # Ensure that from this point onwards, self.active_target_pose is used,
        # which now contains the pose in the robot's base frame.
        if self.current_joint_positions is not None: #
            q_initial_guess = np.array(self.current_joint_positions, dtype=np.float64) #
            source_q0 = "current_joint_positions" #
        else: #
            q_initial_guess = np.zeros(self.num_model_joints, dtype=np.float64) #
            source_q0 = "np.zeros" #
        
        self.get_logger().info(f"Arm {self.arm_index} - IK using q0 from {source_q0}: {[f'{ji:.3f}' for ji in q_initial_guess]}") #
        self.get_logger().info(f"Arm {self.arm_index} - Attempting IK with ikine_NR for target in {self.robot_base_frame}. Tep:\n{target_pose_rtb}") #

        sol = None #
        try: #
            if self.robot.qlim is None: #
                self.get_logger().error(f"Arm {self.arm_index} - self.robot.qlim is None prior to ikine_NR call! Check load_robot_model.") #
                self.target_joint_positions = None #
                self.executing_cartesian_phase = False #
                return #

            sol = self.robot.ikine_LM( #
                Tep=target_pose_rtb, 
                q0=q_initial_guess, 
                joint_limits=True 
            )
        except Exception as e:  #
            self.get_logger().error(f"General exception during ikine_NR call: {e}") #
            self.target_joint_positions = None #
            self.executing_cartesian_phase = False #
            return #

        if sol is None or not hasattr(sol, 'success'): #
            self.get_logger().error(f"Arm {self.arm_index} - IK solution (sol) from ikine_NR is not the expected Solution object or is None. Type: {type(sol)}") #
            self.target_joint_positions = None #
            self.executing_cartesian_phase = False #
            return #

        if sol.success: #
            self.target_joint_positions = sol.q #
            self.get_logger().info(f"Arm {self.arm_index} - IK solution (ikine_NR) found: {[f'{ji:.3f}' for ji in sol.q]}, Iter: {sol.iterations}, Err: {sol.residual:.3e}") #
            self.target_joint_velocities = np.zeros(self.num_model_joints, dtype=np.float64) # For setpoint #
        else: #
            self.target_joint_positions = None #
            self.target_joint_velocities = None #
            self.get_logger().warn(f"Arm {self.arm_index} - IK solution (ikine_NR) NOT found. Status: {sol.reason}, Iter: {sol.iterations}, Err: {sol.residual:.3e}") #
            self.executing_cartesian_phase = False  #
    
    def control_step_callback(self):
        # --- Check for Homing Completion ---
        if self.homing_in_progress:
            if self.current_joint_positions is not None and self.home_position_rad is not None:
                # Check if close to home position
                joint_error_to_home = np.abs(self.current_joint_positions - self.home_position_rad)
                if np.all(joint_error_to_home < self.homing_check_tolerance_rad): 
                    self.get_logger().info(f"Arm {self.arm_index} - Detected robot near home position. Homing considered complete.")
                    self.homing_in_progress = False
                    # Optionally, send a hold command at the actual current (near home) position
                    self.publish_target_joint_positions(self.current_joint_positions, 2.0 / self.controller_frequency)
                else:
                    self.get_logger().debug(f"Arm {self.arm_index} - Homing in progress. PD control deferred. Max joint error to home: {math.degrees(np.max(joint_error_to_home)):.2f} deg", throttle_duration_sec=1.0)
                # While homing, we let the JTC execute the trajectory sent from __init__.
                # No new commands are sent from here unless homing is declared complete or we want active servoing to home.
                return # Skip the rest of the PD control logic while homing command is executing or being checked
            else:
                # Waiting for joint states to become available to check homing completion
                self.get_logger().debug(f"Arm {self.arm_index} - Homing in progress, waiting for joint states to confirm completion.", throttle_duration_sec=1.0)
                return # Skip PD
            
        # --- 1. Prerequisite checks ---
        if self.robot is None:
            self.get_logger().warn("Robot model not loaded, skipping control step.", throttle_duration_sec=5)
            return
        if self.current_joint_positions is None:
            self.get_logger().warn("Current joint positions not available, skipping control step.", throttle_duration_sec=5)
            return
        if self.joint_states_received_once is False:
            self.get_logger().warn("Joint states not received yet, skipping control step.", throttle_duration_sec=5)
            return

        # --- 2. Handle "target reached and holding" state ---
        if not self.executing_cartesian_phase and self.target_reached_for_current_pose:
            # Target was previously reached for the current self.active_target_pose.
            # Publish a command to hold the current joint positions.
            if self.current_joint_positions is not None:
                time_to_command_hold = 2.0 / self.controller_frequency # Consistent duration
                self.publish_target_joint_positions(self.current_joint_positions, time_to_command_hold)
                self.get_logger().debug(f"Arm {self.arm_index} - Holding position (target previously reached).", throttle_duration_sec=5)
            return
        
        # --- 3. Handle "no active target" or "IK failed for current target" state (idle) ---
        if self.active_target_pose is None or self.target_joint_positions is None:
            # self.executing_cartesian_phase should be False if IK failed or no target.
            self.get_logger().debug(f"Arm {self.arm_index} - Idle: No active target or target joints not computed.", throttle_duration_sec=5)
            # The JTC should hold the last commanded position. If explicit re-commanding is needed for idle, add here.
            return
        
        # --- 4. If we are in active Cartesian phase (executing_cartesian_phase is True) ---
        if self.executing_cartesian_phase:
            # 4a. Check for error tolerance
            current_ee_pose_rtb = self.get_current_ee_pose_in_base() #
            if current_ee_pose_rtb: # active_target_pose is guaranteed to be not None here
                pos_err_norm, rot_err_angle = self.calculate_cartesian_error_to_target(self.active_target_pose, current_ee_pose_rtb)
                
                # self.get_logger().warn(f"Arm {self.arm_index} - Cartesian Error: PosNorm={pos_err_norm:.4f}m, RotAngle={rot_err_angle:.4f}rad", throttle_duration_sec=1.0)

                if pos_err_norm < self.cartesian_pos_tolerance and rot_err_angle < self.cartesian_rot_tolerance:
                    self.get_logger().info(f"Arm {self.arm_index} - Target reached within tolerance. PosErr: {pos_err_norm:.4f}, RotErr: {rot_err_angle:.4f}. Switching to hold.")
                    self.executing_cartesian_phase = False  # Stop active PD control for this target
                    self.target_reached_for_current_pose = True # Mark current target as achieved

                    # Publish one command to hold current joint positions
                    if self.current_joint_positions is not None:
                        time_to_command_hold = 2.0 / self.controller_frequency
                        self.publish_target_joint_positions(self.current_joint_positions, time_to_command_hold)
                    return # Done for this control cycle; holding logic will take over next cycle.

            # --- 4b. If target not reached, proceed with PD Control ---
            self.update_current_dynamic_x_plus() # This will set self.current_dynamic_x_plus #
            
            error_q = self.target_joint_positions - self.current_joint_positions  
            error_q_dot = self.target_joint_velocities - self.current_joint_velocities 

            self.calculate_adaptive_terms() 
            #Log lambda values
            self.get_logger().warn(f"Arm {self.arm_index} - lambda_1 = {self.lambda_1:.2f}, lambda_2 = {self.lambda_2:.2f}")
            

            pd_output_velocities = self.K_P @ error_q + self.K_D @ error_q_dot 


            # --- Apply Barrier Logic: Combine PD with scaled avoidance term ---
            
            # 1. Calculate the avoidance direction in joint space
            q_dot_avoid_direction = self.calculate_joint_space_barrier_gradient()

            # 2. Scale the avoidance term by lambda_2 and a gain
            # The barrier_gain_param allows you to tune the "strength" of the repulsion
            avoidance_term = self.lambda_2 * self.barrier_gain_param * q_dot_avoid_direction

            # 3. Combine the goal-seeking term (PD) with the safety term (avoidance)
            # The lambda_1 term scales down the movement towards the goal as you approach a barrier.
            # The avoidance_term pushes you away from the barrier.
            joint_velocities_raw = (self.lambda_1 * pd_output_velocities) + avoidance_term

            # Updated logging to show all components
            if not np.isclose(self.lambda_2, 0.0, atol=0.01): # Log if avoidance is active
                self.get_logger().info(
                    f"Arm {self.arm_index} - Ctrl: L1={self.lambda_1:.2f}, L2={self.lambda_2:.2f} | "
                    f"PD_Vel={[f'{v:.2f}' for v in pd_output_velocities]} | "
                    f"Avoid direction={[f'{v:.2f}' for v in q_dot_avoid_direction]} | "
                    f"Avoid_Term={[f'{v:.2f}' for v in avoidance_term]} | "
                    f"Raw_Vel={[f'{v:.2f}' for v in joint_velocities_raw]}",
                    throttle_duration_sec=0.2
                )

            if not np.isclose(self.lambda_1, 1.0):
                self.get_logger().info(f"Arm {self.arm_index} - Lambda_1: {self.lambda_1:.3f} applied. Original PD: {[f'{v:.3f}' for v in pd_output_velocities]}, Scaled: {[f'{v:.3f}' for v in joint_velocities_raw]}", throttle_duration_sec=1.0)

            joint_velocities_limited = np.clip(joint_velocities_raw.flatten(), #
                                                self.joint_velocity_limits[0], #
                                                self.joint_velocity_limits[1]) #
            
            
            
            # self.get_logger().error(f"Arm {self.arm_index} - Error q: {error_q}") #
            # self.get_logger().error(f"Arm {self.arm_index} - Joint Velocities: {joint_velocities_raw}") #
            # self.get_logger().error(f"Arm {self.arm_index} - Limited Velocities: {joint_velocities_limited}") #
            
            if self.joint_position_limits is not None and self.current_joint_positions is not None and \
                len(joint_velocities_limited) == self.num_model_joints: #
                v_after_pos_limits = np.copy(joint_velocities_limited) #
                for i in range(self.num_model_joints): #
                    q_i = self.current_joint_positions[i] #
                    v_i = v_after_pos_limits[i] #
                    q_min_i = self.joint_position_limits[0, i] #
                    q_max_i = self.joint_position_limits[1, i] #
                    if (q_i <= (q_min_i + self.joint_limit_buffer_fval) and v_i < 0) or \
                       (q_i >= (q_max_i - self.joint_limit_buffer_fval) and v_i > 0): #
                        v_after_pos_limits[i] = 0.0 #
                joint_velocities_limited = v_after_pos_limits #

            dt = 1.0 / self.controller_frequency #
            q_next_target = self.current_joint_positions + joint_velocities_limited * dt #

            if self.joint_position_limits is not None: #
                # Ensure buffer doesn't invert limits
                min_lim_buffered = self.joint_position_limits[0, :] + self.joint_limit_buffer_fval / 2.0 #
                max_lim_buffered = self.joint_position_limits[1, :] - self.joint_limit_buffer_fval / 2.0 #
                # Ensure min_lim_buffered is actually less than max_lim_buffered for each joint
                actual_min_lim = np.minimum(min_lim_buffered, max_lim_buffered)
                actual_max_lim = np.maximum(min_lim_buffered, max_lim_buffered)
                q_next_target = np.clip(q_next_target, actual_min_lim, actual_max_lim) #


            # self.get_logger().info(f"Arm {self.arm_index} - Ctrl Step: q_next={[f'{x:.3f}' for x in q_next_target]}, err_q_norm={np.linalg.norm(error_q):.3f}, L1={self.lambda_1:.2f}") #
            time_to_reach_next_point = 2.0 / self.controller_frequency # Use more stable timing
            self.publish_target_joint_positions(q_next_target, time_to_reach_next_point) #
            return

        # --- 5. Fallback if none of the above states cleanly directed execution ---
        self.get_logger().warn(f"Arm {self.arm_index} - control_step_callback reached end without clear action. State: exec_cart={self.executing_cartesian_phase}, target_reached={self.target_reached_for_current_pose}", throttle_duration_sec=5)

            



def main(args=None):
    rclpy.init(args=args)

    barrier_dynamics_controller = BarrierDynamicsController()

    rclpy.spin(barrier_dynamics_controller)

    barrier_dynamics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

