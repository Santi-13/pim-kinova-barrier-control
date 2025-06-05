import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration as RclpyDuration


from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.robot.IK import IK_NR
from spatialmath import SE3, UnitQuaternion
import tempfile
import os
import subprocess
import math

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
        
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.num_model_joints = len(self.controlled_joint_names) # Initialize based on controller config
        self.euler_input_convention = self.get_parameter('euler_input_convention').value.lower()
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.arm_index = self.get_parameter('arm_index').value
        self.joint_limit_buffer_fval = self.get_parameter('joint_limit_buffer').value


        # Initialize dynamic parameters
        self.declare_parameter('P', [1.0]*12) # Diagonal matrix P for norm calculation
        self.declare_parameter('lambda_1_0', 1.0) # Initial value for lambda_1
        self.declare_parameter('lambda_2_0', 1.0)
        self.declare_parameter('r', 1.0) # Exponent for adaptive terms
        self.declare_parameter('K_P_initial_diag', [1.0]*6) # Initial diagonal values for K_P
        self.declare_parameter('K_D_initial_diag', [1.0]*6)  # Initial diagonal values for K_D

        self.P = np.diag(self.get_parameter('P').get_parameter_value().double_array_value)        
        self.lambda_1_0 = self.get_parameter('lambda_1_0').value
        self.lambda_2_0 = self.get_parameter('lambda_2_0').value
        self.r = self.get_parameter('r').value
        self.K_P = np.diag(self.get_parameter('K_P_initial_diag').value)
        self.K_D = np.diag(self.get_parameter('K_D_initial_diag').value)


        # Calculation parameters
        self.lambda_1 = 0.0
        self.lambda_2 = 0.0


        # Default limits, will be overwritten by URDF if available
        min_position_limits_default = [-1.0 * math.pi,    -2.2,   -2.5,   -1.0 * math.pi,   -2.08, -1.0 * math.pi]
        # min_position_limits_default = [0.0] * 6
        max_position_limits_default = [ 1.0 * math.pi,     2.2,    2.5,    1.0 * math.pi,    2.08,  1.0 * math.pi]
        self.joint_position_limits = np.array((min_position_limits_default, max_position_limits_default))

        min_velocity_limits_default = [0.0]*6
        max_velocity_limits_default = [0.1]*6
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


        # Flags and state for sequential motion
        self.executing_cartesian_phase = False
        self.executing_j6_offset_phase = False
        self.j6_target_q_during_offset = None # Stores the [q1..q6] target during J6 offset
        self.joint_space_error_tolerance_rad = math.radians(1.0) # e.g., 1 degree tolerance for J6 movement


        # After all parameters are set, initialize the robot model
        self.robot = None
        self.load_robot()

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

                try:
                    q_test_inertia = np.zeros(self.num_model_joints, dtype=np.float64)
                    # Or if self.robot.qz is reliably set:
                    # q_test_inertia = np.array(self.robot.qz, dtype=np.float64)
                    self.get_logger().info(f"Testing self.robot.inertia() with q_test: {q_test_inertia}")
                    inertia_matrix_test = self.robot.inertia(q_test_inertia)
                    self.get_logger().info(f"Isolated inertia test PASSED. Matrix shape: {inertia_matrix_test.shape}")
                except TypeError as te:
                    self.get_logger().error(f"Isolated inertia test FAILED with TypeError: {te}")
                except Exception as e:
                    self.get_logger().error(f"Isolated inertia test FAILED with other exception: {e}")


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
                        default_finite_val = 4 * np.pi 
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

        # Use self.current_dynamic_x_plus which is updated by update_current_dynamic_x_plus()
        active_x_plus_boundary = self.current_dynamic_x_plus 

        if active_x_plus_boundary <= 0.0 or norm_x >= active_x_plus_boundary:
            # self.get_logger().warn(f"Arm {self.arm_index} - Barrier issue for lambda calc: x_plus={active_x_plus_boundary:.4g}, norm_x^2_P={norm_x:.4g}. Clamping lambdas.")
            self.lambda_1 = 0.0
            self.lambda_2 = 0.0
        else:
            lambda_factor = (active_x_plus_boundary - norm_x) / active_x_plus_boundary
            # Ensure lambda_factor is not negative due to numerical precision if norm_x is extremely close to active_x_plus_boundary
            lambda_factor = max(0.0, lambda_factor) 
            self.lambda_1 = self.lambda_1_0 * (lambda_factor)**self.r
            self.lambda_2 = self.lambda_2_0 * (lambda_factor)**self.r
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

        closest_point_on_sphere_surface_to_ee = obstacle_center_base + obstacle_radius * (vec_obs_to_ee / dist_obs_to_ee)
        target_barrier_pose_rtb = SE3(closest_point_on_sphere_surface_to_ee) * SE3(current_ee_pose_base.R) # Maintain current orientation

        q_ik_guess_for_barrier = np.copy(self.current_joint_positions) 
        
        try:
            sol = self.robot.ikine_LM(Tep=target_barrier_pose_rtb, q0=q_ik_guess_for_barrier, joint_limits=True, slimit=100, ilimit=50)
        except Exception as e:
            self.get_logger().error(f"Arm {self.arm_index} - Exception during IK for spherical barrier point: {e}")
            return None


        if not sol.success:
            self.get_logger().warn(f"Arm {self.arm_index} - IK for spherical barrier point NOT found. Reason: {sol.reason}. Sphere x_plus not calculated.")
            return None 
        
        q_at_barrier = sol.q
        q_dot_at_barrier = np.zeros_like(q_at_barrier) 
        x_at_barrier = np.concatenate((q_at_barrier, q_dot_at_barrier))
        
        if not (isinstance(self.P, np.ndarray) and self.P.shape == (len(x_at_barrier), len(x_at_barrier))):
            self.get_logger().error(f"P matrix shape {self.P.shape if isinstance(self.P, np.ndarray) else 'Invalid'} not compatible with state vector length {len(x_at_barrier)} for sphere x_plus.")
            return None
            
        x_plus_for_this_sphere = self.norm_squared_P(x_at_barrier, self.P)
        return x_plus_for_this_sphere
        

    def update_current_dynamic_x_plus(self):
        """
        Calculates x_plus based on all relevant barriers (e.g., default, obstacles)
        and updates self.current_dynamic_x_plus to the most restrictive (smallest) one.
        This should be called in your control_step_callback before calculate_adaptive_terms.
        """
        candidate_x_plus_values = [self.default_x_plus_boundary] # Start with the most permissive boundary

        # --- Example: Spherical Obstacle (coordinates in base frame) ---
        obs1_center = np.array([0.5, 0.0, 0.3]) # Example obstacle
        obs1_radius = 0.15
        x_plus_obs1 = self.calculate_x_plus_for_spherical_obstacle(obs1_center, obs1_radius)
        if x_plus_obs1 is not None and x_plus_obs1 > 0: # Ensure positive
            candidate_x_plus_values.append(x_plus_obs1)
        
        # --- Example: Spherical Obstacle 2 ---
        # obs2_center = np.array([0.5, -0.2, 0.3])
        # obs2_radius = 0.05
        # x_plus_obs2 = self.calculate_x_plus_for_spherical_obstacle(obs2_center, obs2_radius)
        # if x_plus_obs2 is not None:
        #     candidate_x_plus_values.append(x_plus_obs2)
        
        # --- Robot-Robot Avoidance (Conceptual) ---
        # If you have the state x_other_robot of the other robot, your markdown says:
        # x_plus_inter_robot = some_function(x_other_robot) 
        # You'd calculate this x_plus_inter_robot here and add to candidates.
        # This x_plus_inter_robot would be the x_plus_current_dynamic value for one robot based on the other's state.
        # Note that your BLF formulation implies x_plus is a function of the OTHER robot's state,
        # so for robot i, x_plus_i = f(x_j). This x_plus_i is what robot i uses.


        positive_candidates = [c for c in candidate_x_plus_values if c > 0] # Ensure we only consider positive x_plus
        if not positive_candidates:
            self.get_logger().warn(f"Arm {self.arm_index} - No positive candidate x_plus values. Using small default fallback.")
            self.current_dynamic_x_plus = 1e-6 
        else:
            new_dynamic_x_plus = min(positive_candidates)
            # Only log if it changes significantly to reduce noise
            if not np.isclose(new_dynamic_x_plus, self.current_dynamic_x_plus):
                 self.get_logger().info(f"Arm {self.arm_index} - Updated current_dynamic_x_plus from {self.current_dynamic_x_plus:.4f} to {new_dynamic_x_plus:.4f}")
            self.current_dynamic_x_plus = new_dynamic_x_plus

    
    ### CALLBACKS ###
    def joint_state_callback(self, msg: JointState):
        if not self.controlled_joint_names: # Should be set by parameters
            self.get_logger().warn("Controlled joint names not set. Cannot process joint states.", throttle_duration_sec=5)
            return
        if self.num_model_joints == 0: 
            self.get_logger().debug("Robot model not loaded or num_model_joints is zero, skipping joint state processing.", throttle_duration_sec=5)
            return

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

                self.x = np.concatenate((self.current_joint_positions, self.current_joint_velocities))
                if not self.joint_states_received_once:
                    self.get_logger().info("First joint states received.")
                    self.joint_states_received_once = True

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None
            self.current_joint_velocities = None
    
    def pose_target_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Arm {self.arm_index} - Received new target pose: Pos(x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f})')

        # 1. Process incoming pose and set self.active_target_pose
        processed_target_pose = PoseStamped()
        processed_target_pose.header = msg.header
        processed_target_pose.pose.position = msg.pose.position

        # Assuming orientation in msg is x,y,z,w (either Euler to be converted or direct Quaternion)
        ros_q_x = msg.pose.orientation.x
        ros_q_y = msg.pose.orientation.y
        ros_q_z = msg.pose.orientation.z
        ros_q_w = msg.pose.orientation.w 

        if self.euler_input_convention == 'xyz':
            # Convert Euler (roll=x, pitch=y, yaw=z) to Quaternion (w,x,y,z) for internal use
            try:
                hr, hp, hy = ros_q_x * 0.5, ros_q_y * 0.5, ros_q_z * 0.5 # Assuming x,y,z are roll,pitch,yaw
                sr, cr = np.sin(hr), np.cos(hr)
                sp, cp = np.sin(hp), np.cos(hp)
                sy, cy = np.sin(hy), np.cos(hy)
                # ROS quaternion order: x, y, z, w
                processed_target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
                processed_target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
                processed_target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
                processed_target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
            except Exception as e:
                self.get_logger().error(f"Error converting Euler to Quaternion: {e}")
                return
        elif self.euler_input_convention == 'quat':
            processed_target_pose.pose.orientation = msg.pose.orientation # Assumes x,y,z,w is already a valid quaternion
        else:
            self.get_logger().error(f"Euler input convention '{self.euler_input_convention}' not supported. Target not updated.")
            return

        self.active_target_pose = processed_target_pose
        self.executing_cartesian_phase = True
        self.executing_j6_offset_phase = False 
        self.j6_target_q_during_offset = None  
        self.pose_error = np.zeros(6)      
        self.get_logger().info(f"Arm {self.arm_index} - New target set. Engaging Cartesian phase.")

        # 2. Prepare inputs for IK
        if not (self.active_target_pose and self.robot): # self.robot should be loaded by now
            self.get_logger().warn(f"Arm {self.arm_index} - Cannot proceed with IK: No active target or robot model not loaded.")
            self.executing_cartesian_phase = False
            self.target_joint_positions = None # Clear previous target
            return

        pos = self.active_target_pose.pose.position
        # Use the quaternion from processed_target_pose, which is now in ROS x,y,z,w order
        orient_q_ros = processed_target_pose.pose.orientation

        try:
            # spatialmath.UnitQuaternion expects s,v (w, [x,y,z])
            sm_quat = UnitQuaternion(s=orient_q_ros.w, v=[orient_q_ros.x, orient_q_ros.y, orient_q_ros.z])
            target_pose_rtb = SE3(pos.x, pos.y, pos.z) * sm_quat.SE3()
        except Exception as e:
            self.get_logger().error(f"Arm {self.arm_index} - Error creating SE3 for IK from pose: {e}")
            self.target_joint_positions = None
            self.executing_cartesian_phase = False 
            return

        # Prepare q0_initial_guess carefully
        if self.current_joint_positions is not None:
            q_initial_guess = np.array(self.current_joint_positions, dtype=np.float64)
            source_q0 = "current_joint_positions"
        else:
            q_initial_guess = np.zeros(self.num_model_joints, dtype=np.float64)
            source_q0 = "np.zeros"
        
        self.get_logger().info(f"Arm {self.arm_index} - IK using q0 from {source_q0}: {[f'{ji:.3f}' for ji in q_initial_guess]}")
        self.get_logger().info(f"Arm {self.arm_index} - Attempting IK with ikine_NR. Tep:\n{target_pose_rtb}")

        # 3. Call ikine_NR
        sol = None
        try:
            # ikine_NR with joint_limits=True uses self.robot.qlim.
            # self.robot.qlim should have been cleaned in load_robot
            if self.robot.qlim is None:
                self.get_logger().error(f"Arm {self.arm_index} - self.robot.qlim is None prior to ikine_NR call! Check load_robot_model.")
                # Fallback or error out
                self.target_joint_positions = None
                self.executing_cartesian_phase = False
                return

            sol = self.robot.ikine_NR(
                Tep=target_pose_rtb, 
                q0=q_initial_guess, 
                joint_limits=True 
            )
        except Exception as e: 
            self.get_logger().error(f"General exception during ikine_NR call: {e}")
            self.target_joint_positions = None
            self.executing_cartesian_phase = False
            return

        # 4. Process IK solution
        if sol is None or not hasattr(sol, 'success'): # Should be a Solution object
            self.get_logger().error(f"Arm {self.arm_index} - IK solution (sol) from ikine_NR is not the expected Solution object or is None. Type: {type(sol)}")
            self.target_joint_positions = None
            self.executing_cartesian_phase = False
            return

        if sol.success:
            self.target_joint_positions = sol.q
            self.get_logger().info(f"Arm {self.arm_index} - IK solution (ikine_NR) found: {[f'{ji:.3f}' for ji in sol.q]}, Iter: {sol.iterations}, Err: {sol.residual:.3e}")
            self.target_joint_velocities = np.zeros(self.num_model_joints, dtype=np.float64) # For setpoint
        else:
            self.target_joint_positions = None
            self.target_joint_velocities = None
            self.get_logger().warn(f"Arm {self.arm_index} - IK solution (ikine_NR) NOT found. Status: {sol.reason}, Iter: {sol.iterations}, Err: {sol.residual:.3e}")
            self.executing_cartesian_phase = False 
            # No return here, the control_step_callback will see target_joint_positions is None
    
    def control_step_callback(self):
        if self.robot is None:
            self.get_logger().warn("Robot model not loaded, skipping control step.", throttle_duration_sec=5)
            return
        if self.current_joint_positions is None:
            self.get_logger().warn("Current joint positions not available, skipping control step.", throttle_duration_sec=5)
            return
        if self.joint_states_received_once is False:
            self.get_logger().warn("Joint states not received yet, skipping control step.", throttle_duration_sec=5)
            return
        if self.active_target_pose is None:
            self.get_logger().warn("No active target pose set, skipping control step.", throttle_duration_sec=5)
            return
        if self.target_joint_positions is None:
            self.get_logger().warn("Target joint positions not available, skipping control step.", throttle_duration_sec=5)
            return

        
        # Update dynamic x_plus based on environment (e.g., obstacles)
        # self.update_current_dynamic_x_plus() # This will set self.current_dynamic_x_plus

        
        # 3. Calculate error
        # q_d is from IK, q_d_dot is likely zero
        error_q = self.target_joint_positions - self.current_joint_positions  # or self.x[:self.num_model_joints]
        error_q_dot = self.target_joint_velocities - self.current_joint_velocities # or self.x[self.num_model_joints:]
        
        # 4. Call self.calculate_adaptive_terms() (which calculates lambda_1, lambda_2 based on current self.x and self.x_plus_scalar)
        # self.calculate_adaptive_terms()

        # 5. Rigid-Body Dynamics Control Law
        joint_velocities_raw = self.K_P @ error_q + self.K_D @ error_q_dot

        joint_velocities_limited = np.clip(joint_velocities_raw.flatten(),
                                               self.joint_velocity_limits[0, :],
                                               self.joint_velocity_limits[1, :])
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



        # --- Integrate to find next target position ---
        dt = 1.0 / self.controller_frequency
        q_next_target = self.current_joint_positions + joint_velocities_limited * dt

        # Clip to joint limits again for safety before publishing (already somewhat handled by vel clamping)
        if self.joint_position_limits is not None:
            q_next_target = np.clip(q_next_target, 
                                    self.joint_position_limits[0, :] + self.joint_limit_buffer_fval / 2.0, # Smaller buffer for final clip
                                    self.joint_position_limits[1, :] - self.joint_limit_buffer_fval / 2.0)

        # self.get_logger().info(f"Arm {self.arm_index} - Ctrl Step: q_next={q_next_target}, err_q_norm={np.linalg.norm(error_q):.3f}, L1={self.lambda_1:.2f}")
        time_to_reach_next_point = max(dt * 1.5, 0.02) 
        self.publish_target_joint_positions(q_next_target, time_to_reach_next_point)


        





def main(args=None):
    rclpy.init(args=args)

    barrier_dynamics_controller = BarrierDynamicsController()

    rclpy.spin(barrier_dynamics_controller)

    barrier_dynamics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

