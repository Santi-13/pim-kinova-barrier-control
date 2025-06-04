import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped


import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
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
        self.declare_parameter('arm_index', 0) 
        
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.num_model_joints = len(self.controlled_joint_names) # Initialize based on controller config
        self.euler_input_convention = self.get_parameter('euler_input_convention').value.lower()
        joint_state_topic = self.get_parameter('joint_state_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        self.arm_index = self.get_parameter('arm_index').value


        # Initialize dynamic parameters
        self.declare_parameter('P', np.diag(np.ones(12))) # Diagonal matrix P for norm calculation
        self.declare_parameter('x_plus', 10.0) # Barrier term
        self.declare_parameter('lambda_1_0', 1.0) # Initial value for lambda_1
        self.declare_parameter('lambda_2_0', 1.0)
        self.declare_parameter('r', 1.0) # Exponent for adaptive terms
        self.declare_parameter('K_P_initial_diag', [10.0]*6) # Initial diagonal values for K_P
        self.declare_parameter('K_D_initial_diag', [1.0]*6)  # Initial diagonal values for K_D

        self.P = self.get_parameter('P').get_parameter_value().double_array_value
        self.x_plus = self.get_parameter('x_plus').get_parameter_value().double_array_value
        self.lambda_1_0 = self.get_parameter('lambda_1_0').value
        self.lambda_2_0 = self.get_parameter('lambda_2_0').value
        self.r = self.get_parameter('r').value
        self.K_P = np.diag(self.get_parameter('K_P_initial_diag').value)
        self.K_D = np.diag(self.get_parameter('K_D_initial_diag').value)


        # Calculation parameters
        self.lambda_1 = 0.0
        self.lambda_2 = 0.0
        self.lambda_1_dot = 0.0
        self.lambda_2_dot = 0.0


        # Default limits, will be overwritten by URDF if available
        min_position_limits_default = [-2 * math.pi, -2.2, -2.5, -2 * math.pi, -2.08, -2 * math.pi]
        max_position_limits_default = -min_position_limits_default
        self.joint_position_limits = np.array([min_position_limits_default, max_position_limits_default])

        min_velocity_limits_default = [0.0]*6
        max_velocity_limits_default = [0.1]*6
        self.joint_velocity_limits = np.array([min_velocity_limits_default, max_velocity_limits_default])


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
    
    def get_error(self):
        """
        Calculate the error vector based on the current joint positions and the target pose.
        This should be implemented based on the specific robot and task.
        """
        if self.active_target_pose is None or self.current_joint_positions is None:
            return np.zeros(6)
        
        # Getting the target joint positions from the target pose
        target_joint_positions = np.zeros(self.num_model_joints)
        

    ### PRIMARY FUNCTIONS ###
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

    def calculate_adaptive_terms(self):

        norm_x = self.norm_squared_P(self.x, self.P)
        if self.x_plus <= 0.0:
            self.get_logger().warn("x_plus is non-positive, cannot compute lambda factors.")
            return

        lambda_factor = (self.x_plus - norm_x) / self.x_plus

        if lambda_factor < 0.0:
            self.get_logger().warn(f"Lambda factor is negative: {lambda_factor}. Setting to zero to avoid negative lambda values.")
            lambda_factor = 0.0
            
        self.lambda_1 = self.lambda_1_0 * lambda_factor**self.r
        self.lambda_2 = self.lambda_2_0 * lambda_factor**self.r


        # self.lambda_1_dot = self.lambda_1_dot + self.lambda_1 * ( 1.0 / self.controller_frequency )
        # self.lambda_2_dot = self.lambda_2_dot + self.lambda_2 * ( 1.0 / self.controller_frequency )
    
    def calculate_barrier_positions(self):
        """
        This function calculates x_plus based on what the joints position of the robot should 
        be if the EE is at the closest point in a radius to a barrier.
        """
        

    # def calculate_barrier_dynamics(self):
    #     self.calculate_adaptive_terms()

    #     self.tau_blf = B * self.K_P + B * self.K_D
    
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

            if self.active_target_pose and self.robot:
                pos = self.active_target_pose.pose.position
                orient_q = self.active_target_pose.pose.orientation # ROS quaternion (x, y, z, w)

                # Convert ROS Pose to SE3 object for roboticstoolbox
                # Note: SE3.Quat often expects (s, v1, v2, v3) which is (w, x, y, z)
                try:
                    target_pose_rtb = SE3(pos.x, pos.y, pos.z) * SE3.Quat(orient_q.w, orient_q.x, orient_q.y, orient_q.z)
                except Exception as e:
                    self.get_logger().error(f"Error creating SE3 from pose: {e}")
                    self.target_joint_positions = None
                    return

                q_initial_guess = self.current_joint_positions if self.current_joint_positions is not None else np.zeros(self.num_model_joints)

                # Solve IK
                # You might need to adjust solver parameters (e.g., iterations, tolerance)
                # mask parameter can be used if you only care about position, not full orientation, e.g. mask = np.array([1,1,1,0,0,0])
                sol = self.robot.ikine_LM(T=target_pose_rtb, q0=q_initial_guess) 

                if sol.success:
                    self.target_joint_positions = sol.q  # This is your q_d
                    self.get_logger().info(f"Arm {self.arm_index} - IK solution for target pose: {self.target_joint_positions}")
                    self.target_joint_velocities = np.zeros(self.num_model_joints) 
                else:
                    self.target_joint_positions = None
                    self.target_joint_velocities = None
                    self.get_logger().warn(f"Arm {self.arm_index} - IK solution NOT found for the target pose. Status: {sol.reason}")
                    # Consider what to do if IK fails: e.g., keep previous target, stop, or try alternative.
                    # For now, setting executing_cartesian_phase to False might be an option
                    self.executing_cartesian_phase = False 
                    return

        except ValueError as e:
             self.get_logger().error(f"Invalid Euler angle convention '{self.euler_input_convention}' or angles: {e}. Target not updated.")
    
    def control_step_callback(self):
        if self.robot is None or self.current_joint_positions is None or \
            not self.joint_states_received_once or self.active_target_pose is None or \
            self.target_joint_positions is None: # Wait for IK solution
                self.get_logger().info("Waiting for prerequisites for control step...", throttle_duration_sec=5)
                return
        
        if self.x is None or self.x_plus <= 0.0:
            self.get_logger().warn("Barrier term x_plus is non-positive, skipping control step.")
            return
        
        # 3. Calculate error
        # q_d is from IK, q_d_dot is likely zero
        error_q = self.target_joint_positions - self.current_joint_positions  # or self.x[:self.num_model_joints]
        error_q_dot = self.target_joint_velocities - self.current_joint_velocities # or self.x[self.num_model_joints:]
        
        # 4. Call self.calculate_adaptive_terms() (which calculates lambda_1, lambda_2 based on current self.x and self.x_plus_scalar)
        self.calculate_adaptive_terms()

        # --- THEORETICAL DERIVATION NEEDED HERE ---
        # Implement the derived update laws for K_P and K_D
        # dot_K_P = ... # Your formula: f(self.x, error_q, error_q_dot, self.lambda_1, self.P, ...)
        # dot_K_D = ... # Your formula: f(self.x, error_q, error_q_dot, self.lambda_2, self.P, ...)
        # dt = 1.0 / self.controller_frequency
        # self.K_P += dot_K_P * dt 
        # self.K_D += dot_K_D * dt

        # Implement the derived control law tau_BLF
        B_el_q = self.robot.inertia(self.q)
        C_el_q_qdot = self.robot.coriolis(self.q, self.q_dot) # If using RTB methods that take q and qd
        # G_el_q = self.robot.gravity_rnea(self.q) # Or other gravity methods
        tau_BLF = B_el_q @ (self.K_P + self.K_D)
        
        # 5. Calculate q_ddot, q_dot, q
        q_ddot = np.linalg.inv(B_el_q) @ (tau_BLF - C_el_q_qdot)
        q_dot = self.current_joint_velocities + q_ddot * (1.0 / self.controller_frequency)
        self.q = self.current_joint_positions + q_dot * (1.0 / self.controller_frequency)


        # --- (END OF THEORETICAL DERIVATION IMPLEMENTATION) ---

        # For now, let's log what we have:
        self.get_logger().info(f"Arm {self.arm_index} - Control Step: q={self.q}, err_q={error_q}, K_P_diag={np.diag(self.K_P)}, lambda1={self.lambda_1}")


        





def main(args=None):
    rclpy.init(args=args)

    barrier_dynamics_controller = BarrierDynamicsController()

    rclpy.spin(barrier_dynamics_controller)

    barrier_dynamics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

