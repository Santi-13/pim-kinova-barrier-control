import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from sensor_msgs.msg import JointState


import numpy as np
import roboticstoolbox as rtb
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
        
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.controlled_joint_names = self.get_parameter('controlled_joint_names').value
        self.num_model_joints = len(self.controlled_joint_names) # Initialize based on controller config

        # Initialize dynamic parameters
        self.declare_parameter('P', np.diag(np.ones(12))) # Diagonal matrix P for norm calculation
        self.declare_parameter('x_plus', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Vector x_plus, barrier state
        self.declare_parameter('lambda_1_0', 1.0) # Initial value for lambda_1
        self.declare_parameter('lambda_2_0', 1.0)
        self.declare_parameter('r', 1.0) # Exponent for adaptive terms

        self.P = self.get_parameter('P').get_parameter_value().double_array_value
        self.x_plus = self.get_parameter('x_plus').get_parameter_value().double_array_value
        self.lambda_1_0 = self.get_parameter('lambda_1_0').value
        self.lambda_2_0 = self.get_parameter('lambda_2_0').value
        self.r = self.get_parameter('r').value


        # Calculation parameters
        self.lambda_1 = 0.0
        self.lambda_2 = 0.0
        self.lambda_1_dot = 0.0
        self.lambda_2_dot = 0.0


        # Default limits, will be overwritten by URDF if available
        min_limits_default = [-2 * math.pi, -2.2, -2.5, -2 * math.pi, -2.08, -2 * math.pi]
        max_limits_default = [ 2 * math.pi,  2.2,  2.5,  2 * math.pi,  2.08,  2 * math.pi]
        self.joint_position_limits = np.array([min_limits_default, max_limits_default])

        # After all parameters are set, initialize the robot model
        self.robot = None
        self.load_robot()

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
                if not self.joint_states_received_once:
                    self.get_logger().info("First joint states received.")
                    self.joint_states_received_once = True

        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
            self.current_joint_positions = None
            self.current_joint_velocities = None
    
    def calculate_adaptive_terms(self):

        norm_x = self.norm_squared_P(self.x, self.P)

        self.lambda_1 = self.lambda_1_0 * ( self.x_plus - norm_x / self.x_plus)**self.r
        self.lambda_2 = self.lambda_2_0 * ( self.x_plus - norm_x / self.x_plus)**self.r

        self.lambda_1_dot = self.lambda_1_dot + self.lambda_1 * ( 1.0 / self.controller_frequency )
        self.lambda_2_dot = self.lambda_2_dot + self.lambda_2 * ( 1.0 / self.controller_frequency )
    
    def calculate_barrier_dynamics(self):
        self.calculate_adaptive_terms()

        





def main(args=None):
    rclpy.init(args=args)

    barrier_dynamics_controller = BarrierDynamicsController()

    rclpy.spin(barrier_dynamics_controller)

    barrier_dynamics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

