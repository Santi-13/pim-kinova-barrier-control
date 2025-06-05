import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import roboticstoolbox as rtb
from roboticstoolbox.robot.IK import IK_LM # Import the IK_LM solver class
from spatialmath import SE3, UnitQuaternion
import numpy as np
import os
import subprocess
import tempfile
import math

class IKTestNode(Node):
    def __init__(self):
        super().__init__('ik_test_node')
        self.get_logger().info("IK Test Node Started")

        self.robot_description_package = 'kortex_description'
        self.robot_description_xacro_path = 'robots/gen3.xacro'
        self.xacro_args_str = 'dof:=6 use_fake_hardware:=true robot_ip:=dummy sim_gazebo:=false'
        self.num_model_joints = 6 

        self.robot = None
        self.load_robot_model() # Call directly

        if self.robot:
            self.run_ik_tests()
        else:
            self.get_logger().error("Robot model not loaded. Cannot run IK tests.")
        
        self.get_logger().info("IK tests finished. Shutting down node.")
        rclpy.try_shutdown()


    def load_robot_model(self):
        tmp_file_path = None
        try:
            package_name = self.robot_description_package
            xacro_r_path = self.robot_description_xacro_path
            xacro_args = self.xacro_args_str.split()
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
                
                if self.robot.qlim is not None:
                    self.get_logger().info(f"Robot qlim:\n{self.robot.qlim}\ndtype: {self.robot.qlim.dtype}")
                    if np.isnan(self.robot.qlim).any(): self.get_logger().warn("NaNs found in robot.qlim!")
                else: self.get_logger().warn("robot.qlim is None.")
                
                try:
                    robot_ets = self.robot.ets()
                    if hasattr(robot_ets, 'jindices'):
                        j_indices = robot_ets.jindices
                        j_indices_dtype = j_indices.dtype if isinstance(j_indices, np.ndarray) else type(j_indices)
                        self.get_logger().info(f"Robot ETS jindices: {j_indices}, dtype: {j_indices_dtype}")
                        if isinstance(j_indices, np.ndarray) and not np.issubdtype(j_indices.dtype, np.integer):
                             self.get_logger().warn(f"CRITICAL WARNING: jindices dtype is {j_indices_dtype}, NOT INTEGER!")
                    else: self.get_logger().warn("Robot ETS does not have 'jindices' attribute.")
                except Exception as e: self.get_logger().error(f"Error getting/inspecting ETS jindices: {e}")
            else: self.get_logger().error("Failed to load robot model with ERobot.URDF.")
        except Exception as e:
            self.get_logger().error(f"Exception during robot model loading: {e}")
            self.robot = None
        finally:
            if tmp_file_path and os.path.exists(tmp_file_path): os.remove(tmp_file_path)

    def run_ik_tests(self):
        self.get_logger().info("--- Starting IK Solver Tests ---")
        
        target_pose_matrix = np.array([ # From your log that failed previously
            [0, 0, -1, -0.5],
            [-1, 0, 0, 0],
            [0, 1, 0, 0.6],
            [0, 0, 0, 1]
        ])
        Tep_target = SE3(target_pose_matrix, check=False)
        self.get_logger().info(f"Target Pose (Tep):\n{Tep_target}")

        q0_guess = np.zeros(self.num_model_joints, dtype=np.float64)
        if hasattr(self.robot, 'qz') and isinstance(self.robot.qz, np.ndarray) and self.robot.qz.shape == (self.num_model_joints,):
            q0_guess = np.array(self.robot.qz, dtype=np.float64)
        self.get_logger().info(f"Initial Guess (q0): {q0_guess}, dtype: {q0_guess.dtype}")

        slimit_for_ik = None
        if self.robot.qlim is not None:
            slimit_for_ik = np.array(self.robot.qlim, dtype=np.float64)
            if np.isnan(slimit_for_ik).any(): 
                self.get_logger().warn("NaNs found in qlim, replacing with 0 for test.")
                slimit_for_ik[np.isnan(slimit_for_ik)] = 0 
            if np.isinf(slimit_for_ik).any(): 
                self.get_logger().warn("Infs found in qlim, replacing with +/- 4*pi for test.")
                slimit_for_ik[np.isinf(slimit_for_ik)] = 4*np.pi * np.sign(slimit_for_ik[np.isinf(slimit_for_ik)])
            self.get_logger().info(f"Using slimit for IK (cleaned for tests): \n{slimit_for_ik}")
        else:
            self.get_logger().warn("robot.qlim is None. IK calls requiring limits might fail or use defaults.")
            # Fallback to some default if robot.qlim is None, to allow tests to proceed
            min_limits_default = np.array([-np.pi] * self.num_model_joints)
            max_limits_default = np.array([np.pi] * self.num_model_joints)
            slimit_for_ik = np.vstack([min_limits_default, max_limits_default])
            self.get_logger().info(f"Using default +/- pi as slimit_for_ik: \n{slimit_for_ik}")


        # === Test 1: Direct NumPy Uniform Test with Kinova's qlim values ===
        if slimit_for_ik is not None:
            self.get_logger().info("--- Direct NumPy Uniform Test with Kinova's qlim ---")
            test_rng = np.random.default_rng()
            num_samples_per_joint = 1000
            all_direct_tests_passed = True
            for i in range(slimit_for_ik.shape[1]): 
                low = slimit_for_ik[0, i]
                high = slimit_for_ik[1, i]
                if low > high:
                    self.get_logger().error(f"Direct Test Joint {i}: low > high ({low} > {high})! problem with qlim.")
                    all_direct_tests_passed = False; continue
                try:
                    for _ in range(num_samples_per_joint):
                        sample = test_rng.uniform(low=low, high=high)
                except OverflowError as oe_direct:
                    self.get_logger().error(f"!!! OverflowError on DIRECT NumPy UNIFORM TEST for Joint {i} (low={low}, high={high}): {oe_direct}")
                    all_direct_tests_passed = False; break 
                except Exception as e_direct:
                    self.get_logger().error(f"!!! Exception on DIRECT NumPy UNIFORM TEST for Joint {i} (low={low}, high={high}): {e_direct}")
                    all_direct_tests_passed = False; break
            if all_direct_tests_passed: self.get_logger().info("--- All Direct NumPy Uniform Tests PASSED ---")
            else: self.get_logger().error("--- One or more Direct NumPy Uniform Tests FAILED ---")
        else:
            self.get_logger().warn("Skipping Direct NumPy Uniform Test as slimit_for_ik is not available.")


        # === Test 2: IK_LM direct instantiation (slimit=1, kq=0.0) ===
        self.get_logger().info("--- Testing IK_LM direct instantiation (slimit=1, kq=0.0) ---")
        try:
            # Ensure robot.qlim (used by solver if joint_limits=True) is our cleaned version
            original_robot_qlim_for_solver_test = None
            if self.robot.qlim is not None and slimit_for_ik is not None:
                original_robot_qlim_for_solver_test = np.copy(self.robot.qlim)
                self.robot.qlim = slimit_for_ik 
                self.get_logger().info("Updated self.robot.qlim to cleaned slimit for IK_LM solver instance.")
            elif self.robot.qlim is None and slimit_for_ik is not None:
                self.get_logger().info("self.robot.qlim was None, setting to cleaned slimit for IK_LM solver instance.")
                self.robot.qlim = slimit_for_ik


            solver_lm_direct = IK_LM(
                ilimit=100,        
                slimit=1,          # Max number of searches = 1 (essentially no random restarts)
                tol=1e-7,
                joint_limits=True, # Should use the self.robot.qlim we just set
                seed=42,           
                kq=0.0             # Disable null-space joint limit avoidance
            )
            
            solution_obj_lm = solver_lm_direct.solve(ets=self.robot.ets(), Tep=Tep_target, q0=q0_guess)
            self.log_ik_solution("IK_LM (direct, slimit=1, kq=0)", solution_obj_lm)

            if original_robot_qlim_for_solver_test is not None: # Restore original
                self.robot.qlim = original_robot_qlim_for_solver_test

        except Exception as e:
            self.get_logger().error(f"Exception during IK_LM direct instantiation/solve: {e}")
            if 'original_robot_qlim_for_solver_test' in locals() and original_robot_qlim_for_solver_test is not None : self.robot.qlim = original_robot_qlim_for_solver_test


        # === Test 3: Standard ikine_LM wrapper method tests ===
        self.get_logger().info("--- Testing robot.ikine_LM (with explicit slimit_for_ik) ---")
        if slimit_for_ik is not None:
            try:
                sol_lm_wrapper_slimit = self.robot.ikine_LM(Tep=Tep_target, q0=q0_guess, slimit=slimit_for_ik)
                self.log_ik_solution("robot.ikine_LM (explicit slimit)", sol_lm_wrapper_slimit)
            except Exception as e: self.get_logger().error(f"Exception during robot.ikine_LM (explicit slimit): {e}")
        else:
            self.get_logger().warn("Skipping robot.ikine_LM with explicit slimit as slimit_for_ik is None.")

        self.get_logger().info("--- Testing robot.ikine_LM (joint_limits=False) ---")
        try:
            sol_lm_wrapper_nolim = self.robot.ikine_LM(Tep=Tep_target, q0=q0_guess, joint_limits=False)
            self.log_ik_solution("robot.ikine_LM (joint_limits=False)", sol_lm_wrapper_nolim)
        except Exception as e: self.get_logger().error(f"Exception during robot.ikine_LM (joint_limits=False): {e}")


        # === Test 4: Standard ikine_NR wrapper method tests ===
        self.get_logger().info("--- Testing robot.ikine_NR (joint_limits=True, uses robot.qlim) ---")
        try:
            original_robot_qlim_for_nr = None
            if self.robot.qlim is not None and slimit_for_ik is not None:
                 original_robot_qlim_for_nr = np.copy(self.robot.qlim)
                 self.robot.qlim = slimit_for_ik # Ensure it uses the clean limits
            elif self.robot.qlim is None and slimit_for_ik is not None:
                 self.robot.qlim = slimit_for_ik
            
            sol_nr_wrapper_lim = self.robot.ikine_NR(Tep=Tep_target, q0=q0_guess, joint_limits=True)
            
            if original_robot_qlim_for_nr is not None : self.robot.qlim = original_robot_qlim_for_nr
            self.log_ik_solution("robot.ikine_NR (joint_limits=True)", sol_nr_wrapper_lim)
        except Exception as e: 
            self.get_logger().error(f"Exception during robot.ikine_NR (joint_limits=True): {e}")
            if 'original_robot_qlim_for_nr' in locals() and original_robot_qlim_for_nr is not None : self.robot.qlim = original_robot_qlim_for_nr

        self.get_logger().info("--- Testing robot.ikine_NR (joint_limits=False) ---")
        try:
            sol_nr_wrapper_nolim = self.robot.ikine_NR(Tep=Tep_target, q0=q0_guess, joint_limits=False)
            self.log_ik_solution("robot.ikine_NR (joint_limits=False)", sol_nr_wrapper_nolim)
        except Exception as e: self.get_logger().error(f"Exception during robot.ikine_NR (joint_limits=False): {e}")


    def log_ik_solution(self, solver_name: str, sol):
        # Check if sol is the expected Solution object (has .success) or just a q array (older RTB or some specific returns)
        if hasattr(sol, 'success'):
            if sol.success:
                self.get_logger().info(f"{solver_name} Solution: q={sol.q}, iter={sol.iterations}, searches={getattr(sol, 'searches', 'N/A')}, err={sol.residual:.3e}")
            else:
                self.get_logger().warn(f"{solver_name} FAILED: {sol.reason}, iter={sol.iterations}, searches={getattr(sol, 'searches', 'N/A')}, err={sol.residual:.3e}")
        elif isinstance(sol, np.ndarray): # If it's just a q array
             self.get_logger().info(f"{solver_name} Solution: q={sol} (Direct q array returned, success not explicitly indicated by solver wrapper)")
        elif sol is None:
             self.get_logger().warn(f"{solver_name} FAILED: Solution was None.")
        else: # Some other unexpected return type
             self.get_logger().warn(f"{solver_name} FAILED: Unexpected solution type: {type(sol)}, value: {sol}")


def main(args=None):
    rclpy.init(args=args)
    ik_test_node = IKTestNode()
    # Node will run tests in __init__ and call rclpy.try_shutdown()
    # No explicit spin needed here as it's a single-shot test.
if __name__ == '__main__':
    main()