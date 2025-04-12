import sys
import threading
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2, Common_pb2, BaseCyclic_pb2, ControlConfig_pb2, ActuatorConfig_pb2
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient


import time

from utils import Utils

class KinovaDualArmController(Utils):
    def __init__(self, 
                 ips: list[str], 
                 ports: list[int] = [10000, 10001],
                 credentials: list[tuple[str, str]] = [("admin", "admin")]*2):
        super().__init__()
        if len(ips) != 2 or len(credentials) != 2:
            raise ValueError("Requires exactly 2 IP addresses and credential pairs")
        
        self.ips = ips
        self.ports = ports
        self.credentials = credentials
        self._setup_connection()

    def _setup_connection(self):
        """Establish secure session-managed connection"""
        try:
            self.arms = []
            try:
                for i in range(2):
                    arm = self._create_arm_connection(self.ips[i], self.ports[i], self.credentials[i])
                    self.arms.append(arm)
                
                print("Homing both arms...")
                
                for i in range(2):
                    self._home_arm(i)  # New homing method
                    
                # Verify both arms are ready
                for i, arm in enumerate(self.arms):
                    servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                    print("Trying to set servoing mode to SINGLE_LEVEL_SERVOING")
                    self._ensure_servoing_mode(arm['base'], servoing_mode)
                        
            except:
                self._safe_teardown()
                raise

        except Exception as e:
            self._safe_teardown()
            raise ConnectionError(f"Connection failed: {str(e)}")

    def _create_arm_connection(self, ip: str, port: int, creds: tuple[str, str]) -> dict:
        """Establish a single robot connection with session management"""
        transport = TCPTransport()
        router = RouterClient(transport, lambda ex: print(f"Connection Error: {ex}"))
        transport.connect(ip, port)
        
        session = SessionManager(router)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username, session_info.password = creds
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000
        
        try:
            session.CreateSession(session_info)
            print("Created session succesfully for " + ip)
            
        except Exception as e:
            transport.disconnect()
            raise ConnectionError(f"Session failed for {ip}: {str(e)}")
        return {
            'transport': transport,
            'router': router,
            'session': session,
            'base': BaseClient(router),                 # High/level
            'base_cyclic': BaseCyclicClient(router),    # Low-level
            'control_config': ControlConfigClient(router), # Low-level
            'actuator_config': ActuatorConfigClient(router), # Low-level
        }
    
    def _ensure_servoing_mode(self, base_client: BaseClient, servoing_mode: Base_pb2) -> bool:
        """Set appropriate servoing mode if not already set"""
        current_mode = base_client.GetServoingMode()
        if current_mode.servoing_mode != servoing_mode:
            base_client.SetServoingMode(Base_pb2.ServoingModeInformation(
                servoing_mode=servoing_mode
            ))
            print("Servoing mode set!")
        else:
            print("Servoing mode is already set")

    def _safe_teardown(self):
        """Guaranteed cleanup of all connections"""
        for arm in self.arms:
            try:
                arm['session'].CloseSession()
                arm['transport'].disconnect()
            except Exception as e:
                print(f"Cleanup error: {str(e)}")


    def _notification_factory(self, event: threading.Event):
        def callback(notification):
            if notification.action_event in [
                Base_pb2.ACTION_END, 
                Base_pb2.ACTION_ABORT
            ]:
                event.set()
        return callback
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._safe_teardown()

    def _home_arm(self, arm_index: int):
        """Home individual arm with feedback"""
        print(f"Homing arm {arm_index}...")
        base = self.arms[arm_index]['base']

        try:
            base.ClearFaults()
        except Exception as e:
            print(f"Fault clear attempt failed: {str(e)}")
        
        # Find Home action
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = base.ReadAllActions(action_type)
        
        home_action = next((a for a in action_list.action_list if a.name == "Home"), None)
        if not home_action:
            raise ValueError(f"Arm {arm_index} missing Home action")

        event = threading.Event()
        handle = base.OnNotificationActionTopic(
            self._notification_factory(event),
            Base_pb2.NotificationOptions()
        )
        
        try:
            base.ExecuteActionFromReference(home_action.handle)
            if not event.wait(30):
                raise TimeoutError(f"Arm {arm_index} homing timed out")
            print(f"Arm {arm_index} homed successfully")
        finally:
            base.Unsubscribe(handle)

    def _build_cartesian_action(self, arm_index: int, pose: list, 
                              max_linear_speed: float = None, 
                              max_angular_speed: float = None):
        """Enhanced Cartesian action builder with speed constraints"""
        if len(pose) not in (3,6):
            raise ValueError("Pose must contain 3 (position) or 6 (position+orientation) elements")
        
        action = Base_pb2.Action()
        action.name = f"Arm_{arm_index}_Cartesian_Action"
        
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = pose[0]  # meters
        cartesian_pose.y = pose[1]
        cartesian_pose.z = pose[2]
        
        # Handle orientation if provided
        if len(pose) == 6:
            cartesian_pose.theta_x = pose[3]
            cartesian_pose.theta_y = pose[4]
            cartesian_pose.theta_z = pose[5]
        else:
            # Maintain current orientation
            current_pose = self.arms[arm_index]['base'].GetMeasuredCartesianPose()
            cartesian_pose.theta_x = current_pose.theta_x
            cartesian_pose.theta_y = current_pose.theta_y
            cartesian_pose.theta_z = current_pose.theta_z
        
        # Add speed constraints if provided
        if max_linear_speed or max_angular_speed:
            constraint = action.reach_pose.constraint
            speed_constraint = constraint.speed
            if max_linear_speed:
                speed_constraint.translation = max_linear_speed  # m/s
            if max_angular_speed:
                speed_constraint.orientation = max_angular_speed  # deg/s

        return action
    
    def _build_joint_action(self, arm_index: int, angles_deg: list):
        """Create a joint angle action for specified arm"""
        if len(angles_deg) != 6:
            raise ValueError("Requires exactly 6 joint angles")
            
        action = Base_pb2.Action()
        action.name = f"Arm_{arm_index}_Joint_Action"
        action.reach_joint_angles.joint_angles.joint_angles.extend([
            Base_pb2.JointAngle(joint_identifier=i, value=angles_deg[i])
            for i in range(6)
        ])
        return action
    
    def coordinated_cartesian_move(self, 
                                            targets: list[list[float]],
                                            max_linear_speeds: list[float] = [0.1, 0.1],
                                            max_angular_speeds: list[float] = [10.0, 10.0],
                                            timeout: float = 30.0):
        """
        Execute Cartesian move with controlled velocity
        :param targets: List of target poses (3 or 6 elements per arm)
        :param max_linear_speeds: Maximum translation speed in m/s for each arm
        :param max_angular_speeds: Maximum rotation speed in deg/s for each arm
        """
        if len(targets) != 2:
            raise ValueError("Requires targets for both arms")
        
        if len(targets[0]) not in (3,6) or len(targets[1]) not in (3,6):
            raise ValueError("Requires 3 or 6 coordinates for each arm (position or full pose)")

        print("Starting coordinated Cartesian move...")
        # Setup event synchronization
        events = [threading.Event() for _ in range(2)]
        handles = []
        
        try:
            # Create and subscribe notifications
            for i in range(2):

                action = self._build_cartesian_action(i, targets[i],
                                                      max_linear_speed=max_linear_speeds[i],
                                                      max_angular_speed=max_angular_speeds[i]
                                                     )
                
                if action is None:
                    raise ValueError(f"Failed to create action for arm {i}")
                
                notif_handle = self.arms[i]['base'].OnNotificationActionTopic(
                    self._notification_factory(events[i]),
                    Base_pb2.NotificationOptions()
                )
    
                handles.append(notif_handle)
                self.arms[i]['base'].ExecuteAction(action)
            
            
            # Wait with combined timeout
            start = time.time()

            while time.time() - start < timeout:
                if all(e.is_set() for e in events):
                    return
                time.sleep(0.1)
            raise TimeoutError("Coordinated move exceeded timeout")
        
        finally:
            for i, handle in enumerate(handles):
                self.arms[i]['base'].Unsubscribe(handle)

    def send_joint_speeds(self, arm_index: int, 
                    joint_speeds: list[float], 
                    duration: float,
                    control_frequency: int = 100):
        """
        Send velocity commands using BaseCyclic ActuatorCommand
        :param joint_speeds: List of 6 velocity values in deg/s
        :param duration: Execution time in seconds
        :param control_frequency: Control loop frequency (1-100Hz)
        """
        if len(joint_speeds) != 6:
            raise ValueError("Requires exactly 6 joint speeds")
        
        base_cyclic = self.arms[arm_index]['base_cyclic']
        base = self.arms[arm_index]['base']
        control_config = self.arms[arm_index]['control_config']
        actuator_config = self.arms[arm_index]['actuator_config']

        try:
            # 1. Clear existing faults
            try:
                base.ClearFaults()
            except Exception as e:
                print(f"Fault clear attempt failed: {str(e)}")
            time.sleep(0.5)

            # 2. Set control mode
            control_mode = ActuatorConfig_pb2.ControlModeInformation()
            control_mode.control_mode = ActuatorConfig_pb2.ControlMode.Value('VELOCITY')
            for actuator_id in range(1, 7):  # Actuator IDs start at 1
                actuator_config.SetControlMode(control_mode, actuator_id)


            # 3. Set servoing mode
            servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            print("Trying to set servoing mode to LOW_LEVEL_SERVOING")
            self._ensure_servoing_mode(base, servoing_mode)

            # 4. Verify LOW_LEVEL_SERVOING
            start_time = time.time()
            while base.GetServoingMode().servoing_mode != Base_pb2.LOW_LEVEL_SERVOING:
                if time.time() - start_time > 5:
                    raise TimeoutError("Failed to enter LOW_LEVEL_SERVOING")
                time.sleep(0.1)

            # 5. Prepare cyclic command 
            command = BaseCyclic_pb2.Command()
            num_actuators = base.GetActuatorCount().count  # Should be 6
            
            # Initialize actuator commands
            for i in range(num_actuators):
                actuator = command.actuators.add()
                actuator.command_id = i
                actuator.flags = 1  # Enable servoing control
                actuator.velocity = joint_speeds[i]  # deg/s
            
            # Send commands in real-time loop
            interval = 1.0 / control_frequency
            start_time = time.time()
        
            while time.time() - start_time < duration:
                # Update frame ID only
                command.frame_id += 1
                
                # Send command (no timestamp needed)
                base_cyclic.Refresh(command)
                
                # Maintain frequency
                target_time = time.perf_counter() + interval
                while time.perf_counter() < target_time: pass
        except Exception as e:
            print(f"Error during joint speed command: {str(e)}")
        finally:
            # Ensure Low Level Servoing
            servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            print("Trying to set servoing mode to LOW_LEVEL_SERVOING")
            self._ensure_servoing_mode(base, servoing_mode)

            # Stop all joints
            for actuator in command.actuators:
                actuator.velocity = 0.0
            base_cyclic.Refresh(command)
    
            # Before setting SINGLE_LEVEL_SERVOING
            control_mode.control_mode = ActuatorConfig_pb2.POSITION
            for actuator_id in range(1, 7):
                actuator_config.SetControlMode(control_mode, actuator_id)

            servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            print("Trying to set servoing mode to SINGLE_LEVEL_SERVOING")
            self._ensure_servoing_mode(base, servoing_mode)

            

    def coordinated_joint_move(self, targets: list, timeout: float = 30.0):
        """Execute synchronized joint movement on both arms"""
        if len(targets) != 2 or any(len(t) != 6 for t in targets):
            raise ValueError("Requires joint targets for both arms (6 angles each)")
        
        events = [threading.Event() for _ in range(2)]
        handles = []
        
        try:
            # Build and execute actions
            for i in range(2):
                action = self._build_joint_action(i, targets[i])
                notif_handle = self.arms[i]['base'].OnNotificationActionTopic(
                    self._notification_factory(events[i]),
                    Base_pb2.NotificationOptions()
                )
                handles.append(notif_handle)
                self.arms[i]['base'].ExecuteAction(action)
            
            # Wait for completion
            start_time = time.time()
            while time.time() - start_time < timeout:
                if all(e.is_set() for e in events):
                    return True
                time.sleep(0.1)
            raise TimeoutError("Joint movement timed out")
        
        finally:
            for i, handle in enumerate(handles):
                self.arms[i]['base'].Unsubscribe(handle)

    def target_objective(self, 
                         target_pose: list[float], 
                         max_linear_speeds: list[float] = [0.1, 0.1],
                         max_angular_speeds: list[float] = [10.0, 10.0],
                         timeout: float = 30.0) -> None:
        """Move both arms to a target pose"""
        if len(target_pose) != 3:
            raise ValueError("Requires exactly 3 coordinates for target pose")
        
        # Check what robot the target is closer to
        if target_pose[1] > 0:
            robot_1_coords = self.base_to_robot_1_coordinates(target_pose)

            # Get current pose to validate against
            current_pose = self.arms[1]['base'].GetMeasuredCartesianPose()
            robot_2_coords = [current_pose.x, current_pose.y, current_pose.z]
        else:
            robot_2_coords = self.base_to_robot_2_coordinates(target_pose)

            # Get current pose to validate against
            current_pose = self.arms[0]['base'].GetMeasuredCartesianPose()
            robot_1_coords = [current_pose.x, current_pose.y, current_pose.z]

        self.coordinated_cartesian_move([robot_1_coords, robot_2_coords], max_linear_speeds, max_angular_speeds, timeout)

# Usage Example
if __name__ == "__main__":
    try:
        ROBOT_IPS = ["192.168.1.10", "192.168.1.11"]
        PORTS = [10000, 10000]
        with KinovaDualArmController(ROBOT_IPS, PORTS) as dual_arm:
            # robot_1_coords = dual_arm.base_to_robot_1_coordinates([0.75, 0.15, 0.4] + dual_arm.default_cartesian_angles_front)
            # robot_2_coords = dual_arm.base_to_robot_2_coordinates([0.75, -0.15, 0.4])
            # dual_arm.coordinated_cartesian_move(
            #     [robot_1_coords, robot_2_coords],
            #     max_linear_speeds=[0.1, 0.1],  # m/s
            #     max_angular_speeds=[15.0, 15.0]  # deg/s
            # )
            # dual_arm.coordinated_joint_move(
            #     [[360, 0, 0, 360, 360, 360], [360, 0, 0, 360, 360, 360]]
            # )

            dual_arm.send_joint_speeds(
                                        arm_index=0,
                                        # joint_speeds=[10.0, 10.0, 10.0, 0.0, 0.0, 0.0],
                                        joint_speeds = [0.0, 0.0, 0.0, 30.0, 0.0, 0.0],
                                        duration=5.0,
                                        control_frequency=100
                                      )
            
            
            
            # dual_arm.send_joint_speeds(
            #                             arm_index=0,
            #                             # joint_speeds=[10.0, 10.0, 10.0, 0.0, 0.0, 0.0],
            #                             joint_speeds = [50.0, 0.0, 0.0, 20.0, 20.0, 0.0],
            #                             duration=1.0,
            #                             control_frequency=100
            #                           )
            

            # target_pose = [0.8, -0.15, 0.3] # Position of a target relative to the robot base
            # dual_arm.target_objective(target_pose,
            #                           max_linear_speeds=[0.4, 0.1],  # m/s
            #                           max_angular_speeds=[15.0, 15.0])  # deg/s)
            print("Dual arm movement successful!")
    except Exception as e:
        print(f"Dual arm operation failed: {str(e)}")
        sys.exit(1)