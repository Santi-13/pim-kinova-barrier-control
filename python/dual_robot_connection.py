import sys
import threading
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2, Common_pb2
import time

class KinovaDualArmController:
    def __init__(self, 
                 ips: list[str], 
                 ports: list[int] = [10000, 10001],
                 credentials: list[tuple[str, str]] = [("admin", "admin")]*2):
        
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
                    self._ensure_servoing_mode(arm['base'])
                        
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
            'base': BaseClient(router)
        }
    
    def _ensure_servoing_mode(self, base_client: BaseClient) -> bool:
        """Set appropriate servoing mode if not already set"""
        current_mode = base_client.GetServoingMode()
        if current_mode.servoing_mode != Base_pb2.SINGLE_LEVEL_SERVOING:
            base_client.SetServoingMode(Base_pb2.ServoingModeInformation(
                servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING
            ))
            print("Set servoing mode to SINGLE_LEVEL_SERVOING")
        print("Servoing mode is already set to SINGLE_LEVEL_SERVOING")

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

    def _is_pose_reachable(self, arm_index: int, target: list) -> bool:
        """Check if target is within workspace boundaries"""
        # Get robot's Cartesian limits
        base = self.arms[arm_index]['base']
        workspace_info = base.GetMeasuredWorkspace()
        
        return (
            workspace_info.x_min <= target[0] <= workspace_info.x_max and
            workspace_info.y_min <= target[1] <= workspace_info.y_max and
            workspace_info.z_min <= target[2] <= workspace_info.z_max
        )

    def _home_arm(self, arm_index: int):
        """Home individual arm with feedback"""
        print(f"Homing arm {arm_index}...")
        base = self.arms[arm_index]['base']
        
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

    def _build_cartesian_action(self, arm_index: int, pose: list):
        """Create Cartesian action with orientation validation"""
        action = Base_pb2.Action()
        action.name = f"Arm_{arm_index}_Cartesian_Action"
        
        # Get current pose to validate against
        current_pose = self.arms[arm_index]['base'].GetMeasuredCartesianPose()
        print(f"Arm {arm_index} current pose: {current_pose}")
        
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = pose[0]  # meters
        cartesian_pose.y = pose[1]
        cartesian_pose.z = pose[2]
        
        # Maintain current orientation unless specified
        cartesian_pose.theta_x = current_pose.theta_x  # Use current orientation
        cartesian_pose.theta_y = current_pose.theta_y
        cartesian_pose.theta_z = current_pose.theta_z
        
        return action
    
    def coordinated_cartesian_move(self, 
                        targets: list[list[float]], 
                        timeout: float = 30.0) -> None:
        """Execute synchronized movement on both arms"""
        if len(targets) != 2:
            raise ValueError("Requires targets for both arms")
        
        if len(targets[0]) != 3 or len(targets[1]) != 3:
            raise ValueError("Requires exactly 3 coordinates for each arm")
        
        # for i in range(2):
        #     if not self._is_pose_reachable(i, targets[i]):
        #         raise ValueError(f"Arm {i} target pose is unreachable")

        print("Starting coordinated Cartesian move...")
        # Setup event synchronization
        events = [threading.Event() for _ in range(2)]
        handles = []
        
        try:
            # Create and subscribe notifications
            for i in range(2):

                action = self._build_cartesian_action(i, targets[i])
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
# Usage Example
if __name__ == "__main__":
    try:
        ROBOT_IPS = ["192.168.1.10", "192.168.1.11"]
        PORTS = [10000, 10000]
        with KinovaDualArmController(ROBOT_IPS, PORTS) as dual_arm:
            dual_arm.coordinated_cartesian_move(
                [[0.75, -0.02, 0.4], [0.75, -0.02, 0.4]]
            )
            print("Dual arm movement successful!")
    except Exception as e:
        print(f"Dual arm operation failed: {str(e)}")
        sys.exit(1)