import sys
import threading
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2, Common_pb2


class SingleRobotController:
    def __init__(self, ip, port=10000, username="admin", password="admin"):
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self._setup_connection()

    def _setup_connection(self):
        """Establish secure session-managed connection"""
        try:
            # 1. Create transport layer
            self.transport = TCPTransport()
            
            # 2. Initialize router with error callback
            self.router = RouterClient(
                self.transport, 
                lambda ex: print(f"Connection Error: {ex}")
            )
            
            # 3. Physical connection
            self.transport.connect(self.ip, self.port)
            print("Session")
            # 4. Session management
            

            self.session = SessionManager(self.router)
            session_info = Session_pb2.CreateSessionInfo(
                username=self.username,
                password=self.password,
                session_inactivity_timeout=60000,    # 60 seconds
                connection_inactivity_timeout=2000  # 2 seconds
            )
            self.session.CreateSession(session_info)
            
            # 5. Initialize service client
            self.base = BaseClient(self.router)
            
            # 6. Initial configuration
            self._ensure_servoing_mode()
            self._home_robot()

        except Exception as e:
            self._safe_teardown()
            raise ConnectionError(f"Connection failed: {str(e)}")

    def _ensure_servoing_mode(self):
        """Set appropriate servoing mode if not already set"""
        current_mode = self.base.GetServoingMode()
        if current_mode.servoing_mode != Base_pb2.SINGLE_LEVEL_SERVOING:
            self.base.SetServoingMode(Base_pb2.ServoingModeInformation(
                servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING
            ))

    def _home_robot(self):
        """Execute homing sequence using predefined Home action"""
        print("Initiating homing sequence...")
        
        # Get all actions of type REACH_JOINT_ANGLES
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)

        # Find the Home action
        home_action = next((a for a in action_list.action_list if a.name == "Home"), None)
        
        if not home_action:
            raise RuntimeError("Home action not found in predefined actions")

        event = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._create_notification_callback(event),
            Base_pb2.NotificationOptions()
        )

        try:
            print("Executing Home action...")
            self.base.ExecuteActionFromReference(home_action.handle)
            if not event.wait(30):  # 30 second timeout for homing
                raise TimeoutError("Homing sequence timed out")
            print("Homing completed successfully")
        finally:
            self.base.Unsubscribe(notification_handle)

    def _safe_teardown(self):
        """Guaranteed resource cleanup"""
        try:
            if hasattr(self, 'session'):
                self.session.CloseSession()
            if hasattr(self, 'transport'):
                self.transport.disconnect()
        except Exception as e:
            print(f"Cleanup error: {str(e)}")

    def maintain_connection(self):
        if self.session.SessionRenew():
            print("Session renewed successfully")
        else:
            raise ConnectionError("Session renewal failed")

    def move_to_joint_angles(self, angles_deg, timeout=30):
        """
        Move robot to specified joint angles (degrees)
        
        Args:
            angles_deg (list[float]): 6 joint angles in degrees
            timeout (float): Maximum execution time in seconds
        """
        if len(angles_deg) != 6:
            raise ValueError("Requires exactly 6 joint angles")
            
        action = Base_pb2.Action()
        action.name = "Target Joint Position"
        action.reach_joint_angles.joint_angles.joint_angles.extend([
            Base_pb2.JointAngle(joint_identifier=i, value=angles_deg[i])
            for i in range(6)
        ])

        event = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._create_notification_callback(event),
            Base_pb2.NotificationOptions()
        )

        try:
            print(f"Moving to {angles_deg} degrees...")
            self.base.ExecuteAction(action)
            if not event.wait(timeout):
                raise TimeoutError("Movement exceeded allowed time")
            print("Movement completed successfully")
        finally:
            self.base.Unsubscribe(notification_handle)

    def _create_notification_callback(self, event):
        def check(notification):
            if notification.action_event in [
                Base_pb2.ACTION_END, 
                Base_pb2.ACTION_ABORT
            ]:
                event.set()
        return check
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._safe_teardown()

    def move_to_coordinates(self, coordinates, timeout=30):
        """
        Move robot to specified Cartesian coordinates
        
        Args:
            coordinates (list[float]): 3D coordinates [x, y, z] in meters
            timeout (float): Maximum execution time in seconds
        """
        if len(coordinates) != 3:
            raise ValueError("Requires exactly 3 coordinates")
        
        action = Base_pb2.Action()
        action.name = "Target Cartesian Position"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose

        cartesian_pose.x = coordinates[0] # (meters)
        cartesian_pose.y = coordinates[1]
        cartesian_pose.z = coordinates[2]
        cartesian_pose.theta_x = 90 # (degrees)
        cartesian_pose.theta_y = 0 # (degrees)
        cartesian_pose.theta_z = 90 # (degrees)

        event = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._create_notification_callback(event),
            Base_pb2.NotificationOptions()
        )

        try:
            print(f"Moving to {coordinates} meters...")
            self.base.ExecuteAction(action)
            if not event.wait(timeout):
                raise TimeoutError("Movement exceeded allowed time")
            print("Movement completed successfully")
        finally:
            self.base.Unsubscribe(notification_handle)
            
# Usage Example
if __name__ == "__main__":
    try:
        # Replace with your robot's IP
        ROBOT_IP = "192.168.1.11"  
        PORT = 10000
        with SingleRobotController(ROBOT_IP, PORT) as robot:
            # Example target position (degrees for each of 6 joints)
            target_angles = [0.0, 15.0, 180.0, 0.0, 30.0, 90.0]
            coordinates = [0.85, -0.02, 0.4] # Meters
            
            #robot.move_to_joint_angles(target_angles)
            robot.move_to_coordinates(coordinates)
            
        print("Operation completed successfully")


        ROBOT_IP = "192.168.1.10"  
        PORT = 10000
        with SingleRobotController(ROBOT_IP, PORT) as robot:
            # Example target position (degrees for each of 6 joints)
            target_angles = [0.0, 15.0, 180.0, 0.0, 30.0, 90.0]
            coordinates = [0.85, -0.02, 0.4] # Meters
            
            #robot.move_to_joint_angles(target_angles)
            robot.move_to_coordinates(coordinates)
            
        print("Operation completed successfully")
    except Exception as e:
        print(f"Error: {str(e)}")
        sys.exit(1)