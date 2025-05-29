class Utils:
    def __init__(self):
        self.default_cartesian_angles_front = [90.0, 0.0, 90.0]
        pass

    def base_to_robot_1_coordinates(self, base_frame: list[float]):
        """
        Convert a base frame to a robot frame.
        """
        # Assuming base_coordinates is a list of [x, y, z] coordinates
        robot_frame = base_frame 
        robot_frame[1] -= 0.15
        
        return robot_frame
    
    def base_to_robot_2_coordinates(self, base_frame: list[float]):
        """
        Convert a base frame to a robot frame.
        """
        # Assuming base_coordinates is a list of [x, y, z] coordinates
        robot_frame = base_frame 
        robot_frame[1] += 0.15
        
        return robot_frame