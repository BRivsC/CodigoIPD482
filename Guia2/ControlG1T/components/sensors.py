import random

class GPSSensor:
    """GPS sensor class that reads data from CoppeliaSim."""
    
    def __init__(self, sim, gps_handle, robot_handle, trailer_handle=None):
        """Initialize GPS sensor.
        
        Args:
            sim: CoppeliaSim simulation object
            gps_handle: Handle to GPS object
            robot_handle: Handle to robot object
            trailer_handle: Handle to trailer object (optional)
        """
        self.sim = sim
        self.gps_handle = gps_handle
        self.robot_handle = robot_handle
        self.trailer_handle = trailer_handle
    
    def get_position(self):
        """Get position from CoppeliaSim GPS.
        
        Returns:
            list: [x, y, z] position
        """
        return self.sim.getObjectPosition(self.gps_handle, -1)
    
    def get_real_position(self):
        """Get real position (same as get_position since we're not simulating errors).
        
        Returns:
            list: [x, y, z] actual position
        """
        return self.get_position()
    
    def get_trailer_position(self):
        """Get trailer position.
        
        Returns:
            list: [x, y, z] trailer position
        """
        if self.trailer_handle:
            return self.sim.getObjectPosition(self.trailer_handle, -1)
        return [0, 0, 0]  # Return zeros if trailer handle not available
        
    def get_orientation(self):
        """Get robot orientation.
        
        Returns:
            list: [roll, pitch, yaw] orientation
        """
        return self.sim.getObjectOrientation(self.robot_handle, -1)
        
    def get_trailer_orientation(self):
        """Get trailer orientation.
        
        Returns:
            list: [roll, pitch, yaw] trailer orientation
        """
        if self.trailer_handle:
            return self.sim.getObjectOrientation(self.trailer_handle, -1)
        return [0, 0, 0]  # Return zeros if trailer handle not available 