from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

class SimulationEnvironment:
    """Class to handle CoppeliaSim simulation environment."""
    
    def __init__(self):
        """Initialize connection to CoppeliaSim."""
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        # Stop any previous simulation
        self.sim.stopSimulation()
        time.sleep(1)
        
        self.handles = {}
        self.started = False
        
    def start(self):
        """Start the simulation."""
        if not self.started:
            self.sim.startSimulation()
            self.started = True
            print("Simulation started")
            
    def stop(self):
        """Stop the simulation."""
        if self.started:
            self.sim.stopSimulation()
            self.started = False
            print("Simulation stopped")
            
    def get_handle(self, object_path, handle_name=None):
        """Get handle for an object in the simulation.
        
        Args:
            object_path: Path to the object in CoppeliaSim
            handle_name: Name to store the handle (optional)
            
        Returns:
            int: Object handle
        """
        try:
            handle = self.sim.getObjectHandle(object_path)
            if handle_name:
                self.handles[handle_name] = handle
            return handle
        except Exception as e:
            print(f"Error getting handle for {object_path}: {e}")
            return None
            
    def get_handles(self, handles_dict):
        """Get multiple handles at once.
        
        Args:
            handles_dict: Dictionary mapping handle names to object paths
            
        Returns:
            dict: Dictionary of handles
        """
        for name, path in handles_dict.items():
            self.get_handle(path, name)
        return self.handles
        
    def get_simulation_time(self):
        """Get current simulation time.
        
        Returns:
            float: Current simulation time
        """
        return self.sim.getSimulationTime()
        
    def set_joint_velocity(self, handle, velocity):
        """Set velocity for a joint.
        
        Args:
            handle: Joint handle
            velocity: Target velocity
        """
        self.sim.setJointTargetVelocity(handle, velocity)
        
    def get_joint_position(self, handle):
        """Get position of a joint.
        
        Args:
            handle: Joint handle
            
        Returns:
            float: Joint position
        """
        return self.sim.getJointPosition(handle) 