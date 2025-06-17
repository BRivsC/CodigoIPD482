from components.controllers import PIDController
from components.utils import distance, angle_to_target
import math

class RobotController:
    """Class to control a robot with a trailer."""
    
    def __init__(self, simulation, sensors):
        """Initialize robot controller.
        
        Args:
            simulation: SimulationEnvironment instance
            sensors: GPSSensor instance
        """
        # Control gains - ajustados para mejor estabilidad
        self.kp = 4.0  # Reducido para menos agresividad
        self.ki = 0.02  # Reducido para menos windup
        self.kd = 0.1   # Agregado para mejor estabilidad
        
        self.sim = simulation
        self.sensors = sensors
        
        # PID controllers
        self.pid_distance = PIDController(self.kp, self.ki, self.kd)
        self.pid_angle = PIDController(self.kp * 1.5, self.ki, self.kd * 2)  # Más agresivo para ángulos
        
        # Control limits
        self.max_error = 0.1
        self.max_velocity = 8.5   # Velocidad máxima [m/s]
        self.max_angular = 0.5    # Velocidad angular máxima [rad/s]
        self.min_velocity = 0.1   # Velocidad mínima para evitar paradas
        
        # Smoothing
        self.velocity_smoothing = 0.8  # Factor de suavizado de velocidad
        self.prev_left_vel = 0.0
        self.prev_right_vel = 0.0
        
    def reset_controllers(self):
        """Reset all PID controllers."""
        self.pid_distance.reset()
        self.pid_angle.reset()
        self.prev_left_vel = 0.0
        self.prev_right_vel = 0.0
        
    def smooth_velocity(self, new_vel, prev_vel):
        """Apply velocity smoothing to avoid abrupt changes."""
        return self.velocity_smoothing * prev_vel + (1 - self.velocity_smoothing) * new_vel
        
    def navigate_to_waypoint(self, target_position, visualizer=None):
        """Navigate robot to a target waypoint.
        
        Args:
            target_position: Target position [x, y, z]
            visualizer: Optional TrajectoryVisualizer
            
        Returns:
            bool: True if waypoint reached, False otherwise
        """
        sim = self.sim.sim
        handles = self.sim.handles
        
        # Get current positions and orientations
        gps_position = self.sensors.get_position()
        real_position = self.sensors.get_real_position()
        trailer_position = self.sensors.get_trailer_position()
        current_orientation = self.sensors.get_orientation()
        
        # Calculate distance error using GPS position
        dist_error = distance(gps_position, target_position)
        
        # If we've reached the waypoint
        if dist_error < self.max_error:
            # Stop the motors gradually
            if 'left_motor' in handles and 'right_motor' in handles:
                self.sim.set_joint_velocity(handles['left_motor'], 0)
                self.sim.set_joint_velocity(handles['right_motor'], 0)
            return True
        
        # Calculate angle error using GPS position
        angle_error = angle_to_target(gps_position, target_position, current_orientation)
        
        # Get time delta
        current_time = sim.getSimulationTime()
        delta_time = 0.05  # Default value if first iteration
        if hasattr(self, 'prev_time'):
            delta_time = current_time - self.prev_time
        self.prev_time = current_time
        
        # Calculate control signals
        velocity_signal = self.pid_distance.compute(dist_error, delta_time)
        angular_signal = self.pid_angle.compute(angle_error, delta_time)
        
        # Apply velocity limits and scaling
        velocity_signal = max(self.min_velocity, min(velocity_signal, self.max_velocity))
        angular_signal = max(-self.max_angular, min(angular_signal, self.max_angular))
        
        # Reduce velocity when making sharp turns
        angle_factor = 1.0 - abs(angle_error) / math.pi
        velocity_signal *= max(0.3, angle_factor)  # Mantener al menos 30% de velocidad
        
        # Convert control signals to motor velocities
        left_velocity = velocity_signal - angular_signal
        right_velocity = velocity_signal + angular_signal
        
        # Apply smoothing
        left_velocity = self.smooth_velocity(left_velocity, self.prev_left_vel)
        right_velocity = self.smooth_velocity(right_velocity, self.prev_right_vel)
        
        # Store for next iteration
        self.prev_left_vel = left_velocity
        self.prev_right_vel = right_velocity
        
        # Apply velocities to tractor motors
        if 'left_motor' in handles and 'right_motor' in handles:
            self.sim.set_joint_velocity(handles['left_motor'], left_velocity)
            self.sim.set_joint_velocity(handles['right_motor'], right_velocity)
        
        # Print debug information (reducido para menos spam)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 10 == 0:  # Print every 10 iterations
            print(f"GPS: {[round(p, 2) for p in gps_position]}, "
                  f"Dist: {round(dist_error, 2)}m, "
                  f"Angle: {round(math.degrees(angle_error), 1)}°, "
                  f"Vel: L={round(left_velocity, 2)}, R={round(right_velocity, 2)}")
        
        # Record data for visualization if provided
        if visualizer:
            elapsed_time = current_time - self.start_time
            visualizer.record_data(elapsed_time, real_position, gps_position, trailer_position)
        
        return False
        
    def set_start_time(self, start_time):
        """Set the simulation start time.
        
        Args:
            start_time: Simulation start time
        """
        self.start_time = start_time
        self.prev_time = start_time
        self.debug_counter = 0 