import matplotlib.pyplot as plt
import numpy as np
import math
import json
import csv
from datetime import datetime

class TrajectoryVisualizer:
    """Class for visualizing robot trajectories and performance analysis."""
    
    def __init__(self):
        """Initialize trajectory visualizer."""
        # Data storage
        self.time_points = []
        self.real_trajectory_x = []
        self.real_trajectory_y = []
        self.gps_trajectory_x = []
        self.gps_trajectory_y = []
        self.trailer_trajectory_x = []
        self.trailer_trajectory_y = []
        self.waypoint_times = [0]  # Time stamps for waypoint changes
        self.has_trailer_data = False
        
        # Performance analysis data
        self.distance_errors = []
        self.velocities = []
        self.angular_velocities = []
        self.previous_position = None
        self.previous_time = None
        
    def save_data_to_json(self, data, filename, description=""):
        """Save data to JSON file with metadata.
        
        Args:
            data: Data dictionary to save
            filename: Output filename
            description: Description of the data
        """
        metadata = {
            "generated_at": datetime.now().isoformat(),
            "description": description,
            "data_type": "robot_trajectory_analysis",
            "units": {
                "time": "seconds",
                "position": "meters", 
                "velocity": "meters/second",
                "angular_velocity": "radians/second",
                "error": "meters"
            }
        }
        
        output = {
            "metadata": metadata,
            "data": data
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"💾 Datos exportados: {filename}")

    def record_data(self, elapsed_time, real_position, gps_position, trailer_position):
        """Record position data at current timestamp.
        
        Args:
            elapsed_time: Current simulation time
            real_position: Real robot position [x, y, z]
            gps_position: GPS position [x, y, z]
            trailer_position: Trailer position [x, y, z] or [0,0,0] if no trailer
        """
        self.time_points.append(elapsed_time)
        self.real_trajectory_x.append(real_position[0])
        self.real_trajectory_y.append(real_position[1])
        self.gps_trajectory_x.append(gps_position[0])
        self.gps_trajectory_y.append(gps_position[1])
        
        # Calculate performance metrics
        if len(self.time_points) > 1:
            dt = elapsed_time - self.previous_time
            if dt > 0:
                # Calculate velocity
                dx = real_position[0] - self.previous_position[0]
                dy = real_position[1] - self.previous_position[1]
                velocity = math.sqrt(dx*dx + dy*dy) / dt
                self.velocities.append(velocity)
                
                # Calculate angular velocity (simplified)
                if len(self.real_trajectory_x) > 2:
                    # Calculate heading change
                    prev_heading = math.atan2(self.real_trajectory_y[-2] - self.real_trajectory_y[-3],
                                            self.real_trajectory_x[-2] - self.real_trajectory_x[-3])
                    curr_heading = math.atan2(dy, dx)
                    angular_vel = abs(curr_heading - prev_heading) / dt
                    self.angular_velocities.append(angular_vel)
                else:
                    self.angular_velocities.append(0.0)
            else:
                self.velocities.append(0.0)
                self.angular_velocities.append(0.0)
        else:
            self.velocities.append(0.0)
            self.angular_velocities.append(0.0)
        
        # Check if we have valid trailer data
        if trailer_position[0] != 0 or trailer_position[1] != 0:
            self.has_trailer_data = True
            self.trailer_trajectory_x.append(trailer_position[0])
            self.trailer_trajectory_y.append(trailer_position[1])
        
        # Store for next iteration
        self.previous_position = real_position
        self.previous_time = elapsed_time
        
    def record_waypoint_reached(self, elapsed_time):
        """Record time when a waypoint is reached.
        
        Args:
            elapsed_time: Current simulation time
        """
        self.waypoint_times.append(elapsed_time)
    
    def calculate_tracking_errors(self, planned_waypoints):
        """Calculate tracking errors between planned and actual trajectory.
        
        Args:
            planned_waypoints: List of planned waypoints [[x, y, z], ...]
        
        Returns:
            dict: Dictionary with error metrics
        """
        if not self.real_trajectory_x or not planned_waypoints:
            return {}
        
        errors = []
        cross_track_errors = []
        
        # For each point in actual trajectory, find closest point on planned path
        for i in range(len(self.real_trajectory_x)):
            actual_x, actual_y = self.real_trajectory_x[i], self.real_trajectory_y[i]
            
            # Find minimum distance to planned path
            min_distance = float('inf')
            closest_segment_error = 0.0
            
            for j in range(len(planned_waypoints) - 1):
                # Distance to line segment between waypoints j and j+1
                x1, y1 = planned_waypoints[j][0], planned_waypoints[j][1]
                x2, y2 = planned_waypoints[j+1][0], planned_waypoints[j+1][1]
                
                # Point-to-line distance
                line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                if line_length > 0:
                    t = max(0, min(1, ((actual_x-x1)*(x2-x1) + (actual_y-y1)*(y2-y1)) / (line_length**2)))
                    projection_x = x1 + t * (x2 - x1)
                    projection_y = y1 + t * (y2 - y1)
                    distance = math.sqrt((actual_x - projection_x)**2 + (actual_y - projection_y)**2)
                    
                    if distance < min_distance:
                        min_distance = distance
                        # Calculate cross-track error (signed distance)
                        cross_product = (actual_x - x1) * (y2 - y1) - (actual_y - y1) * (x2 - x1)
                        closest_segment_error = cross_product / line_length if line_length > 0 else 0
            
            errors.append(min_distance)
            cross_track_errors.append(closest_segment_error)
        
        return {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'rms_error': np.sqrt(np.mean(np.array(errors)**2)),
            'std_error': np.std(errors),
            'cross_track_errors': cross_track_errors,
            'tracking_errors': errors
        }
    
    def plot_trajectory_comparison(self, planned_waypoints, filename='trajectory_comparison.pdf'):
        """Plot comparison between planned and actual trajectories.
        
        Args:
            planned_waypoints: List of planned waypoints
            filename: Output filename
        """
        # Set figure size optimized for double column format
        plt.figure(figsize=(7, 6))
        
        # Extract planned path
        planned_x = [wp[0] for wp in planned_waypoints]
        planned_y = [wp[1] for wp in planned_waypoints]
        
        # Plot planned trajectory
        plt.plot(planned_x, planned_y, 'r--', linewidth=2.5, label='Trayectoria Planificada', alpha=0.8)
        plt.plot(planned_x, planned_y, 'ro', markersize=6, alpha=0.6)
        
        # Plot actual trajectory  
        plt.plot(self.real_trajectory_x, self.real_trajectory_y, 'b-', linewidth=2, label='Trayectoria Real')
        
        # Plot GPS trajectory if different
        if (len(self.gps_trajectory_x) == len(self.real_trajectory_x) and
            (self.real_trajectory_x != self.gps_trajectory_x or self.real_trajectory_y != self.gps_trajectory_y)):
            plt.plot(self.gps_trajectory_x, self.gps_trajectory_y, 'g:', linewidth=1.5, label='GPS', alpha=0.7)
        
        # Plot trailer trajectory if available
        if self.has_trailer_data and len(self.trailer_trajectory_x) > 0:
            plt.plot(self.trailer_trajectory_x, self.trailer_trajectory_y, 'm-', 
                    linewidth=1.8, label='Tráiler', alpha=0.8)
        
        # Mark start and end points
        if self.real_trajectory_x:
            plt.plot(self.real_trajectory_x[0], self.real_trajectory_y[0], 'go', markersize=10, label='Inicio')
            plt.plot(self.real_trajectory_x[-1], self.real_trajectory_y[-1], 'ro', markersize=10, label='Final')
        
        # Add waypoint numbers every 4 waypoints to avoid saturation
        for i, wp in enumerate(planned_waypoints):
            if i % 4 == 0 or i == len(planned_waypoints) - 1:  # Every 4th waypoint + last
                plt.annotate(f'WP{i+1}', (wp[0], wp[1]), xytext=(3, 3), 
                            textcoords='offset points', fontsize=9, fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.8, edgecolor='black'))
        
        # Calculate and display metrics in a more compact way
        error_metrics = self.calculate_tracking_errors(planned_waypoints)
        if error_metrics:
            textstr = f'Error Medio: {error_metrics["mean_error"]:.2f}m\n'
            textstr += f'Error Máx: {error_metrics["max_error"]:.2f}m\n'
            textstr += f'RMS: {error_metrics["rms_error"]:.2f}m'
            plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
        
        plt.title('Comparación de Trayectorias', fontsize=13, fontweight='bold')
        plt.xlabel('Posición X (m)', fontsize=11)
        plt.ylabel('Posición Y (m)', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.legend(loc='upper right', fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"📊 Comparación de trayectorias (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        trajectory_data = {
            "planned_trajectory": {
                "x_coordinates": planned_x,
                "y_coordinates": planned_y,
                "waypoints": [[wp[0], wp[1], wp[2]] for wp in planned_waypoints]
            },
            "actual_trajectory": {
                "x_coordinates": self.real_trajectory_x,
                "y_coordinates": self.real_trajectory_y,
                "time_points": self.time_points
            },
            "gps_trajectory": {
                "x_coordinates": self.gps_trajectory_x,
                "y_coordinates": self.gps_trajectory_y
            },
            "performance_metrics": error_metrics if error_metrics else {},
            "trailer_trajectory": {
                "x_coordinates": self.trailer_trajectory_x if self.has_trailer_data else [],
                "y_coordinates": self.trailer_trajectory_y if self.has_trailer_data else [],
                "available": self.has_trailer_data
            }
        }
        
        self.save_data_to_json(
            trajectory_data, 
            data_filename,
            "Comparación entre trayectoria planificada vs real del robot. Incluye waypoints, métricas de error y datos del tráiler si están disponibles."
        )
    
    def plot_tracking_errors(self, planned_waypoints, filename='tracking_errors.pdf'):
        """Plot tracking errors over time (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        error_metrics = self.calculate_tracking_errors(planned_waypoints)
        if error_metrics and 'tracking_errors' in error_metrics:
            time_subset = self.time_points[:len(error_metrics['tracking_errors'])]
            plt.plot(time_subset, error_metrics['tracking_errors'], 'r-', linewidth=1.5, label='Error de Seguimiento')
            plt.axhline(y=error_metrics['mean_error'], color='blue', linestyle='--', linewidth=2,
                       label=f'Error Promedio: {error_metrics["mean_error"]:.2f}m')
            plt.axhline(y=error_metrics['mean_error'] + error_metrics['std_error'], 
                       color='orange', linestyle=':', alpha=0.8, label=f'±1σ: {error_metrics["std_error"]:.2f}m')
            plt.axhline(y=error_metrics['mean_error'] - error_metrics['std_error'], 
                       color='orange', linestyle=':', alpha=0.8)
            
            # Fill confidence band
            plt.fill_between(time_subset, 
                           error_metrics['mean_error'] - error_metrics['std_error'],
                           error_metrics['mean_error'] + error_metrics['std_error'],
                           alpha=0.2, color='orange')
        
        plt.title('Errores de Seguimiento vs Tiempo', fontsize=13, fontweight='bold')
        plt.xlabel('Tiempo (s)', fontsize=11)
        plt.ylabel('Error de Distancia (m)', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"📈 Errores de seguimiento (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if error_metrics and 'tracking_errors' in error_metrics:
            time_subset = self.time_points[:len(error_metrics['tracking_errors'])]
            errors_data = {
                "time_series": {
                    "time_points": time_subset,
                    "tracking_errors": error_metrics['tracking_errors']
                },
                "statistics": {
                    "mean_error": error_metrics['mean_error'],
                    "max_error": error_metrics['max_error'],
                    "rms_error": error_metrics['rms_error'],
                    "std_error": error_metrics['std_error'],
                    "confidence_bounds": {
                        "upper": error_metrics['mean_error'] + error_metrics['std_error'],
                        "lower": error_metrics['mean_error'] - error_metrics['std_error']
                    }
                }
            }
            
            self.save_data_to_json(
                errors_data,
                data_filename,
                "Evolución temporal de los errores de seguimiento de trayectoria. Incluye serie temporal y estadísticas de error."
            )
    
    def plot_velocity_profile(self, filename='velocity_profile.pdf'):
        """Plot velocity profile over time (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        if self.velocities:
            time_vel = self.time_points[:len(self.velocities)]
            plt.plot(time_vel, self.velocities, 'g-', linewidth=1.5, label='Velocidad Linear')
            plt.axhline(y=np.mean(self.velocities), color='red', linestyle='--', linewidth=2,
                       label=f'Velocidad Promedio: {np.mean(self.velocities):.2f} m/s')
            
            # Mark waypoint changes with reduced density
            for i, wt in enumerate(self.waypoint_times[1:]):  # Skip initial time
                if i % 3 == 0 and wt <= max(time_vel):  # Every 3rd waypoint
                    plt.axvline(x=wt, color='red', linestyle=':', alpha=0.6, linewidth=1)
        
        plt.title('Perfil de Velocidad', fontsize=13, fontweight='bold')
        plt.xlabel('Tiempo (s)', fontsize=11)
        plt.ylabel('Velocidad (m/s)', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"🏃 Perfil de velocidad (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if self.velocities:
            time_vel = self.time_points[:len(self.velocities)]
            velocity_data = {
                "time_series": {
                    "time_points": time_vel,
                    "velocities": self.velocities
                },
                "statistics": {
                    "mean_velocity": float(np.mean(self.velocities)),
                    "max_velocity": float(np.max(self.velocities)),
                    "min_velocity": float(np.min(self.velocities)),
                    "std_velocity": float(np.std(self.velocities)),
                    "median_velocity": float(np.median(self.velocities))
                },
                "waypoint_changes": {
                    "times": self.waypoint_times[1:],  # Skip initial time
                    "description": "Momentos cuando el robot alcanzó cada waypoint"
                }
            }
            
            self.save_data_to_json(
                velocity_data,
                data_filename,
                "Perfil de velocidad lineal del robot a lo largo del tiempo. Incluye estadísticas y marcadores de waypoints."
            )
    
    def plot_angular_velocity(self, filename='angular_velocity.pdf'):
        """Plot angular velocity over time (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        if self.angular_velocities:
            time_ang = self.time_points[:len(self.angular_velocities)]
            angular_vel_degrees = np.array(self.angular_velocities) * 180/np.pi
            plt.plot(time_ang, angular_vel_degrees, 'purple', linewidth=1.5)
            plt.axhline(y=np.mean(angular_vel_degrees), color='orange', linestyle='--', linewidth=2,
                       label=f'Promedio: {np.mean(angular_vel_degrees):.1f}°/s')
        
        plt.title('Velocidad Angular', fontsize=13, fontweight='bold')
        plt.xlabel('Tiempo (s)', fontsize=11)
        plt.ylabel('Velocidad Angular (°/s)', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"🔄 Velocidad angular (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if self.angular_velocities:
            time_ang = self.time_points[:len(self.angular_velocities)]
            angular_vel_degrees = np.array(self.angular_velocities) * 180/np.pi
            angular_data = {
                "time_series": {
                    "time_points": time_ang,
                    "angular_velocities_rad_per_sec": self.angular_velocities,
                    "angular_velocities_deg_per_sec": angular_vel_degrees.tolist()
                },
                "statistics": {
                    "mean_angular_velocity_deg": float(np.mean(angular_vel_degrees)),
                    "max_angular_velocity_deg": float(np.max(angular_vel_degrees)),
                    "min_angular_velocity_deg": float(np.min(angular_vel_degrees)),
                    "std_angular_velocity_deg": float(np.std(angular_vel_degrees)),
                    "mean_angular_velocity_rad": float(np.mean(self.angular_velocities)),
                    "max_angular_velocity_rad": float(np.max(self.angular_velocities))
                }
            }
            
            self.save_data_to_json(
                angular_data,
                data_filename,
                "Velocidad angular del robot indicando intensidad de giros y maniobras. Valores en radianes y grados por segundo."
            )
    
    def plot_cross_track_error(self, planned_waypoints, filename='cross_track_error.pdf'):
        """Plot cross-track error over time (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        error_metrics = self.calculate_tracking_errors(planned_waypoints)
        if error_metrics and 'cross_track_errors' in error_metrics:
            time_cross = self.time_points[:len(error_metrics['cross_track_errors'])]
            plt.plot(time_cross, error_metrics['cross_track_errors'], 'brown', linewidth=1.5, label='Error Transversal')
            plt.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=1)
            plt.axhline(y=np.mean(error_metrics['cross_track_errors']), color='blue', linestyle='--', linewidth=2,
                       label=f'Sesgo: {np.mean(error_metrics["cross_track_errors"]):.3f}m')
            
            # Fill confidence band
            std_cross = np.std(error_metrics['cross_track_errors'])
            plt.fill_between(time_cross, 
                           np.array(error_metrics['cross_track_errors']) - std_cross,
                           np.array(error_metrics['cross_track_errors']) + std_cross,
                           alpha=0.3, color='brown', label=f'±1σ: {std_cross:.3f}m')
        
        plt.title('Error Transversal (Cross-track)', fontsize=13, fontweight='bold')
        plt.xlabel('Tiempo (s)', fontsize=11)
        plt.ylabel('Error Transversal (m)', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"📐 Error transversal (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if error_metrics and 'cross_track_errors' in error_metrics:
            time_cross = self.time_points[:len(error_metrics['cross_track_errors'])]
            std_cross = np.std(error_metrics['cross_track_errors'])
            cross_track_data = {
                "time_series": {
                    "time_points": time_cross,
                    "cross_track_errors": error_metrics['cross_track_errors']
                },
                "statistics": {
                    "mean_bias": float(np.mean(error_metrics['cross_track_errors'])),
                    "std_deviation": float(std_cross),
                    "max_left_deviation": float(np.min(error_metrics['cross_track_errors'])),
                    "max_right_deviation": float(np.max(error_metrics['cross_track_errors'])),
                    "confidence_bounds": {
                        "upper": float(np.mean(error_metrics['cross_track_errors']) + std_cross),
                        "lower": float(np.mean(error_metrics['cross_track_errors']) - std_cross)
                    }
                },
                "interpretation": {
                    "positive_values": "Robot a la derecha del camino planificado",
                    "negative_values": "Robot a la izquierda del camino planificado",
                    "zero_line": "Robot exactamente sobre el camino planificado"
                }
            }
            
            self.save_data_to_json(
                cross_track_data,
                data_filename,
                "Error transversal (cross-track) mostrando desviaciones laterales del robot respecto al camino planificado. Valores positivos = derecha, negativos = izquierda."
            )
    
    def plot_error_distribution(self, planned_waypoints, filename='error_distribution.pdf'):
        """Plot error distribution histogram (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        error_metrics = self.calculate_tracking_errors(planned_waypoints)
        if error_metrics and 'tracking_errors' in error_metrics:
            # Calculate histogram
            hist_counts, bin_edges = np.histogram(error_metrics['tracking_errors'], bins=25, density=True)
            bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
            
            plt.hist(error_metrics['tracking_errors'], bins=25, alpha=0.7, color='skyblue', 
                    edgecolor='black', density=True)
            plt.axvline(error_metrics['mean_error'], color='red', linestyle='--', linewidth=2,
                       label=f'Media: {error_metrics["mean_error"]:.2f}m')
            plt.axvline(error_metrics['mean_error'] + error_metrics['std_error'], 
                       color='orange', linestyle=':', linewidth=1.5, label=f'±1σ: {error_metrics["std_error"]:.2f}m')
            plt.axvline(error_metrics['mean_error'] - error_metrics['std_error'], 
                       color='orange', linestyle=':', linewidth=1.5)
        
        plt.title('Distribución de Errores de Seguimiento', fontsize=13, fontweight='bold')
        plt.xlabel('Error de Distancia (m)', fontsize=11)
        plt.ylabel('Densidad de Probabilidad', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"📊 Distribución de errores (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if error_metrics and 'tracking_errors' in error_metrics:
            hist_counts, bin_edges = np.histogram(error_metrics['tracking_errors'], bins=25, density=True)
            bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
            
            distribution_data = {
                "raw_errors": error_metrics['tracking_errors'],
                "histogram": {
                    "bin_centers": bin_centers.tolist(),
                    "bin_edges": bin_edges.tolist(),
                    "counts": hist_counts.tolist(),
                    "density": True
                },
                "statistics": {
                    "mean": error_metrics['mean_error'],
                    "std": error_metrics['std_error'],
                    "min": float(np.min(error_metrics['tracking_errors'])),
                    "max": float(np.max(error_metrics['tracking_errors'])),
                    "median": float(np.median(error_metrics['tracking_errors'])),
                    "percentiles": {
                        "25th": float(np.percentile(error_metrics['tracking_errors'], 25)),
                        "75th": float(np.percentile(error_metrics['tracking_errors'], 75)),
                        "90th": float(np.percentile(error_metrics['tracking_errors'], 90)),
                        "95th": float(np.percentile(error_metrics['tracking_errors'], 95))
                    }
                }
            }
            
            self.save_data_to_json(
                distribution_data,
                data_filename,
                "Distribución estadística de errores de seguimiento. Histograma y percentiles para análisis de la calidad de seguimiento."
            )
    
    def plot_velocity_distribution(self, filename='velocity_distribution.pdf'):
        """Plot velocity distribution histogram (individual plot)."""
        plt.figure(figsize=(7, 5))
        
        if self.velocities:
            # Calculate histogram
            hist_counts, bin_edges = np.histogram(self.velocities, bins=20, density=True)
            bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
            
            plt.hist(self.velocities, bins=20, alpha=0.7, color='lightgreen', 
                    edgecolor='black', density=True)
            plt.axvline(np.mean(self.velocities), color='red', linestyle='--', linewidth=2,
                       label=f'Media: {np.mean(self.velocities):.2f} m/s')
            plt.axvline(np.median(self.velocities), color='blue', linestyle=':', linewidth=1.5,
                       label=f'Mediana: {np.median(self.velocities):.2f} m/s')
        
        plt.title('Distribución de Velocidades', fontsize=13, fontweight='bold')
        plt.xlabel('Velocidad (m/s)', fontsize=11)
        plt.ylabel('Densidad de Probabilidad', fontsize=11)
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9)
        plt.tight_layout()
        plt.savefig(filename, format='pdf', bbox_inches='tight')
        print(f"🏃 Distribución de velocidades (vectorizado): {filename}")
        
        # Save data to JSON
        data_filename = filename.replace('.pdf', '_data.json')
        if self.velocities:
            hist_counts, bin_edges = np.histogram(self.velocities, bins=20, density=True)
            bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
            
            velocity_dist_data = {
                "raw_velocities": self.velocities,
                "histogram": {
                    "bin_centers": bin_centers.tolist(),
                    "bin_edges": bin_edges.tolist(),
                    "counts": hist_counts.tolist(),
                    "density": True
                },
                "statistics": {
                    "mean": float(np.mean(self.velocities)),
                    "median": float(np.median(self.velocities)),
                    "std": float(np.std(self.velocities)),
                    "min": float(np.min(self.velocities)),
                    "max": float(np.max(self.velocities)),
                    "percentiles": {
                        "25th": float(np.percentile(self.velocities, 25)),
                        "75th": float(np.percentile(self.velocities, 75)),
                        "90th": float(np.percentile(self.velocities, 90)),
                        "95th": float(np.percentile(self.velocities, 95))
                    }
                }
            }
            
            self.save_data_to_json(
                velocity_dist_data,
                data_filename,
                "Distribución estadística de velocidades del robot. Muestra patrones de aceleración/desaceleración y consistencia de velocidad."
            )
    
    def generate_data_summary(self, base_filename='trayectoria_robot'):
        """Generate a summary file explaining all generated data files."""
        summary_filename = f'{base_filename}_data_summary.json'
        
        summary = {
            "metadata": {
                "generated_at": datetime.now().isoformat(),
                "description": "Resumen completo de todos los archivos de datos generados del análisis de trayectoria del robot",
                "total_files": 7,
                "robot_type": "Pioneer P3DX con potencial trailer",
                "control_system": "PID + Campos Potenciales"
            },
            "files_generated": {
                f"{base_filename}_comparison_data.json": {
                    "graph_type": "Comparación de Trayectorias",
                    "purpose": "Análisis espacial de seguimiento de trayectoria",
                    "key_data": [
                        "Coordenadas de trayectoria planificada vs real",
                        "Waypoints originales del algoritmo de campos potenciales", 
                        "Métricas de error de seguimiento (media, máximo, RMS)",
                        "Datos del tráiler si están disponibles",
                        "Lecturas GPS vs posición real"
                    ],
                    "use_case": "Evaluar qué tan bien siguió el robot el camino planificado en el espacio 2D"
                },
                f"{base_filename}_tracking_errors_data.json": {
                    "graph_type": "Errores de Seguimiento vs Tiempo",
                    "purpose": "Análisis temporal de la precisión de seguimiento",
                    "key_data": [
                        "Serie temporal de errores de distancia",
                        "Estadísticas de error (media, desviación estándar, RMS)",
                        "Bandas de confianza (±1σ)",
                        "Evolución del error a lo largo de la misión"
                    ],
                    "use_case": "Identificar momentos de baja precisión y evaluar estabilidad del control"
                },
                f"{base_filename}_velocity_profile_data.json": {
                    "graph_type": "Perfil de Velocidad",
                    "purpose": "Análisis del comportamiento dinámico del robot",
                    "key_data": [
                        "Serie temporal de velocidades lineales",
                        "Estadísticas de velocidad (media, máx, mín, mediana)",
                        "Momentos de cambio de waypoint",
                        "Patrones de aceleración/desaceleración"
                    ],
                    "use_case": "Evaluar suavidad de movimiento y eficiencia de la navegación"
                },
                f"{base_filename}_angular_velocity_data.json": {
                    "graph_type": "Velocidad Angular",
                    "purpose": "Análisis de maniobras y giros del robot",
                    "key_data": [
                        "Serie temporal de velocidades angulares (rad/s y °/s)",
                        "Estadísticas de giros (media, máximo, mínimo)",
                        "Intensidad de maniobras durante la navegación"
                    ],
                    "use_case": "Evaluar agresividad de giros y estabilidad rotacional"
                },
                f"{base_filename}_cross_track_data.json": {
                    "graph_type": "Error Transversal (Cross-track)",
                    "purpose": "Análisis de desviaciones laterales del camino",
                    "key_data": [
                        "Serie temporal de errores transversales",
                        "Sesgo lateral (tendencia izquierda/derecha)",
                        "Interpretación de signos (+ = derecha, - = izquierda)",
                        "Bandas de confianza para desviaciones"
                    ],
                    "use_case": "Detectar problemas de calibración o tendencias sistemáticas de deriva"
                },
                f"{base_filename}_error_distribution_data.json": {
                    "graph_type": "Distribución de Errores",
                    "purpose": "Análisis estadístico de la calidad de seguimiento",
                    "key_data": [
                        "Histograma de errores de seguimiento",
                        "Percentiles de error (25°, 75°, 90°, 95°)",
                        "Distribución de densidad de probabilidad",
                        "Errores brutos para análisis detallado"
                    ],
                    "use_case": "Caracterizar la distribución de errores y identificar outliers"
                },
                f"{base_filename}_velocity_distribution_data.json": {
                    "graph_type": "Distribución de Velocidades",
                    "purpose": "Análisis estadístico del comportamiento dinámico",
                    "key_data": [
                        "Histograma de velocidades",
                        "Percentiles de velocidad",
                        "Media vs mediana (simetría de distribución)",
                        "Velocidades brutas para análisis detallado"
                    ],
                    "use_case": "Evaluar consistencia de velocidad y detectar patrones anómalos"
                }
            },
            "interpretation_guide": {
                "error_thresholds": {
                    "excellent": "< 0.5m error promedio",
                    "good": "0.5-1.0m error promedio", 
                    "regular": "1.0-2.0m error promedio",
                    "needs_improvement": "> 2.0m error promedio"
                },
                "velocity_analysis": {
                    "low_std": "Velocidad consistente (bueno)",
                    "high_std": "Velocidad errática (revisar control)",
                    "low_mean": "Robot lento (posible conservadurismo)",
                    "high_mean": "Robot rápido (posible agresividad)"
                },
                "cross_track_analysis": {
                    "zero_bias": "Sin deriva sistemática (bueno)",
                    "positive_bias": "Deriva hacia la derecha",
                    "negative_bias": "Deriva hacia la izquierda",
                    "high_std": "Mucha variabilidad lateral"
                }
            },
            "ai_analysis_prompts": {
                "overall_performance": "Analiza las métricas de error promedio, máximo y RMS para evaluar la calidad general del seguimiento de trayectoria",
                "temporal_patterns": "Examina las series temporales para identificar patrones, tendencias o momentos problemáticos específicos",
                "statistical_distributions": "Analiza los histogramas para caracterizar la naturaleza de los errores y velocidades (normal, sesgada, multimodal)",
                "cross_track_behavior": "Evalúa el comportamiento lateral para detectar problemas de calibración o algoritmos de control",
                "velocity_consistency": "Examina la consistencia de velocidades para evaluar la suavidad y eficiencia de la navegación"
            }
        }
        
        with open(summary_filename, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"📋 Resumen de datos generado: {summary_filename}")
        return summary_filename

    def plot_results(self, waypoints, filename='trayectoria_robot.png'):
        """Plot all trajectory results and generate comprehensive analysis.
        
        Args:
            waypoints: List of waypoints [[x, y, z], ...]
            filename: Base filename for outputs
        """
        # Generate all individual plots optimized for double column
        base_name = filename.replace('.png', '')
        
        print(f"\n📊 Generando análisis visual vectorizado para LaTeX doble columna...")
        
        # 1. Original single plot for backward compatibility (keep as PNG)
        self._plot_original_results(waypoints, filename)
        
        # 2. Individual vectorized plots for double column format
        self.plot_trajectory_comparison(waypoints, f'{base_name}_comparison.pdf')
        self.plot_tracking_errors(waypoints, f'{base_name}_tracking_errors.pdf')
        self.plot_velocity_profile(f'{base_name}_velocity_profile.pdf')
        self.plot_angular_velocity(f'{base_name}_angular_velocity.pdf')
        self.plot_cross_track_error(waypoints, f'{base_name}_cross_track.pdf')
        self.plot_error_distribution(waypoints, f'{base_name}_error_distribution.pdf')
        self.plot_velocity_distribution(f'{base_name}_velocity_distribution.pdf')
        
        # 3. Generate data summary for AI analysis
        self.generate_data_summary(base_name)
        
        # Print summary to console
        self._print_performance_summary(waypoints)
        
        print(f"\n✅ 7 gráficos vectorizados (PDF) + 7 archivos de datos (JSON) + 1 resumen generados")
        print(f"📋 Para LaTeX: \\includegraphics[width=\\columnwidth]{{archivo.pdf}}")
        print(f"🤖 Para análisis IA: Usar archivos *_data.json + resumen")
        
        # Show all plots
        plt.show()
    
    def _plot_original_results(self, waypoints, filename):
        """Original plotting method for backward compatibility."""
        plt.figure(figsize=(10, 8))
        
        # Create array of waypoints for plotting
        waypoints_x = [wp[0] for wp in waypoints]
        waypoints_y = [wp[1] for wp in waypoints]
        
        # Plot reference (waypoints) and trajectories
        plt.subplot(2, 1, 1)
        plt.plot(waypoints_x, waypoints_y, 'r--', linewidth=2, label='Reference (Waypoints)')
        plt.plot(waypoints_x, waypoints_y, 'ro', markersize=6)
        
        # Plot trajectories
        plt.plot(self.real_trajectory_x, self.real_trajectory_y, 'b-', linewidth=1.5, label='Robot Trajectory')
        
        # Plot GPS trajectory if different from real
        if self.real_trajectory_x != self.gps_trajectory_x or self.real_trajectory_y != self.gps_trajectory_y:
            plt.plot(self.gps_trajectory_x, self.gps_trajectory_y, 'g:', linewidth=1, label='GPS readings')
            
        # Plot trailer trajectory if available
        if self.has_trailer_data:
            plt.plot(self.trailer_trajectory_x, self.trailer_trajectory_y, 'm-', linewidth=1.5, label='Trailer Trajectory')
        
        # Mark starting position
        if self.real_trajectory_x:
            plt.plot(self.real_trajectory_x[0], self.real_trajectory_y[0], 'ko', markersize=8, label='Start')
        
        # Add waypoint numbers (reduced density)
        for i, wp in enumerate(waypoints):
            if i % 3 == 0 or i == len(waypoints) - 1:  # Every 3rd waypoint
                plt.text(wp[0], wp[1], f' WP{i+1}', fontsize=10)
        
        plt.title('Robot Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        
        # Plot X and Y positions vs time
        plt.subplot(2, 1, 2)
        
        # Plot position X vs time
        plt.plot(self.time_points, self.real_trajectory_x, 'b-', label='X Robot')
        plt.plot(self.time_points, self.real_trajectory_y, 'r-', label='Y Robot')
        
        # Plot trailer position if available
        if self.has_trailer_data:
            plt.plot(self.time_points[:len(self.trailer_trajectory_x)], self.trailer_trajectory_x, 'g-', label='X Trailer')
            plt.plot(self.time_points[:len(self.trailer_trajectory_y)], self.trailer_trajectory_y, 'm-', label='Y Trailer')
        
        # Mark waypoint changes (reduced density)
        for i, wt in enumerate(self.waypoint_times):
            if i > 0 and i % 2 == 0:  # Every other waypoint
                plt.axvline(x=wt, color='k', linestyle='--', alpha=0.5)
                plt.text(wt, max(max(self.real_trajectory_x), max(self.real_trajectory_y)), 
                         f' WP{i}', fontsize=9)
        
        plt.title('Position vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"📄 Gráfico original (PNG): {filename}")
    
    def _print_performance_summary(self, waypoints):
        """Print performance summary to console."""
        print("\n" + "="*60)
        print("📊 RESUMEN DE DESEMPEÑO")
        print("="*60)
        
        if not self.real_trajectory_x:
            print("❌ No hay datos de trayectoria para analizar")
            return
        
        # Calculate basic metrics
        error_metrics = self.calculate_tracking_errors(waypoints)
        
        total_distance = 0
        for i in range(1, len(self.real_trajectory_x)):
            dx = self.real_trajectory_x[i] - self.real_trajectory_x[i-1]
            dy = self.real_trajectory_y[i] - self.real_trajectory_y[i-1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        total_time = self.time_points[-1] if self.time_points else 0
        avg_velocity = np.mean(self.velocities) if self.velocities else 0
        max_velocity = np.max(self.velocities) if self.velocities else 0
        
        print(f"🚀 Distancia Total Recorrida: {total_distance:.2f} m")
        print(f"⏱️  Tiempo Total: {total_time:.1f} s")
        print(f"🏃 Velocidad Promedio: {avg_velocity:.2f} m/s")
        print(f"⚡ Velocidad Máxima: {max_velocity:.2f} m/s")
        
        if error_metrics:
            print(f"\n📐 ERRORES DE SEGUIMIENTO:")
            print(f"   📊 Error Promedio: {error_metrics['mean_error']:.3f} m")
            print(f"   📈 Error Máximo: {error_metrics['max_error']:.3f} m") 
            print(f"   📉 Error RMS: {error_metrics['rms_error']:.3f} m")
            print(f"   📏 Desviación Estándar: {error_metrics['std_error']:.3f} m")
            
            # Performance rating
            if error_metrics['mean_error'] < 0.5:
                print(f"   🎯 Calificación: EXCELENTE")
            elif error_metrics['mean_error'] < 1.0:
                print(f"   ✅ Calificación: BUENO")  
            elif error_metrics['mean_error'] < 2.0:
                print(f"   ⚠️  Calificación: REGULAR")
            else:
                print(f"   ❌ Calificación: NECESITA MEJORAS")
        
        print(f"\n🎯 Waypoints Programados: {len(waypoints)}")
        print(f"🏁 Waypoints Alcanzados: {len(self.waypoint_times) - 1}")
        
        if self.has_trailer_data:
            print(f"🚛 Datos de Trailer: Disponibles")
        
        print("="*60) 