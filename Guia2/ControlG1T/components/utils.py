import math

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def angle_to_target(current_pos, target_pos, robot_orientation):
    """Calculate angle to target from current position and orientation."""
    # Get current robot angle (yaw)
    current_angle = robot_orientation[2]
    
    # Calculate angle to target
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    target_angle = math.atan2(dy, dx)
    
    # Calculate angular difference
    angle_diff = target_angle - current_angle
    
    # Normalize to [-pi, pi]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
        
    return angle_diff 