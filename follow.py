import pygame
import math
import random

# ============================================================================
# PD CONTROLLER GAINS (EASILY CONFIGURABLE)
# ============================================================================
KP_DISTANCE = 0.8      # Proportional gain for distance error
KD_DISTANCE = 0.1      # Derivative gain for distance error
KP_ANGLE = 2.5         # Proportional gain for angle error
KD_ANGLE = 0.3         # Derivative gain for angle error

# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================
WINDOW_SIZE = 1200
FPS = 60
PIXELS_PER_METER = 100  # Scale: 100 pixels = 1 meter

# Robot constraints
MAX_LINEAR_VEL = 1.0    # m/s
MAX_ANGULAR_VEL = 2.0   # rad/s

# Leader parameters
LEADER_RADIUS = 0.15    # meters
LEADER_SPEED = 0.3      # m/s
LEADER_TURN_RATE = 0.5  # rad/s

# Robot parameters
ROBOT_WIDTH = 0.25      # meters
ROBOT_LENGTH = 0.35     # meters
TARGET_DISTANCE = 1.0   # meters (goal proximity)

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (50, 120, 200)
RED = (200, 50, 50)
GREEN = (50, 200, 50)
GRAY = (150, 150, 150)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))

# ============================================================================
# LEADER CLASS
# ============================================================================
class Leader:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.theta = random.uniform(0, 2 * math.pi)
        self.change_timer = 0
        self.change_interval = random.uniform(2, 5)
        
    def update(self, dt):
        # Random walk behavior
        self.change_timer += dt
        if self.change_timer > self.change_interval:
            self.theta += random.uniform(-1, 1)
            self.change_interval = random.uniform(2, 5)
            self.change_timer = 0
        
        # Move forward
        self.x += LEADER_SPEED * math.cos(self.theta) * dt
        self.y += LEADER_SPEED * math.sin(self.theta) * dt
        
        # Keep in bounds
        margin = 1.0
        self.x = clamp(self.x, margin, WINDOW_SIZE/PIXELS_PER_METER - margin)
        self.y = clamp(self.y, margin, WINDOW_SIZE/PIXELS_PER_METER - margin)
        
    def draw(self, screen):
        px = int(self.x * PIXELS_PER_METER)
        py = int(self.y * PIXELS_PER_METER)
        radius = int(LEADER_RADIUS * PIXELS_PER_METER)
        
        pygame.draw.circle(screen, BLUE, (px, py), radius)
        
        # Direction indicator
        end_x = px + int(radius * 1.5 * math.cos(self.theta))
        end_y = py + int(radius * 1.5 * math.sin(self.theta))
        pygame.draw.line(screen, WHITE, (px, py), (end_x, end_y), 2)

# ============================================================================
# ROBOT CLASS
# ============================================================================
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.theta = 0
        self.v = 0      # Linear velocity
        self.w = 0      # Angular velocity
        
        # For derivative control
        self.prev_dist_error = 0
        self.prev_angle_error = 0
        
    def update_kinematics(self, v_cmd, w_cmd, dt):
        """Update robot state using unicycle kinematics"""
        # Clamp commands to max velocities
        self.v = clamp(v_cmd, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.w = clamp(w_cmd, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        # Unicycle model
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        self.theta = normalize_angle(self.theta)
        
    def pd_control(self, leader, dt):
        """PD controller for following the leader"""
        # ====================================================================
        # CONTROL LOOP - PD Controller
        # ====================================================================
        
        # 1. Calculate target position (1m behind leader)
        target_x = leader.x - TARGET_DISTANCE * math.cos(leader.theta)
        target_y = leader.y - TARGET_DISTANCE * math.sin(leader.theta)
        
        # 2. Calculate errors
        dx = target_x - self.x
        dy = target_y - self.y
        distance_error = math.sqrt(dx**2 + dy**2)
        
        angle_to_target = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_target - self.theta)
        
        # Also align with leader's orientation
        heading_error = normalize_angle(leader.theta - self.theta)
        
        # 3. Calculate derivatives
        dist_error_derivative = (distance_error - self.prev_dist_error) / dt if dt > 0 else 0
        angle_error_derivative = (angle_error - self.prev_angle_error) / dt if dt > 0 else 0
        
        # 4. PD control laws
        v_cmd = KP_DISTANCE * distance_error + KD_DISTANCE * dist_error_derivative
        
        # Weight between moving toward target vs. aligning with leader
        if distance_error > 0.3:
            # Far away: prioritize moving toward target position
            w_cmd = KP_ANGLE * angle_error + KD_ANGLE * angle_error_derivative
        else:
            # Close enough: prioritize orientation alignment
            w_cmd = KP_ANGLE * heading_error + KD_ANGLE * angle_error_derivative * 0.5
        
        # 5. Update previous errors for next iteration
        self.prev_dist_error = distance_error
        self.prev_angle_error = angle_error
        
        return v_cmd, w_cmd
        
    def draw(self, screen):
        px = int(self.x * PIXELS_PER_METER)
        py = int(self.y * PIXELS_PER_METER)
        w = int(ROBOT_WIDTH * PIXELS_PER_METER)
        h = int(ROBOT_LENGTH * PIXELS_PER_METER)
        
        # Create rectangle points (local frame)
        points = [
            (-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)
        ]
        
        # Rotate and translate to world frame
        rotated_points = []
        for px_local, py_local in points:
            px_world = px + px_local * math.cos(self.theta) - py_local * math.sin(self.theta)
            py_world = py + px_local * math.sin(self.theta) + py_local * math.cos(self.theta)
            rotated_points.append((px_world, py_world))
        
        pygame.draw.polygon(screen, RED, rotated_points)
        
        # Direction indicator (front of robot)
        front_x = px + h/2 * math.cos(self.theta)
        front_y = py + h/2 * math.sin(self.theta)
        pygame.draw.line(screen, WHITE, (px, py), (front_x, front_y), 3)

# ============================================================================
# MAIN SIMULATION
# ============================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Leader-Follower Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)
    
    # Initialize agents
    leader = Leader(4.0, 4.0)
    robot = Robot(2.0, 2.0)
    
    # Control mode
    auto_mode = False
    
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    auto_mode = not auto_mode
        
        # Update leader
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_UP]:
            leader.x += LEADER_SPEED * 2 * math.cos(leader.theta) * dt
            leader.y += LEADER_SPEED * 2 * math.sin(leader.theta) * dt
        if keys[pygame.K_DOWN]:
            leader.x -= LEADER_SPEED * 2 * math.cos(leader.theta) * dt
            leader.y -= LEADER_SPEED * 2 * math.sin(leader.theta) * dt
        if keys[pygame.K_LEFT]:
            leader.theta += LEADER_TURN_RATE * 2 * dt
        if keys[pygame.K_RIGHT]:
            leader.theta -= LEADER_TURN_RATE * 2 * dt
        
        # Keep in bounds
        margin = 1.0
        leader.x = clamp(leader.x, margin, WINDOW_SIZE/PIXELS_PER_METER - margin)
        leader.y = clamp(leader.y, margin, WINDOW_SIZE/PIXELS_PER_METER - margin)
        
        # Control robot
        if auto_mode:
            # Automatic PD control
            v_cmd, w_cmd = robot.pd_control(leader, dt)
            robot.update_kinematics(v_cmd, w_cmd, dt)
        
        # Draw
        screen.fill(BLACK)
        
        # Draw target proximity circle
        target_x = int((leader.x - TARGET_DISTANCE * math.cos(leader.theta)) * PIXELS_PER_METER)
        target_y = int((leader.y - TARGET_DISTANCE * math.sin(leader.theta)) * PIXELS_PER_METER)
        pygame.draw.circle(screen, GREEN, (target_x, target_y), 5)
        pygame.draw.line(screen, GREEN, 
                        (int(leader.x * PIXELS_PER_METER), int(leader.y * PIXELS_PER_METER)),
                        (target_x, target_y), 1)
        
        leader.draw(screen)
        robot.draw(screen)
        
        # Draw UI
        mode_text = "PD CONTROL: ON" if auto_mode else "PD CONTROL: OFF"
        mode_color = GREEN if auto_mode else RED
        text = font.render(f"{mode_text} (SPACE to toggle)", True, mode_color)
        screen.blit(text, (10, 10))
        
        control_text = font.render("Arrow Keys: Control Leader (UP/DOWN=move, LEFT/RIGHT=turn)", True, WHITE)
        screen.blit(control_text, (10, 35))
        
        # Distance to target position
        target_x = leader.x - TARGET_DISTANCE * math.cos(leader.theta)
        target_y = leader.y - TARGET_DISTANCE * math.sin(leader.theta)
        dx = target_x - robot.x
        dy = target_y - robot.y
        dist = math.sqrt(dx**2 + dy**2)
        dist_text = font.render(f"Distance to target: {dist:.2f}m", True, WHITE)
        screen.blit(dist_text, (10, WINDOW_SIZE - 30))
        
        pygame.display.flip()
    
    pygame.quit()

if __name__ == "__main__":
    main()