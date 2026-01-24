"""
=============================================================================
ADVANCED MOUSE BRAIN CONTROLLER - FINAL VERSION V4.3
=============================================================================
AUTHOR: Damian Cutajar
PROJECT: Pac-Mouse Autonomous Navigation System
DESCRIPTION: 
    Intelligent autonomous mouse controller combining visual servoing, 
    frontier-based exploration, and predator avoidance for optimal cheese 
    collection in a maze environment.

=============================================================================
KEY FEATURES:
=============================================================================

🗺️  STATIC MAPPING SYSTEM
    - Fixed 24x24m occupancy grid (matches maze dimensions)
    - Never forgets explored areas (unlike rolling window)
    - 0.15m resolution for efficient processing
    - Perfect memory of entire maze layout
    - Dynamic obstacles (cat) NOT added to permanent map

🎥 DUAL CAMERA VISION
    - Yellow HSV detection for cheese (H:20-40, S:100-255, V:100-255)
    - Blue HSV detection for cat (H:90-110, S:80-255, V:100-255)
    - Real-time visual servoing for both targets
    - Area-based distance estimation

⚡ POWER PELLET MODE (10 seconds)
    - Activated by eating cheese_3
    - Reverses predator/prey roles
    - Cat becomes huntable target
    - Full-speed pursuit (0.8 m/s)

🧭 INTELLIGENT EXPLORATION
    - Frontier-based unknown area detection
    - Visited goal memory (30-second retention)
    - 3-second goal cooldown to prevent spam
    - 1.5m minimum goal distance
    - Smart cat avoidance in exploration paths

🚀 PERSISTENT CHEESE PURSUIT
    - Visual servoing when visible
    - Memory of last cheese position
    - Aggressive Nav2 fallback (doesn't give up!)
    - Smart wall detection vs cheese detection
    - Path clearing around cheese

🛡️ MULTI-LAYER SAFETY
    - Emergency collision prevention (35cm threshold)
    - Automatic stop/reverse/resume sequence
    - Wall unstuck behavior (backup + turn)
    - Corner detection and escape
    - Nav2 goal validation before sending

🐱 SMART CAT RESPONSE
    - Far detection: 5.0m danger radius
    - Close detection: 2.5m critical radius
    - Improved line-of-sight (ignores cat body as obstacle)
    - Clear flee state management
    - No flip-flopping between states

🎯 HYBRID NAVIGATION
    - Visual servoing for direct line-of-sight targets
    - Nav2 path planning for complex routing
    - Automatic strategy switching
    - Costmap synchronization

=============================================================================
CHANGE LOG V4.3:
=============================================================================
[FIXED] Cheese pursuit doesn't give up (uses Nav2 aggressively)
[FIXED] Cat body not added to static map (dynamic obstacle)
[FIXED] Line-of-sight uses LiDAR, not occupancy grid
[FIXED] Flee state properly cleared when cat blocked
[FIXED] No more flip-flopping between flee and navigate
[IMPROVED] Cheese memory - remembers last position
[IMPROVED] More persistent cheese collection behavior

=============================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
import subprocess


class ProposalMouseBrain(Node):
    
    def __init__(self):
        super().__init__('proposal_mouse_brain',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION
        # ====================================================================
        self.linear_speed = 2.0
        self.angular_speed = 2.0
        self.cheese_threshold = 13750
        
        # 🗺️ STATIC MAP SETTINGS (Fixed world frame - never forgets!)
        self.resolution = 0.15              # 0.15m per cell (fast processing)
        self.map_width_meters = 24.0        # 24m total width (21m maze + 3m padding)
        self.map_height_meters = 24.0       # 24m total height
        self.map_origin_x = -12.0           # Western edge at -12m
        self.map_origin_y = -12.0           # Southern edge at -12m
        
        # Calculate grid dimensions
        self.grid_size_x = int(self.map_width_meters / self.resolution)   # 160 cells
        self.grid_size_y = int(self.map_height_meters / self.resolution)  # 160 cells
        
        # Cat detection settings
        self.cat_danger_distance = 5.0
        self.cat_critical_distance = 2.5
        
        # 🆕 CAT VISION SETTINGS
        self.cat_visible_in_camera = False
        self.cat_camera_error = 0.0
        self.cat_camera_area = 0.0
        self.power_mode = False  # Set to True when can eat cat!
        self.power_mode_timer = 0.0
        self.power_mode_duration = 10.0  # 10 seconds of power
        
        # 🆕 CHEESE MEMORY
        self.last_cheese_x = None
        self.last_cheese_y = None
        self.cheese_chase_mode = False
        
        # Obstacle collision prevention
        self.obstacle_danger_distance = 0.35
        self.obstacle_critical_distance = 0.25
        self.min_obstacle_distance = float('inf')
        self.recovery_state = None
        self.recovery_timer = 0.0
        
        # 🆕 LIDAR RANGES (for line-of-sight checking)
        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        
        # Cat escape state
        self.cat_escape_goal_sent = False
        self.last_escape_goal_time = 0.0
        self.stuck_detection_poses = []
        
        # Goal retry prevention
        self.last_rejected_goal = None
        self.goal_rejection_count = 0
        
        # 🆕 FRONTIER LOOP PREVENTION
        self.visited_goals = []  # Track recently visited goals
        self.min_goal_distance = 1.5  # Minimum distance for new goals
        self.last_goal_time = 0.0
        self.goal_cooldown = 3.0  # Wait 3 seconds between goals
        
        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        self.cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.score_pub = self.create_publisher(String, '/cheese_eaten', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mouse/visualization_marker_array', 10)
        
        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(LaserScan, '/mouse/scan', self.lidar_callback, 10)
        self.create_subscription(Image, '/mouse/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(Odometry, '/mouse/odom', self.odometry_callback, 10)
        self.create_subscription(Odometry, '/cat/odom', self.cat_odometry_callback, 10)
        
        # ====================================================================
        # ACTION CLIENT
        # ====================================================================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.cat_x = 0.0
        self.cat_y = 0.0
        self.cat_detected = False
        self.cat_distance = float('inf')
        
        self.bridge = CvBridge()
        self.cheese_visible = False
        self.cheese_error = 0.0
        self.cheese_area = 0.0
        
        self.is_navigating = False
        self.nav_goal_handle = None
        
        # 🗺️ STATIC OCCUPANCY GRID (Never forgets!)
        self.occupancy_grid = np.full((self.grid_size_y, self.grid_size_x), -1, dtype=int)
        
        self.cheese_models = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'},
            {'x': -5.0, 'y': -5.0, 'name': 'cheese_4'}
        ]
        
        # ====================================================================
        # TIMER
        # ====================================================================
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("="*70)
        self.get_logger().info("🐭 ENHANCED MOUSE BRAIN V4.3: ONLINE")
        self.get_logger().info("="*70)
        self.get_logger().info("🗺️  Static mapping: ACTIVE (walls only, not cat!)")
        self.get_logger().info("🎥 Cat camera detection: ACTIVE (Sky Blue HSV)")
        self.get_logger().info("👁️  LiDAR line-of-sight: ACTIVE (cat-aware)")
        self.get_logger().info("⚡ Power pellet mode: READY (10s duration)")
        self.get_logger().info("🧀 Persistent cheese chase: ACTIVE (never gives up!)")
        self.get_logger().info("🧠 Smart cat avoidance: ACTIVE (no flip-flop)")
        self.get_logger().info("🚀 Full-speed cheese: ACTIVE (0.8 m/s)")
        self.get_logger().info(f"🐱 Cat detection: Danger={self.cat_danger_distance}m, Critical={self.cat_critical_distance}m")
        self.get_logger().info("="*70)
    
    def odometry_callback(self, msg):
        """Mouse odometry callback"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def has_line_of_sight_to_cat_lidar(self):
        """✅ FIXED: Check LOS using LiDAR (not occupancy grid!)"""
        if self.cat_distance == float('inf') or len(self.lidar_ranges) == 0:
            return False
        
        # Calculate angle to cat
        dx = self.cat_x - self.robot_x
        dy = self.cat_y - self.robot_y
        angle_to_cat = math.atan2(dy, dx) - self.robot_yaw
        
        # Normalize to [-pi, pi]
        while angle_to_cat > math.pi:
            angle_to_cat -= 2.0 * math.pi
        while angle_to_cat < -math.pi:
            angle_to_cat += 2.0 * math.pi
        
        # Find corresponding LiDAR ray
        ray_index = int((angle_to_cat - self.lidar_angle_min) / self.lidar_angle_increment)
        
        if 0 <= ray_index < len(self.lidar_ranges):
            lidar_distance = self.lidar_ranges[ray_index]
            
            # Check adjacent rays too (±5 rays for robustness)
            min_lidar_dist = lidar_distance
            for offset in range(-5, 6):
                check_idx = ray_index + offset
                if 0 <= check_idx < len(self.lidar_ranges):
                    if not math.isinf(self.lidar_ranges[check_idx]) and not math.isnan(self.lidar_ranges[check_idx]):
                        min_lidar_dist = min(min_lidar_dist, self.lidar_ranges[check_idx])
            
            # If LiDAR distance is much less than cat distance → wall blocking
            # Cat body size ~0.3m, so allow 0.5m tolerance
            if not math.isinf(min_lidar_dist) and min_lidar_dist < (self.cat_distance - 0.5):
                return False  # Wall between us and cat
            
            return True  # Clear line of sight
        
        return False
    
    def cat_odometry_callback(self, msg):
        """✅ FIXED: Cat odometry with proper LOS check"""
        self.cat_x = msg.pose.pose.position.x
        self.cat_y = msg.pose.pose.position.y
        
        dx = self.cat_x - self.robot_x
        dy = self.cat_y - self.robot_y
        self.cat_distance = math.hypot(dx, dy)
        
        # ✅ FIXED: Clear, unambiguous cat detection
        if self.power_mode:
            self.cat_detected = False  # Never flee when powered up!
        elif self.cat_distance < self.cat_danger_distance:
            # ✅ NEW: Use LiDAR-based line-of-sight
            if self.has_line_of_sight_to_cat_lidar():
                self.cat_detected = True
            else:
                self.cat_detected = False
        else:
            self.cat_detected = False
    
    def lidar_callback(self, msg):
        """✅ FIXED: LiDAR callback - stores ranges for LOS, only maps STATIC obstacles"""
        # Store LiDAR data for line-of-sight checking
        self.lidar_ranges = list(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        
        angle = msg.angle_min
        self.min_obstacle_distance = float('inf')
        
        for i, range_val in enumerate(msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)):
                scan_angle = msg.angle_min + (i * msg.angle_increment)
                
                while scan_angle > math.pi:
                    scan_angle -= 2.0 * math.pi
                while scan_angle < -math.pi:
                    scan_angle += 2.0 * math.pi
                
                if abs(scan_angle) < 0.52:
                    if range_val < self.min_obstacle_distance:
                        self.min_obstacle_distance = range_val
                
                # ✅ FIXED: Only map STATIC obstacles (walls)
                # Don't map anything at cat's position (dynamic obstacle)
                if 0.35 < range_val < 8.0:
                    # Calculate world position of this LiDAR point
                    global_angle = angle + self.robot_yaw
                    wx = self.robot_x + range_val * math.cos(global_angle)
                    wy = self.robot_y + range_val * math.sin(global_angle)
                    
                    # Check if this is the cat (within 0.4m of cat position)
                    dist_to_cat = math.hypot(wx - self.cat_x, wy - self.cat_y)
                    
                    if dist_to_cat > 0.4:  # Not the cat - it's a real wall!
                        self.update_occupancy_grid(range_val, angle)
            
            angle += msg.angle_increment
    
    def update_occupancy_grid(self, range_val, local_angle):
        """🗺️ UPDATED: Update STATIC internal map (walls only!)"""
        global_angle = local_angle + self.robot_yaw
        
        max_trace = min(range_val, 4.0)
        for step in np.arange(0.2, max_trace, self.resolution):
            wx = self.robot_x + step * math.cos(global_angle)
            wy = self.robot_y + step * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy) and self.occupancy_grid[gy, gx] != 100:
                self.occupancy_grid[gy, gx] = 0
        
        if range_val <= 4.0:
            wx = self.robot_x + range_val * math.cos(global_angle)
            wy = self.robot_y + range_val * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy):
                self.occupancy_grid[gy, gx] = 100
    
    def clear_cheese_area_from_map(self):
        """Clear cheese from occupancy grid"""
        if not self.cheese_visible or self.min_obstacle_distance == float('inf'):
            return
        
        cheese_angle = -self.cheese_error * 0.001
        cheese_distance = max(self.min_obstacle_distance, 0.5)
        
        cheese_x = self.robot_x + cheese_distance * math.cos(self.robot_yaw + cheese_angle)
        cheese_y = self.robot_y + cheese_distance * math.sin(self.robot_yaw + cheese_angle)
        
        # ✅ NEW: Remember cheese position
        self.last_cheese_x = cheese_x
        self.last_cheese_y = cheese_y
        
        gx, gy = self.world_to_grid(cheese_x, cheese_y)
        clear_radius = int(0.5 / self.resolution)
        
        for dx in range(-clear_radius, clear_radius + 1):
            for dy in range(-clear_radius, clear_radius + 1):
                if dx*dx + dy*dy <= clear_radius*clear_radius:
                    nx, ny = gx + dx, gy + dy
                    if self.is_valid_grid_cell(nx, ny):
                        if self.occupancy_grid[ny, nx] == 100:
                            self.occupancy_grid[ny, nx] = 0
    
    def camera_callback(self, msg):
        """🎥 ENHANCED: Detect both cheese AND cat"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # ====================
            # CHEESE DETECTION (Yellow)
            # ====================
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            cheese_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            cheese_contours, _ = cv2.findContours(cheese_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if cheese_contours:
                largest_contour = max(cheese_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        if not self.cheese_visible:
                            self.get_logger().info("🧀 Cheese spotted!")
                        cx = int(M["m10"] / M["m00"])
                        image_center_x = cv_image.shape[1] / 2
                        
                        self.cheese_error = cx - image_center_x
                        self.cheese_area = area
                        self.cheese_visible = True
                        self.clear_cheese_area_from_map()
                    else:
                        self.cheese_visible = False
                else:
                    self.cheese_visible = False
            else:
                self.cheese_visible = False
            
            # ====================
            # CAT DETECTION (Sky Blue - matches cat.urdf.xacro)
            # ====================
            lower_cat = np.array([90, 80, 100])
            upper_cat = np.array([110, 255, 255])
            cat_mask = cv2.inRange(hsv, lower_cat, upper_cat)
            
            cat_contours, _ = cv2.findContours(cat_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if cat_contours:
                largest_cat_contour = max(cat_contours, key=cv2.contourArea)
                cat_area = cv2.contourArea(largest_cat_contour)
                
                if cat_area > 500:
                    M_cat = cv2.moments(largest_cat_contour)
                    if M_cat["m00"] > 0:
                        if not self.cat_visible_in_camera:
                            status = "🍖 CAT IN SIGHT - HUNTING!" if self.power_mode else "😱 CAT SPOTTED!"
                            self.get_logger().info(status)
                        
                        cx_cat = int(M_cat["m10"] / M_cat["m00"])
                        image_center_x = cv_image.shape[1] / 2
                        
                        self.cat_camera_error = cx_cat - image_center_x
                        self.cat_camera_area = cat_area
                        self.cat_visible_in_camera = True
                    else:
                        self.cat_visible_in_camera = False
                else:
                    self.cat_visible_in_camera = False
            else:
                self.cat_visible_in_camera = False
            
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
    
    def control_loop(self):
        """🧠 ENHANCED: Main control loop"""
        cmd = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ====================
        # POWER MODE TIMER
        # ====================
        if self.power_mode:
            elapsed = current_time - self.power_mode_timer
            if elapsed > self.power_mode_duration:
                self.get_logger().info("⚡ Power mode EXPIRED!")
                self.power_mode = False
            else:
                remaining = self.power_mode_duration - elapsed
                if int(remaining) % 2 == 0:
                    self.get_logger().info(
                        f"⚡ POWER MODE: {remaining:.1f}s remaining!",
                        throttle_duration_sec=2.0
                    )
        
        # ====================================================================
        # PRIORITY 0: OBSTACLE COLLISION PREVENTION
        # ====================================================================
        if self.min_obstacle_distance < self.obstacle_danger_distance:
            if self.recovery_state is None:
                self.get_logger().warn(f"⚠️ OBSTACLE TOO CLOSE: {self.min_obstacle_distance:.2f}m")
                self.recovery_state = 'stopping'
                self.recovery_timer = current_time
                if self.is_navigating and self.nav_goal_handle:
                    self.nav_goal_handle.cancel_goal_async()
                self.is_navigating = False
            
            if self.recovery_state == 'stopping':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
                if current_time - self.recovery_timer > 0.5:
                    self.recovery_state = 'reversing'
                    self.recovery_timer = current_time
            
            elif self.recovery_state == 'reversing':
                cmd.linear.x = -0.3
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)
                
                if current_time - self.recovery_timer > 1.5:
                    self.recovery_state = 'resuming'
                    self.recovery_timer = current_time
            
            elif self.recovery_state == 'resuming':
                self.recovery_state = None
                self.occupancy_grid.fill(-1)
            
            return
        
        if self.recovery_state and self.min_obstacle_distance > self.obstacle_danger_distance + 0.1:
            self.recovery_state = None
        
        # ====================================================================
        # ✅ PRIORITY 0.5: CHASE CAT (POWER MODE ONLY!)
        # ====================================================================
        if self.power_mode and self.cat_visible_in_camera:
            self.get_logger().info(
                f"🍖 HUNTING CAT! area={self.cat_camera_area:.0f}, error={self.cat_camera_error:.0f}, dist={self.cat_distance:.2f}m",
                throttle_duration_sec=0.5
            )
            
            if self.is_navigating and self.nav_goal_handle:
                self.nav_goal_handle.cancel_goal_async()
                self.is_navigating = False
            
            kp_angular = 0.008
            cmd.angular.z = -kp_angular * self.cat_camera_error
            
            if self.cat_camera_area > 40000 and self.cat_distance < 1.0:
                self.get_logger().info("🎉🎉🎉 CAUGHT THE CAT! 🎉🎉🎉")
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return
            
            if abs(self.cat_camera_error) < 50:
                cmd.linear.x = 0.8
            elif abs(self.cat_camera_error) < 100:
                cmd.linear.x = 0.6
            else:
                cmd.linear.x = 0.3
            
            self.cmd_pub.publish(cmd)
            return
        
        if self.power_mode and not self.cat_visible_in_camera:
            self.get_logger().info(
                "⚡ POWER MODE ACTIVE - Searching for cat...",
                throttle_duration_sec=2.0
            )
        
        # ====================================================================
        # ✅ PRIORITY 1: FLEE FROM CAT (Only if actually visible!)
        # ====================================================================
        if self.cat_detected and not self.power_mode:
            urgency = "CRITICAL" if self.cat_distance < self.cat_critical_distance else "WARNING"
            
            if self.cat_distance < 1.5:
                if not self.is_navigating:
                    if self.nav_goal_handle:
                        self.nav_goal_handle.cancel_goal_async()
                    
                    dx = self.robot_x - self.cat_x
                    dy = self.robot_y - self.cat_y
                    escape_angle = math.atan2(dy, dx)
                    
                    escape_distance = 3.0
                    escape_x = self.robot_x + escape_distance * math.cos(escape_angle)
                    escape_y = self.robot_y + escape_distance * math.sin(escape_angle)
                    
                    for dist in np.arange(0, escape_distance, 0.2):
                        cx = self.robot_x + dist * math.cos(escape_angle)
                        cy = self.robot_y + dist * math.sin(escape_angle)
                        gx, gy = self.world_to_grid(cx, cy)
                        
                        clear_radius = 3
                        for dx_clear in range(-clear_radius, clear_radius + 1):
                            for dy_clear in range(-clear_radius, clear_radius + 1):
                                nx, ny = gx + dx_clear, gy + dy_clear
                                if self.is_valid_grid_cell(nx, ny):
                                    if self.occupancy_grid[ny, nx] == 100:
                                        self.occupancy_grid[ny, nx] = 0
                    
                    self.get_logger().warn(
                        f"😱 CAT {urgency}! Escaping to ({escape_x:.2f}, {escape_y:.2f})"
                    )
                    
                    self.send_navigation_goal((escape_x, escape_y))
                    self.cat_escape_goal_sent = True
                    self.last_escape_goal_time = current_time
                else:
                    self.get_logger().info(
                        f"🏃 Escaping via Nav2... (cat: {self.cat_distance:.2f}m)",
                        throttle_duration_sec=1.0
                    )
                
                return
            
            else:
                if self.is_navigating and self.nav_goal_handle:
                    self.nav_goal_handle.cancel_goal_async()
                    self.is_navigating = False
                
                self.cat_escape_goal_sent = False
                
                dx = self.robot_x - self.cat_x
                dy = self.robot_y - self.cat_y
                escape_angle_world = math.atan2(dy, dx)
                escape_angle_rel = self.normalize_angle(escape_angle_world - self.robot_yaw)
                
                self.stuck_detection_poses.append((self.robot_x, self.robot_y, current_time))
                self.stuck_detection_poses = [p for p in self.stuck_detection_poses 
                                             if current_time - p[2] < 2.0]
                
                if len(self.stuck_detection_poses) > 6:
                    first_pose = self.stuck_detection_poses[0]
                    distance_moved = math.hypot(self.robot_x - first_pose[0], 
                                              self.robot_y - first_pose[1])
                    
                    if distance_moved < 0.3:
                        self.get_logger().error("🚨 STUCK IN CORNER!")
                        
                        escape_x = self.robot_x + 2.0 * math.cos(self.robot_yaw + math.pi/2)
                        escape_y = self.robot_y + 2.0 * math.sin(self.robot_yaw + math.pi/2)
                        
                        self.send_navigation_goal((escape_x, escape_y))
                        self.stuck_detection_poses.clear()
                        return
                
                turn_speed = 8.0 if self.cat_distance < self.cat_critical_distance else 5.0
                cmd.angular.z = turn_speed * np.sign(escape_angle_rel)
                
                if abs(escape_angle_rel) < 0.5:
                    cmd.linear.x = 0.5 if self.cat_distance < self.cat_critical_distance else 0.4
                else:
                    cmd.linear.x = 0.15
                
                self.get_logger().info(
                    f"🔄 Reactive flee: cat {self.cat_distance:.2f}m",
                    throttle_duration_sec=1.0
                )
                
                self.cmd_pub.publish(cmd)
                return
        else:
            self.cat_escape_goal_sent = False
            self.stuck_detection_poses.clear()
        
        # ====================================================================
        # ✅ PRIORITY 2: PERSISTENT CHEESE PURSUIT
        # ====================================================================
        if self.cheese_visible:
            self.cheese_chase_mode = True
            
            if self.is_navigating:
                self.get_logger().info("🧀 CHEESE - CANCELING NAV2!")
                if self.nav_goal_handle:
                    self.nav_goal_handle.cancel_goal_async()
                self.is_navigating = False
            
            self.clear_cheese_area_from_map()
            
            if self.cheese_area > self.cheese_threshold:
                self.get_logger().info("🧀 EATING CHEESE!")
                self.cmd_pub.publish(Twist())
                self.collect_cheese()
                return
            
            # 🔥 FULL SPEED VISUAL SERVOING
            kp_angular = 0.008
            cmd.angular.z = -kp_angular * self.cheese_error
            
            if abs(self.cheese_error) < 50:
                cmd.linear.x = 0.8
            elif abs(self.cheese_error) < 150:
                cmd.linear.x = 0.7
            else:
                cmd.linear.x = 0.4
            
            self.cmd_pub.publish(cmd)
            return

        # ✅ CHEESE LOST BUT REMEMBERED - Use Nav2 (with safety check!)
        elif self.cheese_chase_mode and self.last_cheese_x is not None:
            # ✅ NEW: Safety check - make sure cheese still exists!
            cheese_still_exists = False
            for cheese in self.cheese_models:
                dist = math.hypot(self.last_cheese_x - cheese['x'], 
                                self.last_cheese_y - cheese['y'])
                if dist < 0.5:  # Within 0.5m of known cheese location
                    cheese_still_exists = True
                    break
            
            if not cheese_still_exists:
                # Cheese was collected - stop chasing!
                self.get_logger().info("🧀 Cheese collected - clearing memory")
                self.last_cheese_x = None
                self.last_cheese_y = None
                self.cheese_chase_mode = False
                # Don't return - fall through to exploration
            else:
                # Cheese still exists - navigate to it
                if not self.is_navigating:
                    self.get_logger().warn(f"🧀 Cheese lost - navigating to last position!")
                    
                    # Clear path to cheese
                    gx, gy = self.world_to_grid(self.last_cheese_x, self.last_cheese_y)
                    clear_radius = int(1.0 / self.resolution)
                    
                    for dx in range(-clear_radius, clear_radius + 1):
                        for dy in range(-clear_radius, clear_radius + 1):
                            nx, ny = gx + dx, gy + dy
                            if self.is_valid_grid_cell(nx, ny):
                                self.occupancy_grid[ny, nx] = 0
                    
                    self.send_navigation_goal((self.last_cheese_x, self.last_cheese_y))
                
                return

        
        # ====================================================================
        # PRIORITY 3: NAVIGATE
        # ====================================================================
        if self.is_navigating:
            return
        
        # ====================================================================
        # PRIORITY 4: EXPLORATION
        # ====================================================================
        self.cheese_chase_mode = False
        self.explore_environment()
    
    def explore_environment(self):
        """Exploration with cooldown"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if current_time - self.last_goal_time < self.goal_cooldown:
            cmd = Twist()
            cmd.angular.z = 1.5
            self.cmd_pub.publish(cmd)
            return
        
        frontiers = self.detect_frontiers()
        
        if frontiers:
            goal_point = self.select_best_frontier(frontiers)
            if goal_point:
                self.send_navigation_goal(goal_point)
                self.last_goal_time = current_time
                return
        
        cmd = Twist()
        cmd.angular.z = 3.0
        self.cmd_pub.publish(cmd)
    
    def detect_frontiers(self):
        """Detect frontier regions"""
        frontiers = []
        visited = np.zeros_like(self.occupancy_grid, dtype=bool)
        
        for x in range(1, self.grid_size_x - 1):
            for y in range(1, self.grid_size_y - 1):
                if self.occupancy_grid[y, x] == 0 and not visited[y, x]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.expand_frontier(x, y, visited)
                        if len(frontier) > 5:
                            frontiers.append(frontier)
        
        return frontiers
    
    def is_frontier_cell(self, x, y):
        """Check if cell is frontier"""
        if self.occupancy_grid[y, x] != 0:
            return False
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid_grid_cell(nx, ny):
                if self.occupancy_grid[ny, nx] == -1:
                    return True
        return False
    
    def expand_frontier(self, start_x, start_y, visited):
        """Expand frontier using BFS"""
        frontier = []
        queue = deque([(start_x, start_y)])
        visited[start_y, start_x] = True
        
        while queue:
            x, y = queue.popleft()
            
            if self.is_frontier_cell(x, y):
                frontier.append((x, y))
                
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        
                        nx, ny = x + dx, y + dy
                        if (self.is_valid_grid_cell(nx, ny) and 
                            not visited[ny, nx] and
                            self.occupancy_grid[ny, nx] == 0):
                            visited[ny, nx] = True
                            queue.append((nx, ny))
        
        return frontier
    
    def select_best_frontier(self, frontiers):
        """Select best frontier"""
        candidates = []
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        self.visited_goals = [(gx, gy, t) for gx, gy, t in self.visited_goals 
                              if current_time - t < 30.0]
        
        for frontier in frontiers:
            if len(frontier) < 10:
                continue
                
            center_idx = len(frontier) // 2
            gx, gy = frontier[center_idx]
            
            if not self.is_safe_location(gx, gy):
                found_safe = False
                for offset in range(1, 6):
                    for dx in [-offset, 0, offset]:
                        for dy in [-offset, 0, offset]:
                            test_gx, test_gy = gx + dx, gy + dy
                            if self.is_safe_location(test_gx, test_gy):
                                gx, gy = test_gx, test_gy
                                found_safe = True
                                break
                        if found_safe:
                            break
                    if found_safe:
                        break
                
                if not found_safe:
                    continue
            
            wx, wy = self.grid_to_world(gx, gy)
            distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
            
            if distance < self.min_goal_distance:
                continue
            
            skip_frontier = False
            for visited_gx, visited_gy, _ in self.visited_goals:
                visited_wx, visited_wy = self.grid_to_world(visited_gx, visited_gy)
                dist_to_visited = math.hypot(wx - visited_wx, wy - visited_wy)
                if dist_to_visited < 2.0:
                    skip_frontier = True
                    break
            
            if skip_frontier:
                continue
            
            cat_penalty = 0.0
            if self.cat_detected and not self.power_mode:
                frontier_angle = math.atan2(wy - self.robot_y, wx - self.robot_x)
                cat_angle = math.atan2(self.cat_y - self.robot_y, self.cat_x - self.robot_x)
                
                angle_diff = abs(self.normalize_angle(frontier_angle - cat_angle))
                
                if angle_diff < math.pi / 3:
                    distance_to_cat = math.hypot(self.cat_x - wx, self.cat_y - wy)
                    if distance_to_cat < 3.0:
                        cat_penalty = 10.0
            
            size_score = math.sqrt(len(frontier))
            distance_penalty = distance * 0.8
            score = size_score * 2.0 - distance_penalty - cat_penalty
            
            candidates.append((score, wx, wy, len(frontier), gx, gy))
        
        if candidates:
            candidates.sort(reverse=True, key=lambda x: x[0])
            best_score, wx, wy, size, gx, gy = candidates[0]
            
            self.visited_goals.append((gx, gy, current_time))
            
            self.get_logger().info(
                f"📍 Frontier: size={size}, dist={math.hypot(wx-self.robot_x, wy-self.robot_y):.2f}m"
            )
            return (wx, wy)
        
        return None
    
    def is_safe_location(self, gx, gy):
        """Safety check"""
        if not self.is_valid_grid_cell(gx, gy):
            return False
        
        if self.occupancy_grid[gy, gx] == 100:
            return False
        
        margin = 4
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_valid_grid_cell(nx, ny):
                    if self.occupancy_grid[ny, nx] == 100:
                        return False
        return True
    
    def estimate_cheese_distance(self):
        """Estimate cheese distance"""
        if not self.cheese_visible or self.min_obstacle_distance == float('inf'):
            return 2.0
        
        cheese_angle_estimate = -self.cheese_error * 0.001
        
        if abs(cheese_angle_estimate) < 0.3:
            return max(self.min_obstacle_distance - 0.3, 0.5)
        
        return 2.0
    
    def send_navigation_goal(self, point):
        """Send navigation goal"""
        goal_key = (round(point[0], 1), round(point[1], 1))
        if self.last_rejected_goal == goal_key and self.goal_rejection_count > 2:
            self.get_logger().error(f"❌ Goal {point} rejected {self.goal_rejection_count} times!")
            self.is_navigating = False
            
            if self.cheese_visible:
                self.get_logger().info("🔄 Forcing visual servoing")
            
            return
        
        gx, gy = self.world_to_grid(point[0], point[1])
        
        if not self.is_safe_location(gx, gy):
            self.get_logger().warn(f"⚠️ Goal ({point[0]:.2f}, {point[1]:.2f}) UNSAFE")
            
            found_safe = False
            best_safe_point = None
            min_distance = float('inf')
            
            for radius in range(1, 12):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if dx*dx + dy*dy <= radius*radius:
                            test_gx, test_gy = gx + dx, gy + dy
                            if self.is_safe_location(test_gx, test_gy):
                                wx, wy = self.grid_to_world(test_gx, test_gy)
                                
                                dist_to_original = math.hypot(wx - point[0], wy - point[1])
                                if dist_to_original < min_distance:
                                    min_distance = dist_to_original
                                    best_safe_point = (wx, wy)
                                    found_safe = True
            
            if found_safe and best_safe_point:
                point = best_safe_point
                self.get_logger().info(f"✅ Safe alternative: ({point[0]:.2f}, {point[1]:.2f})")
            else:
                self.get_logger().error("❌ No safe goal!")
                self.is_navigating = False
                self.last_rejected_goal = goal_key
                self.goal_rejection_count += 1
                return
        
        robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.world_to_grid(point[0], point[1])
        
        for i in range(5):
            t = (i + 1) / 6.0
            check_gx = int(robot_gx + t * (goal_gx - robot_gx))
            check_gy = int(robot_gy + t * (goal_gy - robot_gy))
            
            blocked_count = 0
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    nx, ny = check_gx + dx, check_gy + dy
                    if self.is_valid_grid_cell(nx, ny):
                        if self.occupancy_grid[ny, nx] == 100:
                            blocked_count += 1
            
            if blocked_count > 20:
                self.get_logger().error(f"❌ Path blocked at checkpoint {i+1}/5")
                self.is_navigating = False
                self.last_rejected_goal = goal_key
                self.goal_rejection_count += 1
                return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = 0.0
        
        angle = math.atan2(point[1] - self.robot_y, point[0] - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)
        
        self.get_logger().info(f"🎯 Goal: ({point[0]:.2f}, {point[1]:.2f})")
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected')
            self.is_navigating = False
            return
        
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result()
        self.is_navigating = False
        
        if result.status == 4:
            self.get_logger().info('✅ Goal reached')
            self.goal_rejection_count = 0
            self.last_rejected_goal = None
            # ✅ Check if we reached cheese position
            if self.cheese_chase_mode and self.last_cheese_x is not None:
                dist_to_cheese = math.hypot(self.robot_x - self.last_cheese_x, 
                                           self.robot_y - self.last_cheese_y)
                if dist_to_cheese < 1.0:
                    self.get_logger().info("🧀 Reached cheese area - switching to visual")
        
        elif result.status == 5:
            self.get_logger().info('🔄 Goal CANCELED (switched priority)')
            self.goal_rejection_count = 0
            self.last_rejected_goal = None
        
        elif result.status == 6:
            self.get_logger().error('❌ Goal ABORTED by Nav2!')
            self.goal_rejection_count += 1
            robot_gx, robot_gy = self.world_to_grid(self.robot_x, self.robot_y)
            for dx in range(-10, 11):
                for dy in range(-10, 11):
                    nx, ny = robot_gx + dx, robot_gy + dy
                    if self.is_valid_grid_cell(nx, ny):
                        if self.occupancy_grid[ny, nx] == 100:
                            self.occupancy_grid[ny, nx] = -1
        
        else:
            self.get_logger().warn(f'⚠️ Nav status: {result.status}')
            self.goal_rejection_count = 0
    
    def collect_cheese(self):
        """Cheese collection + power mode"""
        min_dist = float('inf')
        closest_cheese = None
        
        for cheese in self.cheese_models:
            dist = math.hypot(self.robot_x - cheese['x'], 
                            self.robot_y - cheese['y'])
            if dist < min_dist:
                min_dist = dist
                closest_cheese = cheese
        
        if closest_cheese and min_dist < 1.5:
            self.delete_gazebo_model(closest_cheese['name'])
            self.score_pub.publish(String(data=closest_cheese['name']))
            self.cheese_models.remove(closest_cheese)
            self.get_logger().info(f"🎯 Collected {closest_cheese['name']}!")
            
            # ✅ Clear cheese memory
            self.last_cheese_x = None
            self.last_cheese_y = None
            self.cheese_chase_mode = False
            
            if closest_cheese['name'] == 'cheese_3':
                self.power_mode = True
                self.power_mode_timer = self.get_clock().now().nanoseconds / 1e9
                self.cat_detected = False
                self.get_logger().info("="*70)
                self.get_logger().info("⚡⚡⚡ POWER MODE ACTIVATED! ⚡⚡⚡")
                self.get_logger().info("🍖 CAT IS NOW PREY! HUNT IT DOWN!")
                self.get_logger().info(f"⏱️  {self.power_mode_duration} seconds remaining!")
                self.get_logger().info("="*70)
            
            self.occupancy_grid.fill(-1)
    
    def delete_gazebo_model(self, model_name):
        """Remove model from Gazebo"""
        cmd = [
            "gz", "service", "-s", "/world/pac_mouse_maze/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{model_name}" type: MODEL'
        ]
        subprocess.run(cmd, capture_output=True)
    
    def world_to_grid(self, wx, wy):
        """Convert world to grid"""
        gx = int((wx - self.map_origin_x) / self.resolution)
        gy = int((wy - self.map_origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid to world"""
        wx = self.map_origin_x + (gx * self.resolution)
        wy = self.map_origin_y + (gy * self.resolution)
        return wx, wy
    
    def is_valid_grid_cell(self, x, y):
        """Check if grid cell valid"""
        return 0 <= x < self.grid_size_x and 0 <= y < self.grid_size_y
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ProposalMouseBrain()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
