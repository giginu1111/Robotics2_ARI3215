"""
=============================================================================
ADVANCED MOUSE BRAIN CONTROLLER
=============================================================================
OVERVIEW:
This node implements an intelligent mouse controller that combines multiple
behavioral strategies for optimal cheese collection while evading the cat.

FEATURES:
Visual Servoing: Uses camera to detect and approach cheese
Frontier-Based Exploration: Discovers unknown areas systematically
Path Planning: Uses Nav2 for obstacle-free navigation
Predator Avoidance: Detects and evades the cat
Adaptive Behavior: Switches strategies based on context

BEHAVIORAL STATES:
CHASE_CHEESE: Visual servoing to visible cheese
EXPLORE: Frontier-based autonomous exploration
FLEE: Emergency evasion from cat
NAVIGATE: Goal-directed navigation using Nav2

SENSORS USED:
Camera: Cheese detection (HSV color filtering)
LiDAR: Obstacle detection and mapping
Odometry: Position tracking and state estimation
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
        self.cheese_threshold = 50000
        self.resolution = 0.15
        
        # Cat detection settings
        self.cat_danger_distance = 2.0  # Cat must be within 2m to trigger flee
        self.cat_critical_distance = 1.0  # Urgent flee if cat within 1m
        
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
        
        # NEW: Subscribe to cat position to know where it really is
        self.create_subscription(Odometry, '/cat/odom', self.cat_odometry_callback, 10)
        
        # ====================================================================
        # ACTION CLIENT
        # ====================================================================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        # Mouse state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Cat state (FIXED: Now using actual cat position)
        self.cat_x = 0.0
        self.cat_y = 0.0
        self.cat_detected = False
        self.cat_distance = float('inf')
        
        # Vision
        self.bridge = CvBridge()
        self.cheese_visible = False
        self.cheese_error = 0.0
        self.cheese_area = 0.0
        
        # Navigation
        self.is_navigating = False
        self.nav_goal_handle = None
        
        # Mapping
        self.grid_size = 160
        self.origin = self.grid_size // 2
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=int)
        
        # Cheese models
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
        
        self.get_logger().info("🐭 PROPOSAL MOUSE BRAIN: ONLINE")
    
    def odometry_callback(self, msg):
        """Mouse odometry callback"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def cat_odometry_callback(self, msg):
        """
        Cat odometry callback - FIXED VERSION
        Now we know exactly where the cat is!
        """
        self.cat_x = msg.pose.pose.position.x
        self.cat_y = msg.pose.pose.position.y
        
        # Calculate actual distance to cat
        dx = self.cat_x - self.robot_x
        dy = self.cat_y - self.robot_y
        self.cat_distance = math.hypot(dx, dy)
        
        # Only trigger cat detection if cat is actually close
        if self.cat_distance < self.cat_danger_distance:
            self.cat_detected = True
        else:
            self.cat_detected = False
    
    def lidar_callback(self, msg):
        """LiDAR callback - FIXED: No longer detects walls as cat!"""
        angle = msg.angle_min
        
        for range_val in msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)):
                # Only use LiDAR for mapping, NOT for cat detection
                if 0.35 < range_val < 8.0:
                    self.update_occupancy_grid(range_val, angle)
            
            angle += msg.angle_increment
        
        # Cat detection removed from LiDAR - we use actual cat position now!
    
    def update_occupancy_grid(self, range_val, local_angle):
        """Update internal map with LiDAR data"""
        global_angle = local_angle + self.robot_yaw
        
        max_trace = min(range_val, 4.0)
        for step in np.arange(0.2, max_trace, self.resolution):
            wx = self.robot_x + step * math.cos(global_angle)
            wy = self.robot_y + step * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy) and self.occupancy_grid[gx, gy] != 100:
                self.occupancy_grid[gx, gy] = 0
        
        if range_val <= 4.0:
            wx = self.robot_x + range_val * math.cos(global_angle)
            wy = self.robot_y + range_val * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy):
                self.occupancy_grid[gx, gy] = 100
    
    def camera_callback(self, msg):
        """Camera callback for cheese detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        if self.cheese_visible == False:
                            self.get_logger().info("🧀 Cheese spotted!")
                        cx = int(M["m10"] / M["m00"])
                        image_center_x = cv_image.shape[1] / 2
                        
                        self.cheese_error = cx - image_center_x
                        self.cheese_area = area
                        self.cheese_visible = True
                        return
            
            self.cheese_visible = False
            
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
    
    def control_loop(self):
        """Main control loop with FIXED cat detection"""
        cmd = Twist()
        
        # Priority 1: Flee from cat (FIXED: Only triggers when cat is actually close)
        if self.cat_detected:
            urgency = "CRITICAL" if self.cat_distance < self.cat_critical_distance else "WARNING"
            self.get_logger().warn(
                f"😱 CAT {urgency}! Distance: {self.cat_distance:.2f}m", 
                throttle_duration_sec=1.0
            )
            
            # Calculate escape direction (away from cat)
            dx = self.robot_x - self.cat_x
            dy = self.robot_y - self.cat_y
            escape_angle_world = math.atan2(dy, dx)
            escape_angle_rel = self.normalize_angle(escape_angle_world - self.robot_yaw)
            
            # Faster turning if cat is very close
            turn_speed = 5.0 if self.cat_distance < self.cat_critical_distance else 3.0
            cmd.angular.z = turn_speed * np.sign(escape_angle_rel)
            
            # Move forward if roughly aligned with escape direction
            if abs(escape_angle_rel) < 0.5:
                cmd.linear.x = 0.5 if self.cat_distance < self.cat_critical_distance else 0.4
            else:
                cmd.linear.x = 0.15
            
            self.cmd_pub.publish(cmd)
            return
        
        # Priority 2: Chase cheese
        if self.cheese_visible:
            if self.cheese_area > self.cheese_threshold:
                self.get_logger().info("🧀 EATING CHEESE!")
                self.cmd_pub.publish(Twist())
                self.collect_cheese()
                return
            else:
                # NEED TO ADD WALL COLLISION AVOIDANCE HERE -------------------------------
                kp_angular = 0.05
                cmd.angular.z = -kp_angular * self.cheese_error
                if abs(self.cheese_error) < 50:
                    cmd.linear.x = self.linear_speed
                else:
                    cmd.linear.x = 2.0
                self.cmd_pub.publish(cmd)
                return
        
        # Priority 3: Navigate
        if self.is_navigating:
            return
        
        # Priority 4: Explore
        self.explore_environment()
    
    def explore_environment(self):
        """Exploration behavior"""
        frontiers = self.detect_frontiers()
        
        if frontiers:
            goal_point = self.select_best_frontier(frontiers)
            if goal_point:
                self.send_navigation_goal(goal_point)
                return
        
        # No frontiers - rotate to search
        cmd = Twist()
        cmd.angular.z = 1.5
        self.cmd_pub.publish(cmd)
    
    def detect_frontiers(self):
        """Detect frontier regions between known and unknown space"""
        frontiers = []
        visited = np.zeros_like(self.occupancy_grid, dtype=bool)
        
        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.occupancy_grid[x, y] == 0 and not visited[x, y]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.expand_frontier(x, y, visited)
                        if len(frontier) > 5:
                            frontiers.append(frontier)
        
        return frontiers
    
    def is_frontier_cell(self, x, y):
        """Check if cell is on frontier boundary"""
        if self.occupancy_grid[x, y] != 0:
            return False
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid_grid_cell(nx, ny):
                if self.occupancy_grid[nx, ny] == -1:
                    return True
        return False
    
    def expand_frontier(self, start_x, start_y, visited):
        """Expand frontier region using BFS"""
        frontier = []
        queue = deque([(start_x, start_y)])
        visited[start_x, start_y] = True
        
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
                            not visited[nx, ny] and 
                            self.occupancy_grid[nx, ny] == 0):
                            visited[nx, ny] = True
                            queue.append((nx, ny))
        
        return frontier
    
    def select_best_frontier(self, frontiers):
        """Select best frontier based on size and distance"""
        best_score = -float('inf')
        best_goal = None
        
        for frontier in frontiers:
            center_idx = len(frontier) // 2
            gx, gy = frontier[center_idx]
            wx, wy = self.grid_to_world(gx, gy)
            
            if not self.is_safe_location(gx, gy):
                continue
            
            distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
            size_bonus = len(frontier)
            score = size_bonus * 1.5 - distance * 0.5
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal
    
    def is_safe_location(self, gx, gy):
        """Check if location has sufficient clearance"""
        margin = 2
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_valid_grid_cell(nx, ny):
                    if self.occupancy_grid[nx, ny] == 100:
                        return False
        return True
    
    def send_navigation_goal(self, point):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = 0.0
        
        angle = math.atan2(point[1] - self.robot_y, point[0] - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)
        
        self.get_logger().info(f"📍 Goal: ({point[0]:.2f}, {point[1]:.2f})")
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
    
    def goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_navigating = False
            return
        
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle Nav2 goal completion"""
        result = future.result()
        self.is_navigating = False
        
        if result.status == 4:
            self.get_logger().info('✅ Goal reached')
        else:
            self.get_logger().warn('⚠️ Navigation failed')
    
    def collect_cheese(self):
        """Handle cheese collection"""
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
        """Convert world coordinates to grid"""
        gx = int(wx / self.resolution) + self.origin
        gy = int(wy / self.resolution) + self.origin
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world"""
        wx = (gx - self.origin) * self.resolution
        wy = (gy - self.origin) * self.resolution
        return wx, wy
    
    def is_valid_grid_cell(self, x, y):
        """Check if grid coordinates are valid"""
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size
    
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