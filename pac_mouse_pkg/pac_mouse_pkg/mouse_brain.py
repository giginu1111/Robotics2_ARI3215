#!/usr/bin/env python3
"""
=============================================================================
ADVANCED MOUSE BRAIN CONTROLLER
=============================================================================
Course: Robotics 2 (ARI3215)
Project: Pac-Mouse Autonomous Navigation

OVERVIEW:
This node implements an intelligent mouse controller that combines multiple
behavioral strategies for optimal cheese collection while evading the cat.

FEATURES:
1. Visual Servoing: Uses camera to detect and approach cheese
2. Frontier-Based Exploration: Discovers unknown areas systematically
3. Path Planning: Uses Nav2 for obstacle-free navigation
4. Predator Avoidance: Detects and evades the cat
5. Adaptive Behavior: Switches strategies based on context

BEHAVIORAL STATES:
- CHASE_CHEESE: Visual servoing to visible cheese
- EXPLORE: Frontier-based autonomous exploration
- FLEE: Emergency evasion from cat
- NAVIGATE: Goal-directed navigation using Nav2

SENSORS USED:
- Camera: Cheese detection (HSV color filtering)
- LiDAR: Obstacle detection and mapping
- Odometry: Position tracking and state estimation

Author: Damian Cutajar
Date: January 2026
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

# ROS Messages
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

# Processing Libraries
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
import subprocess

class ProposalMouseBrain(Node):
    """
    Advanced mouse controller implementing hybrid exploration and cheese collection.
    """
    
    def __init__(self):
        super().__init__('proposal_mouse_brain',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION PARAMETERS
        # ====================================================================
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('cheese_threshold_area', 18000)
        self.declare_parameter('exploration_resolution', 0.15)
        self.declare_parameter('safety_distance', 0.4)
        
        # Load parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.cheese_threshold = self.get_parameter('cheese_threshold_area').value
        
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
        
        # ====================================================================
        # ACTION CLIENTS
        # ====================================================================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ====================================================================
        # INTERNAL STATE VARIABLES
        # ====================================================================
        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Vision system
        self.bridge = CvBridge()
        self.cheese_visible = False
        self.cheese_error = 0.0  # Horizontal offset from image center
        self.cheese_area = 0.0
        
        # Navigation state
        self.is_navigating = False
        self.nav_goal_handle = None
        self.current_goal = None
        
        # Mapping system
        self.grid_size = 160  # 24m x 24m world
        self.resolution = 0.15  # 15cm cells
        self.origin = self.grid_size // 2
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), -1, dtype=int)
        # -1 = unknown, 0 = free, 100 = occupied
        
        # Cat detection
        self.cat_detected = False
        self.cat_direction = 0.0
        
        # Cheese database (for deletion after collection)
        self.cheese_models = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'},
            {'x': -5.0, 'y': -5.0, 'name': 'cheese_4'}
        ]
        
        # ====================================================================
        # CONTROL LOOP TIMER
        # ====================================================================
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🐭 PROPOSAL MOUSE BRAIN: ONLINE")
        self.get_logger().info("=" * 60)
    
    # ========================================================================
    # SENSOR CALLBACKS
    # ========================================================================
    
    def odometry_callback(self, msg):
        """Updates robot position from odometry data."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def lidar_callback(self, msg):
        """
        Processes LiDAR data for:
        1. Occupancy grid mapping
        2. Cat detection (close large obstacles)
        3. Obstacle avoidance
        """
        angle = msg.angle_min
        
        # Check for cat (large obstacle within 2m)
        close_obstacle_angles = []
        
        for range_val in msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)):
                # Map building
                if 0.35 < range_val < 8.0:
                    self.update_occupancy_grid(range_val, angle)
                
                # Cat detection
                if 0.5 < range_val < 2.0:
                    close_obstacle_angles.append(angle + self.robot_yaw)
            
            angle += msg.angle_increment
        
        # Update cat detection status
        if close_obstacle_angles:
            self.cat_detected = True
            self.cat_direction = np.mean(close_obstacle_angles)
        else:
            self.cat_detected = False
    
    def update_occupancy_grid(self, range_val, local_angle):
        """Updates internal occupancy grid with LiDAR measurement."""
        global_angle = local_angle + self.robot_yaw
        
        # Raytrace free space
        max_trace = min(range_val, 4.0)
        for step in np.arange(0.2, max_trace, self.resolution):
            wx = self.robot_x + step * math.cos(global_angle)
            wy = self.robot_y + step * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy) and self.occupancy_grid[gx, gy] != 100:
                self.occupancy_grid[gx, gy] = 0  # Free
        
        # Mark obstacle
        if range_val <= 4.0:
            wx = self.robot_x + range_val * math.cos(global_angle)
            wy = self.robot_y + range_val * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            
            if self.is_valid_grid_cell(gx, gy):
                self.occupancy_grid[gx, gy] = 100  # Occupied
    
    def camera_callback(self, msg):
        """
        Detects cheese using HSV color filtering.
        Yellow cheese: H=20-40, S=100-255, V=100-255
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for yellow objects
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Get largest contour (closest cheese)
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum size threshold
                    # Calculate centroid
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        image_center_x = cv_image.shape[1] / 2
                        
                        # Update cheese detection state
                        self.cheese_error = cx - image_center_x
                        self.cheese_area = area
                        self.cheese_visible = True
                        return
            
            # No cheese detected
            self.cheese_visible = False
            
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")
    
    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    
    def control_loop(self):
        """
        Main decision-making loop implementing behavior hierarchy:
        1. FLEE (highest priority) - Evade cat
        2. CHASE - Visual servoing to cheese
        3. NAVIGATE - Goal-directed movement
        4. EXPLORE - Frontier-based exploration
        """
        cmd = Twist()
        
        # PRIORITY 1: Flee from cat
        if self.cat_detected:
            self.get_logger().warn("😱 CAT DETECTED! FLEEING!", throttle_duration_sec=1.0)
            self.flee_from_cat(cmd)
            self.cmd_pub.publish(cmd)
            return
        
        # PRIORITY 2: Chase visible cheese
        if self.cheese_visible:
            if self.cheese_area > self.cheese_threshold:
                # Cheese reached!
                self.get_logger().info("🧀 EATING CHEESE!")
                self.cmd_pub.publish(Twist())  # Stop
                self.collect_cheese()
                return
            else:
                # Visual servoing to cheese
                self.chase_cheese(cmd)
                self.cmd_pub.publish(cmd)
                return
        
        # PRIORITY 3: Navigate to goal
        if self.is_navigating:
            # Nav2 is handling movement
            return
        
        # PRIORITY 4: Explore unknown areas
        self.explore_environment()
    
    # ========================================================================
    # BEHAVIOR IMPLEMENTATIONS
    # ========================================================================
    
    def flee_from_cat(self, cmd):
        """Emergency evasion behavior - move away from cat."""
        # Calculate escape direction (opposite of cat)
        escape_angle = self.cat_direction + math.pi
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(escape_angle - self.robot_yaw)
        
        # Turn towards escape direction
        cmd.angular.z = 3.0 * np.sign(angle_diff)
        
        # Move forward if roughly aligned
        if abs(angle_diff) < 0.5:
            cmd.linear.x = 0.4  # Fast escape
    
    def chase_cheese(self, cmd):
        """Visual servoing to align with and approach cheese."""
        # P-controller for alignment
        kp_angular = 0.005
        cmd.angular.z = -kp_angular * self.cheese_error
        
        # Move forward while aligned
        if abs(self.cheese_error) < 50:  # pixels
            cmd.linear.x = self.linear_speed
        else:
            cmd.linear.x = 0.1  # Slow while turning
    
    def explore_environment(self):
        """
        Frontier-based exploration:
        1. Detect frontiers (boundaries between known and unknown)
        2. Select best frontier
        3. Send navigation goal
        """
        frontiers = self.detect_frontiers()
        
        if frontiers:
            goal_point = self.select_best_frontier(frontiers)
            if goal_point:
                self.send_navigation_goal(goal_point)
                return
        
        # No frontiers found - rotate to find new areas
        self.rotate_search()
    
    def rotate_search(self):
        """Rotate in place to discover new areas."""
        cmd = Twist()
        cmd.angular.z = 1.5
        self.cmd_pub.publish(cmd)
    
    # ========================================================================
    # FRONTIER DETECTION
    # ========================================================================
    
    def detect_frontiers(self):
        """
        Finds frontier regions (boundaries between free and unknown space).
        
        Returns:
            list: List of frontier regions, each region is a list of (x,y) grid cells
        """
        frontiers = []
        visited = np.zeros_like(self.occupancy_grid, dtype=bool)
        
        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.occupancy_grid[x, y] == 0 and not visited[x, y]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.expand_frontier(x, y, visited)
                        if len(frontier) > 5:  # Minimum frontier size
                            frontiers.append(frontier)
        
        return frontiers
    
    def is_frontier_cell(self, x, y):
        """Checks if cell is on frontier (free cell adjacent to unknown)."""
        if self.occupancy_grid[x, y] != 0:
            return False
        
        # Check 4-connected neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid_grid_cell(nx, ny):
                if self.occupancy_grid[nx, ny] == -1:  # Unknown
                    return True
        return False
    
    def expand_frontier(self, start_x, start_y, visited):
        """Expands frontier region using BFS."""
        frontier = []
        queue = deque([(start_x, start_y)])
        visited[start_x, start_y] = True
        
        while queue:
            x, y = queue.popleft()
            
            if self.is_frontier_cell(x, y):
                frontier.append((x, y))
                
                # Expand to 8-connected neighbors
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
        """
        Selects the most promising frontier based on:
        - Size (larger is better)
        - Distance (closer is better)
        - Safety (avoid obstacles)
        
        Returns:
            tuple: (world_x, world_y) of goal point, or None
        """
        best_score = -float('inf')
        best_goal = None
        
        for frontier in frontiers:
            # Use centroid of frontier
            center_idx = len(frontier) // 2
            gx, gy = frontier[center_idx]
            wx, wy = self.grid_to_world(gx, gy)
            
            # Check safety
            if not self.is_safe_location(gx, gy):
                continue
            
            # Calculate score
            distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
            size_bonus = len(frontier)
            score = size_bonus * 1.5 - distance * 0.5
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal
    
    def is_safe_location(self, gx, gy):
        """Checks if location has sufficient clearance from obstacles."""
        margin = 2  # Check 5x5 area
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_valid_grid_cell(nx, ny):
                    if self.occupancy_grid[nx, ny] == 100:
                        return False
        return True
    
    # ========================================================================
    # NAVIGATION
    # ========================================================================
    
    def send_navigation_goal(self, point):
        """Sends goal to Nav2 action server."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = 0.0
        
        # Calculate orientation towards goal
        angle = math.atan2(point[1] - self.robot_y, point[0] - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)
        
        self.get_logger().info(f"📍 Navigating to ({point[0]:.2f}, {point[1]:.2f})")
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
    
    def goal_response_callback(self, future):
        """Handles Nav2 goal acceptance response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.is_navigating = False
            return
        
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handles Nav2 goal completion."""
        result = future.result()
        self.is_navigating = False
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Goal reached')
        else:
            self.get_logger().warn('⚠️ Navigation failed')
    
    # ========================================================================
    # CHEESE COLLECTION
    # ========================================================================
    
    def collect_cheese(self):
        """Handles cheese collection: removal from simulation and scoring."""
        # Find closest cheese
        min_dist = float('inf')
        closest_cheese = None
        
        for cheese in self.cheese_models:
            dist = math.hypot(self.robot_x - cheese['x'], 
                            self.robot_y - cheese['y'])
            if dist < min_dist:
                min_dist = dist
                closest_cheese = cheese
        
        if closest_cheese and min_dist < 0.5:
            # Delete from Gazebo
            self.delete_gazebo_model(closest_cheese['name'])
            
            # Publish score
            self.score_pub.publish(String(data=closest_cheese['name']))
            
            # Remove from internal list
            self.cheese_models.remove(closest_cheese)
            
            self.get_logger().info(f"🎯 Collected {closest_cheese['name']}!")
            
            # Reset map to re-explore
            self.occupancy_grid.fill(-1)
    
    def delete_gazebo_model(self, model_name):
        """Removes model from Gazebo simulation."""
        cmd = [
            "gz", "service", "-s", "/world/pac_mouse_maze/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{model_name}" type: MODEL'
        ]
        subprocess.run(cmd, capture_output=True)
    
    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    
    def world_to_grid(self, wx, wy):
        """Converts world coordinates to grid indices."""
        gx = int(wx / self.resolution) + self.origin
        gy = int(wy / self.resolution) + self.origin
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Converts grid indices to world coordinates."""
        wx = (gx - self.origin) * self.resolution
        wy = (gy - self.origin) * self.resolution
        return wx, wy
    
    def is_valid_grid_cell(self, x, y):
        """Checks if grid indices are within bounds."""
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size
    
    def normalize_angle(self, angle):
        """Normalizes angle to [-pi, pi]."""
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
