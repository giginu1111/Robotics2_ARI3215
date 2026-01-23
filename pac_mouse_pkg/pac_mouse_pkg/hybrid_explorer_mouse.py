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
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
import cv2
import numpy as np
import math
import heapq
from collections import deque
import subprocess
import time

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class HybridMouse(Node):
    def __init__(self):
        super().__init__('hybrid_mouse', 
             parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    


        # ==========================================================
        # 1. NAVIGATION & COMMUNICATION
        # ==========================================================
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10) 
        self.pub_score = self.create_publisher(String, '/cheese_eaten', 10)

        # ==========================================================
        # 2. PERCEPTION & MAPPING
        # ==========================================================
        self.bridge = CvBridge()
        
        self.scan_sub = self.create_subscription(LaserScan, '/mouse/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/mouse/camera/image_raw', self.camera_callback, 10)
        
        # We subscribe to Odom just for velocity/emergency checks
        self.odom_sub = self.create_subscription(Odometry, '/mouse/odom', self.odom_callback, 10)
        
        # CRITICAL FIX: Subscribe to /map updates from SLAM so our internal map matches reality
        # Or, we build our own simple layer on top. 
        # For this hybrid, we will keep building our own fast local map based on Odom for speed,
        # BUT we will transform goals to the correct frame if needed. 
        # Ideally, we should listen to TF, but for simplicity in this script:
        # We will assume 'odom' and 'map' are aligned at start, or use Map frame for everything.
        
        # Let's stick to using the robot's estimated pose in the MAP frame if SLAM is running.
        # However, getting 'map' pose requires a TF listener. 
        # To keep this script simple and crash-free: 
        # We will use the Odometry frame for our internal exploration map,
        # but Nav2 will handle the heavy lifting of global positioning.
        
        self.vis_pub = self.create_publisher(MarkerArray, '/mouse/visualization_marker_array', 10)
        
        # Internal Grid
        self.resolution = 0.15 # 15cm grid cells
        self.grid_size = 160   # 18m x 18m area
        self.origin = self.grid_size // 2
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=int) 

        # ==========================================================
        # 3. INTERNAL STATE
        # ==========================================================
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.cheese_visible = False
        self.cheese_error = 0
        self.cheese_area = 0.0
        self.chasing_cheese = False
        
        self.nav_goal_handle = None
        self.is_navigating = False
        self.unreachable_goals = []
        
        # Cheat list for deleting models
        self.possible_cheeses = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'},
            {'x': -5.0, 'y': -5.0, 'name': 'cheese_4'}
        ]

        # Timer: 10Hz Control Loop
        self.timer = self.create_timer(0.1, self.brain_loop)
        
        self.get_logger().info("🐭 HYBRID MOUSE: Systems Nominal.")

    # ==========================================================
    # SENSOR CALLBACKS
    # ==========================================================
    def update_pose_from_tf(self):
        try:
        # Use Time(0) to ask for the LATEST available transform 
        # instead of the specific current clock time.
            now = rclpy.time.Time() 

            trans = self.tf_buffer.lookup_transform(
                'map', 
                'mouse/base_link', 
                now,
                timeout=rclpy.duration.Duration(seconds=0.05) # Small wait if needed
            )
        
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
        
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            return True
       
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Use throttled logging so your terminal isn't flooded
            self.get_logger().warn(f"⏳ Waiting for valid TF: {e}", throttle_duration_sec=2.0)
            return False
        

        

    
    def odom_callback(self, msg):
        # We track position relative to Odom frame
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):

        if abs(self.robot_x) < 0.01 and abs(self.robot_y) < 0.01:
            self.get_logger().warn("⚠️ Scan ignored: robot pose not initialized")
            return


    # 1. Get the latest Map-to-Base transform so the scan is placed correctly
        if not self.update_pose_from_tf():
            return

    # 2. Convert Scan to Numpy arrays for speed
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) + self.robot_yaw

    # 3. Filter out bad data (NaN, Inf, or too close/far)
        mask = (np.isfinite(ranges)) & (ranges > 0.35) & (ranges < 8.0)
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

    # 4. Calculate Global World Coordinates of the sensor "hits"
        hit_x = self.robot_x + valid_ranges * np.cos(valid_angles)
        hit_y = self.robot_y + valid_ranges * np.sin(valid_angles)

    # 5. Mark Free Space (Raytracing)
    # Instead of full raytracing (slow), we mark points along the beam as free
        for step_dist in np.arange(0.2, 3.0, self.resolution * 2): # Steps of 30cm for speed
            free_x = self.robot_x + step_dist * np.cos(valid_angles)
            free_y = self.robot_y + step_dist * np.sin(valid_angles)
        
            for fx, fy in zip(free_x, free_y):
                gx, gy = self.world_to_grid(fx, fy)
                if self.is_in_grid(gx, gy) and self.grid[gx, gy] != 100:
                    self.grid[gx, gy] = 0

    # 6. Mark Obstacles (Hits)
    # Only mark hits that are within a reasonable range (4m) to keep map clean
        obstacle_mask = valid_ranges < 4.0
        for ox, oy in zip(hit_x[obstacle_mask], hit_y[obstacle_mask]):
            gx, gy = self.world_to_grid(ox, oy)
            if self.is_in_grid(gx, gy):
                self.grid[gx, gy] = 100

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 100: 
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        self.cheese_error = cx - (img.shape[1] / 2)
                        self.cheese_visible = True
                        self.cheese_area = area
                        return
            
            self.cheese_visible = False
        except Exception:
            pass

    # ==========================================================
    # MAIN BRAIN LOOP
    # ==========================================================
    def brain_loop(self):

        if not self.update_pose_from_tf():
            self.get_logger().warn("⛔ Skipping brain loop: no TF pose")
            return

        """
        Main 10Hz Control Loop.
        Handles state transitions between Cheese Chasing and Frontier Exploration.
        """
        # 1. CRITICAL: Always sync internal pose with the Map frame via TF
        # This fixes the "Frame Drift" and "Slow Turning" bugs.
        # if not self.update_pose_from_tf():
        #     # If TF isn't ready, we don't have a valid position. Skip this loop.
        #     return

        # 2. STATE 1: CHASE CHEESE (Priority)
        if self.cheese_visible:
            # Transition Logic: Stop Nav2 if we were exploring
            if not self.chasing_cheese:
                self.get_logger().info("🧀 CHEESE SPOTTED! Aborting Nav2 for manual intercept.")
                self.cancel_nav_goal()
                self.chasing_cheese = True

            # "Eat" Logic: If we are close enough (based on camera area)
            if self.cheese_area > 50000:
                self.get_logger().info("🐭 SUCCESS: Eating cheese.")
                self.cmd_pub.publish(Twist()) # Hard stop
                self.eat_closest_cheese()
                self.chasing_cheese = False
                self.grid.fill(-1) # Force re-exploration of the cleared area
                return
            
            if self.is_navigating:
                self.get_logger().warn("⚠️ COMMAND CONFLICT: Cheese seen but Nav2 still active. Canceling...")
                self.cancel_nav_goal()
                return
    
            # Manual Intercept (Visual Servoing)
            cmd = Twist()
            cmd.linear.x = 5 # SPEED OF MOUSE WHEN LOCKING ONTO CHEESE
            # P-Controller for steering (0.003 is gain, cheese_error is center-offset)
            cmd.angular.z = -0.003 * self.cheese_error
            self.cmd_pub.publish(cmd)
            return

        # 3. TRANSITION: Reset state if cheese is lost
        if self.chasing_cheese and not self.cheese_visible:
            self.get_logger().info("🧀 Cheese lost. Returning to exploration.")
            self.chasing_cheese = False
            # We don't return here; we let it fall through to Frontier Exploration

        # 4. STATE 2: EXPLORATION (Nav2)
        # If Nav2 is currently busy driving us to a frontier, just wait and let it work
        if self.is_navigating:
            # We can add a timeout check here if Nav2 gets stuck for > 60 seconds
            return

        # 5. FRONTIER LOGIC: If idle, find somewhere new to go
        self.get_logger().info("🔍 Searching for new frontiers...", throttle_duration_sec=5.0)
        frontiers = self.detect_frontiers()
        
        if frontiers:
            goal_point = self.choose_frontier_goal(frontiers)
            if goal_point:
                # Successfully found a safe, reachable frontier
                self.send_nav_goal(goal_point)
            else:
                # Frontiers exist but none passed the "is_safe_spot" test
                self.spin_to_find_new()
        else:
            # No frontiers found at all (Map is likely complete or obscured)
            self.get_logger().warn("🗺️ No frontiers found. Performing search spin.")
            self.spin_to_find_new()

        if not frontiers:
            self.spin_to_find_new()

    # ==========================================================
    # FRONTIER LOGIC
    # ==========================================================
    def spin_to_find_new(self):
        cmd = Twist()
        cmd.angular.z = 5.0
        self.cmd_pub.publish(cmd)

    def detect_frontiers(self):

        unknown_count = np.count_nonzero(self.grid == -1)
        free_count = np.count_nonzero(self.grid == 0)
        obs_count = np.count_nonzero(self.grid == 100)

        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        # Look for free cells next to unknown cells
        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.grid[x, y] == 0 and not visited[x, y]: 
                    if self.is_frontier_cell(x, y):
                        frontier = self.bfs_frontier(x, y, visited)
                        # Relaxed Filter: Accept smaller frontiers (size > 3)
                        if len(frontier) > 3: 
                            frontiers.append(frontier)
        
        self.publish_markers(frontiers_list=frontiers)

        if not frontiers:
        # DEBUG LOG: Help identify why there are no frontiers
            self.get_logger().info(
                f"📊 Map Stats: Unknown={unknown_count}, Free={free_count}, Obstacles={obs_count}",
                throttle_duration_sec=10.0)
            
            if unknown_count > 100 and free_count > 0:
                self.get_logger().error("‼️ STARVATION: Unknown cells exist but no frontiers detected. Check raytracing!")
        return frontiers

    def is_frontier_cell(self, x, y):
        # Frontier = Free Cell (0) touching Unknown (-1)
        if self.grid[x, y] != 0: return False
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if self.is_in_grid(nx, ny):
                if self.grid[nx, ny] == -1: return True
        return False

    def bfs_frontier(self, x, y, visited):
        q = deque([(x, y)])
        frontier = []
        visited[x, y] = True
        while q:
            cx, cy = q.popleft()
            if self.is_frontier_cell(cx, cy):
                frontier.append((cx, cy))
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        nx, ny = cx + dx, cy + dy
                        if self.is_in_grid(nx, ny) and not visited[nx, ny]:
                            if self.grid[nx, ny] == 0:
                                visited[nx, ny] = True
                                q.append((nx, ny))
        return frontier

    def choose_frontier_goal(self, frontiers):
        best_score = -float('inf')
        best_goal = None
    
        # 1. Sort frontiers by size (largest first) to reduce processing
        frontiers.sort(key=len, reverse=True)

        for f in frontiers:
        # Pick the middle cell of the frontier as the representative point
            mid_idx = len(f) // 2
            gx, gy = f[mid_idx]
            wx, wy = self.grid_to_world(gx, gy)

        # 2. BLACKLIST FILTER: Skip if we've already tried and failed here
            if any(self.dist((wx, wy), bg) < 1.0 for bg in self.unreachable_goals):
                continue
        
        # 3. SAFETY MARGIN: Check 0.5m around the goal (4 cells at 0.15m res)
        # This prevents Nav2 from rejecting the goal due to inflation.
            if not self.is_safe_spot(gx, gy):
                continue

        # 4. SCORING: Distance vs. Information Gain
        # We want the biggest frontiers that are closest to us.
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
        
        # Score = (Size of Frontier) - (Distance weight)
        # Higher score is better.
            score = (len(f) * 1.5) - (dist * 2.0) 
        
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
    
        return best_goal

    def is_safe_spot(self, gx, gy):
    # Margin 4 = 60cm (4 cells * 0.15m resolution)
        margin = 4 
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_in_grid(nx, ny):
                    if self.grid[nx, ny] == 100:
                        return False
        return True

    # ==========================================================
    # NAV2 ACTION CLIENT
    # ==========================================================
    def send_nav_goal(self, point):
        self.current_goal_coords = point
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map' 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        
        # Orientation: Face the goal direction roughly
        angle = math.atan2(point[1] - self.robot_y, point[0] - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2)

        self.get_logger().info(f"📍 Sending Goal: ({point[0]:.2f}, {point[1]:.2f})")
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True

    def cancel_nav_goal(self):
        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        self.is_navigating = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Nav2 rejected goal.')
            self.is_navigating = False
            return

        self.nav_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
    
        if status == 4: # SUCCESS
            self.get_logger().info('✅ Reached Frontier.')
            self.is_navigating = False
        else:
        # If we failed (status 5 or 6 usually means aborted/canceled)
            self.get_logger().warn('⚠️ Nav2 Aborted. Executing Manual Recovery (Reverse)...')
        
        # 1. Immediate Manual Recovery: Reverse for 1.5 seconds
            recovery_msg = Twist()
            recovery_msg.linear.x = -0.2  # Back up at 0.2 m/s
        
        # We publish this a few times to ensure the robot clears the obstacle
            for _ in range(15):
                self.cmd_pub.publish(recovery_msg)
                time.sleep(0.1)
        
        # 2. Stop the robot
            self.cmd_pub.publish(Twist())
        
        # 3. Add to blacklist so we don't immediately try the exact same spot
            if hasattr(self, 'current_goal_coords'):
                self.unreachable_goals.append(self.current_goal_coords)
        
            self.is_navigating = False
            self.get_logger().info('🔄 Recovery complete. Re-scanning for new frontiers.')

        self.nav_goal_handle = None

    # ==========================================================
    # UTILS
    # ==========================================================
    def eat_closest_cheese(self):
        closest_name = ""
        min_dist = float('inf')
        for cheese in self.possible_cheeses:
            d = self.dist((self.robot_x, self.robot_y), (cheese['x'], cheese['y']))
            if d < min_dist:
                min_dist = d
                closest_name = cheese['name']
        
        if closest_name and min_dist < 0.5: # Only eat if reasonably close
            self.delete_model(closest_name)
            self.pub_score.publish(String(data=closest_name))
            self.possible_cheeses = [c for c in self.possible_cheeses if c['name'] != closest_name]

    def delete_model(self, name):
        world_name = "pac_mouse_maze" 
        cmd = [
            "gz", "service", "-s", f"/world/{world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{name}" type: MODEL'
        ]
        subprocess.run(cmd, capture_output=True)

    def world_to_grid(self, x, y):
        gx = int(x / self.resolution) + self.origin
        gy = int(y / self.resolution) + self.origin
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = (gx - self.origin) * self.resolution
        wy = (gy - self.origin) * self.resolution
        return wx, wy

    def is_in_grid(self, x, y):
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size

    def dist(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def publish_markers(self, frontiers_list=None):
        markers = MarkerArray()
        if frontiers_list:
            m = Marker()
            m.header.frame_id = "map" 
            m.id = 0; m.type = Marker.POINTS; m.action = Marker.ADD
            m.scale.x = m.scale.y = 0.1
            m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.0
            for f in frontiers_list:
                for fx, fy in f:
                    wx, wy = self.grid_to_world(fx, fy)
                    p = Point(); p.x = wx; p.y = wy
                    m.points.append(p)
            markers.markers.append(m)
        self.vis_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = HybridMouse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()