import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import heapq
from collections import deque
import subprocess
import time

class HybridMouse(Node):
    def __init__(self):
        super().__init__('hybrid_mouse')

        # ==========================================================
        # 1. NAVIGATION & COMMUNICATION (From Explorer Mouse)
        # ==========================================================
        # Action Client to talk to Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher to drive manually (only when chasing cheese)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Replaces /mouse/cmd_vel if remapped
        
        # Score Publisher
        self.pub_score = self.create_publisher(String, '/cheese_eaten', 10)

        # ==========================================================
        # 2. PERCEPTION & MAPPING (From Smart Mouse)
        # ==========================================================
        self.bridge = CvBridge()
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Internal Grid for Frontier Exploration (Independent of Global Costmap)
        self.resolution = 0.15
        self.grid_size = 120
        self.origin = self.grid_size // 2
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=int) 
        
        # Visualization (So we can see what the mouse is thinking)
        self.vis_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/mouse_internal_map', map_qos)

        # ==========================================================
        # 3. INTERNAL STATE
        # ==========================================================
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0
        
        # Cheese State
        self.cheese_visible = False
        self.cheese_error = 0
        self.cheese_area = 0.0
        self.chasing_cheese = False
        
        # Nav State
        self.nav_goal_handle = None
        self.is_navigating = False
        self.unreachable_goals = []
        
        # "Cheat List" ONLY for deleting the correct model name when eating
        self.possible_cheeses = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'},
            {'x': -5.0, 'y': -5.0, 'name': 'cheese_4'}
        ]

        # Main Loop (Runs logic 10 times a second)
        self.timer = self.create_timer(0.1, self.brain_loop)
        
        # Map Publish Loop (1Hz)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("🐭 HYBRID MOUSE: Nav2 Brain + CV Eyes Activated!")

    # ==========================================================
    # SENSOR CALLBACKS
    # ==========================================================
    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # Build the internal grid map for finding frontiers
        angle = msg.angle_min
        rx, ry = self.robot_pose
        
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r) or r < 0.35 or r > 8.0:
                angle += msg.angle_increment
                continue

            global_angle = angle + self.robot_yaw
            
            # Clamp for mapping
            valid_range = min(r, 5.0)
            hit = (r <= 5.0)

            # Raytrace Free Space
            for step in np.arange(0.2, valid_range, self.resolution):
                tx = rx + step * math.cos(global_angle)
                ty = ry + step * math.sin(global_angle)
                gx, gy = self.world_to_grid(tx, ty)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 0 # Mark Free

            # Mark Obstacle
            if hit:
                wx = rx + r * math.cos(global_angle)
                wy = ry + r * math.sin(global_angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 100 # Mark Occupied
            
            angle += msg.angle_increment

    def camera_callback(self, msg):
        # OpenCV logic to detect Yellow Cheese
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Yellow Mask
            mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 100: # Sensitivity threshold
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        # Calculate error (center of screen vs center of cheese)
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
        cmd = Twist()
        
        # --- STATE 1: CHASE CHEESE (Visual Servoing) ---
        if self.cheese_visible:
            if not self.chasing_cheese:
                self.get_logger().info("🧀 CHEESE SPOTTED! Cancelling Nav2 to chase...")
                self.cancel_nav_goal()
                self.chasing_cheese = True

            # Visual Servoing Logic (P-Controller)
            if self.cheese_area > 18000:
                # Close enough to eat!
                self.get_logger().info("🐭 CHOMP!")
                self.cmd_pub.publish(Twist()) # Stop
                self.eat_closest_cheese()
                self.chasing_cheese = False
                # Reset map around us to force re-exploration
                self.grid.fill(-1) 
                return
            
            elif abs(self.cheese_error) < 40: 
                # Centered, drive forward
                cmd.linear.x = 0.3
                cmd.angular.z = -0.002 * self.cheese_error # Small corrections
            else:
                # Turn to center
                cmd.linear.x = 0.05
                cmd.angular.z = -0.005 * self.cheese_error
            
            self.cmd_pub.publish(cmd)
            return

        # --- STATE 2: EXPLORATION (Nav2) ---
        self.chasing_cheese = False # Reset flag if we lost sight
        
        if not self.is_navigating:
            # We need a new goal
            self.get_logger().info("🤔 Planning next move...")
            frontiers = self.detect_frontiers()
            
            if frontiers:
                goal_point = self.choose_frontier_goal(frontiers)
                if goal_point:
                    self.send_nav_goal(goal_point)
                else:
                    self.get_logger().warn("Frontiers found but unreachable.")
                    self.spin_to_find_new()
            else:
                self.get_logger().info("No frontiers detected. Spinning to scan...")
                self.spin_to_find_new()

    # ==========================================================
    # EXPLORATION LOGIC
    # ==========================================================
    def spin_to_find_new(self):
        # Rotate in place to update map
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_pub.publish(cmd)

    def detect_frontiers(self):
        # BFS to find edges between Known(0) and Unknown(-1)
        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        for x in range(2, self.grid_size - 2):
            for y in range(2, self.grid_size - 2):
                if self.grid[x, y] == 0 and not visited[x, y]: # If Free Space
                    if self.is_frontier_cell(x, y):
                        frontier = self.bfs_frontier(x, y, visited)
                        if len(frontier) > 5: # Filter tiny noise
                            frontiers.append(frontier)
        
        self.publish_markers(frontiers_list=frontiers)
        return frontiers

    def is_frontier_cell(self, x, y):
        # A cell is a frontier if it is free (0) but touches unknown (-1)
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
        # Pick the largest frontier that is closest
        best_score = -float('inf')
        best_goal = None
        rx, ry = self.robot_pose

        for f in frontiers:
            # Pick center of frontier
            mid_idx = len(f) // 2
            gx, gy = f[mid_idx]
            wx, wy = self.grid_to_world(gx, gy)

            # Don't go back to unreachable places
            if any(self.dist((wx, wy), bg) < 0.5 for bg in self.unreachable_goals):
                continue
            
            # Score = Size - Distance cost
            dist = math.hypot(wx - rx, wy - ry)
            score = (len(f) * 0.5) - (dist * 1.0) 
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal

    # ==========================================================
    # NAV2 ACTION CLIENT WRAPPERS
    # ==========================================================
    def send_nav_goal(self, point):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"📍 Nav2 Goal Sent: ({point[0]:.2f}, {point[1]:.2f})")
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True

    def cancel_nav_goal(self):
        if self.nav_goal_handle:
            self.get_logger().info("🛑 Cancelling Nav2 Goal...")
            future = self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        self.is_navigating = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.is_navigating = False
            return

        self.nav_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info('✅ Arrived at Frontier.')
        else:
            self.get_logger().warn('❌ Failed to reach frontier.')
            # Add to unreachable list so we don't try again immediately
            if self.nav_goal_handle: # If it wasn't cancelled manually
                 # In a real impl, we'd store the coordinate, but for now just reset
                 pass
        
        self.is_navigating = False
        self.nav_goal_handle = None

    # ==========================================================
    # HELPERS
    # ==========================================================
    def eat_closest_cheese(self):
        # We find the closest cheese in our "Cheat List" to call the delete service
        closest_name = ""
        min_dist = float('inf')
        
        for cheese in self.possible_cheeses:
            d = self.dist(self.robot_pose, (cheese['x'], cheese['y']))
            if d < min_dist:
                min_dist = d
                closest_name = cheese['name']
        
        if closest_name:
            self.delete_model(closest_name)
            self.pub_score.publish(String(data=closest_name))
            # Remove from list so we don't try to delete it again
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

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = - (self.grid_size * self.resolution) / 2.0
        msg.info.origin.position.y = - (self.grid_size * self.resolution) / 2.0
        msg.data = self.grid.T.flatten().astype(np.int8).tolist()
        self.map_pub.publish(msg)

    def publish_markers(self, frontiers_list=None):
        markers = MarkerArray()
        if frontiers_list:
            m = Marker()
            m.header.frame_id = "odom"
            m.id = 0; m.type = Marker.POINTS; m.action = Marker.ADD
            m.scale.x = m.scale.y = 0.1
            m.color.a = 1.0; m.color.r = 1.0 # Red Dots for Frontiers
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