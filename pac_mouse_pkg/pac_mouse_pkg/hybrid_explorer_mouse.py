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
        # 1. NAVIGATION & COMMUNICATION
        # ==========================================================
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # NOTE: Verify if your robot listens to /cmd_vel or /mouse/cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
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
        self.grid_size = 120   # 18m x 18m area
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
    def odom_callback(self, msg):
        # We track position relative to Odom frame
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # Update Internal Grid
        angle = msg.angle_min
        rx, ry = self.robot_x, self.robot_y
        
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r) or r < 0.35 or r > 8.0:
                angle += msg.angle_increment
                continue

            global_angle = angle + self.robot_yaw
            
            # Clamp range for mapping (don't map walls 10m away)
            valid_range = min(r, 4.0)
            hit = (r <= 4.0)

            # Raytrace Free Space
            # Step size reduced for better resolution
            for step in np.arange(0.2, valid_range, self.resolution):
                tx = rx + step * math.cos(global_angle)
                ty = ry + step * math.sin(global_angle)
                gx, gy = self.world_to_grid(tx, ty)
                if self.is_in_grid(gx, gy):
                    # Only mark as free if it wasn't already marked as obstacle
                    if self.grid[gx, gy] != 100:
                        self.grid[gx, gy] = 0 

            # Mark Obstacle
            if hit:
                wx = rx + r * math.cos(global_angle)
                wy = ry + r * math.sin(global_angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 100
            
            angle += msg.angle_increment

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
        cmd = Twist()
        
        # --- STATE 1: CHASE CHEESE ---
        if self.cheese_visible:
            if not self.chasing_cheese:
                self.get_logger().info("🧀 CHEESE! Taking manual control.")
                self.cancel_nav_goal()
                self.chasing_cheese = True

            if self.cheese_area > 18000:
                self.get_logger().info("🐭 YUM!")
                self.cmd_pub.publish(Twist()) 
                self.eat_closest_cheese()
                self.chasing_cheese = False
                self.grid.fill(-1) # Reset map to re-explore area
                return
            
            # Simple P-Controller
            cmd.linear.x = 0.25
            cmd.angular.z = -0.003 * self.cheese_error
            self.cmd_pub.publish(cmd)
            return

        # --- STATE 2: EXPLORATION ---
        self.chasing_cheese = False 
        
        if not self.is_navigating:
            self.get_logger().info("🔍 Scanning for frontiers...", throttle_duration_sec=2.0)
            frontiers = self.detect_frontiers()
            
            if frontiers:
                goal_point = self.choose_frontier_goal(frontiers)
                if goal_point:
                    self.send_nav_goal(goal_point)
                else:
                    self.spin_to_find_new()
            else:
                self.spin_to_find_new()

    # ==========================================================
    # FRONTIER LOGIC
    # ==========================================================
    def spin_to_find_new(self):
        cmd = Twist()
        cmd.angular.z = 0.4
        self.cmd_pub.publish(cmd)

    def detect_frontiers(self):
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
        
        for f in frontiers:
            # Pick center of frontier
            mid_idx = len(f) // 2
            gx, gy = f[mid_idx]
            wx, wy = self.grid_to_world(gx, gy)

            # 1. Filter: Don't go to unreachable places
            if any(self.dist((wx, wy), bg) < 0.5 for bg in self.unreachable_goals):
                continue
            
            # 2. Relaxed Safety Check: 
            # Check only the immediate 3x3 area, not a huge box.
            if not self.is_safe_spot(gx, gy):
                continue

            # 3. Score: Prefer Larger frontiers, closer to robot
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            score = (len(f) * 1.0) - (dist * 0.5) 
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal

    def is_safe_spot(self, gx, gy):
        # Small 3x3 check (radius 1)
        margin = 1 
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_in_grid(nx, ny) and self.grid[nx, ny] == 100:
                    return False
        return True

    # ==========================================================
    # NAV2 ACTION CLIENT
    # ==========================================================
    def send_nav_goal(self, point):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'mouse/odom' 
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
        if status == 4: 
            self.get_logger().info('✅ Reached Frontier.')
        else:
            self.get_logger().warn('⚠️ Failed to reach frontier.')
            # Temporarily mark this location as bad
            # (In a full slam system, the map would update, but here we blacklist)
        
        self.is_navigating = False
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
        
        if closest_name and min_dist < 2.0: # Only eat if reasonably close
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
            # FRAME FIX: Markers must match the frame of the coordinates (odom)
            m.header.frame_id = "mouse/odom" 
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