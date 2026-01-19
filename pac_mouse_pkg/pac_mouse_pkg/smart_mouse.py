import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import heapq
from collections import deque
import time

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')

        # --- ROS I/O ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vis_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # QoS for Map (Transient Local ensures RViz sees it even if it joins late)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/mouse_map', map_qos)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.map_timer = self.create_timer(1.0, self.publish_map) 

        # --- INTERNAL STATE ---
        self.cheese_visible = False
        self.cheese_error = 0
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0

        # --- MAP SETTINGS ---
        self.resolution = 0.15
        self.grid_size = 120
        self.origin = self.grid_size // 2
        # -1: Unknown, 0: Free, 100: Wall
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=int) 

        # --- NAVIGATION MEMORY ---
        self.explore_goal = None
        self.path = []
        self.last_goal_time = 0
        self.goal_timeout = 25.0 
        self.unreachable_goals = [] 
        self.is_returning_home = False
        
        # --- STUCK RECOVERY ---
        self.last_position = (0.0, 0.0)
        self.stuck_timer = time.time()
        self.recovery_mode = False
        self.recovery_step = 0

        self.get_logger().info("🐭 ROBUST MOUSE: Nose Dive Filter & Auto-Recovery Active.")

    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # --- LIDAR MAPPING (WITH FILTERS) ---
    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        rx, ry = self.robot_pose

        for i, r in enumerate(msg.ranges):
            # --- FILTER 1: CLEANUP ---
            # Ignore NaNs and Infs
            if math.isnan(r) or math.isinf(r):
                continue
            
            # --- FILTER 2: NOSE DIVE PROTECTION ---
            # Ignore floor hits (> 3.5m) and self-hits (< 0.2m)
            if r > 3.5 or r < 0.2:
                continue

            angle = angle_min + i * angle_inc + self.robot_yaw
            
            # --- CLEARING FREE SPACE (Ghost Wall Fix) ---
            # If the laser travels X meters, everything up to X MUST be empty.
            # We overwrite any '100's here to "heal" the map from previous errors.
            for step in np.arange(0, r, self.resolution):
                tx = rx + step * math.cos(angle)
                ty = ry + step * math.sin(angle)
                gx, gy = self.world_to_grid(tx, ty)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 0  # Force clear

            # --- MARKING OBSTACLES ---
            wx = rx + r * math.cos(angle)
            wy = ry + r * math.sin(angle)
            gx, gy = self.world_to_grid(wx, wy)
            if self.is_in_grid(gx, gy):
                self.grid[gx, gy] = 100

    # --- SAFETY CHECK ---
    def is_safe_cell(self, x, y):
        """Returns True if cell is free AND not hugging a wall."""
        if not self.is_in_grid(x, y): return False
        if self.grid[x, y] == 100: return False
        if self.grid[x, y] == -1: return False 
        
        # INFLATION: Keep distance from walls (Radius 1 cell)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if self.is_in_grid(nx, ny) and self.grid[nx, ny] == 100:
                    return False
        return True

    # --- FRONTIER LOGIC ---
    def detect_frontiers(self):
        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.grid[x, y] == 0 and not visited[x, y]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.bfs_frontier(x, y, visited)
                        if len(frontier) > 3: # Ignore tiny noise holes
                            frontiers.append(frontier)
        
        self.publish_markers(frontiers_list=frontiers)
        return frontiers

    def is_frontier_cell(self, x, y):
        if self.grid[x, y] != 0: return False
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            if self.grid[x+dx, y+dy] == -1: return True
        return False

    def bfs_frontier(self, x, y, visited):
        q = deque([(x, y)])
        frontier = []
        while q:
            cx, cy = q.popleft()
            if visited[cx, cy]: continue
            visited[cx, cy] = True
            if self.is_frontier_cell(cx, cy):
                frontier.append((cx, cy))
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nx, ny = cx+dx, cy+dy
                    if self.is_in_grid(nx, ny): q.append((nx, ny))
        return frontier

    def choose_frontier_goal(self, frontiers):
        best_score = -float('inf')
        best_goal = None
        rx, ry = self.robot_pose

        for f in frontiers:
            fx = sum(p[0] for p in f) / len(f)
            fy = sum(p[1] for p in f) / len(f)
            gx, gy = int(fx), int(fy)
            wx, wy = self.grid_to_world(gx, gy)

            # Skip if we already failed to go here
            if any(self.dist((wx, wy), bg) < 0.6 for bg in self.unreachable_goals):
                continue
            
            if not self.is_safe_cell(gx, gy):
                continue

            dist = math.hypot(wx - rx, wy - ry)
            # Heuristic: Prefer larger frontiers, but balance distance
            score = (len(f) * 0.5) - (dist * 1.0) 
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal

    # --- A* PATHFINDING ---
    def get_path(self, start_world, goal_world):
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        if not self.is_in_grid(gx, gy): return None

        # Spiral to find nearest safe node if goal is slightly inside a wall
        safe_goal = self.find_nearest_safe_goal(goal_world)
        if safe_goal is None: return None
        
        gx, gy = self.world_to_grid(*safe_goal)

        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = { (sx, sy): 0 }
        
        iterations = 0
        while open_set:
            iterations += 1
            if iterations > 4000: return None # Timeout

            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):
                path = self.reconstruct_path(came_from, (cx, cy))
                self.publish_markers(path_points=path)
                return path

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = cx + dx, cy + dy
                if self.is_safe_cell(nx, ny):
                    new_g = g_score[(cx, cy)] + 1
                    if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = new_g
                        # Manhattan heuristic
                        h = abs(nx - gx) + abs(ny - gy)
                        heapq.heappush(open_set, (new_g + h, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        return None

    def find_nearest_safe_goal(self, target_world):
        tx, ty = self.world_to_grid(*target_world)
        if self.is_safe_cell(tx, ty): return target_world
        # Spiral Check
        for r in range(1, 6):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = tx + dx, ty + dy
                    if self.is_safe_cell(nx, ny):
                        return self.grid_to_world(nx, ny)
        return None

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            wx, wy = self.grid_to_world(*current)
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path

    # --- MAIN LOOP ---
    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # 1. CHEESE LOGIC (Priority 1)
        if self.cheese_visible:
            self.get_logger().info(f"🧀 CHEESE FOUND!", throttle_duration_sec=2.0)
            cmd.linear.x = 0.0
            if abs(self.cheese_error) > 20:
                cmd.angular.z = -0.005 * self.cheese_error
            else:
                cmd.linear.x = 0.2 # Charge!
            self.cmd_pub.publish(cmd)
            self.path = [] 
            self.explore_goal = None
            return

        # 2. STUCK RECOVERY LOGIC
        # If we aren't moving but are trying to, trigger recovery
        dist_moved = self.dist(self.robot_pose, self.last_position)
        if now - self.stuck_timer > 2.0:
            if dist_moved < 0.1 and self.path and not self.recovery_mode:
                self.get_logger().warn("⚠️ STUCK DETECTED: Initiating Recovery!")
                self.recovery_mode = True
                self.recovery_step = 0
            
            self.last_position = self.robot_pose
            self.stuck_timer = now

        if self.recovery_mode:
            self.recovery_step += 1
            if self.recovery_step < 15: # Back up for 1.5 sec
                cmd.linear.x = -0.15
                cmd.angular.z = 0.0
            elif self.recovery_step < 30: # Spin for 1.5 sec
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6
            else:
                self.recovery_mode = False
                self.path = [] # Force replan
                self.get_logger().info("✅ Recovery Complete. Replanning.")
            self.cmd_pub.publish(cmd)
            return

        # 3. GOAL MANAGEMENT
        if (self.explore_goal is None or 
            self.dist(self.robot_pose, self.explore_goal) < 0.4 or 
            (now - self.last_goal_time) > self.goal_timeout):
            
            frontiers = self.detect_frontiers()
            
            if not frontiers:
                dist_to_start = self.dist(self.robot_pose, (0.0, 0.0))
                if dist_to_start > 2.0 and not self.is_returning_home:
                    self.get_logger().warn("🔙 Done exploring. Returning Home.")
                    self.is_returning_home = True
                    self.explore_goal = (0.0, 0.0)
                else:
                    self.get_logger().info("✅ Mission Complete.")
                    cmd.angular.z = 0.2
                    self.cmd_pub.publish(cmd)
                    return
            else:
                new_goal = self.choose_frontier_goal(frontiers)
                if new_goal:
                    self.explore_goal = new_goal
                    self.is_returning_home = False
                    self.last_goal_time = now
                    self.path = [] 
                else:
                    # Spin to find new frontiers
                    cmd.angular.z = 0.5
                    self.cmd_pub.publish(cmd)
                    return

        # 4. PATH PLANNING
        if self.explore_goal and not self.path:
            self.path = self.get_path(self.robot_pose, self.explore_goal)
            if not self.path:
                self.get_logger().warn(f"🚫 Unreachable: {self.explore_goal}")
                self.unreachable_goals.append(self.explore_goal)
                self.explore_goal = None
                return

        # 5. PATH FOLLOWING
        if self.path:
            target_x, target_y = self.path[0]
            if self.dist(self.robot_pose, (target_x, target_y)) < 0.3:
                self.path.pop(0)
            else:
                angle_to_target = math.atan2(target_y - self.robot_pose[1], target_x - self.robot_pose[0])
                angle_diff = angle_to_target - self.robot_yaw
                # Normalize angle
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi

                if abs(angle_diff) > 0.4:
                    cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = 0.22
                    cmd.angular.z = 0.6 * angle_diff
        
        self.cmd_pub.publish(cmd)

    # --- UTILS ---
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
        if p1 is None or p2 is None: return float('inf')
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    # --- VISUALIZATION ---
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

    def publish_markers(self, frontiers_list=None, path_points=None):
        markers = MarkerArray()
        # Goal Marker
        if self.explore_goal:
            m = Marker()
            m.header.frame_id = "odom"
            m.id = 0; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = self.explore_goal
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.a = 1.0; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0 # Green Goal
            markers.markers.append(m)

        # Path Marker
        if path_points:
            m = Marker()
            m.header.frame_id = "odom"
            m.id = 1; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.a = 1.0; m.color.b = 1.0 # Blue Path
            for px, py in path_points:
                p = Point(); p.x = px; p.y = py
                m.points.append(p)
            markers.markers.append(m)
        
        # Frontier Marker
        if frontiers_list:
            m = Marker()
            m.header.frame_id = "odom"
            m.id = 2; m.type = Marker.POINTS; m.action = Marker.ADD
            m.scale.x = m.scale.y = 0.1
            m.color.a = 1.0; m.color.r = 1.0 # Red Frontiers
            for f in frontiers_list:
                for fx, fy in f:
                    wx, wy = self.grid_to_world(fx, fy)
                    p = Point(); p.x = wx; p.y = wy
                    m.points.append(p)
            markers.markers.append(m)
        
        self.vis_pub.publish(markers)

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.cheese_visible = False
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) > 200:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        self.cheese_error = cx - (img.shape[1] / 2)
                        self.cheese_visible = True
        except: pass

def main():
    rclpy.init()
    node = SmartMouse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()