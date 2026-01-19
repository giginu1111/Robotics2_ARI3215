import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import heapq
from enum import Enum
from collections import deque
import time

class State(Enum):
    EXPLORE = 1
    COLLECT = 2
    RECOVERY = 3

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')

        # --- ROS I/O ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)

        # --- INTERNAL STATE ---
        self.state = State.EXPLORE
        self.cheese_visible = False
        self.cheese_error = 0
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0

        # --- MAP SETTINGS ---
        self.resolution = 0.15
        self.grid_size = 120
        self.origin = self.grid_size // 2
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=int) 

        # --- NAVIGATION MEMORY ---
        self.explore_goal = None
        self.path = []
        self.last_goal_time = 0
        self.goal_timeout = 20.0 
        
        # FIX: Memory of bad goals to prevent infinite loops
        self.unreachable_goals = [] 
        
        self.get_logger().info("🐭 DEBUG MOUSE LOADED: Ready to print EVERYTHING.")

    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # (Standard Mapping Logic - Kept efficient, no prints here to avoid spam)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        rx, ry = self.robot_pose

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r): continue
            angle = angle_min + i * angle_inc + self.robot_yaw
            dist = min(r, 4.0)
            
            # Clear Free Space
            for step in np.arange(0, dist, self.resolution):
                tx = rx + step * math.cos(angle)
                ty = ry + step * math.sin(angle)
                gx, gy = self.world_to_grid(tx, ty)
                if self.is_in_grid(gx, gy) and self.grid[gx, gy] != 1:
                    self.grid[gx, gy] = 0

            # Mark Walls
            if r < 4.0:
                wx = rx + r * math.cos(angle)
                wy = ry + r * math.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 1

    # --- FRONTIER LOGIC ---
    def detect_frontiers(self):
        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.grid[x, y] == 0 and not visited[x, y]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.bfs_frontier(x, y, visited)
                        if len(frontier) > 4: 
                            frontiers.append(frontier)
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

        self.get_logger().info(f"🔎 Analyzing {len(frontiers)} potential frontiers...")

        for f in frontiers:
            fx = sum(p[0] for p in f) / len(f)
            fy = sum(p[1] for p in f) / len(f)
            wx, wy = self.grid_to_world(fx, fy)

            # FILTER: Check if this goal is in our blacklist
            is_bad = False
            for bad_goal in self.unreachable_goals:
                if self.dist((wx, wy), bad_goal) < 0.5:
                    is_bad = True
                    break
            
            if is_bad:
                continue

            dist = math.hypot(wx - rx, wy - ry)
            score = (len(f) * 0.5) - (dist * 1.5) # Prefer bigger, slightly closer
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        
        return best_goal

    # --- A* PATHFINDING ---
    def get_path(self, start_world, goal_world):
        self.get_logger().info(f"🗺️ Planning path: {start_world} -> {goal_world}")
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        if not self.is_in_grid(gx, gy):
            self.get_logger().warn("⚠️ Planner: Goal is off grid!")
            return None
        
        # Allow goal to be near a wall, but check the exact center
        if self.grid[gx, gy] == 1:
            self.get_logger().warn("⚠️ Planner: Goal is INSIDE a wall!")
            return None

        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = { (sx, sy): 0 }
        
        # Limit iterations to prevent freezing if path is weird
        iterations = 0
        max_iterations = 5000 

        while open_set:
            iterations += 1
            if iterations > max_iterations:
                self.get_logger().warn("⚠️ Planner: Timeout (Too complex)")
                return None

            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):
                self.get_logger().info(f"✅ Path found! Length: {len(came_from)}")
                return self.reconstruct_path(came_from, (cx, cy))

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = cx + dx, cy + dy
                
                # Check bounds and walls
                if self.is_in_grid(nx, ny) and self.grid[nx, ny] != 1:
                    new_g = g_score[(cx, cy)] + 1
                    if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = new_g
                        # Heuristic: Manhattan
                        h = abs(nx - gx) + abs(ny - gy)
                        heapq.heappush(open_set, (new_g + h, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        
        self.get_logger().warn("❌ Planner: No path found (A* exhausted)")
        return None

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            wx, wy = self.grid_to_world(*current)
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path

    # --- MAIN CONTROL LOOP ---
    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # 1. VISUAL OVERRIDE (Cheese)
        if self.cheese_visible:
            self.get_logger().info(f"🧀 CHEESE DETECTED! Error: {self.cheese_error:.2f}")
            cmd.linear.x = 0.2
            cmd.angular.z = -0.005 * self.cheese_error
            self.cmd_pub.publish(cmd)
            self.path = [] 
            self.explore_goal = None
            return

        # 2. GOAL MANAGEMENT
        # If we have no goal, or reached it, or timed out...
        if (self.explore_goal is None or 
            self.dist(self.robot_pose, self.explore_goal) < 0.3 or 
            (now - self.last_goal_time) > self.goal_timeout):
            
            if self.explore_goal:
                self.get_logger().info("🏁 Goal reached or timed out.")
            
            frontiers = self.detect_frontiers()
            if not frontiers:
                self.get_logger().info("🔄 No frontiers visible. Spinning to scan...")
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)
                return

            new_goal = self.choose_frontier_goal(frontiers)
            
            if new_goal:
                self.explore_goal = new_goal
                self.last_goal_time = now
                self.path = [] 
                self.get_logger().info(f"📍 New Goal Selected: {new_goal}")
            else:
                self.get_logger().warn("⚠️ All frontiers are unreachable! Spinning.")
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)
                return

        # 3. PATH PLANNING
        if self.explore_goal and not self.path:
            self.path = self.get_path(self.robot_pose, self.explore_goal)
            
            if not self.path:
                self.get_logger().error(f"🚫 Path failed to {self.explore_goal}. Blacklisting this goal.")
                self.unreachable_goals.append(self.explore_goal)
                self.explore_goal = None # Force re-think next loop
                return

        # 4. PATH FOLLOWING
        if self.path:
            target_x, target_y = self.path[0]
            dist_to_waypoint = self.dist(self.robot_pose, (target_x, target_y))
            
            # Print status every so often (every 10th frame roughly)
            if int(now * 10) % 20 == 0:
                 self.get_logger().info(f"🚗 Driving. Dist to WP: {dist_to_waypoint:.2f} | Remaining WPs: {len(self.path)}")

            if dist_to_waypoint < 0.25:
                self.get_logger().info("🎯 Waypoint hit. Advancing.")
                self.path.pop(0)
            else:
                angle_to_target = math.atan2(target_y - self.robot_pose[1], 
                                           target_x - self.robot_pose[0])
                angle_diff = angle_to_target - self.robot_yaw
                
                # Normalize angle
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi

                # Turn vs Drive logic
                if abs(angle_diff) > 0.4:
                    cmd.angular.z = 0.6 if angle_diff > 0 else -0.6
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = 0.25
                    cmd.angular.z = 0.8 * angle_diff
        
        self.cmd_pub.publish(cmd)

    # --- HELPERS ---
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

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Yellow Mask
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