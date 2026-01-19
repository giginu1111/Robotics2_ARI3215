import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import heapq  # <--- Added for A* Priority Queue
from enum import Enum
from collections import deque
import time

class State(Enum):
    EXPLORE = 1
    COLLECT = 2

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

        # --- FSM & Strategy ---
        self.state = State.EXPLORE
        self.cheese_visible = False
        self.cheese_error = 0
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0

        # --- MAP SETTINGS ---
        self.resolution = 0.15  # Meters per cell
        self.grid_size = 120    # Grid dimensions
        self.origin = self.grid_size // 2
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=int) # -1=Unknown, 0=Free, 1=Wall

        # --- GOAL & PATH PERSISTENCE ---
        self.explore_goal = None
        self.path = []  # <--- Added to store the A* path
        self.last_goal_time = 0
        self.goal_timeout = 15.0 # Seconds to try one goal before switching

        self.get_logger().info("🐭 Enhanced Smart Mouse with A* Initialized")

    # --- ODOMETRY: Tracks real-world position ---
    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Convert quaternion to Euler Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # --- LIDAR: Enhanced with Ray-Casting (Marks Free Space) ---
    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        rx, ry = self.robot_pose

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
                
            angle = angle_min + i * angle_inc + self.robot_yaw
            dist = min(r, 4.0) # Only map up to 4 meters to save CPU
            
            # 1. Clear space along the ray (Mark as 0)
            for step in np.arange(0, dist, self.resolution):
                tx = rx + step * math.cos(angle)
                ty = ry + step * math.sin(angle)
                gx, gy = self.world_to_grid(tx, ty)
                if self.is_in_grid(gx, gy) and self.grid[gx, gy] != 1:
                    self.grid[gx, gy] = 0

            # 2. Mark obstacles (Mark as 1)
            if r < 4.0:
                wx = rx + r * math.cos(angle)
                wy = ry + r * math.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_in_grid(gx, gy):
                    self.grid[gx, gy] = 1

    # --- FRONTIER LOGIC: Finding the "edge" of the map ---
    def detect_frontiers(self):
        frontiers = []
        visited = np.zeros_like(self.grid, dtype=bool)

        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if self.grid[x, y] == 0 and not visited[x, y]:
                    if self.is_frontier_cell(x, y):
                        frontier = self.bfs_frontier(x, y, visited)
                        if len(frontier) > 4: # Filter noise
                            frontiers.append(frontier)
        return frontiers

    def is_frontier_cell(self, x, y):
        # A cell is a frontier if it is FREE (0) and has an UNKNOWN (-1) neighbor
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
            # Get center of the frontier
            fx = sum(p[0] for p in f) / len(f)
            fy = sum(p[1] for p in f) / len(f)
            wx, wy = self.grid_to_world(fx, fy)

            dist = math.hypot(wx - rx, wy - ry)
            # Efficient score: Big frontiers are good, far away ones are bad
            score = (len(f) * 0.2) - dist 
            
            if score > best_score:
                best_score = score
                best_goal = (wx, wy)
        return best_goal

    # --- PATHFINDING: A* Algorithm (NEW) ---
    def get_path(self, start_world, goal_world):
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        # Safety: If goal is inside a wall or out of bounds, fail
        if not self.is_in_grid(gx, gy) or self.grid[gx, gy] == 1:
            return None

        # Priority Queue for A*: (f_score, x, y)
        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        
        came_from = {}
        g_score = { (sx, sy): 0 }
        
        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):
                return self.reconstruct_path(came_from, (cx, cy))

            # Neighbors: Up, Down, Left, Right
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = cx + dx, cy + dy
                
                # Check bounds. We treat -1 (Unknown) and 0 (Free) as valid. 1 is Wall.
                if self.is_in_grid(nx, ny) and self.grid[nx, ny] != 1:
                    new_g = g_score[(cx, cy)] + 1
                    
                    if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = new_g
                        f = new_g + self.heuristic((nx, ny), (gx, gy))
                        heapq.heappush(open_set, (f, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        
        return None

    def heuristic(self, a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            wx, wy = self.grid_to_world(*current)
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path

    # --- CONTROL LOOP (UPDATED) ---
    def control_loop(self):
        cmd = Twist()
        
        # Priority 1: Collect Cheese
        if self.cheese_visible:
            cmd.linear.x = 0.2
            cmd.angular.z = -0.005 * self.cheese_error
            self.cmd_pub.publish(cmd)
            self.path = [] # Clear path if distracted
            return

        # Priority 2: Explore / Pathfind
        now = time.time()
        
        # Check if we need a NEW goal (Timeout or reached target)
        if (self.explore_goal is None or 
            self.dist(self.robot_pose, self.explore_goal) < 0.3 or 
            (now - self.last_goal_time) > self.goal_timeout):
            
            frontiers = self.detect_frontiers()
            new_goal = self.choose_frontier_goal(frontiers)
            
            if new_goal:
                self.explore_goal = new_goal
                self.last_goal_time = now
                self.path = [] # Reset path for new goal
                self.get_logger().info(f"New Goal Selected: {self.explore_goal}")
            else:
                # No frontiers found? Spin to find some
                cmd.angular.z = 0.5
                self.cmd_pub.publish(cmd)
                return

        # If we have a goal but NO path, Calculate one!
        if self.explore_goal and not self.path:
            self.path = self.get_path(self.robot_pose, self.explore_goal)
            if not self.path:
                # Goal unreachable? Invalidate it so we pick a new one next loop
                self.explore_goal = None
                return

        # Follow the Path
        if self.path:
            # Get the next waypoint
            target_x, target_y = self.path[0]
            
            # Are we close to this waypoint?
            if self.dist(self.robot_pose, (target_x, target_y)) < 0.25:
                self.path.pop(0) # Remove it, move to next
            else:
                # Drive towards waypoint
                angle_to_target = math.atan2(target_y - self.robot_pose[1], 
                                           target_x - self.robot_pose[0])
                angle_diff = angle_to_target - self.robot_yaw
                
                # Normalize angle
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi

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