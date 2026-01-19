import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from sensor_msgs.msg import LaserScan, Image # type: ignore
import cv2
from cv_bridge import CvBridge # type: ignore
import numpy as np
import math
from enum import Enum
from collections import deque
from nav_msgs.msg import Odometry

# In __init__
self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

def odom_callback(self, msg):
    self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

# -------------------- STATES --------------------

class State(Enum):
    EXPLORE = 1
    COLLECT = 2
    EVADE = 3
    Victory = 4


# -------------------- NODE --------------------

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)

        # FSM
        self.state = State.EXPLORE

        # Camera state
        self.cheese_visible = False
        self.cheese_error = 0

        # LiDAR distances
        self.front = 10.0
        self.right = 10.0

        # -------------------- MAP --------------------

        self.resolution = 0.2  # meters
        self.grid_size = 100   # 100x100 grid
        self.origin = self.grid_size // 2

        self.grid = np.full((self.grid_size, self.grid_size), -1)  # unknown

        self.robot_pose = (0.0, 0.0)  # assume starting at origin
        self.explore_goal = None

        self.get_logger().info("🐭 Smart Mouse with Frontier Exploration Initialized")

    # -------------------- LIDAR --------------------

    def scan_callback(self, msg):
        ranges = msg.ranges
        size = len(ranges)

        def clean(r):
            return r if 0.05 < r < 10.0 else 10.0

        front_idx = size // 2
        right_idx = int(size * 0.25)

        self.front = clean(ranges[front_idx])
        self.right = clean(ranges[right_idx])

        self.update_occupancy_grid(ranges)

def update_occupancy_grid(self, ranges):
    rx, ry = self.robot_pose
    angle_min = -math.pi
    angle_inc = (2 * math.pi) / len(ranges)

    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_inc
        
        # 1. Determine the end point of the laser ray
        # If the laser didn't hit anything, we treat it as "Max Range" for clearing space
        is_hit = r < 9.5
        dist = r if is_hit else 5.0 # Clear space up to 5m even if no wall hit
        
        # 2. Iterate along the ray to mark FREE space (0)
        # We step through the ray in small increments (resolution / 2)
        for step in np.arange(0, dist, self.resolution / 2):
            tx = rx + step * math.cos(angle)
            ty = ry + step * math.sin(angle)
            gx = int(tx / self.resolution) + self.origin
            gy = int(ty / self.resolution) + self.origin
            
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                # If it's not a wall, mark it as free
                if self.grid[gx, gy] != 1:
                    self.grid[gx, gy] = 0

        # 3. Mark the final point as a WALL (1) if it was a hit
        if is_hit:
            wx = rx + r * math.cos(angle)
            wy = ry + r * math.sin(angle)
            gx = int(wx / self.resolution) + self.origin
            gy = int(wy / self.resolution) + self.origin
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.grid[gx, gy] = 1


    # -------------------- CAMERA --------------------

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower = np.array([20, 100, 100])
            upper = np.array([40, 255, 255])

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            self.cheese_visible = False

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        self.cheese_error = cx - (img.shape[1] / 2)
                        self.cheese_visible = True
        except Exception:
            pass

    # -------------------- FRONTIER LOGIC --------------------

    def detect_frontiers(self):
        frontiers = []
        visited = set()

        for x in range(1, self.grid_size - 1):
            for y in range(1, self.grid_size - 1):
                if (x, y) in visited:
                    continue

                if self.grid[x, y] == 0 and self.is_frontier_cell(x, y):
                    frontier = self.bfs_frontier(x, y, visited)
                    frontiers.append(frontier)

        return frontiers

    def is_frontier_cell(self, x, y):
        if self.grid[x, y] != 0:
            return False
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            if self.grid[x+dx, y+dy] == -1:
                return True
        return False

    def bfs_frontier(self, x, y, visited):
        q = deque()
        q.append((x, y))
        frontier = []

        while q:
            cx, cy = q.popleft()
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))

            if not self.is_frontier_cell(cx, cy):
                continue

            frontier.append((cx, cy))

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx+dx, cy+dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    q.append((nx, ny))

        return frontier

    def choose_frontier_goal(self, frontiers):
        rx, ry = self.origin, self.origin

        best = None
        best_score = -float('inf')

        for f in frontiers:
            mx = sum(p[0] for p in f) / len(f)
            my = sum(p[1] for p in f) / len(f)

            dist = math.hypot(mx - rx, my - ry)
            score = len(f) - dist

            if score > best_score:
                best_score = score
                best = (mx, my)

        return best

    # -------------------- CONTROL LOOP --------------------

    def control_loop(self):
        cmd = Twist()

        # FSM UPDATE
        if self.cheese_visible:
            self.state = State.COLLECT
        else:
            self.state = State.EXPLORE

        # ---------------- COLLECT ----------------
        if self.state == State.COLLECT:
            cmd.linear.x = 0.3
            cmd.angular.z = -0.005 * self.cheese_error
            self.cmd_pub.publish(cmd)
            return

        # ---------------- EXPLORE ----------------
        if self.explore_goal is None:
            frontiers = self.detect_frontiers()
            if frontiers:
                self.explore_goal = self.choose_frontier_goal(frontiers)
                self.get_logger().info(f"Exploring frontier at {self.explore_goal}")
            else:
                self.get_logger().info("No frontiers left")
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                return

        # Simple goal-seeking + obstacle avoidance
        if self.front < 0.4:
            cmd.angular.z = 0.8
        elif self.right < 0.25:
            cmd.angular.z = 0.3
            cmd.linear.x = 0.2
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


# -------------------- MAIN --------------------

def main(args=None):
    rclpy.init(args=args)
    node = SmartMouse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()