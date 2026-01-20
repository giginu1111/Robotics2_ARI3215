import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from enum import Enum
import time
import random

class CatState(Enum):
    PATROL = 1
    ATTACK = 2
    ESCAPE = 3

class SmartCat(Node):
    def __init__(self):
        super().__init__('smart_cat')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cat/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/cat/state', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/cat/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/cat/camera/image_raw', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/cat/odom', self.odom_callback, 10)
        self.mouse_powered_sub = self.create_subscription(Bool, '/mouse/powered_up', self.mouse_powered_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)

        # State Machine
        self.state = CatState.PATROL
        self.mouse_powered = False

        # Sensor Data
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # Detection
        self.mouse_visible = False
        self.mouse_angle = 0.0
        self.mouse_distance = 0.0
        self.mouse_area = 0

        # Attack timer
        self.attack_start_time = 0.0
        self.attack_duration = 15.0  # Chase for 15 seconds max

        # Patrol waypoints
        self.patrol_waypoints = [
            (2.0, 2.0),
            (-2.0, 2.0),
            (-2.0, -2.0),
            (2.0, -2.0)
        ]
        self.current_waypoint_idx = 0
        self.waypoint_reached_dist = 0.5

        self.get_logger().info("🐱 Smart Cat Initialized!")

    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) == 0:
            return

        size = len(ranges)
        idx_front = size // 2
        idx_left = int(size * 0.75)
        idx_right = int(size * 0.25)

        def safe_range(idx_list):
            valid = [r for i in idx_list if 0 <= i < size 
                    for r in [ranges[i]] if not math.isinf(r) and not math.isnan(r)]
            return min(valid) if valid else 10.0

        self.front_dist = safe_range(range(idx_front-10, idx_front+10))
        self.left_dist = safe_range(range(idx_left-10, idx_left+10))
        self.right_dist = safe_range(range(idx_right-10, idx_right+10))

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            h, w, _ = cv_image.shape

            # Detect Grey/White Mouse (adjust color ranges as needed)
            # Grey mouse
            grey_mask = cv2.inRange(hsv, np.array([0, 0, 50]), np.array([180, 50, 200]))
            mouse_contours, _ = cv2.findContours(grey_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.mouse_visible = False
            if mouse_contours:
                c = max(mouse_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 300:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        self.mouse_angle = (cx - w/2) / (w/2) * 0.5
                        self.mouse_area = area
                        self.mouse_visible = True
                        self.mouse_distance = 25000.0 / max(area, 1)

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def mouse_powered_callback(self, msg):
        self.mouse_powered = msg.data

    def update_state(self):
        """State machine logic"""
        current_time = time.time()

        # If mouse is powered up, ESCAPE!
        if self.mouse_powered:
            if self.state != CatState.ESCAPE:
                self.get_logger().info("😨 MOUSE IS POWERED! ESCAPING!")
            self.state = CatState.ESCAPE
            return

        # If mouse visible and close, ATTACK
        if self.mouse_visible and self.mouse_distance < 3.0:
            if self.state != CatState.ATTACK:
                self.attack_start_time = current_time
                self.get_logger().info("😼 MOUSE DETECTED! ATTACKING!")
            self.state = CatState.ATTACK

            # Check if attack duration exceeded
            if current_time - self.attack_start_time > self.attack_duration:
                self.get_logger().info("⏰ Attack timeout, returning to patrol")
                self.state = CatState.PATROL
            return

        # If in attack but lost mouse, return to patrol
        if self.state == CatState.ATTACK and not self.mouse_visible:
            self.get_logger().info("❓ Lost mouse, returning to patrol")
            self.state = CatState.PATROL
            return

        # Default to patrol
        if self.state not in [CatState.ATTACK, CatState.ESCAPE]:
            self.state = CatState.PATROL

    def control_loop(self):
        self.update_state()

        # Publish state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        cmd = Twist()

        # Execute behavior based on state
        if self.state == CatState.PATROL:
            self.behavior_patrol(cmd)
        elif self.state == CatState.ATTACK:
            self.behavior_attack(cmd)
        elif self.state == CatState.ESCAPE:
            self.behavior_escape(cmd)

        self.cmd_pub.publish(cmd)

    def behavior_patrol(self, cmd):
        """Patrol between waypoints"""
        target = self.patrol_waypoints[self.current_waypoint_idx]

        # Check if waypoint reached
        dx = target[0] - self.robot_pose[0]
        dy = target[1] - self.robot_pose[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.waypoint_reached_dist:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.patrol_waypoints)
            target = self.patrol_waypoints[self.current_waypoint_idx]
            dx = target[0] - self.robot_pose[0]
            dy = target[1] - self.robot_pose[1]

        # Navigate to waypoint
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_yaw

        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Obstacle avoidance
        if self.front_dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8
        elif abs(angle_diff) > 0.3:
            cmd.linear.x = 0.1
            cmd.angular.z = 1.0 if angle_diff > 0 else -1.0
        else:
            cmd.linear.x = 0.25
            cmd.angular.z = 0.5 * angle_diff

    def behavior_attack(self, cmd):
        """Chase the mouse"""
        self.get_logger().info("😾 CHASING MOUSE!", throttle_duration_sec=1.0)

        if not self.mouse_visible:
            # Lost mouse, spin to find it
            cmd.angular.z = 0.5
            return

        # Chase toward mouse
        if self.front_dist < 0.4:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0
        elif abs(self.mouse_angle) > 0.2:
            cmd.linear.x = 0.15
            cmd.angular.z = -2.0 * self.mouse_angle
        else:
            cmd.linear.x = 0.35  # Fast chase
            cmd.angular.z = -1.0 * self.mouse_angle

    def behavior_escape(self, cmd):
        """Run away from the powered-up mouse"""
        self.get_logger().info("😱 ESCAPING FROM SUPER MOUSE!", throttle_duration_sec=1.0)

        if self.mouse_visible:
            # Run away from mouse
            escape_angle = self.mouse_angle + math.pi

            if self.front_dist < 0.4:
                cmd.linear.x = -0.2
                cmd.angular.z = 1.2
            else:
                cmd.linear.x = 0.45  # Run very fast
                # Turn away from mouse
                cmd.angular.z = 2.0 * self.mouse_angle
        else:
            # Just run in a safe direction
            if self.front_dist < 0.5:
                cmd.angular.z = 1.0
            else:
                cmd.linear.x = 0.4

def main(args=None):
    rclpy.init(args=args)
    node = SmartCat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
