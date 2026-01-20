import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from enum import Enum
import time

class MouseState(Enum):
    EXPLORE = 1
    COLLECT_CHEESE = 2
    ESCAPE_CAT = 3
    HUNT_CAT = 4
    VICTORY = 5

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mouse/state', 10)
        self.powered_pub = self.create_publisher(Bool, '/mouse/powered_up', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/mouse/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/mouse/camera/image_raw', self.camera_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/mouse/odom', self.odom_callback, 10)
        self.cat_state_sub = self.create_subscription(String, '/cat/state', self.cat_state_callback, 10)
        self.cheese_count_sub = self.create_subscription(Int32, '/cheese/count', self.cheese_count_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)

        # State Machine
        self.state = MouseState.EXPLORE
        self.powered_up = False
        self.power_up_timer = 0.0
        self.power_up_duration = 30.0  # 30 seconds per cheese

        # Sensor Data
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # Detection Flags
        self.cheese_visible = False
        self.cheese_distance = 0.0
        self.cheese_angle = 0.0
        self.cheese_area = 0

        self.cat_visible = False
        self.cat_distance = 0.0
        self.cat_angle = 0.0
        self.cat_area = 0

        self.cat_state = "PATROL"

        # Cheese tracking
        self.cheese_collected = 0
        self.total_cheese = 8
        self.last_cheese_time = time.time()

        self.get_logger().info("🐭 Smart Mouse Initialized! Hunting for cheese!")

    def cheese_count_callback(self, msg):
        """Update cheese count from cheese manager"""
        old_count = self.cheese_collected
        self.cheese_collected = msg.data

        # If we collected new cheese, activate power-up
        if self.cheese_collected > old_count:
            self.powered_up = True
            self.power_up_timer = time.time()
            self.last_cheese_time = time.time()
            self.get_logger().info(
                f"⚡ POWERED UP! Cheese: {self.cheese_collected}/{self.total_cheese}"
            )

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

            # Detect Yellow Cheese
            yellow_mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            cheese_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.cheese_visible = False
            if cheese_contours:
                c = max(cheese_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 200:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        self.cheese_angle = (cx - w/2) / (w/2) * 0.5
                        self.cheese_area = area
                        self.cheese_visible = True
                        self.cheese_distance = 20000.0 / max(area, 1)

            # Detect Red/White Cat
            red_mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            red_mask2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
            white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
            cat_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask1, red_mask2), white_mask)
            cat_contours, _ = cv2.findContours(cat_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.cat_visible = False
            if cat_contours:
                c = max(cat_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 300:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        self.cat_angle = (cx - w/2) / (w/2) * 0.5
                        self.cat_area = area
                        self.cat_visible = True
                        self.cat_distance = 25000.0 / max(area, 1)

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def cat_state_callback(self, msg):
        self.cat_state = msg.data

    def update_state(self):
        """State machine logic"""
        current_time = time.time()

        # Update power-up timer
        if self.powered_up:
            elapsed = current_time - self.power_up_timer
            if elapsed > self.power_up_duration:
                self.powered_up = False
                self.get_logger().info("⚡ Power-up expired!")

        # State transitions
        if self.state == MouseState.VICTORY:
            return  # Terminal state

        # Check for victory (all cheese collected AND cat eaten)
        if self.cheese_collected >= self.total_cheese and self.powered_up and self.cat_visible and self.cat_distance < 0.3:
            self.state = MouseState.VICTORY
            self.get_logger().info("🏆 VICTORY! All cheese collected and cat defeated!")
            return

        # If powered up and cat visible, hunt it (only if all cheese collected)
        if self.powered_up and self.cat_visible and self.cheese_collected >= self.total_cheese:
            self.state = MouseState.HUNT_CAT
            return

        # If cat is close and we're not powered up, escape
        if self.cat_visible and self.cat_distance < 2.0 and not self.powered_up:
            if self.cat_state == "ATTACK":
                self.state = MouseState.ESCAPE_CAT
                return

        # If cheese is visible and safe from cat (prioritize cheese!)
        if self.cheese_visible and self.cheese_collected < self.total_cheese:
            if not self.cat_visible or self.cat_distance > 3.0 or self.powered_up:
                self.state = MouseState.COLLECT_CHEESE
                return

        # Default to exploration
        if self.state not in [MouseState.ESCAPE_CAT, MouseState.HUNT_CAT]:
            self.state = MouseState.EXPLORE

    def control_loop(self):
        self.update_state()

        # Publish state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        powered_msg = Bool()
        powered_msg.data = self.powered_up
        self.powered_pub.publish(powered_msg)

        cmd = Twist()

        # Execute behavior based on state
        if self.state == MouseState.VICTORY:
            self.behavior_victory(cmd)
        elif self.state == MouseState.HUNT_CAT:
            self.behavior_hunt_cat(cmd)
        elif self.state == MouseState.ESCAPE_CAT:
            self.behavior_escape_cat(cmd)
        elif self.state == MouseState.COLLECT_CHEESE:
            self.behavior_collect_cheese(cmd)
        elif self.state == MouseState.EXPLORE:
            self.behavior_explore(cmd)

        self.cmd_pub.publish(cmd)

    def behavior_explore(self, cmd):
        """Wall-following exploration"""
        if self.front_dist < 0.5:
            cmd.angular.z = 0.8
        elif self.right_dist < 0.3:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3
        elif self.right_dist < 0.6:
            cmd.linear.x = 0.3
        else:
            cmd.linear.x = 0.25
            cmd.angular.z = -0.4

    def behavior_collect_cheese(self, cmd):
        """Navigate to and collect cheese (handled by cheese_manager)"""
        # Just get close, cheese_manager handles collection
        if abs(self.cheese_angle) > 0.15:
            cmd.linear.x = 0.1
            cmd.angular.z = -2.0 * self.cheese_angle
        else:
            if self.front_dist > 0.3:
                cmd.linear.x = 0.25
                cmd.angular.z = -0.5 * self.cheese_angle
            else:
                # Very close, stop and let collection happen
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

    def behavior_escape_cat(self, cmd):
        """Evade the cat"""
        self.get_logger().info("😱 ESCAPING CAT!", throttle_duration_sec=1.0)

        if self.front_dist < 0.4:
            cmd.linear.x = -0.15
            cmd.angular.z = 1.0
        else:
            cmd.linear.x = 0.4
            cmd.angular.z = -2.0 * self.cat_angle

    def behavior_hunt_cat(self, cmd):
        """Chase and eat the cat"""
        self.get_logger().info("😈 HUNTING CAT!", throttle_duration_sec=1.0)

        if self.cat_distance < 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("🎉 CAT EATEN!")
        elif abs(self.cat_angle) > 0.15:
            cmd.linear.x = 0.15
            cmd.angular.z = -2.5 * self.cat_angle
        else:
            cmd.linear.x = 0.35
            cmd.angular.z = -1.0 * self.cat_angle

    def behavior_victory(self, cmd):
        """Victory celebration"""
        self.get_logger().info("🎉 VICTORY DANCE!", throttle_duration_sec=2.0)
        cmd.angular.z = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = SmartMouse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
