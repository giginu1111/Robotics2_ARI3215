import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')
        
        # --- PUBLISHERS & SUBSCRIBERS ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        # --- TOOLS ---
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop) 

        # --- STATE MACHINE VARIABLES ---
        self.state = "SEARCH" 
        self.avoid_start_time = 0
        self.cheese_visible = False
        self.cheese_error = 0 
        
        # --- SENSOR DATA ---
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        print("🐭 Squeak Squeak! Smart Mouse Initialized!")

    def scan_callback(self, msg):
        ranges = msg.ranges
        size = len(ranges)
        
        def clean_data(r):
            if r == float('inf'): return 10.0
            if r == 0.0: return 10.0
            return r

        # Front is Index 180 (Middle)
        mid_point = size // 2 
        front_indices = [mid_point-2, mid_point-1, mid_point, mid_point+1, mid_point+2]
        
        valid_front = []
        for i in front_indices:
            if 0 <= i < size:
                valid_front.append(clean_data(ranges[i]))
        
        if valid_front:
            self.front_dist = sum(valid_front) / len(valid_front)
        else:
            self.front_dist = 10.0
        
        self.left_dist = clean_data(ranges[int(size * 0.75)]) 
        self.right_dist = clean_data(ranges[int(size * 0.25)])

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Yellow Cheese Thresholds
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        height, width, _ = cv_image.shape
                        self.cheese_error = cx - (width / 2)
                        self.cheese_visible = True
                        return

            self.cheese_visible = False
        except Exception:
            pass

    def control_loop(self):
        cmd = Twist()
        
        # --- LOGIC UPDATE: PRIORITY SWAP ---
        
        # PRIORITY 1: CHEESE (THE WIN CONDITION)
        # If we see cheese, we IGNORE the walls (until we literally touch it)
        if self.cheese_visible:
            self.state = "HUNT_CHEESE"
            
            # Stop if we are literally touching it (0.2m) so we don't burn the motors
            if self.front_dist < 0.2:
                print("🧀 YUM! Eating Cheese. (Stopped)")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                print(f"🧀 TARGET ACQUIRED! Closing in... (Dist: {self.front_dist:.2f})")
                cmd.linear.x = 0.3 
                cmd.angular.z = -0.005 * self.cheese_error 

        # PRIORITY 2: CONTINUE AVOIDING (If we were already turning and NO cheese)
        elif self.state == "AVOID_OBSTACLE":
            if time.time() - self.avoid_start_time < 1.0: 
                cmd.linear.x = -0.1 
                cmd.angular.z = 0.8 
            else:
                self.state = "SEARCH"

        # PRIORITY 3: DETECT WALL (Only if NO cheese is seen)
        elif self.front_dist < 0.6: 
            print(f"Wall at {self.front_dist:.2f}m! Avoiding...")
            self.state = "AVOID_OBSTACLE"
            self.avoid_start_time = time.time()
            
        # PRIORITY 4: SEARCH
        else:
            self.state = "SEARCH"
            cmd.linear.x = 0.4
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SmartMouse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()