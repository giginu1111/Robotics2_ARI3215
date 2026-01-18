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
        self.timer = self.create_timer(0.1, self.control_loop) # Run loop 10 times a second

        # --- STATE MACHINE VARIABLES ---
        self.state = "SEARCH" # Options: SEARCH, AVOID_OBSTACLE, HUNT_CHEESE
        self.avoid_start_time = 0
        self.cheese_visible = False
        self.cheese_error = 0 # How far left/right is the cheese?
        
        # --- SENSOR DATA ---
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        print("🐭 Squeak Squeak! Smart Mouse Initialized!")

    def scan_callback(self, msg):
        # We need to handle the infinite/zero readings carefully
        ranges = [r if r > 0.0 else 10.0 for r in msg.ranges]
        
        # Assuming 360 Lidar: Front is roughly index 0
        # Check a cone in front (indices 0-10 and 350-360)
        front_cone = ranges[0:10] + ranges[-10:]
        self.front_dist = min(front_cone)
        self.left_dist = min(ranges[80:100]) # Approx 90 degrees left
        self.right_dist = min(ranges[260:280]) # Approx 90 degrees right

    def camera_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- CHEESE DETECTION (Color Thresholding) ---
            # Yellow Cheese in HSV (Hue, Saturation, Value)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define Yellow Range (Adjust these if your cheese is different color!)
            # Standard Gazebo Yellow: Hue 20-40
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])
            
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # Find Contours (Blobs)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # Find the biggest yellow blob
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100: # Ignore tiny noise
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) # Center X of the blob
                        height, width, _ = cv_image.shape
                        
                        # Calculate error: 0 = centered, negative = left, positive = right
                        self.cheese_error = cx - (width / 2)
                        self.cheese_visible = True
                        return

            self.cheese_visible = False
            
        except Exception as e:
            pass

    def control_loop(self):
        cmd = Twist()
        
        # --- STATE MACHINE LOGIC ---
        
        # PRIORITY 1: WALL AVOIDANCE (Safety First!)
        # If we are currently avoiding, continue avoiding until time is up
        if self.state == "AVOID_OBSTACLE":
            if time.time() - self.avoid_start_time < 1.0: # Turn for 1 full second
                cmd.linear.x = -0.1 # Back up slightly
                cmd.angular.z = 0.8 # Turn hard left
            else:
                self.state = "SEARCH" # Done turning, go back to searching
                
        # PRIORITY 2: NEW OBSTACLE DETECTED
        elif self.front_dist < 0.6: # Wall is close!
            print("Wall detected! Switching to AVOID mode.")
            self.state = "AVOID_OBSTACLE"
            self.avoid_start_time = time.time()
            
        # PRIORITY 3: HUNT CHEESE
        elif self.cheese_visible:
            self.state = "HUNT_CHEESE"
            print("🧀 CHEESE DETECTED! CHASING!")
            cmd.linear.x = 0.3 # Charge!
            # Steer towards cheese (-0.005 is a gain factor, adjust as needed)
            cmd.angular.z = -0.005 * self.cheese_error 
            
        # PRIORITY 4: RANDOM SEARCH
        else:
            self.state = "SEARCH"
            cmd.linear.x = 0.4 # Drive forward
            # Slight wander to avoid getting stuck in perfect straight lines
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