import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class SmartMouse(Node):
    def __init__(self):
        super().__init__('smart_mouse')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop) 

        # State Variables
        self.cheese_visible = False
        self.cheese_error = 0 
        
        # Sensor Data (Initialized to 'inf' equivalent)
        self.front = 10.0
        self.right = 10.0
        self.front_right = 10.0

        print("🐭 Wall-Following Mouse Initialized!")

    def scan_callback(self, msg):
        ranges = msg.ranges
        size = len(ranges)
        
        def get_range(index):
            if 0 <= index < size:
                r = ranges[index]
                if r == float('inf') or r == 0.0: return 10.0
                return r
            return 10.0

        # Lidar Configuration (Index 180 is Front)
        idx_front = size // 2
        idx_right = int(size * 0.25)       # -90 degrees
        idx_front_right = int(size * 0.35) # -45 degrees (Diag)

        # Average a few points for stability
        self.front = min([get_range(i) for i in range(idx_front-5, idx_front+5)])
        self.right = min([get_range(i) for i in range(idx_right-5, idx_right+5)])
        self.front_right = min([get_range(i) for i in range(idx_front_right-5, idx_front_right+5)])

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Yellow Cheese
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
        
        # --- PRIORITY 1: CHEESE HUNT ---
        if self.cheese_visible:
            if self.front < 0.2:
                print("🧀 EATING CHEESE!")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                print("🧀 CHARGING!")
                cmd.linear.x = 0.3 
                cmd.angular.z = -0.005 * self.cheese_error 
            self.publisher_.publish(cmd)
            return

        # --- PRIORITY 2: WALL FOLLOWING (Right Hand Rule) ---
        # Logic adapted for "Front=180" lidar
        
        safe_dist = 0.4
        
        # Case A: Wall directly in front? -> Turn Left hard
        if self.front < safe_dist:
            print("Turn Left (Front Blocked)")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8
            
        # Case B: Too close to right wall? -> Nudge Left
        elif self.right < 0.2:
            print("Nudge Left (Too Close)")
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3 # Turn away from wall

        # Case C: Good distance from right wall? -> Go Straight
        elif self.right < 0.5:
            print("Follow Wall")
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        # Case D: Wall lost/Too far? -> Curve Right to find it
        else:
            print("Find Wall (Curve Right)")
            cmd.linear.x = 0.2
            cmd.angular.z = -0.4 # Curve right

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SmartMouse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()