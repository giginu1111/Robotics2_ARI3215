import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class PacMouseNode(Node):
    def __init__(self):
        super().__init__('pac_mouse_brain')

        # --- PERCEPTION (Subscribers) ---
        # Lidar for walls
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Camera for Cheese (Yellow) and Cat (Red)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        # --- ACTION (Publisher) ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- TOOLS ---
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz Decision Cycle

        # --- INTERNAL STATE ---
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        
        self.cheese_detected = False
        self.cheese_center_x = 0
        
        # State: 'EXPLORE', 'CHASE_CHEESE'
        self.state = 'EXPLORE' 

    def scan_callback(self, msg):
        # The Lidar gives us 360 values. We simplify this into 3 zones.
        # Front is index 0, Left is ~90, Right is ~270 (in a 360 array)
        # Note: Depending on the lidar hardware, 0 might be back. 
        # For Gazebo default, 0 is usually front.
        ranges = msg.ranges
        size = len(ranges)
        
        # 1. Handle 'inf' or '0.0'
        def clean_data(r):
            if r == float('inf'): return 10.0
            if r == 0.0: return 10.0
            return r

        # 2. FIX: The Front is in the MIDDLE of the array (Index 180)
        # Because the scan goes from -Pi (Back) to +Pi (Back)
        mid_point = size // 2  # Approx 180
        
        # Take a small average around the center point
        front_indices = [mid_point-2, mid_point-1, mid_point, mid_point+1, mid_point+2]
        valid_front = [clean_data(ranges[i]) for i in front_indices]
        self.front_dist = sum(valid_front) / len(valid_front)
        
        # 3. Left and Right need to shift too
        # Left is +90 deg (approx index 270)
        # Right is -90 deg (approx index 90)
        self.left_dist = clean_data(ranges[int(size * 0.75)]) 
        self.right_dist = clean_data(ranges[int(size * 0.25)])

        self.get_logger().info(f'Front: {self.front_dist:.3f}', throttle_duration_sec=0.5)

    def camera_callback(self, msg):
        # Convert ROS Image -> OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        # --- VISION PROCESSING ---
        # 1. Convert to HSV (Easier to detect colors)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Define Yellow Range (Cheese)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        
        # 3. Create Mask
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find the biggest yellow blob
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                self.cheese_center_x = cx
                self.cheese_detected = True
                
                # Optional: Draw on image for debugging (show in separate window)
                # cv2.circle(cv_image, (cx, 240), 10, (0, 255, 0), -1)
                # cv2.imshow("Mouse Vision", cv_image)
                # cv2.waitKey(1)
            else:
                self.cheese_detected = False
        else:
            self.cheese_detected = False

    def control_loop(self):
        msg = Twist()

        # --- REASONING (State Machine) ---
        
        # Priority 1: Do we see Cheese? -> Visual Servoing
        if self.cheese_detected:
            self.state = 'CHASE_CHEESE'
            self.get_logger().info('CHEESE SPOTTED! HUNGRY!', throttle_duration_sec=1)
            
            # Simple Proportional Controller
            # Image is 640px wide. Center is 320.
            error = 320 - self.cheese_center_x
            
            # Turn towards cheese
            msg.angular.z = 0.005 * error 
            
            # Move forward (but stop if very close to wall/cheese)
            if self.front_dist > 0.3:
                msg.linear.x = 0.2
            else:
                msg.linear.x = 0.0 # Eat it (Stop)
        
        # Priority 2: Explore (Wall Follow / Avoidance)
        else:
            self.state = 'EXPLORE'
            
            if self.front_dist < 0.6:
                # WALL AHEAD! Turn away.
                msg.linear.x = 0.0
                msg.angular.z = 0.5 # Turn left
            else:
                # CLEAR PATH. Drive forward.
                msg.linear.x = 0.3
                msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PacMouseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()