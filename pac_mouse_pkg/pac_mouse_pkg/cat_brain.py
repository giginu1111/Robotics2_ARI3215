import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan  # NEW: We need to see!
from transforms3d.euler import quat2euler
import math

class CatBrain(Node):
    def __init__(self):
        super().__init__('cat_brain')

        # 1. DATA INPUTS
        self.sub_cat = self.create_subscription(Odometry, '/cat/odom', self.update_cat_pose, 10)
        self.sub_mouse = self.create_subscription(Odometry, '/mouse/odom', self.update_mouse_pose, 10)
        
        # NEW: Listen to Lidar
        self.sub_lidar = self.create_subscription(LaserScan, '/cat/scan', self.lidar_callback, 10)
        # 2. MOTOR OUTPUT
        self.publisher_ = self.create_publisher(Twist, '/cat/cmd_vel', 10)

        # Variables
        self.cat_pose = None
        self.mouse_pose = None
        self.lidar_ranges = []
        
        # Patrol Waypoints
        self.waypoints = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        self.current_wp_index = 0
        
        # Brain Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Safe Cat Brain Active!")

    def update_cat_pose(self, msg): self.cat_pose = msg.pose.pose
    def update_mouse_pose(self, msg): self.mouse_pose = msg.pose.pose
    def lidar_callback(self, msg): self.lidar_ranges = msg.ranges

    def get_yaw(self, pose):
        q = pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        return yaw

    def control_loop(self):
        if self.cat_pose is None or self.mouse_pose is None or not self.lidar_ranges:
            return

        twist = Twist()
        cx, cy = self.cat_pose.position.x, self.cat_pose.position.y
        cat_yaw = self.get_yaw(self.cat_pose)

        # --- LAYER 1: OBSTACLE AVOIDANCE (High Priority) ---
        # The Lidar array has 720 points. Index 360 is dead center front.
        # We check a "wedge" in front of the cat (indices 300 to 420).
        front_ranges = self.lidar_ranges[300:420]
        
        # Filter out "inf" (infinite) distances which mean "nothing seen"
        valid_front = [r for r in front_ranges if r < 10.0] 
        
        wall_ahead = False
        if len(valid_front) > 0:
            min_dist = min(valid_front)
            if min_dist < 0.6: # If wall is closer than 0.6 meters
                wall_ahead = True

        if wall_ahead:
            self.get_logger().warn("WALL DETECTED! Turning away!")
            twist.linear.x = 0.0      # Stop forward
            twist.angular.z = 1.0     # Spin left to find path
            self.publisher_.publish(twist)
            return  # SKIP the rest of the logic! Safety first!

        # --- LAYER 2: CHASE / PATROL (Low Priority) ---
        mx, my = self.mouse_pose.position.x, self.mouse_pose.position.y
        dist_to_mouse = math.sqrt((mx - cx)**2 + (my - cy)**2)
        
        target_x, target_y = 0.0, 0.0
        speed = 0.0

        if dist_to_mouse < 3.0:
            # CHASE MODE
            target_x, target_y = mx, my
            speed = 1.5
            self.get_logger().info(f"Chasing! Dist: {dist_to_mouse:.2f}")
        else:
            # PATROL MODE
            target_x, target_y = self.waypoints[self.current_wp_index]
            dist_to_wp = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
            speed = 0.5
            if dist_to_wp < 0.5:
                self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)

        # Drive Math
        target_angle = math.atan2(target_y - cy, target_x - cx)
        
        # Calculate Error
        error_yaw = target_angle - cat_yaw
        while error_yaw > math.pi: error_yaw -= 2*math.pi
        while error_yaw < -math.pi: error_yaw += 2*math.pi

        # Turn logic
        twist.angular.z = 2.0 * error_yaw
        
        # Only drive if facing target
        if abs(error_yaw) < 0.5:
            twist.linear.x = speed
        else:
            twist.linear.x = 0.0 # Stop to turn tightly

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CatBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()