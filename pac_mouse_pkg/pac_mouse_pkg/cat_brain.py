import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
import math

class CatBrain(Node):
    def __init__(self):
        super().__init__('cat_brain_apf')

        # --- TOPICS ---
        self.sub_cat = self.create_subscription(Odometry, '/cat/odom', self.update_cat_pose, 10)
        self.sub_mouse = self.create_subscription(Odometry, '/mouse/odom', self.update_mouse_pose, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/cat/scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cat/cmd_vel', 10)

        # Variables
        self.cat_pose = None
        self.mouse_pose = None
        self.lidar_ranges = []
        
        # Lidar config (will be auto-detected)
        self.angle_min = -3.14
        self.angle_increment = 0.0087 # approx
        
        self.timer = self.create_timer(0.05, self.control_loop) # Faster loop (20Hz)
        self.get_logger().info("Potential Field Brain Active!")

    def update_cat_pose(self, msg): self.cat_pose = msg.pose.pose
    def update_mouse_pose(self, msg): self.mouse_pose = msg.pose.pose
    
    def lidar_callback(self, msg): 
        self.lidar_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def get_yaw(self, pose):
        q = pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        return yaw

    def control_loop(self):
        if self.cat_pose is None or self.mouse_pose is None or not self.lidar_ranges:
            return

        # 1. CAT POSITION
        cx = self.cat_pose.position.x
        cy = self.cat_pose.position.y
        cat_yaw = self.get_yaw(self.cat_pose)

        # 2. ATTRACTION VECTOR (Pull towards Mouse)
        mx = self.mouse_pose.position.x
        my = self.mouse_pose.position.y
        
        vec_x = mx - cx
        vec_y = my - cy
        dist_to_mouse = math.sqrt(vec_x**2 + vec_y**2)

        # Normalize and weigh attraction
        # (We want the cat to want the mouse badly, so weight = 2.0)
        if dist_to_mouse > 0:
            attract_x = (vec_x / dist_to_mouse) * 2.0
            attract_y = (vec_y / dist_to_mouse) * 2.0
        else:
            attract_x, attract_y = 0.0, 0.0

        # 3. REPULSION VECTOR (Push away from Walls)
        repulse_x = 0.0
        repulse_y = 0.0
        
        # Check all lidar rays
        for i, r in enumerate(self.lidar_ranges):
            # Only care about things closer than 1.0 meter
            if r < 1.0 and r > 0.05:
                # Calculate the angle of this specific ray relative to the robot
                angle = self.angle_min + (i * self.angle_increment)
                
                # The "Force" is strong when close, weak when far (1/r^2)
                force = 1.0 / (r * r)
                
                # The direction of the force is OPPOSITE to the obstacle
                # Since 'angle' is where the obstacle is, 'angle + pi' is away
                # We need to rotate this into the GLOBAL map frame
                # Global Angle = Robot Yaw + Ray Angle + PI (180 deg)
                push_angle = cat_yaw + angle + math.pi
                
                repulse_x += math.cos(push_angle) * force
                repulse_y += math.sin(push_angle) * force

        # Weight the repulsion (Make walls SCARY!)
        # Increase this number (0.02) if the cat still hits walls
        repulsion_strength = 0.02 
        repulse_x *= repulsion_strength
        repulse_y *= repulsion_strength

        # 4. SUM OF FORCES
        final_x = attract_x + repulse_x
        final_y = attract_y + repulse_y

        # 5. DRIVE ALONG THE RESULTANT VECTOR
        target_angle = math.atan2(final_y, final_x)
        
        # Calculate steering error
        error_yaw = target_angle - cat_yaw
        while error_yaw > math.pi: error_yaw -= 2*math.pi
        while error_yaw < -math.pi: error_yaw += 2*math.pi

        twist = Twist()
        
        # Turn logic
        twist.angular.z = 2.5 * error_yaw
        
        # Drive logic (Slow down if turning sharply)
        if abs(error_yaw) < 1.0:
            # Base speed 1.0, but slow down if obstacles are pushing us
            twist.linear.x = 1.0 / (1.0 + abs(twist.angular.z))
        else:
            twist.linear.x = 0.0

        # Stop if we caught the mouse
        if dist_to_mouse < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("GOTCHA!")

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CatBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()