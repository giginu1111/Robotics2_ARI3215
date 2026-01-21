import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
from std_msgs.msg import String
import math
import subprocess  # Used to delete cheese

class MouseBrain(Node):
    def __init__(self):
        super().__init__('mouse_brain')

        # --- TOPICS ---
        self.sub_mouse = self.create_subscription(Odometry, '/mouse/odom', self.update_pose, 10)
        self.sub_cat = self.create_subscription(Odometry, '/cat/odom', self.update_cat_pose, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/mouse/scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/mouse/cmd_vel', 10)

        self.pub_score = self.create_publisher(String, '/cheese_eaten', 10)

        # --- GAME STATE ---
        # List of Cheese Locations (x, y, name)
        # You must update these coordinates to match where you put them in Gazebo!
        self.cheeses = [
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_1'}
        ]
        
        self.mouse_pose = None
        self.cat_pose = None
        self.lidar_ranges = []
        self.current_speed = 0.0 # For acceleration ramping

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Mouse Brain Active! Hunting for cheese...")

    def update_pose(self, msg): self.mouse_pose = msg.pose.pose
    def update_cat_pose(self, msg): self.cat_pose = msg.pose.pose
    def lidar_callback(self, msg): 
        self.lidar_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def get_yaw(self, pose):
        q = pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        return yaw

    def delete_cheese(self, cheese_name):
        # This command sends a request to Gazebo to remove the model
        # NOTE: Verify your world name! (Default is usually 'default' or 'world_demo')
        # You can check world name by running: gz service -l | grep world
        world_name = "pac_mouse_maze" 
        
        cmd = [
            "gz", "service", "-s", f"/world/{world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{cheese_name}" type: MODEL'
        ]
        try:
            subprocess.run(cmd, capture_output=True)
            self.get_logger().info(f"ATE {cheese_name}!")
            # NOTIFY GAME MASTER
            msg = String()
            msg.data = cheese_name
            self.pub_score.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to eat cheese: {e}")

    def control_loop(self):
        if self.mouse_pose is None or not self.lidar_ranges:
            return

        mx = self.mouse_pose.position.x
        my = self.mouse_pose.position.y
        mouse_yaw = self.get_yaw(self.mouse_pose)

        # 1. CALCULATE VECTORS
        final_x, final_y = 0.0, 0.0

        # A. FEAR OF CAT (High Priority)
        if self.cat_pose:
            cx = self.cat_pose.position.x
            cy = self.cat_pose.position.y
            dist_cat = math.sqrt((mx-cx)**2 + (my-cy)**2)
            
            # If cat is close (within 3m), RUN AWAY!
            if dist_cat < 3.0:
                # Vector pointing FROM cat TO mouse
                flee_x = mx - cx
                flee_y = my - cy
                # Normalize and Weight heavily
                flee_x = (flee_x / dist_cat) * 4.0 
                flee_y = (flee_y / dist_cat) * 4.0
                
                final_x += flee_x
                final_y += flee_y
                self.get_logger().warn("CAT DETECTED! RUN!", throttle_duration_sec=1)

        # B. HUNGER FOR CHEESE (Medium Priority)
        # Find closest cheese
        closest_cheese = None
        min_dist = 999.0
        
        for cheese in self.cheeses:
            dx = cheese['x'] - mx
            dy = cheese['y'] - my
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_cheese = cheese

        if closest_cheese:
            # Check if close enough to EAT
            if min_dist < 0.4:
                self.delete_cheese(closest_cheese['name'])
                self.cheeses.remove(closest_cheese) # Remove from memory
                return # Stop for a split second while eating
            
            # Attraction Vector
            attract_x = (closest_cheese['x'] - mx) / min_dist * 1.5
            attract_y = (closest_cheese['y'] - my) / min_dist * 1.5
            
            final_x += attract_x
            final_y += attract_y
        else:
            # No cheese left? Victory spin?
            final_x = 0.0 # Just stop (or wander)

        # C. FEAR OF WALLS (Safety Priority)
        # (Same logic as Cat Brain)
        repulse_x, repulse_y = 0.0, 0.0
        for i, r in enumerate(self.lidar_ranges):
            if r < 0.3 and r > 0.05:
                angle = self.angle_min + (i * self.angle_increment)
                force = 1.0 / (r * r)
                push_angle = mouse_yaw + angle + math.pi
                repulse_x += math.cos(push_angle) * force
                repulse_y += math.sin(push_angle) * force
        
        # Wall fear strength
        repulse_x *= 0.01
        repulse_y *= 0.01
        
        final_x += repulse_x
        final_y += repulse_y

        # 2. DRIVE LOGIC
        target_angle = math.atan2(final_y, final_x)
        error_yaw = target_angle - mouse_yaw
        while error_yaw > math.pi: error_yaw -= 2*math.pi
        while error_yaw < -math.pi: error_yaw += 2*math.pi

        twist = Twist()
        twist.angular.z = 3.0 * error_yaw

        # 3. ACCELERATION RAMP (Anti-Wheelie)
        # We calculate desired speed based on turn sharpness
        if abs(error_yaw) < 1.0:
            target_speed = 1.2 # Max speed
        else:
            target_speed = 0.0 # Stop to turn

        # Smoothly change current_speed towards target_speed
        accel_step = 0.1 # How much speed can change per loop (0.1s)
        
        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed + accel_step, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed - accel_step, target_speed)

        twist.linear.x = self.current_speed
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MouseBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()