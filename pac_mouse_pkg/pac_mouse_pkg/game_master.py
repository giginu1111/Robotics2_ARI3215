import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import subprocess

class GameMaster(Node):
    def __init__(self):
        super().__init__('game_master')
        
        # SETTINGS
        self.catch_radius = 0.4
        self.total_cheeses_needed = 1 # Update this if you add more cheese!
        self.cheeses_collected = 0
        self.power_mode = False
        self.game_active = True

        # NOTE: Make sure this matches what you used in the mouse brain!
        self.world_name = "pac_mouse_maze"

        # SUBSCRIBERS
        self.sub_mouse = self.create_subscription(Odometry, '/mouse/odom', self.update_mouse_pos, 10)
        self.sub_cat   = self.create_subscription(Odometry, '/cat/odom',   self.update_cat_pos, 10)
        self.sub_score = self.create_subscription(String,  '/cheese_eaten', self.cheese_callback, 10)

        # PUBLISHERS
        self.pub_mouse_vel = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.pub_cat_vel   = self.create_publisher(Twist, '/cat/cmd_vel', 10)

        self.mouse_pose = None
        self.cat_pose = None
        self.timer = self.create_timer(0.1, self.game_loop)

    def update_mouse_pos(self, msg): self.mouse_pose = msg.pose.pose.position
    def update_cat_pos(self, msg): self.cat_pose = msg.pose.pose.position

    def cheese_callback(self, msg):
        self.cheeses_collected += 1
        remaining = self.total_cheeses_needed - self.cheeses_collected
        self.get_logger().info(f"Cheese Eaten! {remaining} left to POWER MODE.")

        if self.cheeses_collected >= self.total_cheeses_needed:
            self.power_mode = True
            self.get_logger().warn(">>> POWER MODE ACTIVATED! MOUSE CAN NOW EAT CAT! <<<")

    def game_loop(self):
        if not self.game_active or not self.mouse_pose or not self.cat_pose: return

        dx = self.mouse_pose.x - self.cat_pose.x
        dy = self.mouse_pose.y - self.cat_pose.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.catch_radius:
            if self.power_mode:
                self.victory_mouse() # Mouse wins!
            else:
                self.victory_cat(dist) # Cat wins!

    def victory_cat(self, distance):
        self.game_active = False
        self.get_logger().error(f"GAME OVER! Cat caught Mouse at {distance:.2f}m")
        self.force_stop()
        self.delete_mouse()

    def victory_mouse(self):
        self.game_active = False
        self.get_logger().info("VICTORY! Mouse ate the Cat!")
        self.force_stop()
        self.delete_cat()
    
    def delete_cat(self):
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", 'name: "cat" type: MODEL'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True)
            self.get_logger().info(">>> CAT ELIMINATED FROM WORLD <<<")
            
        except Exception as e:
            self.get_logger().error(f"Failed to delete cat: {e}")

    def delete_mouse(self):
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", 'name: "mouse" type: MODEL'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True)
            self.get_logger().error(">>> MOUSE ELIMINATED. GAME OVER. <<<")
        except Exception as e:
            self.get_logger().error(f"Failed to delete mouse: {e}")

    def force_stop(self):
        stop = Twist()
        self.pub_mouse_vel.publish(stop)
        self.pub_cat_vel.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()