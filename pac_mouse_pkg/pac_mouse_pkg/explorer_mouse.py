import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import subprocess
import random
import math
import time

class ExplorerMouse(Node):
    def __init__(self):
        super().__init__('mouse_navigator')
        
        # 1. Action Client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. Publisher to tell Referee we ate cheese
        self.pub_score = self.create_publisher(String, '/cheese_eaten', 10)

        # 3. Subscribe to Odom to know WHERE WE ARE (The "Proprioception")
        self.create_subscription(Odometry, '/mouse/odom', self.pose_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0

        # 4. Hidden "Ground Truth" of Cheese Locations
        # The mouse doesn't "know" these for navigation, 
        # but uses them to check if it's close enough to "eat".
        self.hidden_cheeses = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'},
            {'x': -5.0, 'y': -5.0, 'name': 'cheese_4'} # Added one more!
        ]
        
        # 5. Timer to "Smell" cheese (Checks every 0.5 seconds)
        self.create_timer(0.5, self.check_for_cheese)

        # Flag to prevent spamming goals
        self.is_navigating = False
        self.goal_handle = None

        self.get_logger().info("Smart Mouse: EXPLORER MODE ACTIVATED!")
        # Give Nav2 a moment to wake up
        time.sleep(2.0)
        self.pick_random_goal()

    def pose_callback(self, msg):
        # Update our current position constantly
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def check_for_cheese(self):
        """
        This acts as the mouse's 'Nose'. 
        If we are close to a cheese, we eat it immediately, 
        even if we haven't reached our navigation goal.
        """
        detection_radius = 0.6  # How close to get before eating (0.6 meters)

        for cheese in self.hidden_cheeses[:]: # Iterate over copy to allow removal
            dist = math.sqrt((self.current_x - cheese['x'])**2 + (self.current_y - cheese['y'])**2)
            
            if dist < detection_radius:
                self.get_logger().info(f"!!! I SMELL CHEESE: {cheese['name']} !!!")
                self.eat_cheese(cheese)

    def eat_cheese(self, cheese_dict):
        # 1. Cancel current navigation (Stop the mouse)
        if self.goal_handle:
            self.get_logger().info("Stopping to eat...")
            # Ideally we would cancel the goal here, but for simplicity 
            # we just delete the cheese and let the mouse continue 
            # or pick a new goal in the next cycle.

        # 2. Delete from Simulation
        self.delete_cheese_model(cheese_dict['name'])
        
        # 3. Remove from our hidden list so we don't try to eat it again
        if cheese_dict in self.hidden_cheeses:
            self.hidden_cheeses.remove(cheese_dict)

        # 4. Check if we won
        if not self.hidden_cheeses:
            self.get_logger().info("ALL CHEESE EATEN! VICTORY!")
            # Stop the node? Or hunt cats?
            
    def pick_random_goal(self):
        """
        Generates a random coordinate within the maze bounds.
        """
        # Maze bounds (Adjust these based on your map size!)
        min_x, max_x = -8.0, 8.0
        min_y, max_y = -8.0, 8.0

        rand_x = random.uniform(min_x, max_x)
        rand_y = random.uniform(min_y, max_y)

        self.get_logger().info(f"Exploring to random spot: ({rand_x:.2f}, {rand_y:.2f})...")
        self.send_goal(rand_x, rand_y)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected (probably inside a wall). Picking new spot...')
            self.pick_random_goal()
            return

        self.get_logger().info('Goal accepted. Moving...')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        # Whether we succeeded (reached random spot) or failed (got stuck),
        # we just pick a new random spot to keep exploring.
        if status == 4:
            self.get_logger().info('Finished exploring this spot. Moving to next...')
        else:
            self.get_logger().warn('Could not reach spot. Trying somewhere else...')
        
        # Small pause before next move
        time.sleep(1.0)
        self.pick_random_goal()

    def delete_cheese_model(self, cheese_name):
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
            msg = String()
            msg.data = cheese_name
            self.pub_score.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to eat cheese: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerMouse()
    rclpy.spin(node)