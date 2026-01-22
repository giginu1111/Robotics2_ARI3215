import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import subprocess

class SmartMouse(Node):
    def __init__(self):
        super().__init__('mouse_navigator')
        
        # 1. Action Client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. Publisher to tell Referee we ate cheese
        self.pub_score = self.create_publisher(String, '/cheese_eaten', 10)

        # 3. Define Cheese Locations (Ideally this comes from Vision, but hardcode for now)
        self.cheese_list = [
            {'x': 3.5, 'y': 3.5, 'name': 'cheese_1'},
            {'x': -2.0, 'y': 2.0, 'name': 'cheese_2'},
            {'x': 5.5, 'y': 5.5, 'name': 'cheese_3'} 
        ]
        self.current_cheese_index = 0
        
        self.get_logger().info("Smart Mouse: Waiting for Nav2...")
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_cheese_index >= len(self.cheese_list):
            self.get_logger().info("ALL CHEESE COLLECTED! HUNTING CAT!")
            # Logic to chase cat goes here (or switch state)
            return

        target = self.cheese_list[self.current_cheese_index]
        self.get_logger().info(f"Navigating to {target['name']} at ({target['x']}, {target['y']})...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.orientation.w = 1.0 # Face forward

        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Path planning in progress...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        # STATUS_SUCCEEDED is 4 in ROS 2 action definitions
        if status == 4:
            self.get_logger().info('ARRIVED AT CHEESE!')
            
            # Eat the cheese
            target = self.cheese_list[self.current_cheese_index]
            self.delete_cheese(target['name'])
            
            # Move to next
            self.current_cheese_index += 1
            self.send_next_goal()
        else:
            self.get_logger().info('Failed to reach the goal for some reason.')
            self.send_next_goal()

    def delete_cheese(self, cheese_name):
        # NOTE: Verify your world name!
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
    node = SmartMouse()
    rclpy.spin(node)