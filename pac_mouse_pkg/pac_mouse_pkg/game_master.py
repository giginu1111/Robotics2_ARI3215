"""
=============================================================================
GAME MASTER - PAC-MOUSE SIMULATION REFEREE
=============================================================================
OVERVIEW:
This node acts as the game referee, managing game state, scoring, win/loss
conditions, and performance metrics for the cat and mouse simulation.

RESPONSIBILITIES:
Track positions of cat and mouse
Detect capture events (cat catches mouse)
Monitor cheese collection
Enforce power mode rules
Calculate performance metrics
Display real-time game statistics
Handle game termination conditions

GAME RULES:
Mouse must collect all cheese before being caught
Each cheese collected adds to score
After collecting all cheese, POWER MODE activates
In POWER MODE, mouse can eliminate the cat
Cat wins if it catches mouse before power mode
Mouse wins if it survives until power mode and catches cat

METRICS TRACKED:
Time elapsed
Cheeses collected
Distance traveled by each robot
Number of close encounters
Average speeds
Survival time
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math
import subprocess
import time


class GameMaster(Node):
    
    def __init__(self):
        super().__init__('game_master',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION
        # ====================================================================
        self.capture_radius = 0.35
        self.total_cheeses = 4
        self.world_name = 'pac_mouse_maze'
        self.time_limit = 9999.0
        self.encounter_dist = 2.5
        self.power_mode_enabled = True
        
        # ====================================================================
        # GAME STATE
        # ====================================================================
        self.game_active = True
        self.game_started = False
        self.game_start_time = None
        self.game_end_time = None
        self.winner = None
        
        self.power_mode = False
        self.power_mode_start_time = None
        
        self.cheeses_collected = 0
        self.cheese_collection_times = []
        
        # ====================================================================
        # ROBOT STATE
        # ====================================================================
        self.mouse_pose = None
        self.mouse_last_pose = None
        self.mouse_total_distance = 0.0
        
        self.cat_pose = None
        self.cat_last_pose = None
        self.cat_total_distance = 0.0
        
        self.current_distance = float('inf')
        self.min_distance_achieved = float('inf')
        self.close_encounters = 0
        self.last_encounter_logged = False
        
        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(Odometry, '/mouse/odom', self.mouse_odometry_callback, 10)
        self.create_subscription(Odometry, '/cat/odom', self.cat_odometry_callback, 10)
        self.create_subscription(String, '/cheese_eaten', self.cheese_eaten_callback, 10)
        
        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        self.mouse_cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.cat_cmd_pub = self.create_publisher(Twist, '/cat/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/game_status', 10)
        
        # ====================================================================
        # TIMERS
        # ====================================================================
        self.game_timer = self.create_timer(0.1, self.game_loop)
        self.stats_timer = self.create_timer(2.0, self.display_statistics)
        
        self.event_log = []
        self.log_event("SYSTEM", "Game Master Initialized")
        self.print_startup_banner()
    
    def mouse_odometry_callback(self, msg):
        current_pose = msg.pose.pose.position
        
        if not self.game_started and self.cat_pose is not None:
            self.game_started = True
            self.game_start_time = self.get_clock().now()
            self.log_event("GAME", "Game Started!")
        
        if self.mouse_last_pose is not None:
            dx = current_pose.x - self.mouse_last_pose.x
            dy = current_pose.y - self.mouse_last_pose.y
            distance = math.hypot(dx, dy)
            self.mouse_total_distance += distance
        
        self.mouse_last_pose = current_pose
        self.mouse_pose = current_pose
    
    def cat_odometry_callback(self, msg):
        current_pose = msg.pose.pose.position
        
        if self.cat_last_pose is not None:
            dx = current_pose.x - self.cat_last_pose.x
            dy = current_pose.y - self.cat_last_pose.y
            distance = math.hypot(dx, dy)
            self.cat_total_distance += distance
        
        self.cat_last_pose = current_pose
        self.cat_pose = current_pose
    
    def cheese_eaten_callback(self, msg):
        self.cheeses_collected += 1
        current_time = self.get_clock().now()
        
        if self.game_start_time:
            elapsed = (current_time - self.game_start_time).nanoseconds / 1e9
            self.cheese_collection_times.append(elapsed)
        
        remaining = self.total_cheeses - self.cheeses_collected
        self.log_event("CHEESE", f"Collected {msg.data}! ({remaining} remaining)")
        
        if self.cheeses_collected >= self.total_cheeses and self.power_mode_enabled:
            self.activate_power_mode()
    
    def game_loop(self):
        if not self.game_active or self.mouse_pose is None or self.cat_pose is None:
            return
        
        dx = self.mouse_pose.x - self.cat_pose.x
        dy = self.mouse_pose.y - self.cat_pose.y
        self.current_distance = math.hypot(dx, dy)
        
        if self.current_distance < self.min_distance_achieved:
            self.min_distance_achieved = self.current_distance
        
        if self.current_distance < self.encounter_dist:
            if not self.last_encounter_logged:
                self.close_encounters += 1
                self.last_encounter_logged = True
                self.log_event("ENCOUNTER", 
                              f"Close encounter #{self.close_encounters} at {self.current_distance:.2f}m")
        else:
            self.last_encounter_logged = False
        
        if self.current_distance < self.capture_radius:
            self.handle_capture()
        
        if self.game_started and self.game_start_time:
            elapsed = (self.get_clock().now() - self.game_start_time).nanoseconds / 1e9
            if elapsed > self.time_limit:
                self.handle_timeout()
        
        self.publish_status()
    
    def handle_capture(self):
        if self.power_mode:
            self.end_game("MOUSE", "Mouse ate the cat in POWER MODE!")
            self.delete_gazebo_model("cat")
        else:
            self.end_game("CAT", f"Cat caught mouse at {self.current_distance:.2f}m!")
            self.delete_gazebo_model("mouse")
    
    def handle_timeout(self):
        if self.power_mode:
            self.end_game("MOUSE", "Mouse survived time limit in POWER MODE!")
        else:
            self.end_game("CAT", "Time limit reached - Cat wins by default!")
    
    def activate_power_mode(self):
        self.power_mode = True
        self.power_mode_start_time = self.get_clock().now()
        
        self.log_event("POWER_MODE", "⚡ POWER MODE ACTIVATED! ⚡")
        self.get_logger().warn("=" * 60)
        self.get_logger().warn("⚡ POWER MODE ACTIVATED! MOUSE CAN NOW EAT CAT! ⚡")
        self.get_logger().warn("=" * 60)
    
    def end_game(self, winner, reason):
        if not self.game_active:
            return
        
        self.game_active = False
        self.game_end_time = self.get_clock().now()
        self.winner = winner
        
        self.emergency_stop()
        self.log_event("GAME_OVER", reason)
        self.display_final_statistics(reason)
    
    def display_statistics(self):
        if not self.game_started or not self.game_active:
            return
        
        elapsed = (self.get_clock().now() - self.game_start_time).nanoseconds / 1e9
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("                    🎮 GAME STATISTICS 🎮")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Game Time:       {elapsed:.1f}s / {self.time_limit:.0f}s")
        self.get_logger().info(f"Cheese:          {self.cheeses_collected}/{self.total_cheeses}")
        self.get_logger().info(f"Power Mode:      {'⚡ ACTIVE' if self.power_mode else 'Inactive'}")
        self.get_logger().info("-" * 70)
        self.get_logger().info(f"Current Distance: {self.current_distance:.2f}m")
        self.get_logger().info(f"Min Distance:     {self.min_distance_achieved:.2f}m")
        self.get_logger().info(f"Close Encounters: {self.close_encounters}")
        self.get_logger().info("-" * 70)
        self.get_logger().info(f"🐭 Mouse Distance: {self.mouse_total_distance:.2f}m")
        self.get_logger().info(f"😾 Cat Distance:   {self.cat_total_distance:.2f}m")
        self.get_logger().info("=" * 70)
    
    def display_final_statistics(self, reason):
        elapsed = (self.game_end_time - self.game_start_time).nanoseconds / 1e9
        
        print("\n" + "=" * 70)
        print("                    🏁 GAME OVER 🏁")
        print("=" * 70)
        print(f"Winner:          {self.winner}")
        print(f"Reason:          {reason}")
        print("=" * 70)
        print(f"Total Game Time:     {elapsed:.2f} seconds")
        print(f"Cheese Collected:    {self.cheeses_collected}/{self.total_cheeses}")
        print(f"Power Mode Reached:  {'Yes' if self.power_mode else 'No'}")
        print("-" * 70)
        print("PERFORMANCE METRICS:")
        print(f"  🐭 Mouse:")
        print(f"     Total Distance:   {self.mouse_total_distance:.2f}m")
        print(f"     Avg Speed:        {self.mouse_total_distance/elapsed:.2f}m/s")
        print(f"  😾 Cat:")
        print(f"     Total Distance:   {self.cat_total_distance:.2f}m")
        print(f"     Avg Speed:        {self.cat_total_distance/elapsed:.2f}m/s")
        print("-" * 70)
        print("ENCOUNTERS:")
        print(f"  Close Encounters:    {self.close_encounters}")
        print(f"  Minimum Distance:    {self.min_distance_achieved:.2f}m")
        print("=" * 70)
    
    def publish_status(self):
        if not self.game_start_time:
            return
        
        elapsed = (self.get_clock().now() - self.game_start_time).nanoseconds / 1e9
        status = f"Time:{elapsed:.1f}|Cheese:{self.cheeses_collected}/{self.total_cheeses}|"
        status += f"Dist:{self.current_distance:.2f}|PowerMode:{self.power_mode}"
        self.status_pub.publish(String(data=status))
    
    def emergency_stop(self):
        stop_cmd = Twist()
        self.mouse_cmd_pub.publish(stop_cmd)
        self.cat_cmd_pub.publish(stop_cmd)
        self.get_logger().info("🛑 Emergency stop issued")
    
    def delete_gazebo_model(self, model_name):
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{model_name}" type: MODEL'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True, check=True)
            self.get_logger().info(f"✅ Deleted {model_name}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Failed to delete {model_name}: {e}")
    
    def log_event(self, category, message):
        event_str = f"[{category}] {message}"
        self.event_log.append(event_str)
        
        if category in ["GAME_OVER", "POWER_MODE"]:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
    
    def print_startup_banner(self):
        print("\n" + "=" * 70)
        print("                    🎮 PAC-MOUSE GAME MASTER 🎮")
        print("=" * 70)
        print(f"Course:              Robotics 2 (ARI3215)")
        print(f"Project:             Autonomous Cat and Mouse Chase")
        print("=" * 70)
        print("GAME CONFIGURATION:")
        print(f"  Total Cheese:      {self.total_cheeses}")
        print(f"  Capture Radius:    {self.capture_radius}m")
        print(f"  Time Limit:        {self.time_limit}s")
        print(f"  Power Mode:        {'Enabled' if self.power_mode_enabled else 'Disabled'}")
        print("=" * 70 + "\n")
        
        self.get_logger().info("🎮 Game Master is ready!")


def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n⚠️  Game interrupted by user ⚠️\n")
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()