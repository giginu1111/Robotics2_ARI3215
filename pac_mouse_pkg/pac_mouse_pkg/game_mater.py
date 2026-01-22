#!/usr/bin/env python3
"""
=============================================================================
GAME MASTER - PAC-MOUSE SIMULATION REFEREE
=============================================================================
Course: Robotics 2 (ARI3215)
Project: Pac-Mouse Autonomous Navigation

OVERVIEW:
This node acts as the game referee, managing game state, scoring, win/loss
conditions, and performance metrics for the cat and mouse simulation.

RESPONSIBILITIES:
1. Track positions of cat and mouse
2. Detect capture events (cat catches mouse)
3. Monitor cheese collection
4. Enforce power mode rules
5. Calculate performance metrics
6. Display real-time game statistics
7. Handle game termination conditions

GAME RULES:
- Mouse must collect all cheese before being caught
- Each cheese collected adds to score
- After collecting all cheese, POWER MODE activates
- In POWER MODE, mouse can eliminate the cat
- Cat wins if it catches mouse before power mode
- Mouse wins if it survives until power mode and catches cat

METRICS TRACKED:
- Time elapsed
- Cheeses collected
- Distance traveled by each robot
- Number of close encounters
- Average speeds
- Survival time

Author: Damian Cutajar
Date: January 2026
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# ROS Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker

# Processing
import math
import subprocess
import time
from datetime import datetime


class ProposalGameMaster(Node):
    """
    Game Master node for managing the Pac-Mouse simulation.
    Handles scoring, win conditions, and performance tracking.
    """
    
    def __init__(self):
        super().__init__('proposal_game_master',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION PARAMETERS
        # ====================================================================
        self.declare_parameter('capture_radius', 0.5)
        self.declare_parameter('total_cheese_count', 4)
        self.declare_parameter('world_name', 'pac_mouse_maze')
        self.declare_parameter('game_time_limit', 300.0)  # 5 minutes
        self.declare_parameter('close_encounter_distance', 1.5)
        self.declare_parameter('enable_power_mode', True)
        
        # Load parameters
        self.capture_radius = self.get_parameter('capture_radius').value
        self.total_cheeses = self.get_parameter('total_cheese_count').value
        self.world_name = self.get_parameter('world_name').value
        self.time_limit = self.get_parameter('game_time_limit').value
        self.encounter_dist = self.get_parameter('close_encounter_distance').value
        self.power_mode_enabled = self.get_parameter('enable_power_mode').value
        
        # ====================================================================
        # GAME STATE VARIABLES
        # ====================================================================
        # Game status
        self.game_active = True
        self.game_started = False
        self.game_start_time = None
        self.game_end_time = None
        self.winner = None  # 'MOUSE' or 'CAT' or 'TIMEOUT'
        
        # Power mode
        self.power_mode = False
        self.power_mode_start_time = None
        
        # Cheese tracking
        self.cheeses_collected = 0
        self.cheese_collection_times = []
        
        # ====================================================================
        # ROBOT STATE TRACKING
        # ====================================================================
        # Mouse state
        self.mouse_pose = None
        self.mouse_last_pose = None
        self.mouse_total_distance = 0.0
        self.mouse_velocity = 0.0
        
        # Cat state
        self.cat_pose = None
        self.cat_last_pose = None
        self.cat_total_distance = 0.0
        self.cat_velocity = 0.0
        
        # Distance between them
        self.current_distance = float('inf')
        self.min_distance_achieved = float('inf')
        self.close_encounters = 0
        self.last_encounter_logged = False
        
        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(
            Odometry, '/mouse/odom', self.mouse_odometry_callback, 10
        )
        self.create_subscription(
            Odometry, '/cat/odom', self.cat_odometry_callback, 10
        )
        self.create_subscription(
            String, '/cheese_eaten', self.cheese_eaten_callback, 10
        )
        
        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        self.mouse_cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.cat_cmd_pub = self.create_publisher(Twist, '/cat/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/game_status', 10)
        self.marker_pub = self.create_publisher(Marker, '/game_visualization', 10)
        
        # ====================================================================
        # TIMERS
        # ====================================================================
        # Main game loop
        self.game_timer = self.create_timer(0.1, self.game_loop)  # 10 Hz
        
        # Statistics display
        self.stats_timer = self.create_timer(2.0, self.display_statistics)
        
        # ====================================================================
        # LOGGING
        # ====================================================================
        self.event_log = []
        self.log_event("SYSTEM", "Game Master Initialized")
        
        # Print startup banner
        self.print_startup_banner()
    
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def mouse_odometry_callback(self, msg):
        """Tracks mouse position and calculates travel distance."""
        current_pose = msg.pose.pose.position
        
        # Start game on first movement
        if not self.game_started and self.cat_pose is not None:
            self.game_started = True
            self.game_start_time = self.get_clock().now()
            self.log_event("GAME", "Game Started!")
        
        # Calculate distance traveled
        if self.mouse_last_pose is not None:
            dx = current_pose.x - self.mouse_last_pose.x
            dy = current_pose.y - self.mouse_last_pose.y
            distance = math.hypot(dx, dy)
            self.mouse_total_distance += distance
        
        # Calculate velocity
        self.mouse_velocity = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        
        self.mouse_last_pose = current_pose
        self.mouse_pose = current_pose
    
    def cat_odometry_callback(self, msg):
        """Tracks cat position and calculates travel distance."""
        current_pose = msg.pose.pose.position
        
        # Calculate distance traveled
        if self.cat_last_pose is not None:
            dx = current_pose.x - self.cat_last_pose.x
            dy = current_pose.y - self.cat_last_pose.y
            distance = math.hypot(dx, dy)
            self.cat_total_distance += distance
        
        # Calculate velocity
        self.cat_velocity = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        
        self.cat_last_pose = current_pose
        self.cat_pose = current_pose
    
    def cheese_eaten_callback(self, msg):
        """Handles cheese collection events."""
        self.cheeses_collected += 1
        current_time = self.get_clock().now()
        
        # Record collection time
        if self.game_start_time:
            elapsed = (current_time - self.game_start_time).nanoseconds / 1e9
            self.cheese_collection_times.append(elapsed)
        
        remaining = self.total_cheeses - self.cheeses_collected
        
        # Log event
        self.log_event("CHEESE", f"Collected {msg.data}! ({remaining} remaining)")
        
        # Check for power mode activation
        if self.cheeses_collected >= self.total_cheeses and self.power_mode_enabled:
            self.activate_power_mode()
    
    # ========================================================================
    # MAIN GAME LOOP
    # ========================================================================
    
    def game_loop(self):
        """
        Main game logic executed at 10Hz.
        Checks win conditions and updates game state.
        """
        if not self.game_active:
            return
        
        # Wait for both robots to be spawned
        if self.mouse_pose is None or self.cat_pose is None:
            return
        
        # Calculate current distance
        dx = self.mouse_pose.x - self.cat_pose.x
        dy = self.mouse_pose.y - self.cat_pose.y
        self.current_distance = math.hypot(dx, dy)
        
        # Update minimum distance
        if self.current_distance < self.min_distance_achieved:
            self.min_distance_achieved = self.current_distance
        
        # Track close encounters
        if self.current_distance < self.encounter_dist:
            if not self.last_encounter_logged:
                self.close_encounters += 1
                self.last_encounter_logged = True
                self.log_event("ENCOUNTER", 
                              f"Close encounter #{self.close_encounters} at {self.current_distance:.2f}m")
        else:
            self.last_encounter_logged = False
        
        # Check for capture
        if self.current_distance < self.capture_radius:
            self.handle_capture()
        
        # Check for time limit
        if self.game_started and self.game_start_time:
            elapsed = (self.get_clock().now() - self.game_start_time).nanoseconds / 1e9
            if elapsed > self.time_limit:
                self.handle_timeout()
        
        # Publish game status
        self.publish_status()
    
    # ========================================================================
    # WIN CONDITION HANDLERS
    # ========================================================================
    
    def handle_capture(self):
        """Handles capture event based on power mode status."""
        if self.power_mode:
            # Mouse wins by eating cat
            self.end_game("MOUSE", "Mouse ate the cat in POWER MODE!")
            self.delete_gazebo_model("cat")
        else:
            # Cat wins by catching mouse
            self.end_game("CAT", f"Cat caught mouse at {self.current_distance:.2f}m!")
            self.delete_gazebo_model("mouse")
    
    def handle_timeout(self):
        """Handles game timeout."""
        if self.power_mode:
            # Mouse survived in power mode - mouse wins
            self.end_game("MOUSE", "Mouse survived time limit in POWER MODE!")
        else:
            # Time expired before collecting all cheese - cat wins
            self.end_game("CAT", "Time limit reached - Cat wins by default!")
    
    def activate_power_mode(self):
        """Activates power mode after all cheese collected."""
        self.power_mode = True
        self.power_mode_start_time = self.get_clock().now()
        
        self.log_event("POWER_MODE", "⚡ POWER MODE ACTIVATED! ⚡")
        self.get_logger().warn("=" * 60)
        self.get_logger().warn("⚡ POWER MODE ACTIVATED! MOUSE CAN NOW EAT CAT! ⚡")
        self.get_logger().warn("=" * 60)
        
        # Visual indication
        self.publish_power_mode_marker()
    
    def end_game(self, winner, reason):
        """
        Ends the game and displays final statistics.
        
        Args:
            winner (str): 'MOUSE', 'CAT', or 'TIMEOUT'
            reason (str): Description of win condition
        """
        if not self.game_active:
            return
        
        self.game_active = False
        self.game_end_time = self.get_clock().now()
        self.winner = winner
        
        # Stop all robots
        self.emergency_stop()
        
        # Log final event
        self.log_event("GAME_OVER", reason)
        
        # Display final statistics
        self.display_final_statistics(reason)
    
    # ========================================================================
    # STATISTICS AND DISPLAY
    # ========================================================================
    
    def display_statistics(self):
        """Displays real-time game statistics (called every 2 seconds)."""
        if not self.game_started or not self.game_active:
            return
        
        elapsed = (self.get_clock().now() - self.game_start_time).nanoseconds / 1e9
        
        # Clear terminal (optional - comment out if unwanted)
        # print("\033[2J\033[H")
        
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
        """Displays comprehensive final statistics."""
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
        
        # Cheese collection timeline
        if self.cheese_collection_times:
            print("CHEESE COLLECTION TIMELINE:")
            for i, t in enumerate(self.cheese_collection_times, 1):
                print(f"  Cheese #{i}: {t:.2f}s")
            print("=" * 70)
        
        # Event log
        print("\nEVENT LOG:")
        for event in self.event_log[-10:]:  # Last 10 events
            print(f"  {event}")
        print("=" * 70)
        
        self.get_logger().info("\n🎮 Thank you for playing Pac-Mouse! 🎮\n")
    
    def publish_status(self):
        """Publishes current game status as a string."""
        status = f"Time:{self.get_elapsed_time():.1f}|Cheese:{self.cheeses_collected}/{self.total_cheeses}|"
        status += f"Dist:{self.current_distance:.2f}|PowerMode:{self.power_mode}"
        
        self.status_pub.publish(String(data=status))
    
    def publish_power_mode_marker(self):
        """Publishes visualization marker for power mode."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "power_mode"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.text = "⚡ POWER MODE ⚡"
        
        self.marker_pub.publish(marker)
    
    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    
    def emergency_stop(self):
        """Immediately stops both robots."""
        stop_cmd = Twist()
        self.mouse_cmd_pub.publish(stop_cmd)
        self.cat_cmd_pub.publish(stop_cmd)
        self.get_logger().info("🛑 Emergency stop issued to all robots")
    
    def delete_gazebo_model(self, model_name):
        """
        Removes a model from Gazebo simulation.
        
        Args:
            model_name (str): Name of model to delete ('mouse' or 'cat')
        """
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{model_name}" type: MODEL'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True, check=True)
            self.get_logger().info(f"✅ Deleted {model_name} from simulation")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Failed to delete {model_name}: {e}")
    
    def log_event(self, category, message):
        """
        Logs a game event with timestamp.
        
        Args:
            category (str): Event category (e.g., 'CHEESE', 'ENCOUNTER')
            message (str): Event description
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        event_str = f"[{timestamp}] [{category}] {message}"
        self.event_log.append(event_str)
        
        # Log to ROS logger based on category
        if category in ["GAME_OVER", "POWER_MODE"]:
            self.get_logger().warn(message)
        elif category in ["CHEESE", "ENCOUNTER"]:
            self.get_logger().info(message)
        else:
            self.get_logger().info(message)
    
    def get_elapsed_time(self):
        """Returns elapsed game time in seconds."""
        if not self.game_start_time:
            return 0.0
        
        current = self.game_end_time if self.game_end_time else self.get_clock().now()
        return (current - self.game_start_time).nanoseconds / 1e9
    
    def print_startup_banner(self):
        """Prints startup banner with game information."""
        print("\n" + "=" * 70)
        print("                    🎮 PAC-MOUSE GAME MASTER 🎮")
        print("=" * 70)
        print(f"Course:              Robotics 2 (ARI3215)")
        print(f"Project:             Autonomous Cat and Mouse Chase")
        print(f"Author:              Damian Cutajar")
        print("=" * 70)
        print("GAME CONFIGURATION:")
        print(f"  Total Cheese:      {self.total_cheeses}")
        print(f"  Capture Radius:    {self.capture_radius}m")
        print(f"  Time Limit:        {self.time_limit}s")
        print(f"  Power Mode:        {'Enabled' if self.power_mode_enabled else 'Disabled'}")
        print(f"  World:             {self.world_name}")
        print("=" * 70)
        print("Waiting for robots to spawn...")
        print("=" * 70 + "\n")
        
        self.get_logger().info("🎮 Game Master is ready!")


def main(args=None):
    """Main entry point for the game master node."""
    rclpy.init(args=args)
    node = ProposalGameMaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n⚠️  Game interrupted by user ⚠️\n")
    finally:
        # Ensure robots are stopped
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
