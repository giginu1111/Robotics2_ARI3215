#!/usr/bin/env python3
"""
=============================================================================
ADVANCED CAT BRAIN CONTROLLER
=============================================================================
Course: Robotics 2 (ARI3215)
Project: Pac-Mouse Autonomous Navigation

OVERVIEW:
This node implements an intelligent cat controller that hunts the mouse
using sensor fusion, predictive pursuit, and strategic interception.

FEATURES:
1. Mouse Tracking: Detects mouse using LiDAR and camera
2. Predictive Pursuit: Anticipates mouse movement
3. Interception: Calculates optimal intercept paths
4. Obstacle Avoidance: Navigates around walls while pursuing
5. Search Behavior: Explores when mouse is not visible

PURSUIT STRATEGIES:
- DIRECT_CHASE: Simple pursuit towards last known position
- PREDICTIVE_INTERCEPT: Calculates future mouse position
- SEARCH_PATTERN: Systematic exploration when contact lost

SENSORS USED:
- LiDAR: Obstacle detection and mouse ranging
- Camera: Visual confirmation of mouse (future feature)
- Odometry: Self-localization

Author: Damian Cutajar
Date: January 2026
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# ROS Messages
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Processing
import numpy as np
import math
from collections import deque


class ProposalCatBrain(Node):
    """
    Advanced cat controller implementing intelligent pursuit and hunting behaviors.
    """
    
    def __init__(self):
        super().__init__('proposal_cat_brain',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION PARAMETERS
        # ====================================================================
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('mouse_detection_range', 5.0)
        self.declare_parameter('capture_distance', 0.3)
        self.declare_parameter('prediction_horizon', 1.0)  # seconds
        
        # Load parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.detection_range = self.get_parameter('mouse_detection_range').value
        self.capture_dist = self.get_parameter('capture_distance').value
        self.prediction_time = self.get_parameter('prediction_horizon').value
        
        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        self.cmd_pub = self.create_publisher(Twist, '/cat/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/cat_status', 10)
        
        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(LaserScan, '/cat/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/cat/odom', self.odometry_callback, 10)
        self.create_subscription(Odometry, '/mouse/odom', self.mouse_odometry_callback, 10)
        
        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        # Cat state
        self.cat_x = 0.0
        self.cat_y = 0.0
        self.cat_yaw = 0.0
        self.cat_vx = 0.0  # Linear velocity
        self.cat_vy = 0.0
        
        # Mouse tracking
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.mouse_vx = 0.0
        self.mouse_vy = 0.0
        self.mouse_detected = False
        self.last_mouse_time = self.get_clock().now()
        self.mouse_history = deque(maxlen=10)  # Position history
        
        # LiDAR-based detection
        self.lidar_mouse_range = 0.0
        self.lidar_mouse_angle = 0.0
        self.lidar_detected = False
        
        # Obstacle avoidance
        self.obstacle_ahead = False
        self.clear_direction = 0.0
        
        # Behavior state
        self.state = "SEARCH"  # SEARCH, CHASE, INTERCEPT
        self.search_angle = 0.0
        
        # ====================================================================
        # CONTROL LOOP
        # ====================================================================
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("😾 PROPOSAL CAT BRAIN: ONLINE - HUNTING MODE ENGAGED")
        self.get_logger().info("=" * 60)
    
    # ========================================================================
    # SENSOR CALLBACKS
    # ========================================================================
    
    def odometry_callback(self, msg):
        """Updates cat's own position and velocity."""
        self.cat_x = msg.pose.pose.position.x
        self.cat_y = msg.pose.pose.position.y
        
        # Extract yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cat_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Velocity in world frame
        self.cat_vx = msg.twist.twist.linear.x * math.cos(self.cat_yaw)
        self.cat_vy = msg.twist.twist.linear.x * math.sin(self.cat_yaw)
    
    def mouse_odometry_callback(self, msg):
        """
        Tracks mouse position and velocity.
        In a real scenario, this would come from sensors only.
        """
        self.mouse_x = msg.pose.pose.position.x
        self.mouse_y = msg.pose.pose.position.y
        
        # Calculate velocity
        self.mouse_vx = msg.twist.twist.linear.x * math.cos(self.get_mouse_yaw(msg))
        self.mouse_vy = msg.twist.twist.linear.x * math.sin(self.get_mouse_yaw(msg))
        
        # Update history
        self.mouse_history.append((self.mouse_x, self.mouse_y, self.get_clock().now()))
        
        # Check if mouse is in detection range
        distance = math.hypot(self.mouse_x - self.cat_x, 
                             self.mouse_y - self.cat_y)
        
        if distance < self.detection_range:
            self.mouse_detected = True
            self.last_mouse_time = self.get_clock().now()
        else:
            # Lose track if too far
            time_since_seen = (self.get_clock().now() - self.last_mouse_time).nanoseconds / 1e9
            if time_since_seen > 3.0:  # 3 second timeout
                self.mouse_detected = False
    
    def get_mouse_yaw(self, odom_msg):
        """Extracts yaw angle from mouse odometry message."""
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def lidar_callback(self, msg):
        """
        Processes LiDAR for:
        1. Obstacle detection ahead
        2. Finding clear directions
        3. Detecting mouse (optional)
        """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Remove invalid readings
        valid_mask = np.isfinite(ranges) & (ranges > 0.1) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return
        
        # Check front sector for obstacles (±30 degrees)
        front_mask = np.abs(valid_angles) < 0.52  # 30 degrees
        if np.any(front_mask):
            front_ranges = valid_ranges[front_mask]
            min_front_dist = np.min(front_ranges)
            self.obstacle_ahead = min_front_dist < 0.6  # 60cm threshold
        else:
            self.obstacle_ahead = False
        
        # Find clearest direction
        if self.obstacle_ahead:
            # Sample directions every 15 degrees
            sample_angles = np.linspace(-math.pi, math.pi, 24)
            clearances = []
            
            for sample_angle in sample_angles:
                # Check clearance in this direction
                angle_mask = np.abs(valid_angles - sample_angle) < 0.26  # ±15 degrees
                if np.any(angle_mask):
                    avg_clearance = np.mean(valid_ranges[angle_mask])
                    clearances.append(avg_clearance)
                else:
                    clearances.append(0.0)
            
            # Choose direction with most clearance
            best_idx = np.argmax(clearances)
            self.clear_direction = sample_angles[best_idx]
    
    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    
    def control_loop(self):
        """
        Main decision-making loop implementing behavior state machine:
        1. INTERCEPT - Predictive pursuit if mouse moving
        2. CHASE - Direct pursuit if mouse stationary/slow
        3. SEARCH - Exploration pattern if mouse not detected
        """
        cmd = Twist()
        
        if self.mouse_detected:
            # Calculate distance to mouse
            dx = self.mouse_x - self.cat_x
            dy = self.mouse_y - self.cat_y
            distance = math.hypot(dx, dy)
            
            # Check for capture
            if distance < self.capture_dist:
                self.get_logger().info("🎯 MOUSE CAPTURED!")
                self.status_pub.publish(String(data="CAPTURED"))
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
            
            # Determine pursuit strategy
            mouse_speed = math.hypot(self.mouse_vx, self.mouse_vy)
            
            if mouse_speed > 0.1:
                # Mouse is moving - use predictive intercept
                self.state = "INTERCEPT"
                self.intercept_mouse(cmd)
            else:
                # Mouse is stationary/slow - direct chase
                self.state = "CHASE"
                self.chase_mouse(cmd)
        else:
            # Mouse not detected - search pattern
            self.state = "SEARCH"
            self.search_for_mouse(cmd)
        
        # Apply obstacle avoidance
        if self.obstacle_ahead:
            self.avoid_obstacle(cmd)
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Publish status
        self.status_pub.publish(String(data=self.state))
        
        # Log status
        self.get_logger().info(
            f"State: {self.state} | Mouse Detected: {self.mouse_detected}",
            throttle_duration_sec=2.0
        )
    
    # ========================================================================
    # BEHAVIOR IMPLEMENTATIONS
    # ========================================================================
    
    def chase_mouse(self, cmd):
        """
        Direct pursuit: Move straight towards mouse's current position.
        Uses proportional control for smooth approach.
        """
        # Calculate angle to mouse
        dx = self.mouse_x - self.cat_x
        dy = self.mouse_y - self.cat_y
        angle_to_mouse = math.atan2(dy, dx)
        
        # Calculate heading error
        angle_error = self.normalize_angle(angle_to_mouse - self.cat_yaw)
        
        # Proportional control
        kp_angular = 3.0
        cmd.angular.z = np.clip(kp_angular * angle_error, 
                               -self.max_angular_speed, 
                               self.max_angular_speed)
        
        # Speed based on alignment
        if abs(angle_error) < 0.3:  # Well aligned
            cmd.linear.x = self.max_linear_speed
        elif abs(angle_error) < 0.8:  # Moderate alignment
            cmd.linear.x = self.max_linear_speed * 0.6
        else:  # Poor alignment - turn in place
            cmd.linear.x = 0.1
    
    def intercept_mouse(self, cmd):
        """
        Predictive pursuit: Aims for where mouse will be.
        Uses constant bearing pursuit with prediction.
        """
        # Predict future mouse position
        future_mouse_x = self.mouse_x + self.mouse_vx * self.prediction_time
        future_mouse_y = self.mouse_y + self.mouse_vy * self.prediction_time
        
        # Calculate intercept point
        # Using proportional navigation: aim ahead based on closing velocity
        dx = future_mouse_x - self.cat_x
        dy = future_mouse_y - self.cat_y
        distance = math.hypot(dx, dy)
        
        # Time to intercept
        relative_speed = self.max_linear_speed
        if distance > 0.1:
            tti = distance / relative_speed
            
            # Refine prediction
            intercept_x = self.mouse_x + self.mouse_vx * tti
            intercept_y = self.mouse_y + self.mouse_vy * tti
        else:
            intercept_x = future_mouse_x
            intercept_y = future_mouse_y
        
        # Calculate angle to intercept point
        angle_to_intercept = math.atan2(intercept_y - self.cat_y, 
                                       intercept_x - self.cat_x)
        
        # Control
        angle_error = self.normalize_angle(angle_to_intercept - self.cat_yaw)
        
        kp_angular = 3.5
        cmd.angular.z = np.clip(kp_angular * angle_error,
                               -self.max_angular_speed,
                               self.max_angular_speed)
        
        # Aggressive speed
        if abs(angle_error) < 0.5:
            cmd.linear.x = self.max_linear_speed
        else:
            cmd.linear.x = self.max_linear_speed * 0.5
    
    def search_for_mouse(self, cmd):
        """
        Search pattern when mouse is not detected.
        Implements expanding spiral search.
        """
        # Spiral search: rotate while moving forward
        cmd.linear.x = 0.2  # Slow forward movement
        cmd.angular.z = 0.8  # Constant rotation
        
        self.search_angle += 0.1
        if self.search_angle > 2 * math.pi:
            self.search_angle = 0.0
    
    def avoid_obstacle(self, cmd):
        """
        Emergency obstacle avoidance.
        Overrides current command to prevent collision.
        """
        # Turn towards clearest direction
        angle_error = self.normalize_angle(self.clear_direction)
        
        cmd.angular.z = 2.5 * np.sign(angle_error)
        cmd.linear.x = 0.0  # Stop forward motion
        
        self.get_logger().warn("🚧 Obstacle Avoidance Active", throttle_duration_sec=1.0)
    
    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    
    def normalize_angle(self, angle):
        """Normalizes angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ProposalCatBrain()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
