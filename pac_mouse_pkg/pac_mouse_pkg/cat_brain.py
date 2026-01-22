#!/usr/bin/env python3
"""
=============================================================================
ADVANCED CAT BRAIN CONTROLLER - CORRECTED VERSION
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import math


class ProposalCatBrain(Node):
    
    def __init__(self):
        super().__init__('proposal_cat_brain',
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        # ====================================================================
        # CONFIGURATION
        # ====================================================================
        self.max_linear_speed = 0.35
        self.max_angular_speed = 2.0
        self.detection_range = 5.0
        self.capture_dist = 0.3
        self.prediction_time = 1.0
        
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
        self.cat_x = 0.0
        self.cat_y = 0.0
        self.cat_yaw = 0.0
        
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.mouse_vx = 0.0
        self.mouse_vy = 0.0
        self.mouse_detected = False
        self.last_mouse_time = self.get_clock().now()
        
        self.obstacle_ahead = False
        self.clear_direction = 0.0
        
        self.state = "SEARCH"  # SEARCH, CHASE, INTERCEPT
        self.search_angle = 0.0
        
        # ====================================================================
        # TIMER
        # ====================================================================
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("😾 PROPOSAL CAT BRAIN: HUNTING MODE ENGAGED")
    
    def odometry_callback(self, msg):
        self.cat_x = msg.pose.pose.position.x
        self.cat_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cat_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def mouse_odometry_callback(self, msg):
        self.mouse_x = msg.pose.pose.position.x
        self.mouse_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        mouse_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.mouse_vx = msg.twist.twist.linear.x * math.cos(mouse_yaw)
        self.mouse_vy = msg.twist.twist.linear.x * math.sin(mouse_yaw)
        
        distance = math.hypot(self.mouse_x - self.cat_x, 
                             self.mouse_y - self.cat_y)
        
        if distance < self.detection_range:
            self.mouse_detected = True
            self.last_mouse_time = self.get_clock().now()
        else:
            time_since_seen = (self.get_clock().now() - self.last_mouse_time).nanoseconds / 1e9
            if time_since_seen > 3.0:
                self.mouse_detected = False
    
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        valid_mask = np.isfinite(ranges) & (ranges > 0.1) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return
        
        front_mask = np.abs(valid_angles) < 0.52
        if np.any(front_mask):
            front_ranges = valid_ranges[front_mask]
            min_front_dist = np.min(front_ranges)
            self.obstacle_ahead = min_front_dist < 0.6
        else:
            self.obstacle_ahead = False
        
        if self.obstacle_ahead:
            sample_angles = np.linspace(-math.pi, math.pi, 24)
            clearances = []
            
            for sample_angle in sample_angles:
                angle_mask = np.abs(valid_angles - sample_angle) < 0.26
                if np.any(angle_mask):
                    avg_clearance = np.mean(valid_ranges[angle_mask])
                    clearances.append(avg_clearance)
                else:
                    clearances.append(0.0)
            
            best_idx = np.argmax(clearances)
            self.clear_direction = sample_angles[best_idx]
    
    def control_loop(self):
        cmd = Twist()
        
        if self.mouse_detected:
            dx = self.mouse_x - self.cat_x
            dy = self.mouse_y - self.cat_y
            distance = math.hypot(dx, dy)
            
            if distance < self.capture_dist:
                self.get_logger().info("🎯 MOUSE CAPTURED!")
                self.status_pub.publish(String(data="CAPTURED"))
                self.cmd_pub.publish(Twist())
                return
            
            mouse_speed = math.hypot(self.mouse_vx, self.mouse_vy)
            
            if mouse_speed > 0.1:
                self.state = "INTERCEPT"
                self.intercept_mouse(cmd)
            else:
                self.state = "CHASE"
                self.chase_mouse(cmd)
        else:
            self.state = "SEARCH"
            self.search_for_mouse(cmd)
        
        if self.obstacle_ahead:
            self.avoid_obstacle(cmd)
        
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=self.state))
    
    def chase_mouse(self, cmd):
        dx = self.mouse_x - self.cat_x
        dy = self.mouse_y - self.cat_y
        angle_to_mouse = math.atan2(dy, dx)
        
        angle_error = self.normalize_angle(angle_to_mouse - self.cat_yaw)
        
        kp_angular = 3.0
        cmd.angular.z = np.clip(kp_angular * angle_error, 
                               -self.max_angular_speed, 
                               self.max_angular_speed)
        
        if abs(angle_error) < 0.3:
            cmd.linear.x = self.max_linear_speed
        elif abs(angle_error) < 0.8:
            cmd.linear.x = self.max_linear_speed * 0.6
        else:
            cmd.linear.x = 0.1
    
    def intercept_mouse(self, cmd):
        future_mouse_x = self.mouse_x + self.mouse_vx * self.prediction_time
        future_mouse_y = self.mouse_y + self.mouse_vy * self.prediction_time
        
        dx = future_mouse_x - self.cat_x
        dy = future_mouse_y - self.cat_y
        distance = math.hypot(dx, dy)
        
        if distance > 0.1:
            tti = distance / self.max_linear_speed
            intercept_x = self.mouse_x + self.mouse_vx * tti
            intercept_y = self.mouse_y + self.mouse_vy * tti
        else:
            intercept_x = future_mouse_x
            intercept_y = future_mouse_y
        
        angle_to_intercept = math.atan2(intercept_y - self.cat_y, 
                                       intercept_x - self.cat_x)
        
        angle_error = self.normalize_angle(angle_to_intercept - self.cat_yaw)
        
        kp_angular = 3.5
        cmd.angular.z = np.clip(kp_angular * angle_error,
                               -self.max_angular_speed,
                               self.max_angular_speed)
        
        if abs(angle_error) < 0.5:
            cmd.linear.x = self.max_linear_speed
        else:
            cmd.linear.x = self.max_linear_speed * 0.5
    
    def search_for_mouse(self, cmd):
        cmd.linear.x = 0.2
        cmd.angular.z = 0.8
        
        self.search_angle += 0.1
        if self.search_angle > 2 * math.pi:
            self.search_angle = 0.0
    
    def avoid_obstacle(self, cmd):
        angle_error = self.normalize_angle(self.clear_direction)
        cmd.angular.z = 2.5 * np.sign(angle_error)
        cmd.linear.x = 0.0
        self.get_logger().warn("🚧 Obstacle Avoidance", throttle_duration_sec=1.0)
    
    def normalize_angle(self, angle):
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
