#!/usr/bin/env python3
"""
cat_brain_v2.py (Option A upgrade)

Key upgrades:
- Clearance-based steering (uses FULL LiDAR, not just front ray)
- Hard wall exclusion: avoid anything closer than avoid_dist (default 0.30m) ALL AROUND
- Goal direction is blended with "most free space" direction (smooth, no corner wedging)
- Stuck detection based on ODOM movement (not commanded speed)
- Still supports PATROL / CHASE / INVESTIGATE / ESCAPE

Assumptions:
- LaserScan angles are in the robot frame (standard): angle=0 is forward, + is left (CCW).
"""
"""
=============================================================================
ADVANCED CAT BRAIN CONTROLLER
=============================================================================
OVERVIEW:
This node implements an intelligent cat controller that hunts the mouse
using sensor fusion, predictive pursuit, and strategic interception.

FEATURES:
Mouse Tracking: Detects mouse using camera
Predictive Pursuit: Anticipates mouse movement
Interception: Calculates optimal intercept paths
Obstacle Avoidance: Navigates around walls while pursuing
Search Behavior: Explores when mouse is not visible

PURSUIT STRATEGIES:
DIRECT_CHASE: Simple pursuit towards last known position
PREDICTIVE_INTERCEPT: Calculates future mouse position
SEARCH_PATTERN: Systematic exploration when contact lost

SENSORS USED:
LiDAR: Obstacle detection and mouse ranging
Camera: Visual confirmation of mouse
Odometry: Self-localisation
=============================================================================
"""
import math
import time
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from transforms3d.euler import quat2euler

PATROL = 0
CHASE = 1
INVESTIGATE = 2
ESCAPE = 3


class CatBrainV2(Node):
    def __init__(self):
        super().__init__('cat_brain_v2')

        # -------------------------
        # SUBSCRIBERS
        # -------------------------
        self.sub_cat_odom = self.create_subscription(Odometry, '/cat/odom', self.cat_odom_cb, 10)
        self.sub_mouse_odom = self.create_subscription(Odometry, '/mouse/odom', self.mouse_odom_cb, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/cat/scan', self.lidar_cb, 10)
        self.sub_cheese = self.create_subscription(String, '/cheese_eaten', self.cheese_cb, 10)

        # -------------------------
        # PUBLISHER
        # -------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cat/cmd_vel', 10)

        # -------------------------
        # POSE / SENSOR STATE
        # -------------------------
        self.cat_pose = None
        self.cat_yaw = 0.0
        self.mouse_pose = None

        self.lidar_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0

        # -------------------------
        # FSM STATE
        # -------------------------
        self.state = PATROL
        self.last_state = None
        self.power_mode = False

        self.last_seen_pos = None
        self.last_seen_time = None

        # -------------------------
        # BEHAVIOUR PARAMETERS
        # -------------------------
        # perception
        self.sniff_radius = 1.2
        self.belief_timeout = 5.0
        self.los_epsilon = 0.15

        # motion limits
        self.max_lin = 0.5
        self.max_ang = 1.8

        # steering gains
        self.k_ang = 1.8
        self.turn_only_thresh = 1.0

        # --- Option A: Clearance-based avoidance ---
        self.avoid_dist = 0.55           # HARD rule: do not drive toward space < 0.30m
        self.slow_dist = 0.85            # start slowing down if anything is within this
        self.front_window_deg = 30.0     # "front" window for speed limiting
        self.goal_blend = 0.65           # weight on goal vs free-space (0..1). Higher = more aggressive chase
        self.escape_goal_blend = 0.35    # in ESCAPE, prefer free space more to avoid wall pinning
        self.max_considered_range = 6.0  # cap for "infinite" lidar

        # patrol behaviour
        self.patrol_speed = 0.70
        self.patrol_turn_speed = 0.8
        self.patrol_switch_time = 2.5
        self.current_patrol_dir = random.choice([-1, 1])
        self.last_patrol_switch = time.time()

        # investigate behaviour
        self.investigate_speed = 0.6
        self.arrival_radius = 0.35

        # escape behaviour
        self.escape_speed = 0.60

        self.cheese_count = 0

        # --- Real stuck detection (ODOM-based) ---
        self.last_progress_check_time = time.time()
        self.last_progress_pos = None
        self.stuck_timeout = 2.0          # seconds with < progress_dist movement
        self.progress_dist = 0.06         # meters
        self.recovering_until = 0.0
        self.recovery_turn_time = 0.7     # seconds
        self.recovery_forward_time = 0.7  # seconds
        self.recovery_phase = "turn"      # "turn" then "forward"
        self.recovery_dir = 1.0

        # control loop
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info("🐱 Cat brain v2 (Option A clearance steering) online")
        self.log_state(force=True)

    # =========================
    # Callbacks
    # =========================
    def cat_odom_cb(self, msg: Odometry):
        self.cat_pose = msg.pose.pose
        q = self.cat_pose.orientation
        _, _, self.cat_yaw = quat2euler([q.w, q.x, q.y, q.z])

    def mouse_odom_cb(self, msg: Odometry):
        self.mouse_pose = msg.pose.pose

    def lidar_cb(self, msg: LaserScan):
        self.lidar_ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def cheese_cb(self, msg: String):
        self.cheese_count = self.cheese_count+1
        self.get_logger().info(f"🧀 Cheese eaten! Total: {self.cheese_count}")
        if self.cheese_count == 4:
            if not self.power_mode:
                self.power_mode = True
                self.get_logger().warn("😱 POWER MODE — CAT ESCAPING!")
                self.set_state(ESCAPE)

    # =========================
    # State helpers
    # =========================
    def log_state(self, force: bool = False):
        if force or self.state != self.last_state:
            names = {PATROL: "PATROL", CHASE: "CHASE", INVESTIGATE: "INVESTIGATE", ESCAPE: "ESCAPE"}
            self.get_logger().info(f"🐱 STATE → {names[self.state]}")
            self.last_state = self.state

    def set_state(self, new_state: int):
        if new_state != self.state:
            self.state = new_state
            self.log_state()

    # =========================
    # Core loop
    # =========================
    def loop(self):
        if self.cat_pose is None or self.mouse_pose is None:
            return

        now = time.time()

        # 0) Recovery mode (ODOM-based stuck recovery)
        if now < self.recovering_until and not self.power_mode:
            self.run_recovery()
            return

        # 1) Power mode overrides everything
        if self.power_mode:
            self.set_state(ESCAPE)
            self.escape()
            self.check_progress_and_stuck(now, allow_recovery=False)
            return

        # 2) Perception
        mouse_visible = self.mouse_is_visible()

        if mouse_visible:
            self.last_seen_pos = self.mouse_pose.position
            self.last_seen_time = time.time()
            self.set_state(CHASE)
        else:
            if self.state == CHASE:
                self.set_state(INVESTIGATE)

        # 3) Execute state
        if self.state == CHASE:
            self.go_to_point(self.last_seen_pos, speed=self.max_lin, goal_blend=self.goal_blend)

        elif self.state == INVESTIGATE:
            if self.last_seen_time and (time.time() - self.last_seen_time) < self.belief_timeout:
                arrived = self.go_to_point(
                    self.last_seen_pos, speed=self.investigate_speed, goal_blend=self.goal_blend, return_arrived=True
                )
                if arrived:
                    # scan gently, but still respect clearance
                    self.clearance_turn_in_place(0.9 * self.current_patrol_dir)
            else:
                self.set_state(PATROL)
                self.patrol()

        elif self.state == PATROL:
            self.patrol()

        # 4) Check real progress (ODOM) and trigger recovery if truly stuck
        self.check_progress_and_stuck(now, allow_recovery=True)

    # =========================
    # Behaviours
    # =========================
    def patrol(self):
        if time.time() - self.last_patrol_switch > self.patrol_switch_time:
            self.current_patrol_dir *= -1
            self.last_patrol_switch = time.time()

        # Patrol "goal": a slight turn bias, but always clearance-safe
        desired_rel = math.radians(25.0) * self.current_patrol_dir

        # Choose free-space direction and blend
        free_rel = self.best_free_space_angle()
        rel = self.blend_angles(desired_rel, free_rel, 0.45)

        self.drive_with_clearance(rel, base_speed=self.patrol_speed)

    def escape(self):
        # Flee direction = away from mouse, then blended with clearance direction
        cx, cy = self.cat_pose.position.x, self.cat_pose.position.y
        mx, my = self.mouse_pose.position.x, self.mouse_pose.position.y

        dx = cx - mx
        dy = cy - my
        flee_angle_world = math.atan2(dy, dx)
        flee_rel = self.normalize_angle(flee_angle_world - self.cat_yaw)

        free_rel = self.best_free_space_angle()
        rel = self.blend_angles(flee_rel, free_rel, self.escape_goal_blend)

        self.drive_with_clearance(rel, base_speed=self.escape_speed)

    # =========================
    # Motion helpers (Option A)
    # =========================
    def drive_with_clearance(self, desired_rel_angle: float, base_speed: float):
        """
        Clearance-safe driving:
        - If desired direction points into "forbidden" space (< avoid_dist), override toward free space.
        - Speed is reduced by front clearance, and goes 0 if too close.
        """
        if not self.lidar_ranges or self.angle_inc == 0.0:
            # no lidar -> minimal safe behavior
            ang = self.clamp(self.k_ang * desired_rel_angle, -self.max_ang, self.max_ang)
            lin = 0.10
            self.publish_cmd(lin, ang)
            return

        # If the direction we're trying to go is unsafe, override with the best free direction.
        if self.direction_is_forbidden(desired_rel_angle):
            desired_rel_angle = self.best_free_space_angle()

        # angular control
        ang = self.clamp(self.k_ang * desired_rel_angle, -self.max_ang, self.max_ang)

        # speed control based on front clearance
        front_min = self.get_min_range_in_window(center_angle=0.0, half_width_deg=self.front_window_deg)

        if front_min < self.avoid_dist:
            lin = 0.0
        else:
            lin = base_speed
            if front_min < self.slow_dist:
                # scale down smoothly between avoid_dist..slow_dist
                t = (front_min - self.avoid_dist) / max(1e-6, (self.slow_dist - self.avoid_dist))
                lin *= self.clamp(t, 0.0, 1.0)

        # If we need to turn a lot, slow slightly (prevents scraping)
        if abs(desired_rel_angle) > self.turn_only_thresh:
            lin = min(lin, 0.15)

        self.publish_cmd(lin, ang)

    def clearance_turn_in_place(self, turn_sign: float):
        """Turn in place, but if a side is too close, prefer the safer side."""
        free_rel = self.best_free_space_angle()
        # If free space is clearly on one side, turn that way.
        if abs(free_rel) > math.radians(10):
            turn_sign = 1.0 if free_rel > 0 else -1.0
        self.publish_cmd(0.0, self.clamp(0.9 * turn_sign, -self.max_ang, self.max_ang))

    def go_to_point(self, point, speed: float, goal_blend: float, return_arrived: bool = False) -> bool:
        if point is None:
            self.publish_cmd(0.0, 0.0)
            return True

        cx, cy = self.cat_pose.position.x, self.cat_pose.position.y
        tx, ty = point.x, point.y

        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)

        if dist < self.arrival_radius:
            self.publish_cmd(0.0, 0.0)
            return True

        target_world = math.atan2(dy, dx)
        goal_rel = self.normalize_angle(target_world - self.cat_yaw)

        free_rel = self.best_free_space_angle()
        rel = self.blend_angles(goal_rel, free_rel, goal_blend)

        self.drive_with_clearance(rel, base_speed=speed)

        return dist < self.arrival_radius if return_arrived else False

    # =========================
    # Stuck detection & recovery (ODOM-based)
    # =========================
    def check_progress_and_stuck(self, now: float, allow_recovery: bool):
        if not allow_recovery:
            return

        if self.last_progress_pos is None:
            self.last_progress_pos = (self.cat_pose.position.x, self.cat_pose.position.y)
            self.last_progress_check_time = now
            return

        x, y = self.cat_pose.position.x, self.cat_pose.position.y
        px, py = self.last_progress_pos
        moved = math.hypot(x - px, y - py)

        if moved >= self.progress_dist:
            self.last_progress_pos = (x, y)
            self.last_progress_check_time = now
            return

        if (now - self.last_progress_check_time) > self.stuck_timeout:
            # True stuck -> recovery: turn toward best free space, then push forward a bit
            free_rel = self.best_free_space_angle()
            self.recovery_dir = 1.0 if free_rel >= 0.0 else -1.0
            self.recovery_phase = "turn"
            self.recovering_until = now + self.recovery_turn_time + self.recovery_forward_time
            # reset timer so it doesn't retrigger instantly
            self.last_progress_check_time = now
            self.last_progress_pos = (x, y)

    def run_recovery(self):
        # Recovery runs open-loop but based on free-space preference
        now = time.time()

        # split recovery into turn then forward
        # We compute where we are in recovery based on remaining time.
        # Easier: keep it simple with an internal phase switch.
        if self.recovery_phase == "turn":
            self.clearance_turn_in_place(self.recovery_dir)
            # after turn_time, switch phase
            # (we detect this by comparing with remaining time estimate)
            # Since we don't store start time, we switch when front is safe enough.
            front_min = self.get_min_range_in_window(0.0, self.front_window_deg)
            if front_min > (self.avoid_dist + 0.10):
                self.recovery_phase = "forward"
            return

        # forward phase: drive straight with clearance speed limiting
        self.drive_with_clearance(0.0, base_speed=0.35)

    # =========================
    # Perception / Visibility
    # =========================
    def mouse_is_visible(self) -> bool:
        cx = self.cat_pose.position.x
        cy = self.cat_pose.position.y
        mx = self.mouse_pose.position.x
        my = self.mouse_pose.position.y

        dx = mx - cx
        dy = my - cy
        dist = math.hypot(dx, dy)

        if dist < self.sniff_radius:
            return True

        if not self.lidar_ranges or self.angle_inc == 0.0:
            return False

        angle_to_mouse = math.atan2(dy, dx)
        rel_angle = self.normalize_angle(angle_to_mouse - self.cat_yaw)

        idx = self.angle_to_index(rel_angle)
        lidar_dist = self.clean_range(self.lidar_ranges[idx])

        # If lidar gives no return, treat as clear in sim
        if lidar_dist >= self.max_considered_range:
            return True

        return lidar_dist >= (dist - self.los_epsilon)

    # =========================
    # Clearance / LiDAR helpers
    # =========================
    def best_free_space_angle(self) -> float:
        """
        Returns the relative angle (rad) pointing to the "best" free direction.
        We choose the direction with maximum range, excluding anything < avoid_dist.
        """
        if not self.lidar_ranges or self.angle_inc == 0.0:
            return 0.0

        best_idx = None
        best_score = -1.0

        for i, r in enumerate(self.lidar_ranges):
            rr = self.clean_range(r)
            if rr < self.avoid_dist:
                continue
            # prefer bigger clearance; slight bias to forward-ish (reduces spinning)
            ang = self.index_to_angle(i)
            forward_bias = 0.15 * math.cos(ang)  # + when near 0 rad
            score = rr + forward_bias
            if score > best_score:
                best_score = score
                best_idx = i

        if best_idx is None:
            # Everything is too close (rare). Pick the least-bad direction.
            best_idx = max(range(len(self.lidar_ranges)), key=lambda k: self.clean_range(self.lidar_ranges[k]))

        return self.index_to_angle(best_idx)

    def direction_is_forbidden(self, rel_angle: float) -> bool:
        """
        If the ray (and nearby rays) in that direction are below avoid_dist, treat as forbidden.
        Using a small angular window makes this robust.
        """
        d = self.get_min_range_in_window(center_angle=rel_angle, half_width_deg=22.0)
        return d < self.avoid_dist

    def get_min_range_in_window(self, center_angle: float, half_width_deg: float) -> float:
        if not self.lidar_ranges or self.angle_inc == 0.0:
            return self.max_considered_range

        half = math.radians(half_width_deg)
        a0 = center_angle - half
        a1 = center_angle + half

        i0 = self.angle_to_index(a0)
        i1 = self.angle_to_index(a1)
        if i0 > i1:
            i0, i1 = i1, i0

        m = self.max_considered_range
        for i in range(i0, i1 + 1):
            m = min(m, self.clean_range(self.lidar_ranges[i]))
        return m

    def angle_to_index(self, rel_angle: float) -> int:
        # clamp angle into scan bounds
        if not self.lidar_ranges:
            return 0
        rel_angle = self.normalize_angle(rel_angle)
        idx = int(round((rel_angle - self.angle_min) / self.angle_inc))
        return max(0, min(idx, len(self.lidar_ranges) - 1))

    def index_to_angle(self, idx: int) -> float:
        return self.angle_min + idx * self.angle_inc

    def clean_range(self, r: float) -> float:
        if math.isinf(r) or math.isnan(r) or r <= 0.0:
            return self.max_considered_range
        return min(r, self.max_considered_range)

    # =========================
    # Publishing
    # =========================
    def publish_cmd(self, lin: float, ang: float):
        lin = self.clamp(lin, -self.max_lin, self.max_lin)
        ang = self.clamp(ang, -self.max_ang, self.max_ang)

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    # =========================
    # Math helpers
    # =========================
    @staticmethod
    def normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    @staticmethod
    def clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def blend_angles(self, a: float, b: float, w_a: float) -> float:
        """
        Blend angles a and b with weight w_a for a, (1-w_a) for b, using unit vectors.
        """
        w_a = self.clamp(w_a, 0.0, 1.0)
        w_b = 1.0 - w_a
        xa, ya = math.cos(a), math.sin(a)
        xb, yb = math.cos(b), math.sin(b)
        x = w_a * xa + w_b * xb
        y = w_a * ya + w_b * yb
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            return 0.0
        return math.atan2(y, x)


def main(args=None):
    rclpy.init(args=args)
    node = CatBrainV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()