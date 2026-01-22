#!/usr/bin/env python3
"""
cat_brain_v2.py

Stable Pac-Man-style cat:
- PATROL (default)
- CHASE (only when mouse is visible / sniff radius)
- INVESTIGATE (go to last seen location, then give up)
- ESCAPE (when power mode / cheese eaten)

Fixes vs “spasming”:
- clamps angular velocity + smooth forward policy (no stop/go jitter)
- wall-aware escape + wall-aware goal seeking
- “unstuck” recovery if not making forward progress
- state logging only on transitions
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


# =========================
# CAT STATES
# =========================
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
        self.sub_cat_odom = self.create_subscription(
            Odometry, '/cat/odom', self.cat_odom_cb, 10)

        # NOTE: we still subscribe to mouse odom for:
        #  - visibility test (LOS via lidar)
        #  - escape direction
        # But the cat ONLY CHASES when "visible".
        self.sub_mouse_odom = self.create_subscription(
            Odometry, '/mouse/odom', self.mouse_odom_cb, 10)

        self.sub_lidar = self.create_subscription(
            LaserScan, '/cat/scan', self.lidar_cb, 10)

        # Power mode trigger (your project uses this)
        self.sub_cheese = self.create_subscription(
            String, '/cheese_eaten', self.cheese_cb, 10)

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
        # BEHAVIOUR PARAMETERS (tunable)
        # -------------------------
        # perception
        self.sniff_radius = 1.2          # always detect if within this distance
        self.belief_timeout = 5.0        # seconds to keep investigating after losing sight
        self.los_epsilon = 0.15          # LOS slack for lidar comparison

        # motion limits (key for stability)
        self.max_lin = 1.4               # m/s (keep modest for maze)
        self.max_ang = 1.7               # rad/s (clamped)

        # steering
        self.k_ang = 1.8                 # angular proportional gain (then clamped)
        self.turn_only_thresh = 1.0      # if |err| > this, don't drive forward

        # obstacle safety
        self.front_stop_dist = 0.25      # if obstacle closer than this straight ahead -> stop/turn
        self.front_slow_dist = 0.55      # if obstacle closer than this -> slow down

        # patrol behaviour
        self.patrol_speed = 0.9
        self.patrol_turn_speed = 0.9
        self.patrol_switch_time = 2.5
        self.current_patrol_dir = random.choice([-1, 1])
        self.last_patrol_switch = time.time()

        # investigate behaviour
        self.investigate_speed = 0.6
        self.arrival_radius = 0.35       # consider arrived at last_seen

        # escape behaviour
        self.escape_speed = 0.8
        self.escape_turn_speed = 1.0

        # unstuck recovery
        self.last_forward_time = time.time()
        self.stuck_timeout = 2.0         # seconds without forward motion -> recovery turn
        self.recovery_turn_time = 0.6    # seconds to force-turn when stuck
        self.recovering_until = 0.0

        # control loop (slower = less jitter)
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info("🐱 Cat brain v2 online")
        self.log_state(force=True)

    # =========================
    # Callbacks
    # =========================
    def cat_odom_cb(self, msg: Odometry):
        self.cat_pose = msg.pose.pose
        q = self.cat_pose.orientation
        # transforms3d expects [w, x, y, z]
        _, _, self.cat_yaw = quat2euler([q.w, q.x, q.y, q.z])

    def mouse_odom_cb(self, msg: Odometry):
        self.mouse_pose = msg.pose.pose

    def lidar_cb(self, msg: LaserScan):
        self.lidar_ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def cheese_cb(self, msg: String):
        # Your system appears to publish on /cheese_eaten when cheese is collected.
        # We treat FIRST receipt as power mode trigger.
        if not self.power_mode:
            self.power_mode = True
            self.get_logger().warn("😱 POWER MODE — CAT ESCAPING!")
            self.set_state(ESCAPE)

    # =========================
    # State helpers
    # =========================
    def log_state(self, force: bool = False):
        if force or self.state != self.last_state:
            names = {
                PATROL: "PATROL",
                CHASE: "CHASE",
                INVESTIGATE: "INVESTIGATE",
                ESCAPE: "ESCAPE"
            }
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

        # 0) Unstuck recovery takes priority (but not during ESCAPE: ESCAPE has its own wall logic)
        now = time.time()
        if not self.power_mode and now < self.recovering_until:
            self.publish_cmd(0.0, self.patrol_turn_speed * 1.2)  # strong turn
            return

        # 1) Power mode overrides everything
        if self.power_mode:
            self.set_state(ESCAPE)
            self.escape()
            return

        # 2) Perception (visibility)
        mouse_visible = self.mouse_is_visible()

        if mouse_visible:
            self.last_seen_pos = self.mouse_pose.position
            self.last_seen_time = time.time()
            self.set_state(CHASE)
        else:
            # if we just lost the mouse during chase, investigate
            if self.state == CHASE:
                self.set_state(INVESTIGATE)

        # 3) Execute state
        if self.state == CHASE:
            # chase toward last_seen_pos (only updates while visible)
            self.go_to_point(self.last_seen_pos, speed=self.max_lin)

        elif self.state == INVESTIGATE:
            if self.last_seen_time and (time.time() - self.last_seen_time) < self.belief_timeout:
                # move to last seen, then spin a bit (small “search”)
                arrived = self.go_to_point(self.last_seen_pos, speed=self.investigate_speed, return_arrived=True)
                if arrived:
                    # small scan to look around instead of vibrating
                    self.publish_cmd(0.0, 0.8 * self.current_patrol_dir)
            else:
                self.set_state(PATROL)
                self.patrol()

        elif self.state == PATROL:
            self.patrol()

        # 4) If we haven’t moved forward in a while, trigger recovery turn
        # (Avoids “stuck pointing at wall forever”)
        if (time.time() - self.last_forward_time) > self.stuck_timeout:
            self.recovering_until = time.time() + self.recovery_turn_time
            self.last_forward_time = time.time()  # reset so it doesn’t retrigger instantly

    # =========================
    # Behaviours
    # =========================
    def patrol(self):
        # simple wandering: forward + gentle turning, flipping direction occasionally
        if time.time() - self.last_patrol_switch > self.patrol_switch_time:
            self.current_patrol_dir *= -1
            self.last_patrol_switch = time.time()

        # obstacle aware patrol: if blocked, turn
        front = self.get_front_range()
        if front < self.front_stop_dist:
            self.publish_cmd(0.0, self.patrol_turn_speed * self.current_patrol_dir)
            return

        ang = 0.35 * self.current_patrol_dir
        lin = self.patrol_speed

        # slow if something close ahead
        if front < self.front_slow_dist:
            lin *= 0.5

        self.publish_cmd(lin, ang)

    def escape(self):
        """
        Flee behaviour, but wall-aware:
        - If obstacle close ahead, turn first.
        - Else flee away from mouse.
        """
        front = self.get_front_range()
        if front < self.front_stop_dist:
            # hard turn when blocked
            turn_dir = self.pick_turn_dir()
            self.publish_cmd(0.0, self.escape_turn_speed * turn_dir)
            return

        cx, cy = self.cat_pose.position.x, self.cat_pose.position.y
        mx, my = self.mouse_pose.position.x, self.mouse_pose.position.y

        dx = cx - mx
        dy = cy - my
        flee_angle = math.atan2(dy, dx)
        err = self.normalize_angle(flee_angle - self.cat_yaw)

        ang = self.clamp(self.k_ang * err, -self.max_ang, self.max_ang)

        # forward policy: keep moving even if turning, but not if huge error
        if abs(err) > self.turn_only_thresh:
            lin = 0.15
        else:
            lin = self.escape_speed

        # slow near walls
        if front < self.front_slow_dist:
            lin *= 0.5

        self.publish_cmd(lin, ang)

    # =========================
    # Motion / Control
    # =========================
    def publish_cmd(self, lin: float, ang: float):
        """Central place to clamp and publish, also tracks forward progress."""
        lin = self.clamp(lin, -self.max_lin, self.max_lin)
        ang = self.clamp(ang, -self.max_ang, self.max_ang)

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

        if lin > 0.05:
            self.last_forward_time = time.time()

    def go_to_point(self, point, speed: float, return_arrived: bool = False) -> bool:
        """
        Stable go-to:
        - clamp angular velocity
        - forward speed depends on heading error and obstacle ahead
        - if front is blocked: rotate away, don't keep pushing
        """
        if point is None:
            self.publish_cmd(0.0, 0.0)
            return True

        cx, cy = self.cat_pose.position.x, self.cat_pose.position.y
        tx, ty = point.x, point.y

        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)

        # arrived?
        if dist < self.arrival_radius:
            self.publish_cmd(0.0, 0.0)
            return True

        target_angle = math.atan2(dy, dx)
        err = self.normalize_angle(target_angle - self.cat_yaw)

        # obstacle check
        front = self.get_front_range()
        if front < self.front_stop_dist:
            # rotate away from obstacle
            turn_dir = self.pick_turn_dir()
            self.publish_cmd(0.0, 0.9 * turn_dir)
            return False

        # angular control (clamped)
        ang = self.clamp(self.k_ang * err, -self.max_ang, self.max_ang)

        # forward policy (smooth, no stop/go jitter)
        abs_err = abs(err)
        if abs_err > self.turn_only_thresh:
            lin = 0.12
        elif abs_err > 0.6:
            lin = 0.12
        elif abs_err > 0.3:
            lin = 0.22
        else:
            lin = speed

        # slow near walls
        if front < self.front_slow_dist:
            lin *= 0.6

        self.publish_cmd(lin, ang)

        return dist < self.arrival_radius if return_arrived else False

    # =========================
    # Perception / Visibility
    # =========================
    def mouse_is_visible(self) -> bool:
        """
        Visibility model:
        - If within sniff radius => visible.
        - Else: raycast approximation using lidar beam closest to mouse bearing.
          If lidar range >= mouse distance - epsilon => line-of-sight.
        """
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

        # Convert relative angle to index in scan array
        idx = int(round((rel_angle - self.angle_min) / self.angle_inc))
        idx = max(0, min(idx, len(self.lidar_ranges) - 1))

        lidar_dist = self.lidar_ranges[idx]
        if math.isinf(lidar_dist) or math.isnan(lidar_dist):
            # if lidar says "no return", treat as clear line (in sim this can happen)
            return True

        return lidar_dist >= (dist - self.los_epsilon)

    # =========================
    # Lidar helpers
    # =========================
    def get_front_range(self) -> float:
        """Approx front distance from lidar."""
        if not self.lidar_ranges:
            return 10.0
        mid = len(self.lidar_ranges) // 2
        r = self.lidar_ranges[mid]
        if math.isinf(r) or math.isnan(r):
            return 10.0
        return r

    def pick_turn_dir(self) -> float:
        """
        Pick a safer turn direction based on side ranges.
        Returns +1 for left, -1 for right.
        """
        if not self.lidar_ranges:
            return 1.0

        n = len(self.lidar_ranges)
        mid = n // 2
        # sample a bit to left/right of front
        left_idx = min(n - 1, mid + n // 6)
        right_idx = max(0, mid - n // 6)

        left = self.lidar_ranges[left_idx]
        right = self.lidar_ranges[right_idx]

        left = 10.0 if (math.isinf(left) or math.isnan(left)) else left
        right = 10.0 if (math.isinf(right) or math.isnan(right)) else right

        # turn towards the larger clearance
        return 1.0 if left >= right else -1.0

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


def main(args=None):
    rclpy.init(args=args)
    node = CatBrainV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()