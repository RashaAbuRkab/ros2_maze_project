import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from collections import deque
import random

# ─────────────────────────────────────────────
#  TOPIC NAMES
# ─────────────────────────────────────────────
TOPIC_CMD_VEL = 'cmd_vel'
TOPIC_ODOM    = 'odom'
TOPIC_SCAN    = 'scan'
TOPIC_POSE    = '/world/complex_maze_world/dynamic_pose/info'

# ─────────────────────────────────────────────
#  STATE MACHINE CONSTANTS
# ─────────────────────────────────────────────
STATE_APF         = 0   # normal artificial-potential-field navigation
STATE_WALL_FOLLOW = 1   # wall-following escape from local minima
STATE_PERTURB     = 2   # random velocity to break symmetry

# ═════════════════════════════════════════════════════════════════════════════
#  ENHANCED POTENTIAL FIELD PLANNER NODE
# ═════════════════════════════════════════════════════════════════════════════
class EnhancedPotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('enhanced_potential_field_planner')

        # ── ROS 2 parameters ──────────────────────────────────────────────
        self.declare_parameter('goal_x',          11.4)
        self.declare_parameter('goal_y',          11.4)
        self.declare_parameter('k_att',           1.2)
        self.declare_parameter('k_rep',           4.0)
        self.declare_parameter('d_obs',           1.5)
        self.declare_parameter('max_linear_vel',  0.4)
        self.declare_parameter('max_angular_vel', 1.2)

        self.xdp             = self.get_parameter('goal_x').value
        self.ydp             = self.get_parameter('goal_y').value
        self.kap             = self.get_parameter('k_att').value
        self.krp             = self.get_parameter('k_rep').value
        self.gstarp          = self.get_parameter('d_obs').value
        self.max_linear_vel  = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # ── Control constants ─────────────────────────────────────────────
        self.kthetap     = 5.0
        self.eps_orient  = math.pi / 4   # rotate-in-place threshold [rad]
        self.eps_control = 0.3           # goal-reached radius [m]
        self.dt          = 0.1           # control loop period [s]

        self.FORCE_MIN_MAG = 0.05

        # ── Message stores ────────────────────────────────────────────────
        self.OdometryMsg = Odometry()
        self.LidarMsg    = LaserScan()

        # ── Ground-truth pose (Gazebo TFMessage) ──────────────────────────
        self._gt_x     = 0.0
        self._gt_y     = 0.0
        self._gt_theta = 0.0
        self._gt_ready = False

        # ── Velocity command ──────────────────────────────────────────────
        self.controlVel = Twist()

        # ── Publisher / Subscribers ───────────────────────────────────────
        self.ControlPublisher = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)

        self.create_subscription(Odometry,  TOPIC_ODOM, self.odom_callback,    10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback,    10)
        self.create_subscription(TFMessage, TOPIC_POSE, self.gt_pose_callback, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        # ── Goal flag ─────────────────────────────────────────────────────
        self.goal_reached = False

        # ─────────────────────────────────────────────────────────────────
        #  STUCK DETECTION
        # ─────────────────────────────────────────────────────────────────
        self.position_window      = deque()
        self.stuck_window_sec     = 4.0    # observe over 4 s window
        self.stuck_disp_threshold = 0.10   # 10 cm net displacement → stuck
        self.stuck_timer          = 0.0
        self.stuck_threshold      = 1.5    # trigger after 1.5 s of no progress
        self._time_started        = None

        # ─────────────────────────────────────────────────────────────────
        #  VIRTUAL CHARGES (History Repulsion)
        # ─────────────────────────────────────────────────────────────────
        self.visited_positions = deque(maxlen=200) # Store recent positions
        self.k_hist = 0.5 # Repulsion gain for history
        self.d_hist = 1.0 # Distance threshold for history repulsion

        # ─────────────────────────────────────────────────────────────────
        #  WALL-FOLLOWING STATE MACHINE
        # ─────────────────────────────────────────────────────────────────
        self.state = STATE_APF
        self.wall_target_dist   = 0.45   # desired clearance to the wall [m]
        self.k_wall             = 1.5    # gain on distance error
        self.k_align            = 2.5    # gain on alignment error
        self.wall_follow_speed  = 0.25   # forward speed while following [m/s]

        self.wall_side           = 0     # +1 = right, −1 = left, 0 = unset
        self.wall_follow_timer   = 0.0
        self.wall_follow_max_sec = 30.0  # Max time in wall follow
        self.wall_follow_min_sec = 3.0   # Min time in wall follow
        self.entry_dist_to_goal  = 0.0   # Distance to goal when entering wall follow

        # ─────────────────────────────────────────────────────────────────
        #  PERTURBATION
        # ─────────────────────────────────────────────────────────────────
        self.perturb_timer = 0.0
        self.perturb_duration = 1.0

        self.get_logger().info(f'Enhanced Planner ready — goal=({self.xdp}, {self.ydp})')

    # ═════════════════════════════════════════════════════════════════════
    #  HELPERS
    # ═════════════════════════════════════════════════════════════════════

    @staticmethod
    def orientation_error(theta: float, thetad: float) -> float:
        return (thetad - theta + math.pi) % (2.0 * math.pi) - math.pi

    def lidar_index(self, angle_rad: float) -> int:
        if not self.LidarMsg.ranges: return 0
        n   = len(self.LidarMsg.ranges)
        idx = int(round((angle_rad - self.LidarMsg.angle_min)
                        / self.LidarMsg.angle_increment))
        return idx % n

    def sector_min_range(self, ranges: np.ndarray,
                         centre: float, half_width: float = 0.25) -> float:
        if not self.LidarMsg.ranges: return np.inf
        n      = len(ranges)
        n_rays = max(1, int(round(half_width / self.LidarMsg.angle_increment)))
        ci     = self.lidar_index(centre)
        vals   = [ranges[(ci + d) % n]
                  for d in range(-n_rays, n_rays + 1)
                  if np.isfinite(ranges[(ci + d) % n])]
        return float(min(vals)) if vals else np.inf

    def publish(self, linear_x: float, angular_z: float) -> None:
        self.controlVel.linear.x  = float(np.clip(linear_x, -self.max_linear_vel, self.max_linear_vel))
        self.controlVel.angular.z = float(np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel))
        self.ControlPublisher.publish(self.controlVel)

    # ═════════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═════════════════════════════════════════════════════════════════════

    def odom_callback(self, msg: Odometry) -> None:
        self.OdometryMsg = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self.LidarMsg = msg

    def gt_pose_callback(self, msg: TFMessage) -> None:
        if not msg.transforms: return
        # Try to find the robot transform (usually the one with small z)
        robot_tf = None
        for tf in msg.transforms:
            if 0.005 < tf.transform.translation.z < 0.1:
                robot_tf = tf
                break
        if robot_tf is None: robot_tf = msg.transforms[0]
        t = robot_tf.transform
        _, _, yaw = euler_from_quaternion([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        self._gt_x, self._gt_y, self._gt_theta = t.translation.x, t.translation.y, yaw
        self._gt_ready = True

    # ═════════════════════════════════════════════════════════════════════
    #  LIDAR PROCESSING
    # ═════════════════════════════════════════════════════════════════════

    def _extract_obstacles(self, x: float, y: float, theta: float):
        raw   = np.array(self.LidarMsg.ranges, dtype=np.float64)
        a_min, a_inc = self.LidarMsg.angle_min, self.LidarMsg.angle_increment
        r_min, r_max = self.LidarMsg.range_min, self.LidarMsg.range_max

        valid   = np.isfinite(raw) & (raw > r_min) & (raw <= r_max)
        lr      = np.where(valid, raw, np.inf)
        clusters = []
        if np.any(valid):
            idx_v  = np.where(np.isfinite(lr))[0]
            splits = np.where(np.diff(idx_v) > 1)[0] + 1
            for part in np.split(idx_v, splits):
                seg     = lr[part]
                best    = int(np.argmin(seg))
                r       = float(seg[best])
                bearing = float(a_min + a_inc * part[best])
                wx      = x + r * math.cos(bearing + theta)
                wy      = y + r * math.sin(bearing + theta)
                clusters.append((wx, wy, r, bearing))
        return raw, valid, lr, clusters

    # ═════════════════════════════════════════════════════════════════════
    #  STUCK DETECTION
    # ═════════════════════════════════════════════════════════════════════

    def _update_stuck(self, x: float, y: float, now: float) -> bool:
        self.position_window.append((now, x, y))
        cutoff = now - self.stuck_window_sec
        while self.position_window and self.position_window[0][0] < cutoff:
            self.position_window.popleft()
        
        if len(self.position_window) < 2:
            self.stuck_timer = 0.0
            return False
            
        _, x0, y0 = self.position_window[0]
        net_disp   = math.hypot(x - x0, y - y0)
        if net_disp < self.stuck_disp_threshold:
            self.stuck_timer += self.dt
        else:
            self.stuck_timer = 0.0
        return self.stuck_timer >= self.stuck_threshold

    # ═════════════════════════════════════════════════════════════════════
    #  WALL-FOLLOWING STEP
    # ═════════════════════════════════════════════════════════════════════

    def _wall_follow_step(self, x: float, y: float, theta: float, lr: np.ndarray):
        side_90    = self.wall_side * (-math.pi / 2)
        side_front = self.wall_side * (-math.pi / 4)
        side_rear  = self.wall_side * (-3.0 * math.pi / 4)

        wall_dist  = self.sector_min_range(lr, side_90,    half_width=0.20)
        d_fs       = self.sector_min_range(lr, side_front, half_width=0.20)
        d_rs       = self.sector_min_range(lr, side_rear,  half_width=0.20)
        front_dist = self.sector_min_range(lr, 0.0,        half_width=0.35)

        if self.wall_side == 0:
            right_dist = self.sector_min_range(lr, -math.pi / 2, half_width=0.3)
            left_dist  = self.sector_min_range(lr,  math.pi / 2, half_width=0.3)
            # Pick the side that is closer to a wall
            self.wall_side = 1 if right_dist <= left_dist else -1
            self.entry_dist_to_goal = math.hypot(self.xdp - x, self.ydp - y)
            self.get_logger().info(f'[WALL] Entering wall-follow on {"RIGHT" if self.wall_side == 1 else "LEFT"} side')

        eff_dist = min(wall_dist, self.gstarp)
        dist_err = eff_dist - self.wall_target_dist
        align_err = min(d_fs, self.gstarp) - min(d_rs, self.gstarp)

        angular_z = -(self.wall_side * (self.k_wall * dist_err + self.k_align * align_err))
        
        if front_dist < 0.4:
            angular_z = self.wall_side * self.max_angular_vel * 0.8
            linear_x  = 0.05
        else:
            linear_x  = self.wall_follow_speed
            
        # Exit conditions:
        # 1. Goal path is clear AND we are closer to goal than when we started wall following
        # 2. Hard timeout
        goal_bearing_world = math.atan2(self.ydp - y, self.xdp - x)
        goal_bearing_robot = self.orientation_error(theta, goal_bearing_world)
        goal_sector_dist   = self.sector_min_range(lr, goal_bearing_robot, half_width=0.4)
        
        current_dist_to_goal = math.hypot(self.xdp - x, self.ydp - y)
        path_is_clear = goal_sector_dist > self.gstarp * 1.2
        progress_made = current_dist_to_goal < (self.entry_dist_to_goal - 0.5)
        dwell_satisfied = self.wall_follow_timer >= self.wall_follow_min_sec

        exit_flag = False
        if self.wall_follow_timer >= self.wall_follow_max_sec:
            exit_flag = True
        elif dwell_satisfied and path_is_clear and progress_made:
            exit_flag = True
            self.get_logger().info(f'[WALL] Progress made and path clear — returning to APF')

        return linear_x, angular_z, exit_flag

    # ═════════════════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP
    # ═════════════════════════════════════════════════════════════════════

    def control_loop(self) -> None:
        if not self.LidarMsg.ranges or not self._gt_ready: return
        if self.goal_reached:
            self.publish(0.0, 0.0)
            return

        x, y, theta = self._gt_x, self._gt_y, self._gt_theta
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._time_started is None: self._time_started = now

        raw, valid, lr, clusters = self._extract_obstacles(x, y, theta)
        dist_to_goal = math.hypot(self.xdp - x, self.ydp - y)

        # 1. Goal-reached check
        if dist_to_goal < self.eps_control:
            self.goal_reached = True
            self.get_logger().info(f'Goal reached at ({x:.2f}, {y:.2f})!')
            self.publish(0.0, 0.0)
            return

        # 2. State Machine
        if self.state == STATE_PERTURB:
            self.perturb_timer += self.dt
            if self.perturb_timer >= self.perturb_duration:
                self.state = STATE_APF
                self.perturb_timer = 0.0
            else:
                # Apply random velocity
                self.publish(0.1, random.uniform(-1.0, 1.0))
                return

        if self.state == STATE_WALL_FOLLOW:
            self.wall_follow_timer += self.dt
            lx, az, exit_flag = self._wall_follow_step(x, y, theta, lr)
            self.publish(lx, az)
            if exit_flag:
                self.state = STATE_APF
                self.wall_side = 0
                self.wall_follow_timer = 0.0
                self.stuck_timer = 0.0
                self.position_window.clear()
            return

        # STATE: APF
        is_stuck = self._update_stuck(x, y, now)
        if is_stuck:
            # Try perturbation first, then wall follow
            if random.random() < 0.3:
                self.state = STATE_PERTURB
                self.get_logger().warn('[APF] Stuck — Perturbing')
            else:
                self.state = STATE_WALL_FOLLOW
                self.get_logger().warn('[APF] Stuck — Wall Following')
            return

        # 3. Calculate Forces
        # Attractive
        vectorD = np.array([[x - self.xdp], [y - self.ydp]])
        AF = -(self.kap * vectorD)

        # Repulsive (Obstacles)
        RF = np.zeros((2, 1))
        for (wx, wy, _, __) in clusters:
            g = math.hypot(x - wx, y - wy)
            if 1e-6 < g <= self.gstarp:
                coeff = self.krp * (1.0 / self.gstarp - 1.0 / g) / (g**3)
                RF[0, 0] += coeff * (x - wx)
                RF[1, 0] += coeff * (y - wy)
        
        # Repulsive (History/Virtual Charges)
        HF = np.zeros((2, 1))
        # Add current position to history every 0.5s
        if int(now * 2) % 2 == 0:
            self.visited_positions.append((x, y))
        
        for (hx, hy) in self.visited_positions:
            gh = math.hypot(x - hx, y - hy)
            if 0.1 < gh <= self.d_hist:
                coeff_h = self.k_hist * (1.0 / self.d_hist - 1.0 / gh) / (gh**3)
                HF[0, 0] += coeff_h * (x - hx)
                HF[1, 0] += coeff_h * (y - hy)

        F = AF - RF - HF
        fmag = math.hypot(float(F[0, 0]), float(F[1, 0]))

        # 4. Convert Force to Velocity
        if fmag < self.FORCE_MIN_MAG:
            thetaD, eorient = theta, 0.0
        else:
            thetaD = math.atan2(float(F[1, 0]), float(F[0, 0]))
            eorient = self.orientation_error(theta, thetaD)

        thetavel = float(np.clip(self.kthetap * eorient, -self.max_angular_vel, self.max_angular_vel))

        if abs(eorient) > self.eps_orient:
            xvel = 0.05 # Slow rotation
        else:
            # Scale speed by proximity to obstacles
            closest = np.min(lr) if np.any(valid) else self.gstarp
            spd_scale = float(np.clip((closest / self.gstarp), 0.2, 1.0))
            xvel = float(np.clip(fmag, 0.1, self.max_linear_vel)) * spd_scale

        self.publish(xvel, thetavel)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedPotentialFieldPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
