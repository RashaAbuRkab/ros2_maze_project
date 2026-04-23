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

# ─────────────────────────────────────────────
#  TOPIC NAMES
# ─────────────────────────────────────────────
TOPIC_CMD_VEL = 'cmd_vel'
TOPIC_ODOM    = 'odom'
TOPIC_SCAN    = 'scan'
TOPIC_POSE    = '/world/maze_world/dynamic_pose/info'

# ─────────────────────────────────────────────
#  STATE MACHINE CONSTANTS
# ─────────────────────────────────────────────
STATE_APF         = 0   # normal artificial-potential-field navigation
STATE_WALL_FOLLOW = 1   # wall-following escape from local minima

# ═════════════════════════════════════════════════════════════════════════════
#  POTENTIAL FIELD PLANNER NODE
# ═════════════════════════════════════════════════════════════════════════════
class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        # ── ROS 2 parameters ──────────────────────────────────────────────
        self.declare_parameter('goal_x',          9.0)
        self.declare_parameter('goal_y',          9.0)
        self.declare_parameter('k_att',           1.0)
        self.declare_parameter('k_rep',           3.0)
        self.declare_parameter('d_obs',           1.2)
        self.declare_parameter('max_linear_vel',  0.4)
        self.declare_parameter('max_angular_vel', 1.0)

        self.xdp             = self.get_parameter('goal_x').value
        self.ydp             = self.get_parameter('goal_y').value
        self.kap             = self.get_parameter('k_att').value
        self.krp             = self.get_parameter('k_rep').value
        self.gstarp          = self.get_parameter('d_obs').value
        self.max_linear_vel  = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # ── Control constants ─────────────────────────────────────────────
        self.kthetap     = 4.0
        self.eps_orient  = math.pi / 6   # rotate-in-place threshold [rad]
        self.eps_control = 0.2           # goal-reached radius [m]
        self.dt          = 0.1           # control loop period [s]

        self.FORCE_MIN_MAG = 0.05

        # ── Message stores ────────────────────────────────────────────────
        self.OdometryMsg = Odometry()
        self.LidarMsg    = LaserScan()

        # ── Ground-truth pose (Gazebo TFMessage) ──────────────────────────
        self._gt_x     = 0.5
        self._gt_y     = 0.5
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
        self.stuck_window_sec     = 4.0
        self.stuck_disp_threshold = 0.20
        self.stuck_timer          = 0.0
        self.stuck_threshold      = 3.0
        self._time_started        = None

        # ─────────────────────────────────────────────────────────────────
        #  WALL-FOLLOWING STATE MACHINE
        # ─────────────────────────────────────────────────────────────────
        self.state = STATE_APF

        # Two-sensor distance + alignment controller
        # ┌─────────────────────────────────────────────────────────────┐
        # │  THREE LiDAR rays used on the wall side (right wall shown): │
        # │                                                             │
        # │      side_rear (−135°)                                      │
        # │           ╲   side_90 (−90°)                               │
        # │            ╲  │                                             │
        # │  ══════════╗╲ ↓    ROBOT →                                 │
        # │   WALL     ║  ●────── forward                               │
        # │  ══════════╝  │ ╲                                           │
        # │               │  ╲ side_front (−45°)                       │
        # │                                                             │
        # │  dist_err  = wall_dist(−90°) − target_dist                 │
        # │  align_err = d_fs(−45°)     − d_rs(−135°)                  │
        # │                                                             │
        # │  angular_z = −wall_side × (k_wall×dist_err                 │
        # │                           + k_align×align_err)             │
        # └─────────────────────────────────────────────────────────────┘
        self.wall_target_dist   = 0.30   # desired clearance to the wall [m]
        self.k_wall             = 1.2    # gain on distance error
        self.k_align            = 2.0    # gain on alignment error
        self.wall_follow_speed  = 0.18   # forward speed while following [m/s]

        self.wall_side           = 0     # +1 = right, −1 = left, 0 = unset
        self.wall_follow_timer   = 0.0
        self.wall_follow_max_sec = 40.0
        self.wall_follow_min_sec = 3.0

        self.get_logger().info(
            f'Planner ready — goal=({self.xdp}, {self.ydp})  '
            f'pose_topic={TOPIC_POSE}')

    # ═════════════════════════════════════════════════════════════════════
    #  HELPERS
    # ═════════════════════════════════════════════════════════════════════

    @staticmethod
    def orientation_error(theta: float, thetad: float) -> float:
        """Signed heading error wrapped to (−π, π]."""
        return (thetad - theta + math.pi) % (2.0 * math.pi) - math.pi

    def lidar_index(self, angle_rad: float) -> int:
        n   = len(self.LidarMsg.ranges)
        idx = int(round((angle_rad - self.LidarMsg.angle_min)
                        / self.LidarMsg.angle_increment))
        return idx % n

    def sector_min_range(self, ranges: np.ndarray,
                         centre: float, half_width: float = 0.25) -> float:
        n      = len(ranges)
        n_rays = max(1, int(round(half_width / self.LidarMsg.angle_increment)))
        ci     = self.lidar_index(centre)
        vals   = [ranges[(ci + d) % n]
                  for d in range(-n_rays, n_rays + 1)
                  if np.isfinite(ranges[(ci + d) % n])]
        return float(min(vals)) if vals else np.inf

    def publish(self, linear_x: float, angular_z: float) -> None:
        self.controlVel.linear.x  = float(np.clip(linear_x,
                                                   -self.max_linear_vel,
                                                    self.max_linear_vel))
        self.controlVel.linear.y  = 0.0
        self.controlVel.linear.z  = 0.0
        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = float(np.clip(angular_z,
                                                   -self.max_angular_vel,
                                                    self.max_angular_vel))
        self.ControlPublisher.publish(self.controlVel)

    # ═════════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═════════════════════════════════════════════════════════════════════

    def odom_callback(self, msg: Odometry) -> None:
        self.OdometryMsg = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self.LidarMsg = msg

    def gt_pose_callback(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        robot_tf = None
        for tf in msg.transforms:
            if 0.005 < tf.transform.translation.z < 0.05:
                robot_tf = tf
                break
        if robot_tf is None:
            robot_tf = msg.transforms[0]
        t = robot_tf.transform
        _, _, yaw = euler_from_quaternion(
            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        self._gt_x     = t.translation.x
        self._gt_y     = t.translation.y
        self._gt_theta = yaw
        self._gt_ready = True

    # ═════════════════════════════════════════════════════════════════════
    #  LIDAR PROCESSING
    # ═════════════════════════════════════════════════════════════════════

    def _extract_obstacles(self, x: float, y: float, theta: float):
        raw   = np.array(self.LidarMsg.ranges, dtype=np.float64)
        a_min = self.LidarMsg.angle_min
        a_inc = self.LidarMsg.angle_increment
        r_min = self.LidarMsg.range_min
        r_max = self.LidarMsg.range_max

        valid   = np.isfinite(raw) & (raw > r_min) & (raw <= r_max)
        lr      = np.where(valid, raw, np.inf)
        has_obs = bool(np.any(valid))

        clusters = []
        if has_obs:
            idx_v  = np.where(np.isfinite(lr))[0]
            splits = np.where(np.diff(idx_v) > 1)[0] + 1
            for part in np.split(idx_v, splits):
                seg     = lr[part]
                best    = int(np.argmin(seg))
                r       = float(seg[best])
                ray_i   = part[best]
                bearing = float(a_min + a_inc * ray_i)
                wx      = x + r * math.cos(bearing + theta)
                wy      = y + r * math.sin(bearing + theta)
                clusters.append((wx, wy, r, bearing))

        return raw, valid, lr, has_obs, clusters

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

    def _wall_follow_step(self, x: float, y: float,
                          theta: float, lr: np.ndarray):
        """
        Drive straight, parallel to the right wall (right-hand rule).

        The key improvement over a single-sensor P-controller is that
        TWO diagonal rays measure the wall on the robot's side.  This lets
        the controller fix both the DISTANCE to the wall AND the HEADING
        relative to it in one step, so the robot moves in a straight line
        instead of oscillating or circling.

        Ray geometry (right wall shown, wall_side = +1):

            Robot frame  (+x forward, +y left)

                              forward (+x)
                                  ↑
                          d_fs  ╱ │
                       (−45°) ╱   │
                             ╱    │
            ◄────────────── ●     │ ──── right side (−y)
            left (+y)        ╲    │
                        d_rs  ╲   │
                       (−135°) ╲  │
                                 ╲│
                           wall_dist (−90°) = perpendicular to wall

        dist_err  = wall_dist − target_dist
            > 0  robot too far from wall   → steer toward wall  (−ω for right)
            < 0  robot too close to wall   → steer away         (+ω for right)

        align_err = d_fs − d_rs
            > 0  front further from wall → nose angling away  → steer right (−ω)
            < 0  front closer  to  wall  → nose angling into  → steer left  (+ω)

        Combined:
            ω = −wall_side × (k_wall × dist_err + k_align × align_err)

        The wall_side sign makes the same formula work for left or right.
        """
        # ── Three LiDAR sectors on the wall side ──────────────────────────
        #   Multiply by wall_side so angles flip correctly for left/right.
        side_90    = self.wall_side * (-math.pi / 2)           # perpendicular
        side_front = self.wall_side * (-math.pi / 4)           # 45° fwd-side
        side_rear  = self.wall_side * (-3.0 * math.pi / 4)     # 45° rwd-side

        wall_dist  = self.sector_min_range(lr, side_90,    half_width=0.20)
        d_fs       = self.sector_min_range(lr, side_front, half_width=0.20)
        d_rs       = self.sector_min_range(lr, side_rear,  half_width=0.20)
        front_dist = self.sector_min_range(lr, 0.0,        half_width=0.35)

        # ── Right-hand rule: always choose right wall on state entry ──────
        if self.wall_side == 0:
            self.wall_side = 1
            self.get_logger().info(
                f'[WALL] Entering wall-follow — RIGHT-HAND RULE  '
                f'(right wall dist={wall_dist:.2f} m)')

        # ── Distance error ────────────────────────────────────────────────
        #   Cap at gstarp so free-space (inf) doesn't produce a huge term.
        eff_dist = min(wall_dist, self.gstarp)
        dist_err = eff_dist - self.wall_target_dist

        # ── Alignment error ───────────────────────────────────────────────
        #   Cap readings so inf doesn't skew the difference.
        align_err = min(d_fs, self.gstarp) - min(d_rs, self.gstarp)

        # ── Combined angular command ──────────────────────────────────────
        angular_z = -(self.wall_side * (self.k_wall  * dist_err
                                      + self.k_align * align_err))

        # ── Corner handling: front blocked → pivot away from wall ─────────
        front_clear_thresh = 0.35
        if front_dist < front_clear_thresh:
            # Right-hand rule corner: cannot go right or straight → turn left
            angular_z = self.wall_side * self.max_angular_vel * 0.75
            linear_x  = 0.05
            self.get_logger().info(
                '[WALL] Corner — pivoting away from wall',
                throttle_duration_sec=0.5)
        else:
            linear_x  = self.wall_follow_speed
            angular_z = float(np.clip(angular_z,
                                      -self.max_angular_vel,
                                       self.max_angular_vel))

        self.get_logger().info(
            f'[WALL]  dist_err={dist_err:+.3f}  align_err={align_err:+.3f}  '
            f'ω={angular_z:+.2f}  front={front_dist:.2f}',
            throttle_duration_sec=0.3)

        # ── Exit condition ────────────────────────────────────────────────
        goal_bearing_world = math.atan2(self.ydp - y, self.xdp - x)
        goal_bearing_robot = self.orientation_error(theta, goal_bearing_world)
        goal_sector_dist   = self.sector_min_range(lr, goal_bearing_robot,
                                                   half_width=0.45)

        if self.wall_follow_timer >= self.wall_follow_max_sec:
            self.get_logger().warn(
                '[WALL] Hard timeout — forcing return to APF')
            return linear_x, angular_z, True

        path_is_clear   = goal_sector_dist > self.gstarp * 1.1
        dwell_satisfied = self.wall_follow_timer >= self.wall_follow_min_sec

        if path_is_clear and dwell_satisfied:
            self.get_logger().info(
                f'[WALL] Goal path clear ({goal_sector_dist:.2f} m) '
                f'after {self.wall_follow_timer:.1f} s — returning to APF')
            return linear_x, angular_z, True

        return linear_x, angular_z, False

    # ═════════════════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP  (10 Hz)
    # ═════════════════════════════════════════════════════════════════════

    def control_loop(self) -> None:

        if len(self.LidarMsg.ranges) == 0:
            self.get_logger().info(
                'Waiting for LiDAR…', throttle_duration_sec=2.0)
            return

        if not self._gt_ready:
            self.get_logger().info(
                f'Waiting for pose on {TOPIC_POSE} …',
                throttle_duration_sec=2.0)
            return

        if self.goal_reached:
            self.publish(0.0, 0.0)
            return

        x, y, theta = self._gt_x, self._gt_y, self._gt_theta
        now         = self.get_clock().now().nanoseconds * 1e-9

        if self._time_started is None:
            self._time_started = now

        raw, valid, lr, has_obs, clusters = self._extract_obstacles(x, y, theta)

        # ═══════════════════════════════════════════
        #  STATE: WALL FOLLOWING
        # ═══════════════════════════════════════════
        if self.state == STATE_WALL_FOLLOW:

            self.wall_follow_timer += self.dt

            linear_x, angular_z, exit_flag = self._wall_follow_step(
                x, y, theta, lr)

            self.publish(linear_x, angular_z)

            self.get_logger().info(
                f'[WALL]  pose=({x:.2f},{y:.2f})  θ={math.degrees(theta):.1f}°  '
                f'side={"R" if self.wall_side == 1 else "L"}  '
                f't={self.wall_follow_timer:.1f} s  '
                f'v={linear_x:.2f}  ω={angular_z:.2f}',
                throttle_duration_sec=0.3)

            if exit_flag:
                self.state             = STATE_APF
                self.wall_side         = 0
                self.wall_follow_timer = 0.0
                self.stuck_timer       = 0.0
                self.position_window.clear()
                self.get_logger().info('[WALL→APF] Resuming potential field navigation')

            return

        # ═══════════════════════════════════════════
        #  STATE: APF
        # ═══════════════════════════════════════════

        is_stuck = self._update_stuck(x, y, now)
        if is_stuck:
            self.get_logger().warn(
                f'[APF] Stuck detected (stuck_timer={self.stuck_timer:.1f} s) '
                f'— switching to WALL FOLLOW')
            self.state             = STATE_WALL_FOLLOW
            self.wall_side         = 0
            self.wall_follow_timer = 0.0
            self.stuck_timer       = 0.0
            self.position_window.clear()
            self.publish(0.0, 0.0)
            return

        # 1. Attractive force
        vectorD = np.array([[x - self.xdp], [y - self.ydp]])
        AF      = -(self.kap * vectorD)

        # 2. Repulsive force
        RF = np.zeros((2, 1))
        if has_obs:
            for (wx, wy, _, __) in clusters:
                g = math.hypot(x - wx, y - wy)
                if 1e-6 < g <= self.gstarp:
                    coeff = self.krp * (1.0 / self.gstarp - 1.0 / g) / g ** 3
                    RF[0, 0] += coeff * (x - wx)
                    RF[1, 0] += coeff * (y - wy)

        RF = -RF

        # 3. Combined force
        F    = AF + RF if has_obs else AF
        fmag = math.hypot(float(F[0, 0]), float(F[1, 0]))

        # 4. Goal-reached check
        dist_to_goal = float(np.linalg.norm(vectorD))
        if dist_to_goal < self.eps_control:
            self.goal_reached = True
            self.get_logger().info(f'[APF] Goal reached!  pose=({x:.3f}, {y:.3f})')
            self.publish(0.0, 0.0)
            self.timer.cancel()
            return

        # 5. Heading
        if fmag < self.FORCE_MIN_MAG:
            thetaD  = theta
            eorient = 0.0
        else:
            thetaD  = math.atan2(float(F[1, 0]), float(F[0, 0]))
            eorient = self.orientation_error(theta, thetaD)

        thetavel = float(np.clip(self.kthetap * eorient,
                                 -self.max_angular_vel, self.max_angular_vel))

        # 6. Linear speed
        if abs(eorient) > self.eps_orient:
            xvel = 0.05
        else:
            if has_obs:
                all_valid = raw[valid]
                closest   = float(np.min(all_valid)) if all_valid.size else self.gstarp
                spd_scale = float(np.clip((closest / self.gstarp) ** 0.5, 0.0, 1.0))
            else:
                spd_scale = 1.0

            goal_scale = float(np.clip(dist_to_goal / 0.5, 0.2, 1.0))
            base_speed = min(self.max_linear_vel, fmag)
            xvel       = max(0.05, base_speed * min(spd_scale, goal_scale))

        self.publish(xvel, thetavel)

        self.get_logger().info(
            f'[APF]  pose=({x:.2f},{y:.2f})  θ={math.degrees(theta):.1f}°  '
            f'dist={dist_to_goal:.2f} m  '
            f'eθ={math.degrees(eorient):.1f}°  '
            f'v={xvel:.2f}  ω={thetavel:.2f}  '
            f'stuck={self.stuck_timer:.1f} s',
            throttle_duration_sec=0.3)


# ═════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
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
