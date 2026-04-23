# # import math
# # import numpy as np
# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import Twist
# # from sensor_msgs.msg import LaserScan
# # from nav_msgs.msg import Odometry
# # from tf_transformations import euler_from_quaternion
# # from tf2_msgs.msg import TFMessage
# # from collections import deque

# # # ─────────────────────────────────────────────
# # #  TOPIC NAMES
# # # ─────────────────────────────────────────────
# # TOPIC_CMD_VEL = 'cmd_vel'
# # TOPIC_ODOM    = 'odom'
# # TOPIC_SCAN    = 'scan'
# # TOPIC_POSE    = '/world/maze_world/dynamic_pose/info'

# # # ─────────────────────────────────────────────
# # #  HIGH-LEVEL NAVIGATION STATES
# # # ─────────────────────────────────────────────
# # STATE_APF    = 0
# # # STATE_ESCAPE = 1      # active escape manoeuvre

# # # # Escape sub-phases (executed in order)
# # # ESCAPE_BACKUP = 0     # reverse away from the corner
# # # ESCAPE_ROTATE = 1     # spin toward the open side
# # # ESCAPE_FORWARD = 2
# # STATE_WALL_FOLLOW = 1

# # # ═════════════════════════════════════════════════════════════════════════════
# # #  POTENTIAL FIELD PLANNER NODE
# # # ═════════════════════════════════════════════════════════════════════════════
# # class PotentialFieldPlanner(Node):

# #     def __init__(self):
# #         super().__init__('potential_field_planner')

# #         # ── ROS 2 parameters ──────────────────────────────────────────────
# #         self.declare_parameter('goal_x',          9.0)
# #         self.declare_parameter('goal_y',          9.0)
# #         self.declare_parameter('k_att',           1.0)
# #         self.declare_parameter('k_rep',           3.0)
# #         self.declare_parameter('d_obs',           1.2)
# #         self.declare_parameter('max_linear_vel',  0.4)
# #         self.declare_parameter('max_angular_vel', 1.0)

# #         self.xdp             = self.get_parameter('goal_x').value
# #         self.ydp             = self.get_parameter('goal_y').value
# #         self.kap             = self.get_parameter('k_att').value
# #         self.krp             = self.get_parameter('k_rep').value
# #         self.gstarp          = self.get_parameter('d_obs').value
# #         self.max_linear_vel  = self.get_parameter('max_linear_vel').value
# #         self.max_angular_vel = self.get_parameter('max_angular_vel').value

# #         # ── Control constants ─────────────────────────────────────────────
# #         self.kthetap     = 4.0
# #         self.eps_orient  = math.pi / 6   # rotate-in-place threshold [rad]
# #         self.eps_control = 0.2           # goal-reached radius [m]
# #         self.dt          = 0.1           # control loop period [s]

# #         # Below this combined-force magnitude the gradient is noise —
# #         # skip the heading command and let stuck-detection fire instead.
# #         self.FORCE_MIN_MAG = 0.05

# #         # ── Message stores ────────────────────────────────────────────────
# #         self.OdometryMsg = Odometry()
# #         self.LidarMsg    = LaserScan()

# #         # ── Ground-truth pose (Gazebo TFMessage) ──────────────────────────
# #         self._gt_x     = 0.5
# #         self._gt_y     = 0.5
# #         self._gt_theta = 0.0
# #         self._gt_ready = False

# #         # ── Velocity command ──────────────────────────────────────────────
# #         self.controlVel = Twist()

# #         # ── Publisher / Subscribers ───────────────────────────────────────
# #         self.ControlPublisher = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)

# #         self.create_subscription(Odometry,  TOPIC_ODOM, self.odom_callback,    10)
# #         self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback,    10)
# #         self.create_subscription(TFMessage, TOPIC_POSE, self.gt_pose_callback, 10)

# #         self.timer = self.create_timer(self.dt, self.control_loop)

# #         # ── Goal flag ─────────────────────────────────────────────────────
# #         self.goal_reached = False

# #         # ─────────────────────────────────────────────────────────────────
# #         #  STUCK DETECTION
# #         #
# #         #  A sliding time window of (timestamp, x, y) entries is kept.
# #         #  Each tick the net displacement from the OLDEST entry to the
# #         #  CURRENT position is measured.  This stays small whether the
# #         #  robot is fully stationary OR slowly circling in place, so both
# #         #  failure modes trigger the escape correctly.
# #         # ─────────────────────────────────────────────────────────────────
# #         self.position_window      = deque()
# #         self.stuck_window_sec     = 4.0    # look-back period  [s]
# #         self.stuck_disp_threshold = 0.20   # net travel threshold [m]
# #         self.stuck_timer          = 0.0    # cumulative "stuck" seconds [s]
# #         self.stuck_threshold      = 3.0    # fire escape after this [s]
# #         self._time_started = None  # set on first control loop tick

# #         # ── Wall_Follow Parameters ────────────────────────────────────────
# #         self.wall_follow_dist = 1.0   # desired distance from wall
# #         self.wall_kp = 2.0           # control gain
# #         self.wall_side = None    
# #         self.wf_start_x       = None
# #         self.wf_start_y       = None
# #         self.wf_min_travel    = 2.5  
# #         self.nav_state = STATE_APF
# #         self.virtual_obstacles    = []
# #         self.virtual_obs_radius   = 2.0
# #         self.virtual_obs_strength = 4.0
# #         self.wf_max_time  = 60.0   # [s] hard timeout
# #         self.wf_start_time = None
# #         self.mline_start_x = None
# #         self.mline_start_y = None
        
# #         self.get_logger().info(
# #             f'Planner Ready — goal=({self.xdp}, {self.ydp})  '
# #             f'pose_topic={TOPIC_POSE}')

# #     # ═════════════════════════════════════════════════════════════════════
# #     #  HELPERS
# #     # ═════════════════════════════════════════════════════════════════════

# #     @staticmethod
# #     def orientation_error(theta: float, thetad: float) -> float:
# #         """Signed heading error wrapped to (−π, π]."""
# #         return (thetad - theta + math.pi) % (2.0 * math.pi) - math.pi

# #     def lidar_index(self, angle_rad: float) -> int:
# #         """Array index for a robot-frame angle."""
# #         n   = len(self.LidarMsg.ranges)
# #         idx = int(round((angle_rad - self.LidarMsg.angle_min)
# #                         / self.LidarMsg.angle_increment))
# #         return idx % n

# #     def sector_min_range(self, ranges: np.ndarray,
# #                          centre: float, half_width: float = 0.25) -> float:
# #         """
# #         Minimum finite range within [centre − half_width, centre + half_width].
# #         Returns np.inf when no valid reading exists in the sector.

# #         Parameters
# #         ----------
# #         ranges     : np.ndarray  — full validated range array (inf = invalid)
# #         centre     : float       — sector centre angle in robot frame [rad]
# #         half_width : float       — half-arc width [rad]
# #         """
# #         n      = len(ranges)          # always an ndarray — never a scalar
# #         n_rays = max(1, int(round(half_width / self.LidarMsg.angle_increment)))
# #         ci     = self.lidar_index(centre)
# #         vals   = [ranges[(ci + d) % n]
# #                   for d in range(-n_rays, n_rays + 1)
# #                   if np.isfinite(ranges[(ci + d) % n])]
# #         return float(min(vals)) if vals else np.inf

# #     def publish(self, linear_x: float, angular_z: float) -> None:
# #         self.controlVel.linear.x  = float(np.clip(linear_x,
# #                                                    -self.max_linear_vel,
# #                                                     self.max_linear_vel))
# #         self.controlVel.linear.y  = 0.0
# #         self.controlVel.linear.z  = 0.0
# #         self.controlVel.angular.x = 0.0
# #         self.controlVel.angular.y = 0.0
# #         self.controlVel.angular.z = float(np.clip(angular_z,
# #                                                    -self.max_angular_vel,
# #                                                     self.max_angular_vel))
# #         self.ControlPublisher.publish(self.controlVel)

# #     # ═════════════════════════════════════════════════════════════════════
# #     #  CALLBACKS
# #     # ═════════════════════════════════════════════════════════════════════

# #     def odom_callback(self, msg: Odometry) -> None:
# #         self.OdometryMsg = msg   # pose ignored; only twist could be used

# #     def scan_callback(self, msg: LaserScan) -> None:
# #         self.LidarMsg = msg

# #     def gt_pose_callback(self, msg: TFMessage) -> None:
# #         """
# #         Extract robot base pose from the Gazebo dynamic-pose TFMessage.

# #         The message contains transforms for ALL model links.  Identify the
# #         robot's base_link by its z-translation height:
# #           • base_link  ≈ 0.010 m  →  0.005 < z < 0.05   ← target
# #           • wheel / caster links have z ≈ 0 or slightly negative
# #         Falls back to transforms[0] when no link passes the height filter.
# #         """
# #         if not msg.transforms:
# #             return

# #         robot_tf = None
# #         for tf in msg.transforms:
# #             if 0.005 < tf.transform.translation.z < 0.05:
# #                 robot_tf = tf
# #                 break

# #         if robot_tf is None:
# #             robot_tf = msg.transforms[0]

# #         t = robot_tf.transform
# #         _, _, yaw = euler_from_quaternion(
# #             [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

# #         self._gt_x     = t.translation.x
# #         self._gt_y     = t.translation.y
# #         self._gt_theta = yaw
# #         self._gt_ready = True

# #     # ═════════════════════════════════════════════════════════════════════
# #     #  LIDAR PROCESSING
# #     # ═════════════════════════════════════════════════════════════════════

# #     def _extract_obstacles(self, x: float, y: float, theta: float):
# #         """
# #         Segment valid LiDAR returns into contiguous clusters.
# #         Returns the closest point per cluster together with its world-frame
# #         position, range, and robot-frame bearing.

# #         Returns
# #         -------
# #         raw        : np.ndarray  — raw ranges (may contain inf / nan)
# #         valid      : np.ndarray  — boolean mask of usable returns
# #         lr         : np.ndarray  — validated ranges (np.inf where invalid)
# #         has_obs    : bool        — True if any valid return exists
# #         clusters   : list of (world_x, world_y, range, bearing)
# #         """
# #         raw   = np.array(self.LidarMsg.ranges, dtype=np.float64)
# #         a_min = self.LidarMsg.angle_min
# #         a_inc = self.LidarMsg.angle_increment
# #         r_min = self.LidarMsg.range_min
# #         r_max = self.LidarMsg.range_max

# #         valid   = np.isfinite(raw) & (raw > r_min) & (raw <= r_max)
# #         lr      = np.where(valid, raw, np.inf)
# #         has_obs = bool(np.any(valid))

# #         clusters = []
# #         if has_obs:
# #             idx_v  = np.where(np.isfinite(lr))[0]
# #             splits = np.where(np.diff(idx_v) > 1)[0] + 1
# #             for part in np.split(idx_v, splits):
# #                 seg     = lr[part]
# #                 best    = int(np.argmin(seg))
# #                 r       = float(seg[best])
# #                 ray_i   = part[best]
# #                 bearing = float(a_min + a_inc * ray_i)
# #                 wx      = x + r * math.cos(bearing + theta)
# #                 wy      = y + r * math.sin(bearing + theta)
# #                 clusters.append((wx, wy, r, bearing))

# #         return raw, valid, lr, has_obs, clusters
    
# #     def run_wall_follow(self, lr):
# #         front = self.sector_min_range(lr, 0.0, 0.20)
        
# #         if self.wall_side == "RIGHT":
# #             side     = self.sector_min_range(lr, -math.pi / 2, 0.25)
# #             if not np.isfinite(side):
# #                 side = self.wall_follow_dist * 2.5  # treat open space as far wall → gentle drift back
# #             error    =  self.wall_follow_dist - side
# #             angular  =  self.wall_kp * error
# #             turn_dir =  1.5
# #         else:  # LEFT
# #             side     = self.sector_min_range(lr,  math.pi / 2, 0.25)
# #             if not np.isfinite(side):
# #                 side = self.wall_follow_dist * 2.5  # treat open space as far wall → gentle drift back
# #             error    =  self.wall_follow_dist - side
# #             angular  = -self.wall_kp * error
# #             turn_dir = -1.5

# #         if front < 0.4:
# #             angular = turn_dir

# #         self.publish(0.15, angular)
# #         self.get_logger().info(
# #             f'[WF-{self.wall_side}] front={front:.2f} side={side:.2f} '
# #             f'err={error:.2f} ω={angular:.2f}',
# #             throttle_duration_sec=0.4)
    
# #     # ═════════════════════════════════════════════════════════════════════
# #     #  MAIN CONTROL LOOP  (10 Hz)
# #     # ═════════════════════════════════════════════════════════════════════

# #     def control_loop(self) -> None:

# #         # ── Guards ────────────────────────────────────────────────────────
# #         if len(self.LidarMsg.ranges) == 0:
# #             self.get_logger().info(
# #                 'Waiting for LiDAR…', throttle_duration_sec=2.0)
# #             return

# #         if not self._gt_ready:
# #             self.get_logger().info(
# #                 f'Waiting for pose on {TOPIC_POSE} …',
# #                 throttle_duration_sec=2.0)
# #             return

# #         if self.goal_reached:
# #             self.publish(0.0, 0.0)
# #             return
        

# #         # ── Ground-truth pose ─────────────────────────────────────────────
# #         x, y, theta = self._gt_x, self._gt_y, self._gt_theta

# #         # ── LiDAR processing ──────────────────────────────────────────────
# #         raw, valid, lr, has_obs, clusters = self._extract_obstacles(x, y, theta)
        
# #         # ─────────────────────────────────────────────────────────────────
# #         #  STUCK DETECTION  (APF mode only)
# #         # ─────────────────────────────────────────────────────────────────
# #         now = self.get_clock().now().nanoseconds * 1e-9

# #         if self._time_started is None:
# #             self._time_started = now

# #         # ── Wall-follow dispatch ──────────────────────────────────────────
# #         if self.nav_state == STATE_WALL_FOLLOW:
# #             # Goal check — must work in ALL states
# #             if math.hypot(self.xdp - x, self.ydp - y) < self.eps_control:
# #                 self.goal_reached = True
# #                 self.get_logger().info(f'[WF] Goal reached!  pose=({x:.3f}, {y:.3f})')
# #                 self.publish(0.0, 0.0)
# #                 self.timer.cancel()
# #                 return
# #             self.run_wall_follow(lr)
# #             if self.wf_start_x is not None:
# #                 travelled = math.hypot(x - self.wf_start_x, y - self.wf_start_y)
# #                 front     = self.sector_min_range(lr, 0.0, 0.25)
# #                 # goal_dist_now   = math.hypot(self.xdp - x, self.ydp - y)
# #                 # goal_dist_start = math.hypot(self.xdp - self.wf_start_x, self.ydp - self.wf_start_y)
# #                 # progress_exit = travelled >= self.wf_min_travel and front > 0.8 and goal_dist_now < goal_dist_start - 0.3
# #                 # timeout_exit  = travelled >= 6.0   # hard cap — never circle more than 6 m
# #                 # time_exit = (now - self.wf_start_time) >= self.wf_max_time
# #                 # if progress_exit or timeout_exit or time_exit:
# #                 goal_dist_now   = math.hypot(self.xdp - x, self.ydp - y)
# #                 goal_dist_start = math.hypot(self.xdp - self.mline_start_x, self.ydp - self.mline_start_y)
# #                 # Distance from robot to the M-line (stuck_point → goal)
# #                 dx = self.xdp - self.mline_start_x
# #                 dy = self.ydp - self.mline_start_y
# #                 mline_len = math.hypot(dx, dy)
# #                 if mline_len > 1e-6:
# #                     dist_to_mline = abs(dy * x - dx * y + self.xdp * self.mline_start_y
# #                         - self.ydp * self.mline_start_x) / mline_len
# #                 else:
# #                     dist_to_mline = 0.0

# #                 on_mline      = dist_to_mline < 0.25          # within 25 cm of M-line
# #                 closer_to_goal = goal_dist_now < goal_dist_start - 0.5
# #                 mline_exit    = on_mline and closer_to_goal and travelled >= self.wf_min_travel and front > 0.8
# #                 timeout_exit  = (now - self.wf_start_time) >= self.wf_max_time

# #                 if mline_exit or timeout_exit:
# #                     self.get_logger().info(
# #                         f'[WF] Exit after {travelled:.2f} m — back to APF')
# #                     self.nav_state     = STATE_APF
# #                     self.wf_start_x    = None
# #                     self.wf_start_y    = None
# #                     self.stuck_timer   = 0.0
# #                     self.position_window.clear()
# #                     self._time_started = now
# #             return
        
# #         # ── Stuck detection (APF mode only) ──────────────────────────────
# #         self.position_window.append((now, x, y))

# #         # Evict entries older than the look-back window
# #         while (self.position_window and
# #                now - self.position_window[0][0] > self.stuck_window_sec):
# #             self.position_window.popleft()

# #         # Measure net displacement from the oldest entry to now
# #         if len(self.position_window) >= 2:
# #             _, x0, y0 = self.position_window[0]
# #             net_disp  = math.sqrt((x - x0)**2 + (y - y0)**2)
# #             window_span = now - self.position_window[0][0]
# #             if window_span >= self.stuck_window_sec * 0.9:
# #                 if net_disp < self.stuck_disp_threshold:
# #                     self.stuck_timer += self.dt
# #                 else:
# #                     self.stuck_timer = 0.0
# #             else:
# #                 self.stuck_timer = 0.0

# #         # Trigger escape once stuck long enough
# #         grace_elapsed = (now - self._time_started) >= 5.0   # 5 ثواني grace period
# #         if self.stuck_timer >= self.stuck_threshold and grace_elapsed:
# #             self.get_logger().warn(f'[STUCK] at ({x:.2f}, {y:.2f}) — wall follow RIGHT')
# #             self.nav_state   = STATE_WALL_FOLLOW
# #             self.wf_start_x  = x
# #             self.wf_start_y  = y
# #             self.mline_start_x = x
# #             self.mline_start_y = y
# #             self.wf_start_time = now
# #             self.virtual_obstacles.append((x, y))
# #             self.stuck_timer      = 0.0
# #             self.position_window.clear()
# #             self._time_started = now
            
# #             # ── NEW: SVD wall-side selection ──────────────────────────────
# #             a_min = self.LidarMsg.angle_min
# #             a_inc = self.LidarMsg.angle_increment

# #             front_pts = [
# #                 [x + lr[i] * math.cos(a_min + a_inc * i + theta),
# #                 y + lr[i] * math.sin(a_min + a_inc * i + theta)]
# #                 for i in range(len(lr))
# #                 if abs(a_min + a_inc * i) <= math.pi / 4 and np.isfinite(lr[i])
# #             ]
# #             if len(front_pts) >= 2:
# #                 pts   = np.array(front_pts)
# #                 diffs = pts - pts.mean(axis=0)
# #                 _, _, Vt = np.linalg.svd(diffs, full_matrices=False)
# #                 wt = Vt[0]
# #             else:
# #                 wt = np.array([-math.sin(theta), math.cos(theta)])
            
# #             goal_vec = np.array([self.xdp - x, self.ydp - y])
# #             if np.dot(wt, goal_vec) < 0:
# #                 wt = -wt

# #             heading = np.array([math.cos(theta), math.sin(theta)])
# #             cross   = heading[0] * wt[1] - heading[1] * wt[0]
# #             self.wall_side = "LEFT" if cross > 0 else "RIGHT"
# #             # ─────────────────────────────────────────────────────────────

# #             self.get_logger().warn(
# #                 f'[STUCK] at ({x:.2f}, {y:.2f}) — wall follow {self.wall_side} '
# #                 f'(wt=[{wt[0]:.2f},{wt[1]:.2f}] cross={cross:.2f})')
# #             self.run_wall_follow(lr)
# #             return
                        
# #         # ═════════════════════════════════════════
# #         #  APF CONTROL
# #         # ═════════════════════════════════════════
# #         # 1. Attractive force
# #         vectorD = np.array([[x - self.xdp], [y - self.ydp]])
# #         AF      = -(self.kap * vectorD)

# #         # 2. Repulsive force
# #         RF = np.zeros((2, 1))
# #         if has_obs:
# #             for (wx, wy, _, __) in clusters:
# #                 g = math.hypot(x - wx, y - wy)
# #                 if 1e-6 < g <= self.gstarp:
# #                     coeff = self.krp * (1.0 / self.gstarp - 1.0 / g) / g ** 3
# #                     RF[0, 0] += coeff * (x - wx)
# #                     RF[1, 0] += coeff * (y - wy)

# #         RF = -RF

# #         # Virtual obstacles (previously stuck positions)
# #         for (vx, vy) in self.virtual_obstacles:
# #             g = math.hypot(x - vx, y - vy)
# #             if 1e-6 < g <= self.virtual_obs_radius:
# #                 coeff = self.virtual_obs_strength * (
# #                     1.0 / self.virtual_obs_radius - 1.0 / g) / g ** 3
# #                 RF[0, 0] -= coeff * (x - vx)
# #                 RF[1, 0] -= coeff * (y - vy)

# #         # 3. Combined force
# #         F    = AF + RF if has_obs else AF
# #         fmag = math.hypot(float(F[0, 0]), float(F[1, 0]))

# #         # 4. Goal-reached check
# #         # dist_to_goal = math.hypot(dx_goal, dy_goal)
# #         dist_to_goal = float(np.linalg.norm(vectorD))
# #         if dist_to_goal < self.eps_control:
# #             self.goal_reached = True
# #             self.get_logger().info(f'[APF] Goal reached!  pose=({x:.3f}, {y:.3f})')
# #             self.publish(0.0, 0.0)
# #             self.timer.cancel()
# #             return

# #         # 5. Heading
# #         if fmag < self.FORCE_MIN_MAG:
# #             # Near-zero gradient: hold heading, let stuck-detection react
# #             thetaD  = theta
# #             eorient = 0.0
# #         else:
# #             thetaD  = math.atan2(float(F[1, 0]), float(F[0, 0]))
# #             eorient = self.orientation_error(theta, thetaD)

# #         thetavel = float(np.clip(self.kthetap * eorient,
# #                                 -self.max_angular_vel, self.max_angular_vel))

# #             # 6. Linear speed
# #         if abs(eorient) > self.eps_orient:
# #             xvel = 0.05   # rotate in place for large heading errors
# #         else:
# #             if has_obs:
# #                 all_valid = raw[valid]
# #                 closest   = float(np.min(all_valid)) if all_valid.size else self.gstarp
# #                 spd_scale = float(np.clip((closest / self.gstarp) ** 0.5, 0.0, 1.0))
# #             else:
# #                 spd_scale = 1.0

# #             goal_scale = float(np.clip(dist_to_goal / 0.5, 0.2, 1.0))
# #             base_speed = min(self.max_linear_vel, fmag)
# #             xvel       = max(0.05, base_speed * min(spd_scale, goal_scale))

# #         self.publish(xvel, thetavel)

# #         self.get_logger().info(
# #             f'[APF]  pose=({x:.2f},{y:.2f})  θ={math.degrees(theta):.1f}°  '
# #             f'dist={dist_to_goal:.2f} m  '
# #             f'eθ={math.degrees(eorient):.1f}°  '
# #             f'v={xvel:.2f}  ω={thetavel:.2f}  '
# #             f'stuck={self.stuck_timer:.1f} s',
# #             throttle_duration_sec=0.3)


# # # ═════════════════════════════════════════════════════════════════════════════
# # #  ENTRY POINT
# # # ═════════════════════════════════════════════════════════════════════════════

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = PotentialFieldPlanner()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.publish(0.0, 0.0)
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()

# import math
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
# from tf2_msgs.msg import TFMessage
# from collections import deque

# # ─────────────────────────────────────────────
# #  TOPIC NAMES
# # ─────────────────────────────────────────────
# TOPIC_CMD_VEL = 'cmd_vel'
# TOPIC_ODOM    = 'odom'
# TOPIC_SCAN    = 'scan'
# TOPIC_POSE    = '/world/maze_world/dynamic_pose/info'

# # ─────────────────────────────────────────────
# #  WALL-FOLLOW STATE CONSTANTS
# # ─────────────────────────────────────────────
# # The robot operates in one of three states:
# #   APF        – normal potential-field navigation
# #   WF_FOLLOW  – wall-follow escape: moving parallel to the front wall
# #   WF_TURN    – wall-follow turn: the front wall ended, turning right to exit
# WF_APF    = 'APF'
# WF_FOLLOW = 'WF_FOLLOW'
# WF_TURN   = 'WF_TURN'

# # ═════════════════════════════════════════════════════════════════════════════
# #  POTENTIAL FIELD PLANNER NODE
# # ═════════════════════════════════════════════════════════════════════════════
# class PotentialFieldPlanner(Node):

#     def __init__(self):
#         super().__init__('potential_field_planner')

#         # ── ROS 2 parameters ──────────────────────────────────────────────
#         self.declare_parameter('goal_x',          9.0)
#         self.declare_parameter('goal_y',          9.0)
#         self.declare_parameter('k_att',           1.0)
#         self.declare_parameter('k_rep',           3.0)
#         self.declare_parameter('d_obs',           1.2)
#         self.declare_parameter('max_linear_vel',  0.4)
#         self.declare_parameter('max_angular_vel', 1.0)

#         self.xdp             = self.get_parameter('goal_x').value
#         self.ydp             = self.get_parameter('goal_y').value
#         self.kap             = self.get_parameter('k_att').value
#         self.krp             = self.get_parameter('k_rep').value
#         self.gstarp          = self.get_parameter('d_obs').value
#         self.max_linear_vel  = self.get_parameter('max_linear_vel').value
#         self.max_angular_vel = self.get_parameter('max_angular_vel').value

#         # ── Control constants ─────────────────────────────────────────────
#         self.kthetap     = 4.0
#         self.eps_orient  = math.pi / 6   # rotate-in-place threshold [rad]
#         self.eps_control = 0.2           # goal-reached radius [m]
#         self.dt          = 0.1           # control loop period [s]

#         self.FORCE_MIN_MAG = 0.05

#         # ── Message stores ────────────────────────────────────────────────
#         self.OdometryMsg = Odometry()
#         self.LidarMsg    = LaserScan()

#         # ── Ground-truth pose (Gazebo TFMessage) ──────────────────────────
#         self._gt_x     = 0.5
#         self._gt_y     = 0.5
#         self._gt_theta = 0.0
#         self._gt_ready = False

#         # ── Velocity command ──────────────────────────────────────────────
#         self.controlVel = Twist()

#         # ── Publisher / Subscribers ───────────────────────────────────────
#         self.ControlPublisher = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)

#         self.create_subscription(Odometry,  TOPIC_ODOM, self.odom_callback,    10)
#         self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback,    10)
#         self.create_subscription(TFMessage, TOPIC_POSE, self.gt_pose_callback, 10)

#         self.timer = self.create_timer(self.dt, self.control_loop)

#         # ── Goal flag ─────────────────────────────────────────────────────
#         self.goal_reached = False

#         # ── Stuck detection ───────────────────────────────────────────────
#         self.position_window      = deque()
#         self.stuck_window_sec     = 4.0
#         self.stuck_disp_threshold = 0.20
#         self.stuck_timer          = 0.0
#         self.stuck_threshold      = 3.0
#         self._time_started        = None

#         # ── Virtual obstacles (fix: initialise the list that the APF loop references) ──
#         self.virtual_obstacles   = []
#         self.virtual_obs_radius  = 0.6
#         self.virtual_obs_strength = 2.0

#         # ─────────────────────────────────────────────────────────────────
#         #  CORNER / WALL-FOLLOW STATE MACHINE
#         #
#         #  Corner geometry (from the image):
#         #    • Front wall  – obstacle directly ahead  (bearing ≈ 0)
#         #    • Left wall   – obstacle to the left     (bearing ≈ +π/2)
#         #
#         #  Entry condition  (ENTER_FRONT / ENTER_SIDE, with hysteresis):
#         #    Both front sector AND left sector are closer than CORNER_ENTER_DIST.
#         #    Using a tighter entry than exit prevents re-entry the moment the
#         #    robot barely clears the threshold.
#         #
#         #  WF_FOLLOW behaviour:
#         #    Keep the front wall on the LEFT of the robot.  Drive forward
#         #    while a P-controller holds the left-sector range at WALL_TARGET_DIST.
#         #    A heading drift guard (MAX_WF_HEADING_DEG) prevents the robot from
#         #    circling: if the robot's heading deviates too far from the wall's
#         #    perpendicular, a corrective angular command is added.
#         #
#         #  WF_TURN behaviour:
#         #    The front wall has ended (front sector > CORNER_EXIT_DIST).
#         #    Rotate right (clockwise) in place until the robot is aimed into
#         #    open space, then return to APF.
#         #
#         #  Exit condition  (back to APF):
#         #    Front sector > CORNER_EXIT_DIST  AND  left sector > CORNER_EXIT_DIST.
#         #    Both must be open so the attractive force cannot immediately pull
#         #    the robot back into the corner.
#         # ─────────────────────────────────────────────────────────────────

#         # Detection thresholds [m]
#         self.CORNER_ENTER_DIST = 0.50   # both sectors must be closer than this to enter WF
#         self.CORNER_EXIT_DIST  = 0.90   # both sectors must be farther than this to leave WF
#                                         # (hysteresis gap = 0.20 m prevents flickering)

#         # Wall-follow speed / controller
#         self.WF_LINEAR_VEL    = 0.15    # forward speed during wall-follow [m/s]
#         self.WALL_TARGET_DIST = 0.35    # desired distance from the front wall [m]
#         self.WF_KP            = 1.8     # P-gain for wall-distance controller

#         # Anti-circling: maximum allowed heading deviation from the
#         # wall-parallel direction before a corrective turn is applied.
#         self.MAX_WF_HEADING_ERR = math.radians(30)   # [rad]

#         # Turn speed used in WF_TURN state [rad/s]  (negative = clockwise = right)
#         self.WF_TURN_VEL = -0.5

#         # Maximum heading error to goal allowed when exiting WF_TURN [rad].
#         # The robot must be facing within this tolerance of the goal bearing
#         # before wall-follow hands back to APF.
#         self.WF_EXIT_HEADING_TOL = math.radians(40)

#         # Heading locked at the moment wall-follow was entered.
#         # The robot must move PARALLEL to the front wall, which means it should
#         # maintain the heading it had when the front wall was first detected.
#         self._wf_entry_heading = None

#         # Current navigation state
#         self._nav_state = WF_APF

#         self.get_logger().info(
#             f'Planner Ready — goal=({self.xdp}, {self.ydp})  '
#             f'pose_topic={TOPIC_POSE}')

#     # ═════════════════════════════════════════════════════════════════════
#     #  HELPERS
#     # ═════════════════════════════════════════════════════════════════════

#     @staticmethod
#     def orientation_error(theta: float, thetad: float) -> float:
#         """Signed heading error wrapped to (−π, π]."""
#         return (thetad - theta + math.pi) % (2.0 * math.pi) - math.pi

#     def lidar_index(self, angle_rad: float) -> int:
#         n   = len(self.LidarMsg.ranges)
#         idx = int(round((angle_rad - self.LidarMsg.angle_min)
#                         / self.LidarMsg.angle_increment))
#         return idx % n

#     def sector_min_range(self, ranges: np.ndarray,
#                          centre: float, half_width: float = 0.25) -> float:
#         n      = len(ranges)
#         n_rays = max(1, int(round(half_width / self.LidarMsg.angle_increment)))
#         ci     = self.lidar_index(centre)
#         vals   = [ranges[(ci + d) % n]
#                   for d in range(-n_rays, n_rays + 1)
#                   if np.isfinite(ranges[(ci + d) % n])]
#         return float(min(vals)) if vals else np.inf

#     def publish(self, linear_x: float, angular_z: float) -> None:
#         self.controlVel.linear.x  = float(np.clip(linear_x,
#                                                    -self.max_linear_vel,
#                                                     self.max_linear_vel))
#         self.controlVel.linear.y  = 0.0
#         self.controlVel.linear.z  = 0.0
#         self.controlVel.angular.x = 0.0
#         self.controlVel.angular.y = 0.0
#         self.controlVel.angular.z = float(np.clip(angular_z,
#                                                    -self.max_angular_vel,
#                                                     self.max_angular_vel))
#         self.ControlPublisher.publish(self.controlVel)

#     # ═════════════════════════════════════════════════════════════════════
#     #  CALLBACKS
#     # ═════════════════════════════════════════════════════════════════════

#     def odom_callback(self, msg: Odometry) -> None:
#         self.OdometryMsg = msg

#     def scan_callback(self, msg: LaserScan) -> None:
#         self.LidarMsg = msg

#     def gt_pose_callback(self, msg: TFMessage) -> None:
#         if not msg.transforms:
#             return
#         robot_tf = None
#         for tf in msg.transforms:
#             if 0.005 < tf.transform.translation.z < 0.05:
#                 robot_tf = tf
#                 break
#         if robot_tf is None:
#             robot_tf = msg.transforms[0]
#         t = robot_tf.transform
#         _, _, yaw = euler_from_quaternion(
#             [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
#         self._gt_x     = t.translation.x
#         self._gt_y     = t.translation.y
#         self._gt_theta = yaw
#         self._gt_ready = True

#     # ═════════════════════════════════════════════════════════════════════
#     #  LIDAR PROCESSING
#     # ═════════════════════════════════════════════════════════════════════

#     def _extract_obstacles(self, x, y, theta):
#         raw   = np.array(self.LidarMsg.ranges, dtype=np.float64)
#         a_min = self.LidarMsg.angle_min
#         a_inc = self.LidarMsg.angle_increment
#         r_min = self.LidarMsg.range_min
#         r_max = self.LidarMsg.range_max

#         valid   = np.isfinite(raw) & (raw > r_min) & (raw <= r_max)
#         lr      = np.where(valid, raw, np.inf)
#         has_obs = bool(np.any(valid))

#         clusters = []
#         if has_obs:
#             idx_v  = np.where(np.isfinite(lr))[0]
#             splits = np.where(np.diff(idx_v) > 1)[0] + 1
#             for part in np.split(idx_v, splits):
#                 seg     = lr[part]
#                 best    = int(np.argmin(seg))
#                 r       = float(seg[best])
#                 ray_i   = part[best]
#                 bearing = float(a_min + a_inc * ray_i)
#                 wx      = x + r * math.cos(bearing + theta)
#                 wy      = y + r * math.sin(bearing + theta)
#                 clusters.append((wx, wy, r, bearing))

#         return raw, valid, lr, has_obs, clusters

#     # ═════════════════════════════════════════════════════════════════════
#     #  CORNER DETECTION
#     # ═════════════════════════════════════════════════════════════════════

#     def _detect_corner(self, lr: np.ndarray) -> bool:
#         """
#         Returns True when the robot is inside the concave corner:
#           front wall close  AND  left wall close.

#         Sector centres in robot frame:
#           front = 0 rad,  left = +π/2 rad.
#         A half-width of 0.35 rad (≈ 20°) captures the wall reliably
#         without confusing adjacent open sectors.
#         """
#         front_dist = self.sector_min_range(lr, centre=0.0,            half_width=0.35)
#         right_dist = self.sector_min_range(lr, centre=-math.pi / 2.0, half_width=0.35)

#         # Both walls must be closer than the entry threshold
#         in_corner = (front_dist < self.CORNER_ENTER_DIST and
#                      right_dist < self.CORNER_ENTER_DIST)
#         return in_corner

#     def _corner_cleared(self, lr: np.ndarray, x: float, y: float, theta: float) -> bool:
#         """
#         Returns True only when BOTH conditions hold simultaneously:

#         1. FRONT IS CLEAR — the sector directly ahead of the robot's current
#            heading is free beyond CORNER_EXIT_DIST.  This confirms the robot
#            has physically rotated into open space and can drive forward.

#         2. FACING THE GOAL — the robot's current heading is within
#            WF_EXIT_HEADING_TOL of the bearing to the goal.  This prevents
#            exiting while still pointing at a wall even if a side gap happens
#            to be open.

#         Requiring both conditions means the robot keeps turning in WF_TURN
#         until it is simultaneously in open space AND aimed usefully at the
#         goal, at which point handing back to APF makes full sense.
#         """
#         # -- Condition 1: front sector is physically open --
#         front_dist = self.sector_min_range(lr, centre=0.0, half_width=0.35)
#         front_clear = front_dist > self.CORNER_EXIT_DIST

#         # -- Condition 2: current heading is close to goal bearing --
#         goal_bearing  = math.atan2(self.ydp - y, self.xdp - x)
#         heading_error = abs(self.orientation_error(theta, goal_bearing))
#         facing_goal   = heading_error < self.WF_EXIT_HEADING_TOL

#         return front_clear and facing_goal

#     # ═════════════════════════════════════════════════════════════════════
#     #  WALL-FOLLOW ESCAPE CONTROLLER
#     # ═════════════════════════════════════════════════════════════════════

#     def _wall_follow_control(self, lr: np.ndarray, theta: float, x: float, y: float):
#         """
#         Executes one control tick of the wall-follow escape and returns
#         (linear_x, angular_z, new_nav_state).

#         Strategy
#         --------
#         WF_FOLLOW
#           The robot drives forward keeping the front wall on its LEFT.
#           A P-controller adjusts angular velocity so the left-sector range
#           stays at WALL_TARGET_DIST (tracks the wall).

#           Anti-circling guard: the robot's heading is compared to the
#           wall-parallel direction (_wf_entry_heading).  If it drifts more
#           than MAX_WF_HEADING_ERR the corrective term dominates, preventing
#           the robot from spiralling.

#           Transition to WF_TURN when the front sector opens up — the corner
#           has ended and the robot must turn right to exit.

#         WF_TURN
#           Rotate clockwise (negative ω) in place.
#           Transition back to APF once BOTH front and left sectors are clear.
#         """
#         front_dist = self.sector_min_range(lr, centre=0.0,            half_width=0.35)
#         right_dist = self.sector_min_range(lr, centre=-math.pi / 2.0, half_width=0.35)
#         left_dist = self.sector_min_range(lr, centre=math.pi / 2.0, half_width=0.35)

#         # ── WF_FOLLOW ────────────────────────────────────────────────────
#         if self._nav_state == WF_FOLLOW:

#             # -- Transition: front wall ended → turn LEFT (counter-clockwise) --
#             if front_dist > self.CORNER_EXIT_DIST:
#                 self.get_logger().info('[WF] Front wall ended → WF_TURN')
#                 return 0.0, -self.WF_TURN_VEL, WF_TURN   # -WF_TURN_VEL = positive = CCW

#             # -- Wall-distance P-controller (keeps RIGHT range at target) --
#             # Positive error → drifted away from right wall → turn right (- ω)
#             # Negative error → too close to right wall     → turn left  (+ ω)
#             wall_error = right_dist - self.WALL_TARGET_DIST
#             omega_wall = self.WF_KP * wall_error

#             # -- Anti-circling: heading correction toward wall-parallel --
#             # The wall-parallel heading is the entry heading recorded when WF started.
#             # If the current heading has drifted, add a restoring term.
#             heading_err = self.orientation_error(theta, self._wf_entry_heading)
#             if abs(heading_err) > self.MAX_WF_HEADING_ERR:
#                 # Large drift: heading correction dominates to prevent circling
#                 omega_heading = self.kthetap * heading_err
#                 angular_z = float(np.clip(omega_heading,
#                                           -self.max_angular_vel,
#                                            self.max_angular_vel))
#             else:
#                 # Normal: blend wall-distance correction into forward drive
#                 angular_z = float(np.clip(omega_wall,
#                                           -self.max_angular_vel,
#                                            self.max_angular_vel))

#             return self.WF_LINEAR_VEL, angular_z, WF_FOLLOW

#         # ── WF_TURN ──────────────────────────────────────────────────────
#         elif self._nav_state == WF_TURN:

#             # -- Transition: both sectors open → corner fully escaped → APF --
#             if self._corner_cleared(lr, x, y, theta):
#                 self.get_logger().info('[WF] Corner cleared → APF')
#                 self._wf_entry_heading = None   # reset for next potential corner
#                 # Give the robot fresh stuck-detection budget so the timer
#                 # doesn't immediately re-fire in the same area.
#                 self.stuck_timer = 0.0
#                 self.position_window.clear()
#                 return 0.0, 0.0, WF_APF

#             # -- Keep turning right (clockwise) until clear --
#             return 0.0, self.WF_TURN_VEL, WF_TURN

#         # Should never reach here
#         return 0.0, 0.0, WF_APF

#     # ═════════════════════════════════════════════════════════════════════
#     #  MAIN CONTROL LOOP  (10 Hz)
#     # ═════════════════════════════════════════════════════════════════════

#     def control_loop(self) -> None:

#         # ── Guards ────────────────────────────────────────────────────────
#         if len(self.LidarMsg.ranges) == 0:
#             self.get_logger().info(
#                 'Waiting for LiDAR…', throttle_duration_sec=2.0)
#             return

#         if not self._gt_ready:
#             self.get_logger().info(
#                 f'Waiting for pose on {TOPIC_POSE} …',
#                 throttle_duration_sec=2.0)
#             return

#         if self.goal_reached:
#             self.publish(0.0, 0.0)
#             return

#         # ── Ground-truth pose ─────────────────────────────────────────────
#         x, y, theta = self._gt_x, self._gt_y, self._gt_theta

#         # ── LiDAR processing ──────────────────────────────────────────────
#         raw, valid, lr, has_obs, clusters = self._extract_obstacles(x, y, theta)

#         # ═════════════════════════════════════════════════════════════════
#         #  CORNER DETECTION & STATE TRANSITION  (APF → WF_FOLLOW)
#         #
#         #  Check BEFORE the APF block so the wall-follower takes priority
#         #  the moment the corner is detected.  The entry condition is only
#         #  evaluated when already in APF state — once inside WF the robot
#         #  stays there until _corner_cleared() returns True.
#         # ═════════════════════════════════════════════════════════════════
#         if self._nav_state == WF_APF and self._detect_corner(lr):
#             # Lock the wall-parallel heading at the current robot heading.
#             # The robot should move SIDEWAYS relative to the front wall, i.e.
#             # keep driving in the direction it was already facing when it hit
#             # the corner.  This is the heading that runs parallel to the
#             # front wall.
#             self._wf_entry_heading = theta
#             self._nav_state = WF_FOLLOW
#             # Reset stuck state so the timer doesn't fire again immediately
#             # after the robot returns to APF from this wall-follow run.
#             self.stuck_timer = 0.0
#             self.position_window.clear()
#             self.get_logger().info(
#                 f'[WF] Corner detected at ({x:.2f},{y:.2f}) '
#                 f'heading={math.degrees(theta):.1f}° → WF_FOLLOW')

#         # ═════════════════════════════════════════════════════════════════
#         #  WALL-FOLLOW ESCAPE  (overrides APF while active)
#         # ═════════════════════════════════════════════════════════════════
#         if self._nav_state in (WF_FOLLOW, WF_TURN):
#             lin, ang, self._nav_state = self._wall_follow_control(lr, theta, x, y)
#             self.publish(lin, ang)
#             self.get_logger().info(
#                 f'[{self._nav_state}]  pose=({x:.2f},{y:.2f})  '
#                 f'θ={math.degrees(theta):.1f}°  '
#                 f'v={lin:.2f}  ω={ang:.2f}',
#                 throttle_duration_sec=0.3)
#             return   # skip APF this tick

#         # ═════════════════════════════════════════
#         #  STUCK DETECTION  (APF only)
#         #
#         #  Every tick the current (x, y, timestamp) is pushed into a sliding
#         #  window of length stuck_window_sec.  The net displacement from the
#         #  oldest entry to the current position is measured.  If it stays
#         #  below stuck_disp_threshold for stuck_threshold seconds the robot
#         #  is considered stuck and wall-follow is forced regardless of the
#         #  LiDAR corner geometry (handles the case where walls are farther
#         #  than CORNER_ENTER_DIST but the APF gradient is still a local min).
#         #
#         #  When triggered, the wall-parallel heading is chosen as the
#         #  direction perpendicular to whichever wall is closest (front or
#         #  left), so the robot always moves along — not into — the wall.
#         # ═════════════════════════════════════════
#         now = self.get_clock().now().nanoseconds * 1e-9

#         # Push current position into the window
#         self.position_window.append((now, x, y))

#         # Discard entries older than the look-back period
#         while (self.position_window and
#                now - self.position_window[0][0] > self.stuck_window_sec):
#             self.position_window.popleft()

#         # Only evaluate once the window has filled for at least stuck_window_sec
#         if (len(self.position_window) > 1 and
#                 now - self.position_window[0][0] >= self.stuck_window_sec):
#             oldest_x = self.position_window[0][1]
#             oldest_y = self.position_window[0][2]
#             net_disp = math.hypot(x - oldest_x, y - oldest_y)

#             if net_disp < self.stuck_disp_threshold:
#                 self.stuck_timer += self.dt
#             else:
#                 self.stuck_timer = 0.0  # moving fine — reset

#             # Stuck threshold reached → force wall-follow entry
#             if self.stuck_timer >= self.stuck_threshold:
#                 self.stuck_timer = 0.0
#                 self.position_window.clear()

#                 # Choose wall-parallel heading based on the nearest wall.
#                 # If the front wall is closer, move perpendicular to it
#                 # (i.e. turn 90° right so we travel along the front wall).
#                 # If the left wall is closer, keep the current heading so we
#                 # travel parallel to it.
#                 front_d = self.sector_min_range(lr, centre=0.0,            half_width=0.35)
#                 right_d = self.sector_min_range(lr, centre=-math.pi / 2.0, half_width=0.35)

#                 if front_d <= right_d:
#                     # Front wall dominates: wall-parallel = current heading rotated +90° (left)
#                     self._wf_entry_heading = (theta + math.pi / 2.0 + math.pi) % (2 * math.pi) - math.pi
#                 else:
#                     # Right wall dominates: already moving roughly parallel to it
#                     self._wf_entry_heading = theta

#                 self._nav_state = WF_FOLLOW
#                 self.get_logger().info(
#                     f'[STUCK] Timer fired at ({x:.2f},{y:.2f}) '
#                     f'front={front_d:.2f} right={right_d:.2f} '
#                     f'→ WF_FOLLOW  entry_heading={math.degrees(self._wf_entry_heading):.1f}°')
#                 return   # next tick will enter the WF branch

#         # ═════════════════════════════════════════
#         #  APF CONTROL  (normal operation)
#         # ═════════════════════════════════════════
#         # 1. Attractive force
#         vectorD = np.array([[x - self.xdp], [y - self.ydp]])
#         AF      = -(self.kap * vectorD)

#         # 2. Repulsive force
#         RF = np.zeros((2, 1))
#         if has_obs:
#             for (wx, wy, _, __) in clusters:
#                 g = math.hypot(x - wx, y - wy)
#                 if 1e-6 < g <= self.gstarp:
#                     coeff = self.krp * (1.0 / self.gstarp - 1.0 / g) / g ** 3
#                     RF[0, 0] += coeff * (x - wx)
#                     RF[1, 0] += coeff * (y - wy)

#         RF = -RF

#         # Virtual obstacles (previously stuck positions)
#         for (vx, vy) in self.virtual_obstacles:
#             g = math.hypot(x - vx, y - vy)
#             if 1e-6 < g <= self.virtual_obs_radius:
#                 coeff = self.virtual_obs_strength * (
#                     1.0 / self.virtual_obs_radius - 1.0 / g) / g ** 3
#                 RF[0, 0] -= coeff * (x - vx)
#                 RF[1, 0] -= coeff * (y - vy)

#         # 3. Combined force
#         F    = AF + RF if has_obs else AF
#         fmag = math.hypot(float(F[0, 0]), float(F[1, 0]))

#         # 4. Goal-reached check
#         dist_to_goal = float(np.linalg.norm(vectorD))
#         if dist_to_goal < self.eps_control:
#             self.goal_reached = True
#             self.get_logger().info(f'[APF] Goal reached!  pose=({x:.3f}, {y:.3f})')
#             self.publish(0.0, 0.0)
#             self.timer.cancel()
#             return

#         # 5. Heading
#         if fmag < self.FORCE_MIN_MAG:
#             thetaD  = theta
#             eorient = 0.0
#         else:
#             thetaD  = math.atan2(float(F[1, 0]), float(F[0, 0]))
#             eorient = self.orientation_error(theta, thetaD)

#         thetavel = float(np.clip(self.kthetap * eorient,
#                                  -self.max_angular_vel, self.max_angular_vel))

#         # 6. Linear speed
#         if abs(eorient) > self.eps_orient:
#             xvel = 0.05
#         else:
#             if has_obs:
#                 all_valid = raw[valid]
#                 closest   = float(np.min(all_valid)) if all_valid.size else self.gstarp
#                 spd_scale = float(np.clip((closest / self.gstarp) ** 0.5, 0.0, 1.0))
#             else:
#                 spd_scale = 1.0

#             goal_scale = float(np.clip(dist_to_goal / 0.5, 0.2, 1.0))
#             base_speed = min(self.max_linear_vel, fmag)
#             xvel       = max(0.05, base_speed * min(spd_scale, goal_scale))

#         self.publish(xvel, thetavel)

#         self.get_logger().info(
#             f'[APF]  pose=({x:.2f},{y:.2f})  θ={math.degrees(theta):.1f}°  '
#             f'dist={dist_to_goal:.2f} m  '
#             f'eθ={math.degrees(eorient):.1f}°  '
#             f'v={xvel:.2f}  ω={thetavel:.2f}  '
#             f'stuck={self.stuck_timer:.1f} s',
#             throttle_duration_sec=0.3)


# # ═════════════════════════════════════════════════════════════════════════════
# #  ENTRY POINT
# # ═════════════════════════════════════════════════════════════════════════════

# def main(args=None):
#     rclpy.init(args=args)
#     node = PotentialFieldPlanner()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.publish(0.0, 0.0)
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


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