#!/usr/bin/env python3
"""Replay a fixed path as an open-loop stream of /cmd_vel.

There is no feedback here. Nothing measures where the robot is; each segment is
"hold this velocity for this long", and the errors accumulate. That is a
deliberate choice for this stage of the project -- see INSTRUCTIONS.md -- and it
sets the expectations: a 2 m square will not close, and the further the robot
walks the further it is from where the file says it should be.

What that buys is that this runs with nothing but the robot: no camera, no
visual odometry, no map.

Topics
    /cmd_vel            Twist, published at `rate` Hz
    ~/status            String, published on every segment change
    ~/planned_path      nav_msgs/Path, latched. The path as *written*, in the
                        `path_frame` frame. Nominal, not measured.
    ~/commanded_odom    nav_msgs/Odometry, the integral of what we commanded.
                        Note this integrates the *commanded* velocity including
                        yaw_trim, so it still will not match reality.
                        Also nominal -- it is our own output fed back, and it
                        cannot see the robot slipping, drifting or being
                        blocked. Never treat it as localisation.

Services (std_srvs/Trigger)
    ~/start             begin at segment 1 (restarts if already finished)
    ~/pause  ~/resume
    ~/stop              abort and command zero velocity

The bridge's gate must be open too, otherwise these Twists are dropped:

    ros2 service call /g1_loco_bridge/enable std_srvs/srv/Trigger
    ros2 service call /path_follower/start  std_srvs/srv/Trigger
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from g1_walk.path import PathError, load_path
from g1_walk.single_instance import AlreadyRunning, claim

IDLE = "idle"
RUNNING = "running"
PAUSED = "paused"
FINISHED = "finished"


def _yaw_to_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")

        # Two followers publishing /cmd_vel means the robot follows whichever
        # message arrived last -- an interleaving of two different paths.
        self._lock = claim("g1_walk_path_follower")

        self.declare_parameter("path_file", "")
        self.declare_parameter("rate", 20.0)
        # Open-loop calibration. Measure the robot's actual travel over a
        # straight 3 m run and divide: if it covered 2.7 m of a commanded 3.0,
        # set linear_scale to 3.0/2.7 = 1.11 so the command lasts longer. Same
        # idea for angular_scale over a commanded 360 deg turn. These are the
        # only knobs that fight drift, because nothing here measures anything.
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        # Counteract a consistent veer while walking straight. Added to omega
        # only on segments that command no turn, in rad/s: positive steers
        # left. A robot that drifts 30 deg right over 1 m at 0.3 m/s is turning
        # about -0.16 rad/s, so yaw_trim would be about +0.16.
        #
        # This is a bias corrector, not feedback. It cannot fix a drift that
        # varies run to run, and if the number needed is large the gait is
        # wrong and should be fixed rather than trimmed -- see INSTRUCTIONS.md
        # section 2. Measure it the same way as linear_scale: walk a straight
        # segment, measure the heading error, divide by the seconds of motion.
        self.declare_parameter("yaw_trim", 0.0)
        # Tuning the ramps without editing the path file. NEGATIVE means "use
        # whatever the file says" -- not zero, because zero is a meaningful
        # value here (a step command, which on this robot tracks better than a
        # ramp). Using 0 as the sentinel made `ramp_down:=0` silently keep the
        # file's 1.5 s, which is exactly the experiment it was added for.
        self.declare_parameter("ramp_down_override", -1.0)
        self.declare_parameter("ramp_override", -1.0)
        # Metres added to every translation segment, to be given back by the
        # robot's settle-backwards at the end of it. Measured, not guessed:
        # walk a straight segment, note how far it rolls back after stopping.
        # See DEFAULT_DISTANCE_OFFSET in path.py.
        self.declare_parameter("distance_offset", 0.0)
        # Off by default. A path follower that starts walking the moment it is
        # launched is a bad thing to have on a humanoid.
        self.declare_parameter("autostart", False)
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_commanded_tf", True)

        # -- closed loop ------------------------------------------------
        # Off by default: enabling feedback changes what the robot does, and
        # that should be a deliberate act rather than a surprise after a pull.
        self.declare_parameter("feedback", False)
        # Heading hold. The robot veers right by 20-35 deg over a 1.4 m walk
        # (measured 2026-08-13), which no open-loop knob can correct because
        # linear_scale only stretches time and yaw_trim is a fixed guess.
        self.declare_parameter("yaw_gain", 1.2)          # rad/s per rad
        # Steer back onto the line, expressed as a heading offset. Bounded so
        # cross-track never dominates heading hold.
        self.declare_parameter("cross_track_gain", 0.8)  # rad per m
        self.declare_parameter("max_cross_track_angle", 0.4)  # rad
        self.declare_parameter("max_correction_omega", 0.4)   # rad/s
        # How close counts as arrived.
        self.declare_parameter("distance_tolerance", 0.05)    # m
        self.declare_parameter("yaw_tolerance", 0.035)        # rad, ~2 deg
        # After arriving and settling, measure and top up if short. Absorbs
        # both overshoot and the backwards settle without modelling either.
        self.declare_parameter("max_corrections", 2)
        # Below this, a corrective move is futile: the gait does not translate
        # over very short distances, so it would stomp and achieve nothing.
        self.declare_parameter("min_correction_distance", 0.15)  # m
        self.declare_parameter("min_correction_angle", 0.09)     # rad, ~5 deg
        # A closed loop with bad odometry must not walk indefinitely.
        self.declare_parameter("timeout_factor", 3.0)
        # Odometry older than this is not a position.
        self.declare_parameter("odom_timeout", 0.5)              # s

        path_file = self.get_parameter("path_file").value
        if not path_file:
            raise RuntimeError(
                "path_file parameter is required, e.g.\n"
                "  ros2 run g1_walk path_follower --ros-args "
                "-p path_file:=$(ros2 pkg prefix g1_walk)"
                "/share/g1_walk/config/paths/straight_3m.yaml"
            )

        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.rate = float(self.get_parameter("rate").value)
        self.yaw_trim = float(self.get_parameter("yaw_trim").value)
        self.path_frame = self.get_parameter("path_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(
            self.get_parameter("publish_commanded_tf").value)

        self.feedback = bool(self.get_parameter("feedback").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.cross_track_gain = float(
            self.get_parameter("cross_track_gain").value)
        self.max_cross_track_angle = float(
            self.get_parameter("max_cross_track_angle").value)
        self.max_correction_omega = float(
            self.get_parameter("max_correction_omega").value)
        self.distance_tolerance = float(
            self.get_parameter("distance_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.max_corrections = int(self.get_parameter("max_corrections").value)
        self.min_correction_distance = float(
            self.get_parameter("min_correction_distance").value)
        self.min_correction_angle = float(
            self.get_parameter("min_correction_angle").value)
        self.timeout_factor = float(self.get_parameter("timeout_factor").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)

        # Latest measured pose, or None until odometry arrives.
        self.odom_pose = None          # (x, y, yaw)
        self.odom_stamp = None         # rclpy Time of arrival

        # How far the robot carries on -- or rolls back -- after being told to
        # stop. LEARNED during the run, not configured: it is measured as the
        # difference between where the robot was when we commanded zero and
        # where it came to rest. Positive means it settles backwards, which is
        # what this robot does (~0.35 m).
        #
        # Without this, closed loop cannot converge. Each approach ends in a
        # stop, and each stop costs another retreat, so a correction gains the
        # missing distance and immediately gives it back -- observed exactly
        # that against the simulator: three corrections, identical result.
        # Knowing the number lets the approach aim past the target instead.
        self.stop_lead = 0.0
        self.stop_lead_seen = False

        ramp_down_override = float(
            self.get_parameter("ramp_down_override").value)
        ramp_override = float(self.get_parameter("ramp_override").value)
        self.distance_offset = float(
            self.get_parameter("distance_offset").value)

        try:
            self.path = load_path(path_file, self.linear_scale,
                                  self.angular_scale,
                                  None if ramp_down_override < 0.0
                                  else ramp_down_override,
                                  None if ramp_override < 0.0
                                  else ramp_override,
                                  self.distance_offset)
        except PathError as exc:
            # Refuse to come up rather than run a path we half-understood.
            raise RuntimeError(f"cannot load path: {exc}") from exc

        self.state = IDLE
        self.segment_index = 0
        self.segment_elapsed = 0.0
        self.dt = 1.0 / self.rate

        # Commanded pose -- the integral of our own output, nothing more.
        self.cx = 0.0
        self.cy = 0.0
        self.cyaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.odom_pub = self.create_publisher(Odometry, "~/commanded_odom", 10)
        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(PathMsg, "~/planned_path",
                                              latched)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf \
            else None

        if self.feedback:
            # SENSOR_DATA to match odom_bridge's publisher. A default (reliable)
            # subscription is silently incompatible with a best-effort
            # publisher: ROS logs a QoS warning and delivers nothing, which
            # presents as "odometry is stale" with the data flowing perfectly
            # well one node away.
            self.create_subscription(
                Odometry, "/odom", self._on_odom,
                QoSPresetProfiles.SENSOR_DATA.value)

        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/pause", self._srv_pause)
        self.create_service(Trigger, "~/resume", self._srv_resume)
        self.create_service(Trigger, "~/stop", self._srv_stop)

        self.path_pub.publish(self._planned_path_msg())

        self.get_logger().info("loaded " + self.path.describe())
        if (self.linear_scale != 1.0 or self.angular_scale != 1.0
                or self.yaw_trim != 0.0 or self.distance_offset != 0.0):
            self.get_logger().info(
                f"calibration: linear_scale={self.linear_scale}, "
                f"angular_scale={self.angular_scale}, "
                f"yaw_trim={self.yaw_trim} rad/s, "
                f"distance_offset={self.distance_offset} m"
            )
        # 2.0 is a measured, repeatable value on this robot (8 runs on
        # 2026-08-11 covered 1.5-1.65 m of a commanded 3.0 m at 0.4 m/s), so it
        # is no longer suspicious. Beyond ~2.5 it probably is.
        if self.linear_scale > 2.5:
            self.get_logger().warn(
                f"linear_scale={self.linear_scale} is beyond anything measured "
                f"on this robot.\nCheck the gait and the FSM state before "
                f"scaling around a problem."
            )
        if self.feedback:
            self.get_logger().info(
                "CLOSED LOOP -- each segment runs until /odom says it has "
                "arrived,\nwith heading hold. Waiting for the first pose."
            )
            if self.linear_scale != 1.0 or self.distance_offset != 0.0:
                self.get_logger().warn(
                    f"linear_scale={self.linear_scale} and "
                    f"distance_offset={self.distance_offset} are OPEN-LOOP "
                    f"corrections.\nWith feedback on they double-correct: the "
                    f"loop already measures what they\nwere guessing. Set "
                    f"them back to 1.0 and 0.0."
                )
        else:
            self.get_logger().warn(
                "OPEN LOOP -- no pose feedback. Errors accumulate over the "
                "path and are never corrected."
            )

        self.create_timer(self.dt, self._tick)

        if bool(self.get_parameter("autostart").value):
            self.get_logger().warn("autostart:=true -- starting immediately")
            self._begin()
        else:
            self.get_logger().info(
                "idle. Open the bridge gate, then: "
                "ros2 service call /path_follower/start std_srvs/srv/Trigger"
            )

    # -- planned path -----------------------------------------------------

    def _planned_path_msg(self):
        """Integrate the segments once, to show the intended route in RViz.

        Uses the same trapezoid integral the follower will execute, so the
        drawn path is exactly what is commanded -- which is still not what the
        robot will do, but it does make an off-by-a-turn path file obvious
        before anything moves.
        """
        message = PathMsg()
        message.header.frame_id = self.path_frame
        message.header.stamp = self.get_clock().now().to_msg()

        x = y = yaw = 0.0
        message.poses.append(self._pose_stamped(x, y, yaw))

        for segment in self.path.segments:
            steps = max(1, int(round(segment.move_time / self.dt)))
            for step in range(steps):
                vx, vy, omega = segment.velocity_at(step * self.dt)
                x, y, yaw = self._integrate(x, y, yaw, vx, vy, omega, self.dt)
            message.poses.append(self._pose_stamped(x, y, yaw))

        return message

    def _pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.path_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = _yaw_to_quaternion(yaw)
        return pose

    @staticmethod
    def _integrate(x, y, yaw, vx, vy, omega, dt):
        # Midpoint yaw: with a turn and a translation in the same tick, using
        # the start or end heading biases the arc to one side. Cheap to do
        # right, and the planned path is drawn from this.
        mid_yaw = yaw + 0.5 * omega * dt
        x += (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
        y += (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
        return x, y, yaw + omega * dt

    # -- execution --------------------------------------------------------

    def _begin(self):
        self.segment_index = 0
        self.segment_elapsed = 0.0
        self.cx = self.cy = self.cyaw = 0.0
        self.state = RUNNING
        if self.feedback:
            self.path_start_pose = self.odom_pose
            self._begin_segment_feedback()
        self._announce(f"start: {self.path.name}, "
                       f"{len(self.path.segments)} segments, "
                       f"{self.path.total_time:.1f} s nominal")
        self._log_segment()

    def _log_segment(self):
        segment = self.path.segments[self.segment_index]
        self.get_logger().info(
            f"[{self.segment_index + 1}/{len(self.path.segments)}] "
            f"{segment.label} -- {segment.move_time:.1f} s move, "
            f"{segment.settle:.1f} s settle"
        )

    def _announce(self, text):
        self.get_logger().info(text)
        message = String()
        message.data = text
        self.status_pub.publish(message)

    # -- closed loop ------------------------------------------------------

    def _on_odom(self, message):
        orientation = message.pose.pose.orientation
        # Yaw from the quaternion; roll and pitch are irrelevant on the floor.
        siny = 2.0 * (orientation.w * orientation.z
                      + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y
                            + orientation.z * orientation.z)
        self.odom_pose = (message.pose.pose.position.x,
                          message.pose.pose.position.y,
                          math.atan2(siny, cosy))
        self.odom_stamp = self.get_clock().now()

    def _odom_is_fresh(self):
        if self.odom_pose is None or self.odom_stamp is None:
            return False
        age = (self.get_clock().now() - self.odom_stamp).nanoseconds * 1e-9
        return age <= self.odom_timeout

    @staticmethod
    def _wrap(angle):
        """Fold an angle into (-pi, pi]. Without this a 359 deg error reads as
        a huge one and the controller spins the long way round."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _begin_segment_feedback(self):
        """Latch the pose a segment is measured against."""
        segment = self.path.segments[self.segment_index]
        x, y, yaw = self.odom_pose

        self.seg_start = (x, y, yaw)
        self.along_at_stop = None
        self.seg_corrections = 0
        self.seg_elapsed_move = 0.0
        self.phase = "approach"
        self.phase_elapsed = 0.0

        # What this segment is trying to achieve, in measured terms. Distances
        # come from the parsed Segment, which already holds the speeds; the
        # target is recovered from the profile's own integral so a path file
        # remains the single source of truth.
        nominal = segment.plateau + 0.5 * (segment.ramp_up + segment.ramp_down)
        self.seg_target_distance = math.hypot(segment.vx, segment.vy) * nominal
        self.seg_target_yaw_change = segment.omega * nominal

        # Direction of travel in the world, from the heading at segment start.
        # Strafe moves sideways, so its direction is rotated 90 degrees.
        if abs(segment.vx) >= abs(segment.vy):
            self.seg_direction = yaw if segment.vx >= 0 else yaw + math.pi
        else:
            self.seg_direction = yaw + (math.pi / 2 if segment.vy >= 0
                                        else -math.pi / 2)

        self.seg_timeout = max(4.0, segment.move_time * self.timeout_factor)

    def _measured_progress(self):
        """Distance advanced along the segment's intended direction, and the
        lateral offset from that line."""
        x0, y0, _yaw0 = self.seg_start
        x, y, _yaw = self.odom_pose
        dx, dy = x - x0, y - y0
        cos_d, sin_d = math.cos(self.seg_direction), math.sin(self.seg_direction)
        along = dx * cos_d + dy * sin_d
        cross = -dx * sin_d + dy * cos_d
        return along, cross

    def _measured_yaw_change(self):
        return self._wrap(self.odom_pose[2] - self.seg_start[2])

    def _steer(self, cross_track):
        """Heading-hold plus line-following, as a yaw rate.

        Holds the heading the segment started with, biased toward the line by
        the cross-track error. Bounded twice -- the cross-track contribution
        cannot swamp heading hold, and the total cannot fight the walk."""
        x0, y0, yaw0 = self.seg_start
        offset = max(-self.max_cross_track_angle,
                     min(self.max_cross_track_angle,
                         -self.cross_track_gain * cross_track))
        error = self._wrap(yaw0 + offset - self.odom_pose[2])
        omega = self.yaw_gain * error
        return max(-self.max_correction_omega,
                   min(self.max_correction_omega, omega))

    def _tick_feedback(self):
        """One control step of a segment driven by measured pose."""
        segment = self.path.segments[self.segment_index]

        if not self._odom_is_fresh():
            # Never silently change control mode. Stop, say so, and let the
            # operator decide -- a closed loop steering on a stale pose is
            # worse than one that refuses.
            self._publish(0.0, 0.0, 0.0)
            if self.phase != "stale":
                self.phase = "stale"
                self.get_logger().error(
                    "odometry is stale -- holding position. Is "
                    "g1_odom_bridge running?")
            return
        if self.phase == "stale":
            self.phase = "approach"
            self.get_logger().info("odometry recovered, resuming")

        self.phase_elapsed += self.dt

        if self.phase == "settle":
            self._publish(0.0, 0.0, 0.0)
            if self.phase_elapsed < max(segment.settle, 1.0):
                return
            self._finish_or_correct()
            return

        # -- approach ----------------------------------------------------
        self.seg_elapsed_move += self.dt
        if self.seg_elapsed_move > self.seg_timeout:
            self.get_logger().error(
                f"segment {self.segment_index + 1} timed out after "
                f"{self.seg_timeout:.0f} s without arriving. Stopping.\n"
                f"  Either the robot cannot reach the target or the odometry "
                f"is wrong.")
            self._publish(0.0, 0.0, 0.0)
            self.state = IDLE
            self._announce("aborted on timeout")
            return

        if self.seg_target_distance > 0.0:
            along, cross = self._measured_progress()
            # Aim past the target by whatever the last stop cost, so that the
            # position AFTER settling is the one that matches the path file.
            remaining = (self.seg_target_distance + self.stop_lead) - along
            if remaining <= 0.0:
                self._enter_settle()
                return
            speed = math.hypot(segment.vx, segment.vy)
            # Full speed to the target. Easing off early does not help on this
            # robot -- below ~0.4 m/s the gait stops translating, so a slow
            # approach covers no ground. Overshoot is handled after the settle,
            # by measuring.
            if abs(segment.vx) >= abs(segment.vy):
                vx = math.copysign(speed, segment.vx)
                vy = 0.0
            else:
                vx = 0.0
                vy = math.copysign(speed, segment.vy)
            self._publish(vx, vy, self._steer(cross))
            return

        if self.seg_target_yaw_change != 0.0:
            turned = self._measured_yaw_change()
            remaining = self.seg_target_yaw_change - turned
            if (self.seg_target_yaw_change > 0.0) == (remaining <= 0.0):
                self._enter_settle()
                return
            self._publish(0.0, 0.0,
                          math.copysign(abs(segment.omega), remaining))
            return

        # A wait segment: no target, just time.
        self._publish(0.0, 0.0, 0.0)
        if self.seg_elapsed_move >= segment.move_time:
            self._enter_settle()

    def _enter_settle(self):
        self.phase = "settle"
        self.phase_elapsed = 0.0
        # Where we were at the instant of the stop command, so the settle
        # distance can be measured against it.
        if self.seg_target_distance > 0.0:
            self.along_at_stop, _cross = self._measured_progress()
        self._publish(0.0, 0.0, 0.0)

    def _learn_stop_lead(self, along_now):
        """Update the learned stopping distance from what just happened."""
        if self.along_at_stop is None:
            return
        observed = self.along_at_stop - along_now
        # Clamp: a wild value here would send the robot past the target on
        # every later segment, which is worse than not correcting at all.
        observed = max(-0.5, min(1.0, observed))
        if not self.stop_lead_seen:
            self.stop_lead_seen = True
            self.stop_lead = observed
        else:
            # Gently, so one odd segment does not dominate.
            self.stop_lead = 0.7 * self.stop_lead + 0.3 * observed
        self.get_logger().info(
            f"  settle cost {observed:+.2f} m; aiming "
            f"{self.stop_lead:+.2f} m past the target from now on")

    def _finish_or_correct(self):
        """After settling, measure. Top up if short and worth attempting."""
        segment = self.path.segments[self.segment_index]

        if self.seg_target_distance > 0.0:
            along, cross = self._measured_progress()
            self._learn_stop_lead(along)
            self.along_at_stop = None
            error = self.seg_target_distance - along
            worth_it = (abs(error) > self.distance_tolerance
                        and error > self.min_correction_distance
                        and self.seg_corrections < self.max_corrections)
            self.get_logger().info(
                f"  measured {along:.2f} m of {self.seg_target_distance:.2f} m "
                f"(error {error:+.2f} m, lateral {cross:+.2f} m)")
            if worth_it:
                self.seg_corrections += 1
                self.get_logger().info(
                    f"  correcting: {error:.2f} m short "
                    f"({self.seg_corrections}/{self.max_corrections})")
                self.phase = "approach"
                self.phase_elapsed = 0.0
                return
            if 0.0 < error <= self.min_correction_distance:
                self.get_logger().info(
                    f"  {error:.2f} m short, below the "
                    f"{self.min_correction_distance:.2f} m the gait can "
                    f"usefully move. Accepting.")

        elif self.seg_target_yaw_change != 0.0:
            turned = self._measured_yaw_change()
            error = self._wrap(self.seg_target_yaw_change - turned)
            self.get_logger().info(
                f"  measured {math.degrees(turned):.1f} deg of "
                f"{math.degrees(self.seg_target_yaw_change):.1f} deg "
                f"(error {math.degrees(error):+.1f} deg)")
            if (abs(error) > self.yaw_tolerance
                    and abs(error) > self.min_correction_angle
                    and self.seg_corrections < self.max_corrections):
                self.seg_corrections += 1
                self.seg_target_yaw_change = turned + error
                self.get_logger().info(
                    f"  correcting: {math.degrees(error):.1f} deg short "
                    f"({self.seg_corrections}/{self.max_corrections})")
                self.phase = "approach"
                self.phase_elapsed = 0.0
                return

        self._advance_segment()

    def _tick(self):
        if self.state != RUNNING:
            # Keep publishing zeros while idle/paused/finished. The bridge
            # treats a stale Twist as no Twist, so silence would work too, but
            # an explicit zero is what a `ros2 topic echo` should show.
            self._publish(0.0, 0.0, 0.0)
            self._publish_commanded_pose()
            return

        if self.feedback:
            self._tick_feedback()
            self._publish_commanded_pose()
            return

        segment = self.path.segments[self.segment_index]
        vx, vy, omega = segment.velocity_at(self.segment_elapsed)
        omega = self._apply_trim(vx, vy, omega)

        self._publish(vx, vy, omega)
        self.cx, self.cy, self.cyaw = self._integrate(
            self.cx, self.cy, self.cyaw, vx, vy, omega, self.dt)
        self._publish_commanded_pose()

        self.segment_elapsed += self.dt
        if self.segment_elapsed < segment.total_time:
            return

        self._advance_segment()

    def _advance_segment(self):
        """Move to the next segment, or finish the path."""
        self.segment_index += 1
        self.segment_elapsed = 0.0

        if self.segment_index >= len(self.path.segments):
            self.state = FINISHED
            self._publish(0.0, 0.0, 0.0)
            if self.feedback and self.odom_pose is not None:
                x, y, yaw = self.odom_pose
                sx, sy, syaw = self.path_start_pose
                self._announce(
                    f"finished {self.path.name}. MEASURED displacement: "
                    f"x={x - sx:+.2f} m, y={y - sy:+.2f} m, "
                    f"yaw={math.degrees(self._wrap(yaw - syaw)):+.1f} deg "
                    f"(from /odom, not commanded)."
                )
            else:
                self._announce(
                    f"finished {self.path.name}. Commanded end pose: "
                    f"x={self.cx:.2f} m, y={self.cy:.2f} m, "
                    f"yaw={math.degrees(self.cyaw):.1f} deg (nominal -- "
                    f"measure the real one and update "
                    f"linear_scale/angular_scale)."
                )
            return

        self._log_segment()
        if self.feedback:
            self._begin_segment_feedback()

    def _apply_trim(self, vx, vy, omega):
        """Add yaw_trim on translation segments only.

        Deliberately not applied to turns: a turn already commands omega, and
        trimming it would corrupt the angle the path file asked for. It is also
        not applied while settling (all velocities zero), because the robot is
        meant to be standing still then, not creeping round.
        """
        if not self.yaw_trim:
            return omega
        if omega != 0.0:
            return omega                    # a commanded turn, leave it alone
        if vx == 0.0 and vy == 0.0:
            return omega                    # settling
        return self.yaw_trim

    def _publish(self, vx, vy, omega):
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(omega)
        self.cmd_pub.publish(message)

    def _publish_commanded_pose(self):
        stamp = self.get_clock().now().to_msg()
        orientation = _yaw_to_quaternion(self.cyaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.path_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.cx
        odom.pose.pose.position.y = self.cy
        odom.pose.pose.orientation = orientation
        # Large, honest covariances: this pose is an open-loop guess and any
        # consumer that fuses it should weight it accordingly. -1 in [0] is the
        # REP-103 way to say "pose unknown", but some tools choke on it, so use
        # a big number instead.
        odom.pose.covariance[0] = 1.0
        odom.pose.covariance[7] = 1.0
        odom.pose.covariance[35] = 1.0
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.path_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.cx
        transform.transform.translation.y = self.cy
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)

    # -- services ---------------------------------------------------------

    def _srv_start(self, _request, response):
        if self.state == RUNNING:
            response.success = False
            response.message = "already running; call ~/stop first"
            return response
        if self.feedback and not self._odom_is_fresh():
            # Starting a closed loop with no pose would silently degrade to
            # something worse than open loop: it would stand still.
            response.success = False
            response.message = (
                "no fresh /odom -- refusing to start in feedback mode. "
                "Start g1_loco_server and g1_odom_bridge, or launch with "
                "feedback:=false.")
            self.get_logger().error(response.message)
            return response
        self._begin()
        response.success = True
        response.message = f"running {self.path.name}"
        return response

    def _srv_pause(self, _request, response):
        if self.state != RUNNING:
            response.success = False
            response.message = f"not running (state: {self.state})"
            return response
        self.state = PAUSED
        self._publish(0.0, 0.0, 0.0)
        self._announce(f"paused during segment {self.segment_index + 1}")
        response.success = True
        response.message = "paused"
        return response

    def _srv_resume(self, _request, response):
        if self.state != PAUSED:
            response.success = False
            response.message = f"not paused (state: {self.state})"
            return response
        self.state = RUNNING
        self._announce(f"resumed at segment {self.segment_index + 1}")
        response.success = True
        response.message = "resumed"
        return response

    def _srv_stop(self, _request, response):
        self.state = IDLE
        self._publish(0.0, 0.0, 0.0)
        self._announce("stopped by request")
        response.success = True
        response.message = "stopped"
        return response

    def destroy_node(self):
        self.state = IDLE
        try:
            self._publish(0.0, 0.0, 0.0)
        except Exception:                       # noqa: BLE001
            pass    # context already torn down; the bridge stops on its own
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathFollower()
    except AlreadyRunning as exc:
        print(f"path_follower: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
