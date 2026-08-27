#!/usr/bin/env python3
"""Forward /cmd_vel to the G1's locomotion controller, via g1_loco_server.

The ROS half of the command path. It owns no policy about *where* the robot
goes -- it takes a Twist and keeps the robot moving at that velocity until told
otherwise. path_follower is the thing with an opinion about the path.

Topics
    /cmd_vel                Twist, subscribed. linear.x forward, linear.y left,
                            angular.z yaw (left-positive). Everything else is
                            ignored; the G1 has no z/roll/pitch velocity input.
    ~/enabled               Bool, published, latched. Whether the gate is open.

Services (all std_srvs/Trigger)
    ~/enable  ~/disable     open / close the /cmd_vel gate
    ~/stop                  disable *and* stop the robot now
    ~/stand_up              FSM 4: stand
    ~/main_operation        FSM 200: enter main operation -- REQUIRED before
                            any velocity command has an effect
    ~/start                 FSM 500, the SDK's name for it. Accepted and
                            ignored on this robot; use ~/main_operation.
    ~/squat  ~/sit  ~/damp  ~/zero_torque
    ~/balance_stand         balance in place (balance mode 0)
    ~/continuous_gait       keep stepping in place (balance mode 1)
    ~/status                read FSM id / mode / balance mode back

The gate matters. On startup `enabled` is false and Twists are dropped, so a
node that was already publishing /cmd_vel when this came up cannot walk the
robot before anyone has looked at it. Enabling is a deliberate act:

    ros2 service call /g1_loco_bridge/enable std_srvs/srv/Trigger

Run g1_loco_server first:
    ros2 run g1_loco_server g1_loco_server --iface=eno1
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from g1_walk.loco_protocol import (
    LocoClient, LocoError, LocoTimeout, describe_error,
)
from g1_walk.single_instance import AlreadyRunning, claim


class LocoBridge(Node):
    def __init__(self):
        super().__init__("g1_loco_bridge")

        # Before any parameter or socket work: two bridges commanding one robot
        # is the failure this guards against, and it should fail immediately
        # and loudly rather than half-start. Held for the process lifetime.
        self._lock = claim("g1_loco_bridge")

        self.declare_parameter("server_address", "127.0.0.1")
        self.declare_parameter("port", 5558)
        self.declare_parameter("rate", 20.0)
        # How long the robot honours each velocity command. This is the
        # robot-side deadline that stops it if we stop talking, so it wants to
        # be a small multiple of the send period -- long enough to bridge one
        # or two dropped ticks, short enough that a stall is over in well under
        # a stride. g1_loco_server clamps it to --max-duration regardless.
        self.declare_parameter("command_duration", 0.5)
        # A Twist older than this counts as no Twist at all. Guards against a
        # publisher that dies while the robot is mid-stride.
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("request_timeout", 2.0)
        # FSM transitions and status reads block until the robot finishes
        # the motion. Must exceed the server's SDK timeout, or a service
        # call reports failure for a command that is still running.
        self.declare_parameter("slow_request_timeout", 15.0)

        address = self.get_parameter("server_address").value
        port = int(self.get_parameter("port").value)
        self.rate = float(self.get_parameter("rate").value)
        self.command_duration = float(
            self.get_parameter("command_duration").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.enabled = bool(self.get_parameter("start_enabled").value)

        self.client = LocoClient(
            address=address, port=port,
            timeout=float(self.get_parameter("request_timeout").value),
            slow_timeout=float(
                self.get_parameter("slow_request_timeout").value),
        )

        # [SAFETY] The velocity tick and the services must not share a
        # callback group. A slow reply from the robot blocks the tick for up to
        # `request_timeout`, and on a single-threaded executor that also blocks
        # ~/stop from even being dispatched -- the one call you need while the
        # robot is walking. Separate groups plus a MultiThreadedExecutor keep
        # the services answerable.
        #
        # This is not full isolation: LocoClient serialises on one socket and
        # the server's REP loop is serial, so a stop still queues behind an
        # in-flight SDK call. It bounds the delay rather than removing it.
        self.tick_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()

        self.command = Twist()
        self.last_command_time = None
        self.was_moving = False
        self.server_ok = False
        self.errors = 0
        self.warned_stale = False
        self.consecutive_failures = 0
        self.failure_backoff = 0
        # Diagnostics for "the robot travelled less than commanded". Without
        # these there is no way to tell a follower problem from a bridge
        # problem from a robot problem.
        self.sent = 0
        self.peak_vx = 0.0
        self.clamped_seen = False
        self.slowest_call = 0.0

        # Latched: a monitor that starts later still learns the gate state,
        # which is the one thing you want to know before touching anything.
        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, "~/enabled", latched)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10,
                                 callback_group=self.service_group)

        for name, handler in (
            ("enable", self._srv_enable),
            ("disable", self._srv_disable),
            ("stop", self._srv_stop),
            ("stand_up", self._fsm_service("stand_up")),
            # main_operation (FSM 200) is the one that actually enables
            # walking. `start` (FSM 500) is the SDK's name for it and is
            # accepted-then-ignored on this robot -- kept only so the SDK
            # vocabulary still works. See FSM_IDS in loco_protocol.py.
            ("main_operation", self._fsm_service("main_operation")),
            ("start", self._fsm_service("start")),
            ("squat", self._fsm_service("squat")),
            ("sit", self._fsm_service("sit")),
            ("damp", self._fsm_service("damp")),
            ("zero_torque", self._fsm_service("zero_torque")),
            ("balance_stand", self._balance_service(0)),
            ("continuous_gait", self._balance_service(1)),
            ("status", self._srv_status),
        ):
            self.create_service(Trigger, f"~/{name}", handler,
                                callback_group=self.service_group)

        self.create_timer(1.0 / self.rate, self._tick,
                          callback_group=self.tick_group)
        self.create_timer(5.0, self._report,
                          callback_group=self.service_group)

        self._publish_enabled()
        self._check_server()

        self.get_logger().info(
            f"bridging /cmd_vel -> {self.client.endpoint} at {self.rate:.0f} Hz. "
            f"Gate is {'OPEN' if self.enabled else 'CLOSED'}; "
            f"call ~/enable to open it."
        )

    # -- plumbing ---------------------------------------------------------

    def _check_server(self):
        try:
            self.client.ping()
        except (LocoError, LocoTimeout) as exc:
            self.server_ok = False
            self.get_logger().warn(
                f"g1_loco_server not responding: {exc}\n"
                f"  Start it with: ros2 run g1_loco_server g1_loco_server "
                f"--iface=eno1"
            )
            return False
        if not self.server_ok:
            self.get_logger().info("g1_loco_server is responding.")
        self.server_ok = True
        return True

    def _publish_enabled(self):
        message = Bool()
        message.data = self.enabled
        self.enabled_pub.publish(message)

    def _on_cmd_vel(self, message):
        self.command = message
        self.last_command_time = self.get_clock().now()

    def _command_is_fresh(self):
        if self.last_command_time is None:
            return False
        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        return age <= self.input_timeout

    def _tick(self):
        """Send the current velocity while active; stay quiet while idle.

        Originally this sent unconditionally, including zeros, on the theory
        that a steady stream keeps the server's watchdog fed. That was wrong in
        two ways. The watchdog only cares about silence *while moving*, so zeros
        buy nothing; and when the robot is unreachable every call blocks for the
        full client timeout, so a closed gate with nobody publishing turned into
        20 blocked calls a second and an unreadable error log. Observed exactly
        that on 2026-08-10 with the robot offline.

        So: transmit when there is something to say, send one explicit stop on
        the moving->idle edge, and otherwise say nothing. Health is covered by
        the periodic ping in _report, which never touches DDS.
        """
        if self.enabled and self._command_is_fresh():
            vx = self.command.linear.x
            vy = self.command.linear.y
            omega = self.command.angular.z
            self.warned_stale = False
        else:
            vx = vy = omega = 0.0
            if (self.enabled and self.was_moving and not self.warned_stale
                    and self.last_command_time is not None):
                self.warned_stale = True
                self.get_logger().warn(
                    f"no /cmd_vel for {self.input_timeout:.1f} s while moving "
                    f"-- commanding zero velocity"
                )

        moving = (vx, vy, omega) != (0.0, 0.0, 0.0)

        if not moving and not self.was_moving:
            return          # idle: nothing to say, so say nothing

        # Back off after repeated failures rather than blocking the tick on
        # every period. With the robot unreachable each call costs a full
        # timeout, and retrying at 20 Hz just floods the log while making the
        # node unresponsive.
        if self.consecutive_failures >= 3:
            self.failure_backoff -= 1
            if self.failure_backoff > 0:
                return
            self.failure_backoff = int(self.rate)    # retry about once a second

        call_start = time.monotonic()
        try:
            if moving:
                reply = self.client.velocity(
                    vx, vy, omega, self.command_duration)
            else:
                # One explicit stop on the moving -> idle edge.
                reply = self.client.stop()
        except (LocoError, LocoTimeout) as exc:
            self.errors += 1
            self.consecutive_failures += 1
            self.server_ok = False
            if self.errors == 1:
                self.get_logger().error(f"command failed: {exc}")
            self.was_moving = False
            return

        self.consecutive_failures = 0
        self.slowest_call = max(self.slowest_call,
                                time.monotonic() - call_start)
        if moving:
            self.sent += 1
            self.peak_vx = max(self.peak_vx, abs(vx))
            # The server clamps to its own limits and reports what it actually
            # sent. A silently clamped velocity is a prime suspect when the
            # robot under-travels.
            if abs(reply.vx - vx) > 1e-3 or abs(reply.omega - omega) > 1e-3:
                self.clamped_seen = True

        self.server_ok = True
        self.was_moving = moving

        if not reply.ok:
            self.errors += 1
            if self.errors == 1:
                self.get_logger().error(
                    f"g1_loco_server returned {describe_error(reply.ret)} "
                    f"for a velocity "
                    f"command. Is the robot in main-operation FSM? "
                    f"Call ~/start."
                )

    def _report(self):
        if self.sent:
            # Only while moving, so an idle bridge stays quiet.
            self.get_logger().info(
                f"commanding: {self.sent / 5.0:.1f} Hz (timer is "
                f"{self.rate:.0f} Hz), peak vx {self.peak_vx:.2f} m/s, "
                f"slowest round trip {self.slowest_call * 1000:.0f} ms"
                + ("  [VELOCITY WAS CLAMPED by g1_loco_server]"
                   if self.clamped_seen else "")
            )
            self.sent = 0
            self.peak_vx = 0.0
            self.clamped_seen = False
            self.slowest_call = 0.0

        if self.errors:
            self.get_logger().warn(
                f"{self.errors} command errors in the last 5 s "
                f"(gate {'open' if self.enabled else 'closed'})"
            )
            self.errors = 0
        elif not self.server_ok:
            self._check_server()

    # -- services ---------------------------------------------------------

    def _srv_enable(self, _request, response):
        if not self._check_server():
            response.success = False
            response.message = ("g1_loco_server is not responding; refusing to "
                                "open the gate")
            return response
        self.enabled = True
        # Any Twist from before this moment is history, not intent.
        self.last_command_time = None
        self._publish_enabled()
        self.get_logger().warn("/cmd_vel gate OPEN -- the robot will now move")
        response.success = True
        response.message = "gate open"
        return response

    def _srv_disable(self, _request, response):
        self.enabled = False
        self._publish_enabled()
        self.get_logger().info("/cmd_vel gate closed")
        response.success = True
        response.message = "gate closed"
        return response

    def _srv_stop(self, _request, response):
        self.enabled = False
        self._publish_enabled()
        try:
            # halt: a robot in continuous gait ignores a zero velocity.
            reply = self.client.halt()
        except (LocoError, LocoTimeout) as exc:
            response.success = False
            response.message = f"gate closed, but stop failed: {exc}"
            self.get_logger().error(response.message)
            return response
        self.was_moving = False
        self.get_logger().warn("STOP: gate closed and robot commanded to stop")
        response.success = reply.ok
        response.message = ("stopped" if reply.ok
                            else describe_error(reply.ret))
        return response

    def _fsm_service(self, name):
        def handler(_request, response):
            # An FSM change while walking would fight the velocity stream.
            self.enabled = False
            self._publish_enabled()
            try:
                reply = self.client.fsm(name)
            except (LocoError, LocoTimeout) as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"{name} failed: {exc}")
                return response
            self.was_moving = False
            response.success = reply.ok
            response.message = (f"{name} accepted (gate closed)" if reply.ok
                                else f"{name} rejected: {describe_error(reply.ret)}")
            self.get_logger().info(response.message)
            return response
        return handler

    def _balance_service(self, mode):
        def handler(_request, response):
            try:
                reply = self.client.balance_mode(mode)
            except (LocoError, LocoTimeout) as exc:
                response.success = False
                response.message = str(exc)
                return response
            response.success = reply.ok
            response.message = (f"balance mode {mode}" if reply.ok
                                else f"rejected: {describe_error(reply.ret)}")
            self.get_logger().info(response.message)
            return response
        return handler

    def _srv_status(self, _request, response):
        try:
            reply = self.client.status()
        except (LocoError, LocoTimeout) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = reply.ok
        response.message = (
            f"fsm_id={reply.fsm_id} fsm_mode={reply.fsm_mode} "
            f"balance_mode={reply.balance_mode} "
            f"gate={'open' if self.enabled else 'closed'}"
        )
        return response

    def destroy_node(self):
        # Best effort: if the server is already gone its own SIGINT handler
        # stopped the robot, and the per-command duration expires regardless.
        try:
            self.client.halt()
        except (LocoError, LocoTimeout):
            pass
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocoBridge()
    except AlreadyRunning as exc:
        print(f"g1_loco_bridge: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    # MultiThreadedExecutor so a blocked velocity call cannot stop ~/stop from
    # being served. See the callback-group comment in __init__.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
