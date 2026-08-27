#!/usr/bin/env python3
"""Publish the robot's own state estimate as /odom and a TF.

This is the piece that turns "hold a velocity and hope" into "walk until you
have actually arrived". Everything upstream of it is unchanged.

WHERE THIS COMES FROM

The G1 runs its own state estimator and publishes it on the DDS topic
rt/odommodestate. That is not documented for the G1 and is not reachable
through the loco RPC -- it was found by enumerating DDS discovery after two
guessed leads turned out to be a dead end and a false negative. g1_loco_server
subscribes to it and streams it here over ZMQ, because unitree_sdk2 and ROS 2
cannot share an address space.

It is NOT LiDAR-derived, which matters because this project is deliberately
LiDAR-free: it arrives at ~500 Hz (no LiDAR SLAM publishes pose that fast),
its position.z tracks pelvis height, and the same values appear on
rt/dog_odom. It is leg odometry fused with the IMU, computed on the robot.

WHAT IT IS WORTH

Validated against a tape measure on 2026-08-13: over one 7.5 s walk it
reported 1.37 m forward and 0.52 m right, matching both the measured distance
and the observed rightward drift, and it captured the backwards retreat at the
end of the segment.

It is odometry. It drifts, it has no loop closure, and it is not a map. Good
enough to close a loop over one path segment; do not build a world model on it.

Publishes:
    /odom                    nav_msgs/Odometry, frame odom -> base_footprint
    odom -> base_footprint   TF, unless publish_tf is false

Run g1_loco_server first -- it is the source of this stream:
    ros2 run g1_loco_server g1_loco_server --iface=eno1
"""

import struct
import sys
import threading

import rclpy
import zmq
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster

from g1_walk.loco_protocol import LocoError, ODOM_PORT, ODOM_SIZE, decode_odom
from g1_walk.single_instance import AlreadyRunning, claim


class OdomBridge(Node):
    def __init__(self):
        super().__init__("g1_odom_bridge")

        # Two bridges publishing /odom and the same TF would fight, and TF
        # consumers would see the frame jitter between two sources.
        self._lock = claim("g1_odom_bridge")

        self.declare_parameter("server_address", "127.0.0.1")
        self.declare_parameter("port", ODOM_PORT)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        # The robot stamps nothing; g1_loco_server stamps with CLOCK_REALTIME
        # on this PC, so the stamp is directly usable. Set false to restamp on
        # arrival instead.
        self.declare_parameter("use_server_timestamp", True)

        address = self.get_parameter("server_address").value
        port = int(self.get_parameter("port").value)
        self.endpoint = f"tcp://{address}:{port}"
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_server_stamp = bool(
            self.get_parameter("use_server_timestamp").value)

        self.odom_pub = self.create_publisher(
            Odometry, "/odom", QoSPresetProfiles.SENSOR_DATA.value)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf \
            else None

        self.packets = 0
        self.bad_packets = 0
        self.logged_first = False

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        # Always take the newest sample. A queued backlog of stale poses is
        # worse than none: a controller would steer on where the robot was.
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(self.endpoint)

        # A blocking poll on its own thread rather than a fast spin timer: at
        # 100 Hz a polling timer either burns a core or adds jitter.
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"listening on {self.endpoint} -> /odom "
            f"({self.odom_frame} -> {self.base_frame})"
            + ("" if self.publish_tf else ", TF disabled")
        )

    def _receive_loop(self):
        while self.running:
            # rclpy tears the context down on Ctrl-C while this thread is
            # still mid-loop; publishing after that raises and looks like a
            # fault when it is only shutdown ordering.
            if not rclpy.ok():
                break
            try:
                if self.sock.poll(500) == 0:
                    continue
                raw = self.sock.recv()
            except zmq.ZMQError:
                break

            try:
                odom = decode_odom(raw)
            except LocoError as exc:
                self.bad_packets += 1
                if self.bad_packets == 1:
                    self.get_logger().error(str(exc))
                continue

            try:
                self._publish(odom)
            except Exception:                       # noqa: BLE001
                if not rclpy.ok():
                    break                           # shutting down
                raise
            self.packets += 1

            if not self.logged_first:
                self.logged_first = True
                self.get_logger().info(
                    f"receiving the robot's state estimate. First pose: "
                    f"x={odom.x:.2f} y={odom.y:.2f} z={odom.z:.2f}"
                )

    def _publish(self, odom):
        if self.use_server_stamp:
            stamp = rclpy.time.Time(nanoseconds=odom.stamp_ns).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame

        message.pose.pose.position.x = float(odom.x)
        message.pose.pose.position.y = float(odom.y)
        # Deliberately flat. odom.z tracks pelvis height, which is the robot
        # crouching and rising, not the floor moving -- feeding it into a
        # planar odom frame would make the robot appear to bob underground.
        message.pose.pose.position.z = 0.0

        # Unitree orders the quaternion (w, x, y, z); ROS wants (x, y, z, w).
        # Swapping these silently produces a plausible but wrong orientation.
        message.pose.pose.orientation.x = float(odom.qx)
        message.pose.pose.orientation.y = float(odom.qy)
        message.pose.pose.orientation.z = float(odom.qz)
        message.pose.pose.orientation.w = float(odom.qw)

        message.twist.twist.linear.x = float(odom.vx)
        message.twist.twist.linear.y = float(odom.vy)
        message.twist.twist.angular.z = float(odom.yaw_speed)

        # Honest and large. This drifts, has no loop closure, and Unitree
        # publishes no uncertainty with it. Anything fusing this should weight
        # it accordingly rather than trust the zeros a default would imply.
        for index in (0, 7, 35):
            message.pose.covariance[index] = 0.05
        for index in (0, 7, 35):
            message.twist.covariance[index] = 0.05

        self.odom_pub.publish(message)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(odom.x)
        transform.transform.translation.y = float(odom.y)
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _report(self):
        if self.packets == 0:
            self.get_logger().warn(
                f"no odometry from {self.endpoint} in 5 s. Is "
                f"g1_loco_server running, and is it new enough to stream "
                f"odometry?"
            )
        elif self.bad_packets:
            self.get_logger().warn(
                f"{self.packets / 5.0:.0f} Hz, but {self.bad_packets} "
                f"malformed packets"
            )
        self.packets = 0
        self.bad_packets = 0

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.sock.close()
        self.ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OdomBridge()
    except AlreadyRunning as exc:
        print(f"g1_odom_bridge: {exc}", file=sys.stderr)
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
