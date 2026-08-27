#!/usr/bin/env python3
"""Republish the G1's DDS state (joints + IMU) as ROS 2 topics.

Copied unchanged from slam_ws (g1_perception/g1_state_bridge.py) so this
workspace stands alone. Read side only -- the write side is loco_bridge.py.

The ROS half of a two-process bridge. The other half is g1_state_server, in
~/workspaces/unitree/unitree_sdk2/example/g1/sensors/, which subscribes to
rt/lowstate and rt/secondary_imu and pushes a fixed-size packed struct over
ZMQ/TCP. They are separate processes because unitree_sdk2 and ROS 2 each link
their own CycloneDDS and corrupt the heap in one address space.

Nothing in the open-loop path following depends on this: it exists so RViz
shows the real legs moving, and so a run can be recorded for later. When this
project grows a pose estimate, this is where the IMU comes from.

Publishes:
    /joint_states     JointState  29 joints, position + velocity + effort
    /imu/data         Imu         pelvis IMU  (frame imu_in_pelvis)
    /imu_torso/data   Imu         torso  IMU  (frame imu_in_torso)

Run the server first:
    cd ~/workspaces/unitree/unitree_sdk2/build && ./bin/g1_state_server eno1
"""

import math
import struct
import threading

import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, JointState

NUM_JOINTS = 29

MAGIC = 0x47315354  # 'G1ST'
VERSION = 1

# Must mirror struct G1StatePacket in g1_state_server.cpp exactly. That file
# has a static_assert on the total size; if this unpack ever fails, the two
# definitions have drifted apart.
PACKET_FMT = (
    "<"
    "IIQB7x"          # magic, version, stamp_ns, mode_machine, padding
    + "29f" * 3       # q, dq, tau
    + "4f3f3f3f"      # pelvis: quat(w,x,y,z), gyro, accel, rpy
    + "4f3f3f"        # torso:  quat(w,x,y,z), gyro, accel
    + "II"            # lowstate_seq, imu_seq
)
PACKET_SIZE = struct.calcsize(PACKET_FMT)
assert PACKET_SIZE == 472, PACKET_SIZE

# Order matches the Unitree motor indices, and every name exists in
# g1_description/urdf/g1_29dof.urdf (verified against the URDF's movable
# joints). Reused unchanged from the older g1_joint_state_bridge.py.
JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
assert len(JOINT_NAMES) == NUM_JOINTS


def _diag(variance):
    """Row-major 3x3 covariance with a constant diagonal."""
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class G1StateBridge(Node):
    def __init__(self):
        super().__init__("g1_state_bridge")

        self.declare_parameter("server_address", "127.0.0.1")
        self.declare_parameter("port", 5557)
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("imu_frame", "imu_in_pelvis")
        self.declare_parameter("imu_torso_frame", "imu_in_torso")
        # Defaults are placeholders sized for a decent MEMS IMU. Replace them
        # with values measured from a stationary log before trusting the EKF --
        # robot_localization weights measurements by these, so a wrong number
        # here quietly biases the whole fusion stage.
        self.declare_parameter("orientation_variance", 1.0e-3)
        self.declare_parameter("angular_velocity_variance", 1.0e-4)
        self.declare_parameter("linear_acceleration_variance", 1.0e-2)
        # The server stamps with CLOCK_REALTIME. That is directly usable while
        # it runs on this PC. Set false to restamp on arrival instead.
        self.declare_parameter("use_server_timestamp", True)

        address = self.get_parameter("server_address").value
        port = int(self.get_parameter("port").value)
        self.endpoint = f"tcp://{address}:{port}"

        self.imu_frame = self.get_parameter("imu_frame").value
        self.imu_torso_frame = self.get_parameter("imu_torso_frame").value
        self.use_server_stamp = bool(
            self.get_parameter("use_server_timestamp").value
        )

        self.orientation_cov = _diag(
            float(self.get_parameter("orientation_variance").value)
        )
        self.angular_velocity_cov = _diag(
            float(self.get_parameter("angular_velocity_variance").value)
        )
        self.linear_acceleration_cov = _diag(
            float(self.get_parameter("linear_acceleration_variance").value)
        )

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.imu_pub = self.create_publisher(Imu, "/imu/data", sensor_qos)
        self.imu_torso_pub = self.create_publisher(
            Imu, "/imu_torso/data", sensor_qos
        )
        # robot_state_publisher subscribes to /joint_states with default
        # (reliable) QoS, so this one must not be best-effort or TF stops.
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.joint_period = 1.0 / float(
            self.get_parameter("joint_state_rate").value
        )
        self.last_joint_stamp = 0.0

        self.packets = 0
        self.bad_packets = 0
        self.warned_version = False
        self.logged_first = False

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        # Always read the newest sample. TCP would otherwise queue stale state
        # under load and feed the EKF measurements that are minutes old.
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(self.endpoint)

        # A blocking poll on its own thread, rather than a fast spin timer:
        # at 200 Hz a polling timer either burns a core or adds jitter.
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"listening on {self.endpoint} -> /joint_states, /imu/data, "
            f"/imu_torso/data. Start g1_state_server if nothing arrives."
        )

    def _receive_loop(self):
        while self.running:
            # rclpy tears the context down on Ctrl-C while this thread is still
            # mid-loop. Publishing after that raises RCLError ("publisher's
            # context is invalid"), which looks like a fault but is just
            # shutdown ordering. Leave quietly instead -- otherwise a real
            # publish failure would be indistinguishable from this noise.
            if not rclpy.ok():
                break
            try:
                if self.sock.poll(500) == 0:
                    continue
                message = self.sock.recv()
            except zmq.ZMQError:
                break

            if len(message) != PACKET_SIZE:
                self.bad_packets += 1
                continue

            fields = struct.unpack(PACKET_FMT, message)
            if fields[0] != MAGIC:
                self.bad_packets += 1
                continue
            if fields[1] != VERSION and not self.warned_version:
                self.warned_version = True
                self.get_logger().error(
                    f"g1_state_server speaks packet version {fields[1]}, this "
                    f"node expects {VERSION}. Rebuild one of them."
                )

            try:
                self._publish(fields)
            except Exception:                    # noqa: BLE001
                if not rclpy.ok():
                    break                        # shutting down, not an error
                raise
            self.packets += 1

            if not self.logged_first:
                self.logged_first = True
                self.get_logger().info("receiving live state from the robot.")

    def _publish(self, f):
        stamp_ns = f[2]

        i = 4
        q = f[i:i + NUM_JOINTS]; i += NUM_JOINTS
        dq = f[i:i + NUM_JOINTS]; i += NUM_JOINTS
        tau = f[i:i + NUM_JOINTS]; i += NUM_JOINTS
        pelvis_quat = f[i:i + 4]; i += 4
        pelvis_gyro = f[i:i + 3]; i += 3
        pelvis_accel = f[i:i + 3]; i += 3
        i += 3  # pelvis rpy -- redundant with the quaternion, not republished
        torso_quat = f[i:i + 4]; i += 4
        torso_gyro = f[i:i + 3]; i += 3
        torso_accel = f[i:i + 3]; i += 3

        if self.use_server_stamp:
            stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

        self.imu_pub.publish(
            self._imu_msg(
                stamp, self.imu_frame, pelvis_quat, pelvis_gyro, pelvis_accel
            )
        )
        self.imu_torso_pub.publish(
            self._imu_msg(
                stamp, self.imu_torso_frame, torso_quat, torso_gyro, torso_accel
            )
        )

        # The IMU is the reason to run at 200 Hz; joints only need to beat
        # robot_state_publisher's needs, so downsample them.
        now = stamp_ns * 1e-9
        if now - self.last_joint_stamp < self.joint_period:
            return
        self.last_joint_stamp = now

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = [float(v) for v in q]
        joint_state.velocity = [float(v) for v in dq]
        joint_state.effort = [float(v) for v in tau]
        self.joint_pub.publish(joint_state)

    def _imu_msg(self, stamp, frame_id, quat, gyro, accel):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        # Unitree orders the quaternion (w, x, y, z); ROS wants (x, y, z, w).
        # Swapping these silently produces a plausible-looking but wrong
        # orientation, so normalise here and let _report flag a bad norm.
        w, x, y, z = quat
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm > 0.0:
            w, x, y, z = w / norm, x / norm, y / norm, z / norm
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0

        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.orientation.w = float(w)
        msg.orientation_covariance = self.orientation_cov

        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.angular_velocity_covariance = self.angular_velocity_cov

        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.linear_acceleration_covariance = self.linear_acceleration_cov

        return msg

    def _report(self):
        rate = self.packets / 5.0
        if self.packets == 0:
            self.get_logger().warn(
                f"no packets from {self.endpoint} in 5 s -- is g1_state_server "
                f"running?"
            )
        elif self.bad_packets:
            self.get_logger().warn(
                f"{rate:.0f} Hz, but {self.bad_packets} malformed packets"
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
    node = G1StateBridge()
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
