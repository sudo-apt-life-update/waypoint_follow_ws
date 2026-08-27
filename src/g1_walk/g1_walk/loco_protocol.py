"""Wire protocol for talking to g1_loco_server, and a small blocking client.

This is the Python half of the two-process split described in
g1_loco_server.cpp: unitree_sdk2 and ROS 2 each link their own CycloneDDS and
corrupt the heap in one address space, so the SDK lives in a separate binary and
we reach it over a ZMQ REP socket.

No ROS imports here on purpose -- loco_cli uses this without a running graph,
which is how you check the robot before bringing anything else up.
"""

import math
import struct
import threading

import zmq

REQ_MAGIC = 0x47314344  # 'G1CD'
REP_MAGIC = 0x47315250  # 'G1RP'
VERSION = 1

# Must mirror struct G1CmdRequest / G1CmdReply in g1_loco_server.cpp exactly.
# That file has static_asserts on both sizes; if either unpack starts failing,
# the two definitions have drifted apart.
REQUEST_FMT = (
    "<"
    "IIII"      # magic, version, seq, command
    "ffff"      # vx, vy, omega, duration
    "fi"        # fvalue, ivalue
)
REPLY_FMT = (
    "<"
    "IIIi"      # magic, version, seq, ret
    "iii"       # fsm_id, fsm_mode, balance_mode
    "ffff"      # clamped vx, vy, omega, duration
)
REQUEST_SIZE = struct.calcsize(REQUEST_FMT)
REPLY_SIZE = struct.calcsize(REPLY_FMT)
assert REQUEST_SIZE == 40, REQUEST_SIZE
assert REPLY_SIZE == 44, REPLY_SIZE


# Mirrors struct G1OdomPacket in g1_loco_server.cpp, which has a static_assert
# on its size. The robot's own state estimator, streamed on a PUB socket rather
# than polled: odometry is state, and 500 Hz state wants a stream.
ODOM_FMT = (
    "<"
    "IIQ"       # magic, version, stamp_ns
    "fff"       # x, y, z
    "ffff"      # quaternion, Unitree order (w, x, y, z)
    "fff"       # vx, vy, vz
    "f"         # yaw_speed
    "II"        # seq, pad
)
ODOM_SIZE = struct.calcsize(ODOM_FMT)
assert ODOM_SIZE == 68, ODOM_SIZE
ODOM_MAGIC = 0x47314F44  # 'G1OD'
ODOM_PORT = 5560


class Odom:
    """One decoded G1OdomPacket."""

    __slots__ = ("stamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz",
                 "vx", "vy", "vz", "yaw_speed", "seq")

    def __init__(self, fields):
        (_magic, _version, self.stamp_ns, self.x, self.y, self.z,
         self.qw, self.qx, self.qy, self.qz,
         self.vx, self.vy, self.vz, self.yaw_speed, self.seq, _pad) = fields

    @property
    def yaw(self):
        """Yaw in radians from the attitude quaternion."""
        siny = 2.0 * (self.qw * self.qz + self.qx * self.qy)
        cosy = 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        return math.atan2(siny, cosy)


def decode_odom(raw):
    """Decode a packet, or raise LocoError. Returns an Odom."""
    if len(raw) != ODOM_SIZE:
        raise LocoError(f"odom packet was {len(raw)} bytes, expected "
                        f"{ODOM_SIZE}")
    fields = struct.unpack(ODOM_FMT, raw)
    if fields[0] != ODOM_MAGIC:
        raise LocoError("odom packet had bad magic")
    if fields[1] != VERSION:
        raise LocoError(
            f"g1_loco_server speaks odom packet version {fields[1]}, this "
            f"client expects {VERSION}. Rebuild one of them.")
    return Odom(fields)


class Command:
    """Mirrors enum Command in g1_loco_server.cpp."""

    PING = 0
    VELOCITY = 1
    STOP = 2
    FSM = 3
    BALANCE_MODE = 4
    STAND_HEIGHT = 5
    SWING_HEIGHT = 6
    SPEED_MODE = 7
    STATUS = 8
    HALT = 9


# FSM ids, from LocoClient's high-level wrappers. The server rejects anything
# not in its own copy of this table, so keep the two in step.
# [THE 500 vs 200 TRAP] LocoClient::Start() hardcodes 500, but on this robot
# sending 500 returns success and leaves the FSM at 4 -- accepted, ignored, and
# every later SetVelocity silently discarded because the robot never entered
# main operation. 200 is the G1's "Main Operation Control". Prefer
# `main_operation`; `start` is kept only because the SDK calls it that.
#
# A return code of 0 from an FSM command does NOT mean the state changed.
# Always confirm with `loco_cli status`.
FSM_IDS = {
    "zero_torque": 0,
    "damp": 1,
    "squat": 2,
    "sit": 3,
    "stand_up": 4,
    "main_operation": 200,
    "start": 500,
    "lie2standup": 702,
    "squat2standup": 706,
}

ERR_BAD_COMMAND = -1001

# Return codes worth naming. Transcribed from the SDK headers under
# /opt/unitree_robotics/include/unitree/ -- common/error.hpp,
# robot/internal/internal_error.hpp and robot/g1/loco/g1_loco_error.hpp. A bare
# "ret=7301" sends you reading headers; the text says what to do about it.
ERROR_NAMES = {
    0: "success",
    -1: "unknown error",
    ERR_BAD_COMMAND: "rejected by g1_loco_server (bad or unknown command)",

    1011: "network error",
    1012: "timeout",
    2001: "DDS error",

    3001: "unknown robot error",
    3102: "send request error",
    3103: "API not registered",
    3104: "call API timeout -- the robot's service did not answer",
    3105: "response API did not match the request",
    3106: "response data error",
    3107: "lease invalid",
    3201: "server send response error",
    3202: "server internal error",
    3203: "API not implemented on the robot",
    3204: "API parameter error",
    3205: "request denied by lease -- something else holds control",
    3206: "lease not found on the robot",

    7301: "LocoState not available -- the robot's locomotion controller is "
          "not running or has not published state yet",
    7302: "invalid FSM id",
    7303: "invalid task id",
}


def describe_error(ret):
    """Human-readable form of a return code, always including the number."""
    if ret in ERROR_NAMES:
        return f"{ret} ({ERROR_NAMES[ret]})"
    return str(ret)


class LocoError(RuntimeError):
    """The server was reached but refused, or the SDK returned non-zero."""


class LocoTimeout(RuntimeError):
    """No reply within the timeout -- the server is down, or DDS is wedged."""


class Reply:
    """Decoded G1CmdReply. `ret` is 0 on success, else the SDK's error code."""

    __slots__ = (
        "seq", "ret", "fsm_id", "fsm_mode", "balance_mode",
        "vx", "vy", "omega", "duration",
    )

    def __init__(self, fields):
        (_magic, _version, self.seq, self.ret, self.fsm_id, self.fsm_mode,
         self.balance_mode, self.vx, self.vy, self.omega,
         self.duration) = fields

    @property
    def ok(self):
        return self.ret == 0

    def __repr__(self):
        return (f"Reply(ret={self.ret}, fsm_id={self.fsm_id}, "
                f"fsm_mode={self.fsm_mode}, balance_mode={self.balance_mode}, "
                f"v=({self.vx:.2f}, {self.vy:.2f}, {self.omega:.2f}) for "
                f"{self.duration:.2f}s)")


class LocoClient:
    """Blocking REQ client for g1_loco_server.

    REQ sockets are a strict send/recv lockstep: a timed-out request leaves the
    socket in a state where the next send raises. Rather than track that, a
    timeout closes the socket and opens a fresh one, which is cheap on loopback
    and means a caller can retry after the server is restarted without
    rebuilding anything.

    Thread-safe: one lock around the whole request/reply exchange, because that
    lockstep cannot be interleaved. Callers from a ROS timer and a service
    callback at the same time are the expected case.

    Two timeouts, and the split matters.

    `timeout` covers the fast commands -- velocity, stop, ping. Those are sent
    at 20 Hz and a late reply is worthless, so they must fail quickly rather
    than stall the caller's control loop.

    `slow_timeout` covers everything that makes the robot *do* something it
    takes time to finish: the FSM transitions, the heights, and the status read.
    A G1 stand-up runs for several seconds and the service only replies when the
    motion completes. This must stay comfortably above the server's own SDK
    timeout (`--sdk-timeout`, 10 s), because a client that gives up first
    reports a failure for a command the robot is busy executing -- which is the
    single most dangerous kind of wrong answer this class can give. Learned the
    hard way: a 2 s client timeout made `stand_up` look like it failed while the
    robot was standing up.
    """

    def __init__(self, address="127.0.0.1", port=5558, timeout=2.0,
                 slow_timeout=15.0):
        self.endpoint = f"tcp://{address}:{port}"
        self.timeout_ms = int(timeout * 1000)
        self.slow_timeout_ms = int(slow_timeout * 1000)
        self._lock = threading.Lock()
        self._seq = 0
        self._ctx = zmq.Context.instance()
        self._sock = None
        self._connect()

    def _connect(self):
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._sock.connect(self.endpoint)

    def _reconnect(self):
        self._sock.close()
        self._connect()

    def close(self):
        with self._lock:
            if self._sock is not None:
                self._sock.close()
                self._sock = None

    def _call(self, command, vx=0.0, vy=0.0, omega=0.0, duration=0.0,
              fvalue=0.0, ivalue=0, slow=False):
        timeout_ms = self.slow_timeout_ms if slow else self.timeout_ms

        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            seq = self._seq
            message = struct.pack(
                REQUEST_FMT, REQ_MAGIC, VERSION, seq, command,
                float(vx), float(vy), float(omega), float(duration),
                float(fvalue), int(ivalue),
            )

            try:
                self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
                self._sock.send(message)
                raw = self._sock.recv()
            except zmq.Again as exc:
                self._reconnect()
                # Say plainly that the command may have landed. A timeout here
                # is ambiguous, not a failure, and treating it as a failure is
                # how someone re-sends a motion command to a moving robot.
                raise LocoTimeout(
                    f"no reply from g1_loco_server at {self.endpoint} within "
                    f"{timeout_ms} ms.\n"
                    f"  The command may still have been delivered and be "
                    f"executing -- check the robot before retrying.\n"
                    f"  If the server is not running: "
                    f"ros2 run g1_loco_server g1_loco_server --iface=eno1"
                ) from exc
            except zmq.ZMQError as exc:
                self._reconnect()
                raise LocoError(f"socket error talking to {self.endpoint}: "
                                f"{exc}") from exc

        if len(raw) != REPLY_SIZE:
            raise LocoError(f"reply was {len(raw)} bytes, expected {REPLY_SIZE}")

        fields = struct.unpack(REPLY_FMT, raw)
        if fields[0] != REP_MAGIC:
            raise LocoError("reply had bad magic")
        if fields[1] != VERSION:
            raise LocoError(
                f"g1_loco_server speaks packet version {fields[1]}, this "
                f"client expects {VERSION}. Rebuild one of them."
            )

        reply = Reply(fields)
        if reply.seq != seq:
            # Cannot happen on a healthy REQ/REP pair; if it does, the socket
            # is out of step and every later reply would be off by one.
            self._reconnect()
            raise LocoError(f"reply seq {reply.seq} does not match request "
                            f"{seq}; socket reset")
        return reply

    # -- commands ---------------------------------------------------------

    def ping(self):
        return self._call(Command.PING)

    def velocity(self, vx, vy, omega, duration):
        """Walk at this velocity for `duration` seconds.

        The duration is the robot-side deadline: the controller stops on its
        own when it expires, so a client that dies mid-stride does not leave
        the robot walking. Send at a rate comfortably faster than 1/duration.
        """
        return self._call(Command.VELOCITY, vx=vx, vy=vy, omega=omega,
                          duration=duration)

    def stop(self):
        """Zero the velocity. Does NOT leave continuous gait -- use halt().

        This is the per-segment stop. Changing balance mode between segments
        would alter the gait mid-path, so it deliberately does not.
        """
        return self._call(Command.STOP)

    def halt(self):
        """Actually stop the robot: zero velocity AND leave continuous gait.

        In balance mode 1 the robot marches in place at zero velocity, so
        `stop()` does not stop it. Anything operator-facing -- the stop
        service, the CLI, shutdown paths -- wants this one.
        """
        return self._call(Command.HALT, slow=True)

    def fsm(self, name):
        if name not in FSM_IDS:
            raise LocoError(f"unknown FSM state {name!r}; expected one of "
                            f"{sorted(FSM_IDS)}")
        # Slow: a stand-up or sit motion runs for seconds before the
        # service replies.
        return self._call(Command.FSM, ivalue=FSM_IDS[name], slow=True)

    def fsm_id(self, raw_id):
        """Send an FSM id the table does not name.

        For exploring a controller whose id mapping does not match the SDK
        headers -- which is exactly the situation that cost us an afternoon.
        The server still refuses unless started with --allow-any-fsm.
        """
        return self._call(Command.FSM, ivalue=int(raw_id), slow=True)

    def balance_mode(self, mode):
        return self._call(Command.BALANCE_MODE, ivalue=int(mode), slow=True)

    def stand_height(self, metres):
        return self._call(Command.STAND_HEIGHT, fvalue=metres, slow=True)

    def swing_height(self, metres):
        return self._call(Command.SWING_HEIGHT, fvalue=metres, slow=True)

    def speed_mode(self, mode):
        return self._call(Command.SPEED_MODE, ivalue=int(mode), slow=True)

    def status(self):
        # Slow: three round trips to the controller, any of which can
        # block if it is busy.
        return self._call(Command.STATUS, slow=True)
