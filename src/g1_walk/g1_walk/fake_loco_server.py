#!/usr/bin/env python3
"""Stand in for g1_loco_server, with no robot and no Unitree SDK.

Speaks the same wire protocol on the same port, so the entire ROS stack can be
brought up and a path run end to end at a desk. It integrates the velocities it
is given and prints the pose it would have reached, which is the same number
path_follower predicts -- if they disagree, the bug is in the plumbing between
them and not in the path.

    ros2 run g1_walk fake_loco_server
    ros2 launch g1_walk walk.launch.py path:=square_2m.yaml

By default it is a perfect robot. Pass --efficiency 0.5 --retreat 0.35 to make
it misbehave the way the real G1 measurably does, which is the only way to
test a controller meant to cope with that:

    ros2 run g1_walk fake_loco_server --efficiency 0.5 --retreat 0.35

What it cannot tell you is anything about the robot: whether the gait is
stable, whether the FSM would have accepted the command, or how far the real
machine actually walks. It is a test of this codebase, not of the robot.
"""

import argparse
import math
import struct
import sys
import time

import zmq

from g1_walk.loco_protocol import (
    Command, FSM_IDS, ODOM_FMT, ODOM_MAGIC, ODOM_PORT, REPLY_FMT, REP_MAGIC,
    REQUEST_FMT, REQUEST_SIZE, REQ_MAGIC, VERSION,
)

FSM_NAMES = {value: key for key, value in FSM_IDS.items()}
ERR_BAD_COMMAND = -1001


def main(argv=None):
    parser = argparse.ArgumentParser(prog="fake_loco_server",
                                     description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind", default="tcp://127.0.0.1:5558")
    parser.add_argument("--odom-bind",
                        default=f"tcp://127.0.0.1:{ODOM_PORT}",
                        help="where to publish simulated odometry, mirroring "
                             "the real server's PUB socket")
    parser.add_argument("--drift", type=float, default=0.0,
                        help="rad/s of rightward veer while walking. -0.05 "
                             "reproduces the real robot's 20-35 deg drift and "
                             "is what exercises heading hold.")
    parser.add_argument("--report", type=float, default=2.0,
                        help="seconds between pose reports")
    # Defaults reproduce the real robot as measured on 2026-08-12 (remote off,
    # 8 runs): it covers about half the commanded distance, then settles
    # backwards by a fixed amount when the commands stop. A simulator that
    # tracks perfectly cannot test a controller whose entire job is to cope
    # with a robot that does not.
    parser.add_argument("--efficiency", type=float, default=1.0,
                        help="fraction of commanded velocity actually "
                             "achieved. 0.5 matches the real G1 at 0.4 m/s; "
                             "1.0 is a perfect robot.")
    parser.add_argument("--retreat", type=float, default=0.0,
                        help="metres rolled backwards when a motion ends, "
                             "simulating the balance controller recovering. "
                             "0.35 matches the real G1.")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, 0)
    try:
        sock.bind(args.bind)
    except zmq.ZMQError as exc:
        print(f"cannot bind {args.bind}: {exc}\n"
              f"  A real g1_loco_server may already be running. "
              f"Check: ss -ltnp | grep 5558", file=sys.stderr)
        return 1

    odom_pub = ctx.socket(zmq.PUB)
    odom_pub.setsockopt(zmq.SNDHWM, 1)
    odom_pub.setsockopt(zmq.LINGER, 0)
    odom_pub.bind(args.odom_bind)

    print(f"fake_loco_server on {args.bind} -- NO ROBOT, NO SDK")
    print(f"  simulated odometry on {args.odom_bind}")
    if args.drift:
        print(f"  veering {args.drift} rad/s while walking")
    if args.efficiency != 1.0 or args.retreat:
        print(f"  simulating an imperfect robot: efficiency "
              f"{args.efficiency}, retreat {args.retreat} m per motion")

    fsm_id = 4              # pretend the robot is stood up
    balance_mode = 0
    x = y = yaw = 0.0
    last_velocity_time = None
    commands = 0
    last_report = time.monotonic()
    moving = False
    odom_seq = 0

    try:
        while True:
            got_request = sock.poll(100) != 0

            # Outside the poll branch on purpose. The bridge sends at 20 Hz, so
            # a 100 ms poll almost never times out -- housekeeping hung off the
            # idle path would then never run, which is exactly when the pose
            # report is most wanted.
            now = time.monotonic()
            # Mirror the real server's watchdog, so a stalled client looks the
            # same here as it would on the robot.
            if (moving and last_velocity_time is not None
                    and now - last_velocity_time > 0.5):
                moving = False
                print("[watchdog] no velocity command for 0.5 s -- stopping")
            if now - last_report >= args.report:
                last_report = now
                print(f"pose x={x:6.2f} m  y={y:6.2f} m  "
                      f"yaw={math.degrees(yaw):7.1f} deg   "
                      f"({commands} commands, "
                      f"fsm={FSM_NAMES.get(fsm_id, fsm_id)})")

            # Stream the simulated pose, exactly as the real server does.
            odom_seq += 1
            odom_pub.send(struct.pack(
                ODOM_FMT, ODOM_MAGIC, VERSION, int(time.time() * 1e9),
                x, y, 0.70,
                math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0),
                0.0, 0.0, 0.0, 0.0, odom_seq, 0))

            if not got_request:
                continue

            raw = sock.recv()
            reply_ret = 0

            if len(raw) != REQUEST_SIZE:
                sock.send(struct.pack(REPLY_FMT, REP_MAGIC, VERSION, 0,
                                      ERR_BAD_COMMAND, -1, -1, -1,
                                      0.0, 0.0, 0.0, 0.0))
                continue

            (magic, version, seq, command, vx, vy, omega, duration,
             fvalue, ivalue) = struct.unpack(REQUEST_FMT, raw)

            out_fsm = out_mode = out_balance = -1

            if magic != REQ_MAGIC or version != VERSION:
                reply_ret = ERR_BAD_COMMAND
            elif command == Command.VELOCITY:
                now = time.monotonic()
                was_moving = moving
                moving = (vx, vy, omega) != (0.0, 0.0, 0.0)
                if last_velocity_time is not None:
                    dt = min(now - last_velocity_time, 0.5)
                    # The robot achieves only a fraction of what it is told.
                    eff = args.efficiency
                    # An uncommanded veer, like the real robot's.
                    omega = omega + (args.drift if moving else 0.0)
                    mid = yaw + 0.5 * omega * eff * dt
                    x += (vx * math.cos(mid) - vy * math.sin(mid)) * eff * dt
                    y += (vx * math.sin(mid) + vy * math.cos(mid)) * eff * dt
                    yaw += omega * eff * dt
                if was_moving and not moving and args.retreat:
                    # Motion just ended: give back a fixed distance along the
                    # current heading, as the real robot does when its balance
                    # controller brings the torso back over its feet.
                    x -= args.retreat * math.cos(yaw)
                    y -= args.retreat * math.sin(yaw)
                    print(f"[sim] motion ended -- retreating "
                          f"{args.retreat} m")
                last_velocity_time = now
                commands += 1
            elif command == Command.STOP:
                if moving and args.retreat:
                    x -= args.retreat * math.cos(yaw)
                    y -= args.retreat * math.sin(yaw)
                    print(f"[sim] stop -- retreating {args.retreat} m")
                moving = False
                last_velocity_time = time.monotonic()
            elif command == Command.FSM:
                if ivalue in FSM_NAMES:
                    fsm_id = ivalue
                    moving = False
                    print(f"FSM -> {FSM_NAMES[ivalue]} ({ivalue})")
                else:
                    reply_ret = ERR_BAD_COMMAND
            elif command == Command.BALANCE_MODE:
                if ivalue in (0, 1):
                    balance_mode = ivalue
                    print(f"balance mode -> {ivalue}")
                else:
                    reply_ret = ERR_BAD_COMMAND
            elif command in (Command.STAND_HEIGHT, Command.SWING_HEIGHT,
                             Command.SPEED_MODE):
                print(f"command {command} -> {fvalue if command != Command.SPEED_MODE else ivalue}")
            elif command == Command.STATUS:
                out_fsm, out_mode, out_balance = fsm_id, 0, balance_mode
            elif command != Command.PING:
                reply_ret = ERR_BAD_COMMAND

            sock.send(struct.pack(
                REPLY_FMT, REP_MAGIC, VERSION, seq, reply_ret,
                out_fsm, out_mode, out_balance, vx, vy, omega, duration))

    except KeyboardInterrupt:
        print(f"\nfinal pose x={x:.2f} m  y={y:.2f} m  "
              f"yaw={math.degrees(yaw):.1f} deg after {commands} commands")
    finally:
        sock.close()
        odom_pub.close()
        ctx.term()

    return 0


if __name__ == "__main__":
    sys.exit(main())
