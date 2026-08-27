#!/usr/bin/env python3
"""Poke g1_loco_server by hand, without a ROS graph.

This is the tool for the first two minutes of any session: check the server is
up, read the FSM state, stand the robot up. It talks to the same ZMQ socket the
bridge uses, so both can be connected at once -- ZMQ fair-queues the requests.

    ros2 run g1_walk loco_cli status
    ros2 run g1_walk loco_cli stand_up
    ros2 run g1_walk loco_cli start
    ros2 run g1_walk loco_cli stop
    ros2 run g1_walk loco_cli damp

`move` is here for calibration runs, and it is the one subcommand that walks the
robot from a shell, so it insists on being told for how long and refuses to run
longer than 10 s:

    ros2 run g1_walk loco_cli move --vx 0.3 --seconds 5
"""

import argparse
import sys
import time

from g1_walk.loco_protocol import (
    FSM_IDS, LocoClient, LocoError, LocoTimeout, describe_error,
)
from g1_walk.path import Segment

# How long to wait for an FSM transition to actually land before giving up.
# A G1 stand-up takes a few seconds; 12 s is generous without being a hang.
FSM_SETTLE_TIMEOUT = 12.0
FSM_POLL_PERIOD = 0.5

# Transitions the controller will not make directly. Keyed by the state you are
# stuck in; the value is what to do about it. Discovered the hard way -- from
# zero_torque the robot accepts stand_up and stays put, with no error anywhere.
STUCK_HINTS = {
    0: ("zero_torque -- the robot is limp and has no holding force.\n"
        "  It cannot go straight to standing. SUPPORT THE ROBOT, then:\n"
        "    ros2 run g1_walk loco_cli damp        (take up the joints)\n"
        "    ros2 run g1_walk loco_cli stand_up\n"
        "  If it is lying down, try lie2standup instead of stand_up."),
    3: ("sit -- stand up before anything else:\n"
        "    ros2 run g1_walk loco_cli stand_up"),
}

# A hand-typed shell command is the least supervised way to move the robot, so
# it gets the tightest leash of any path in this project.
MAX_CLI_SECONDS = 10.0
MAX_CLI_SPEED = 0.4
MAX_CLI_RATE = 0.5


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="loco_cli", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="reply timeout for fast commands [s]")
    parser.add_argument("--slow-timeout", type=float, default=15.0,
                        help="reply timeout for FSM/status commands, which\nthe robot can take seconds to finish [s]")

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("ping", help="check the server is alive")
    subparsers.add_parser("status", help="read FSM id / mode / balance mode")
    subparsers.add_parser(
        "stop", help="zero the velocity (does NOT leave continuous gait)")
    subparsers.add_parser(
        "halt", help="really stop: zero velocity AND leave continuous gait")
    subparsers.add_parser("balance_stand", help="balance in place (mode 0)")
    subparsers.add_parser(
        "continuous_gait",
        help="balance mode 1: MARCHES IN PLACE CONTINUOUSLY until halted")

    for name in FSM_IDS:
        subparsers.add_parser(name, help=f"FSM -> {name} ({FSM_IDS[name]})")

    height = subparsers.add_parser("stand_height", help="set stand height [m]")
    height.add_argument("value", type=float)

    swing = subparsers.add_parser("swing_height", help="set swing height [m]")
    swing.add_argument("value", type=float)

    raw = subparsers.add_parser(
        "fsm_id", help="send a raw FSM id (needs server --allow-any-fsm)")
    raw.add_argument("value", type=int)
    raw.add_argument("--yes", action="store_true")

    move = subparsers.add_parser(
        "move", help="walk at a fixed velocity for N seconds (calibration)")
    move.add_argument("--vx", type=float, default=0.0,
                      help="forward [m/s]")
    move.add_argument("--vy", type=float, default=0.0,
                      help="left [m/s]")
    move.add_argument("--omega", type=float, default=0.0,
                      help="yaw, left-positive [rad/s]")
    move.add_argument("--seconds", type=float, required=True,
                      help=f"how long to walk, max {MAX_CLI_SECONDS}")
    move.add_argument("--rate", type=float, default=20.0,
                      help="command rate [Hz]")
    move.add_argument("--duration", type=float, default=None,
                      help="how long the robot honours each command [s]. "
                           "Defaults to 3 command periods, capped at 0.5. The "
                           "bridge uses 0.5; set it here to compare like with "
                           "like.")
    move.add_argument("--ramp", type=float, default=0.5,
                      help="seconds to ease IN")
    move.add_argument("--ramp-down", type=float, default=1.5,
                      help="seconds to ease OUT. Longer than --ramp on "
                           "purpose: cutting a walking humanoid to zero makes "
                           "it step to arrest its own momentum, which is "
                           "motion you did not ask for.")
    move.add_argument("--settle", type=float, default=2.0,
                      help="seconds of commanded zero velocity after the move, "
                           "before halting")
    move.add_argument("--yes", action="store_true",
                      help="skip the confirmation prompt")
    move.add_argument("--repeat", type=int, default=1,
                      help="run the move N times, pausing between. This "
                           "robot's distance is repeatable but its "
                           "post-move drift is not, so a single run tells "
                           "you very little.")
    move.add_argument("--pause", type=float, default=5.0,
                      help="seconds between repeats, to let the robot settle "
                           "and to give you time to measure")

    return parser.parse_args(argv)


def _do_move(client, args):
    """Run the move, `--repeat` times, refusing to start in the wrong FSM."""
    # Two of the runs in the 2026-08-11 A/B were issued at fsm_id 4, where
    # velocity commands are accepted and ignored -- so those data points meant
    # nothing and nobody noticed until the logs were read afterwards. Check
    # once, up front, rather than trusting the operator to remember.
    try:
        status = client.status()
        if status.fsm_id >= 0 and status.fsm_id != 200:
            print(f"refusing to move: fsm_id is {status.fsm_id}, not 200 "
                  f"(main_operation).\n"
                  f"  Velocity commands are accepted and ignored in this "
                  f"state, so the run would\n"
                  f"  look like a result and be meaningless.\n"
                  f"  Fix: ros2 run g1_walk loco_cli main_operation",
                  file=sys.stderr)
            return 2
    except (LocoTimeout, LocoError) as exc:
        print(f"could not read the FSM before moving: {exc}", file=sys.stderr)
        return 1

    if args.repeat < 1:
        print("--repeat must be at least 1", file=sys.stderr)
        return 2

    worst = 0
    for run in range(args.repeat):
        if args.repeat > 1:
            print(f"\n--- run {run + 1} of {args.repeat} ---")
        result = _do_one_move(client, args)
        worst = max(worst, result)
        if run + 1 < args.repeat:
            print(f"pausing {args.pause:.0f} s -- measure now")
            try:
                time.sleep(args.pause)
            except KeyboardInterrupt:
                print("\ninterrupted")
                break
    return worst


def _do_one_move(client, args):
    if not 0.0 < args.seconds <= MAX_CLI_SECONDS:
        print(f"--seconds must be in (0, {MAX_CLI_SECONDS}]", file=sys.stderr)
        return 2
    if abs(args.vx) > MAX_CLI_SPEED or abs(args.vy) > MAX_CLI_SPEED:
        print(f"speed limited to {MAX_CLI_SPEED} m/s from the CLI",
              file=sys.stderr)
        return 2
    if abs(args.omega) > MAX_CLI_RATE:
        print(f"yaw rate limited to {MAX_CLI_RATE} rad/s from the CLI",
              file=sys.stderr)
        return 2
    if (args.vx, args.vy, args.omega) == (0.0, 0.0, 0.0):
        print("all velocities are zero; nothing to do", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    # Outlive one dropped tick, no more: if this process is killed mid-run, the
    # robot stops when the last duration expires. Overridable so this can be
    # compared directly against the bridge, which uses 0.5 s.
    duration = args.duration if args.duration else min(3.0 * period, 0.5)

    print(f"About to walk the robot: vx={args.vx} vy={args.vy} "
          f"omega={args.omega} for {args.seconds} s "
          f"(+{args.ramp} s in, +{args.ramp_down} s out, "
          f"+{args.settle} s settle, {duration:.2f} s command window).\n"
          f"  Expected ground covered: "
          f"{abs(args.vx) * args.seconds:.2f} m forward, "
          f"{abs(args.vy) * args.seconds:.2f} m sideways.")
    if not args.yes:
        try:
            if input("Type 'go' to continue: ").strip() != "go":
                print("aborted")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\naborted")
            return 1

    # The same trapezoid the path follower uses, from the same class -- so a
    # distance measured with this command transfers to a path run instead of
    # being a different experiment. The ramps matter physically: cutting from
    # 0.4 m/s to zero in one tick makes the robot step to catch itself, and
    # that recovery is motion nobody commanded (observed 2026-08-11: ~0.5 m
    # backwards and 20 deg off heading after an unramped stop).
    segment = Segment("cli move", args.vx, args.vy, args.omega,
                      nominal_time=args.seconds, ramp=args.ramp,
                      settle=args.settle, ramp_down=args.ramp_down)

    # A single slow reply is not a reason to abandon a walk. The robot's own
    # per-command deadline means a gap just makes it pause, and aborting the
    # loop on the first hiccup is both fragile and worse for the gait. Give up
    # only when several in a row fail, which is what a real fault looks like.
    max_consecutive_timeouts = 5
    consecutive = 0
    timeouts = 0
    elapsed = 0.0

    try:
        # total_time includes the settle, during which velocity_at returns
        # zeros -- so the robot is actively told to hold still rather than
        # simply hearing nothing.
        while elapsed < segment.total_time:
            vx, vy, omega = segment.velocity_at(elapsed)
            elapsed += period
            try:
                reply = client.velocity(vx, vy, omega, duration)
            except LocoTimeout:
                consecutive += 1
                timeouts += 1
                print(f"  slow reply ({consecutive}/"
                      f"{max_consecutive_timeouts})", file=sys.stderr)
                if consecutive >= max_consecutive_timeouts:
                    print("giving up: the server stopped answering",
                          file=sys.stderr)
                    break
                continue

            consecutive = 0
            if not reply.ok:
                print(f"server returned {describe_error(reply.ret)}; "
                      f"stopping", file=sys.stderr)
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stopped = _stop_hard(client)

    if timeouts:
        print(f"note: {timeouts} slow replies during the run")
    return 0 if stopped else 1


def _wait_for_fsm(client, name, target):
    """Poll until the FSM actually reaches `target`, or time out saying so.

    SetFsmId returns on *acceptance*, not on completion -- the robot may still
    be mid-motion, and a second FSM command sent into that window lands it
    somewhere unintended (observed 2026-08-11: stand_up immediately followed by
    main_operation left the robot in FSM 801, a state in no table, with
    balance_mode unreadable).

    Waiting here means a caller cannot chain transitions too fast even by
    accident, and turns "accepted" into an answer you can trust.
    """
    deadline = time.monotonic() + FSM_SETTLE_TIMEOUT
    last = None

    while time.monotonic() < deadline:
        try:
            reply = client.status()
        except (LocoTimeout, LocoError):
            time.sleep(FSM_POLL_PERIOD)
            continue

        if reply.fsm_id == target:
            print(f"{name}: ok (fsm_id {target})")
            return 0

        if reply.fsm_id != last and reply.fsm_id >= 0:
            last = reply.fsm_id
            print(f"  ... fsm_id {reply.fsm_id}, waiting for {target}")

        time.sleep(FSM_POLL_PERIOD)

    current = last if last is not None else "unreadable"
    print(f"{name}: ACCEPTED BUT NOT REACHED. fsm_id is {current}, wanted "
          f"{target}.", file=sys.stderr)

    hint = STUCK_HINTS.get(last)
    if hint:
        print(f"  The robot is in {hint}", file=sys.stderr)
    else:
        print(f"  The robot may still be finishing the previous transition. "
              f"Let it settle,\n  then re-send: ros2 run g1_walk loco_cli "
              f"{name}\n"
              f"  If it is stuck in a state you do not recognise, support the "
              f"robot and go back\n  to a known one: damp, then stand_up, "
              f"then main_operation -- one at a time,\n"
              f"  waiting for each to report ok.", file=sys.stderr)
    return 1


def _stop_hard(client):
    """Stop the robot, retrying, and say plainly whether it worked.

    This runs in a `finally`, so it may be cleaning up after a timeout -- which
    is exactly when a single stop attempt is most likely to fail too. Letting
    that exception escape would replace the real error with a confusing one and
    leave the operator with no idea whether the robot is still walking.
    """
    for attempt in range(3):
        try:
            # halt, not stop: if the robot is in continuous gait, zeroing the
            # velocity leaves it marching.
            client.halt()
            print("stopped")
            return True
        except (LocoTimeout, LocoError) as exc:
            print(f"stop attempt {attempt + 1}/3 failed: "
                  f"{str(exc).splitlines()[0]}", file=sys.stderr)

    # Not fatal in itself: every velocity command carried its own expiry, so the
    # robot halts on its own within that window. Say so rather than leaving the
    # operator guessing.
    print("COULD NOT CONFIRM STOP. The robot should halt on its own when the "
          "last command's duration expires (<1 s). Watch it, and use the "
          "remote if it does not.", file=sys.stderr)
    return False


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    client = LocoClient(address=args.address, port=args.port,
                        timeout=args.timeout,
                        slow_timeout=args.slow_timeout)

    try:
        if args.command == "move":
            return _do_move(client, args)

        if args.command == "ping":
            client.ping()
            print(f"g1_loco_server at {client.endpoint} is alive")
            return 0

        if args.command == "status":
            reply = client.status()
            # -1 means that particular getter was refused. They are reported
            # per field because they do not necessarily fail together, and a
            # robot whose setters all work but whose getters do not is a
            # firmware limitation, not a fault to chase.
            def show(label, value):
                print(f"{label:<13}"
                      + ("unavailable" if value < 0 else str(value)))

            show("fsm_id", reply.fsm_id)
            show("fsm_mode", reply.fsm_mode)
            show("balance_mode", reply.balance_mode)

            if not reply.ok:
                print(f"\nall state getters refused: "
                      f"{describe_error(reply.ret)}\n"
                      f"  Commands may still work -- try `loco_cli stop`, "
                      f"which is a setter.\n"
                      f"  If setters work and getters do not, this robot's "
                      f"firmware does not publish\n"
                      f"  LocoState. That blocks state readback only, not "
                      f"walking.", file=sys.stderr)
                return 1
            return 0

        if args.command == "halt":
            reply = client.halt()
            print("halt: ok" if reply.ok
                  else f"halt: {describe_error(reply.ret)}",
                  file=sys.stderr if not reply.ok else sys.stdout)
            return 0 if reply.ok else 1

        if args.command == "fsm_id":
            print(f"About to send raw FSM id {args.value}. This can move the "
                  f"robot.")
            if not args.yes and input("Type 'go' to continue: ").strip() != "go":
                print("aborted")
                return 1
            reply = client.fsm_id(args.value)
        elif args.command == "stop":
            reply = client.stop()
        elif args.command == "balance_stand":
            reply = client.balance_mode(0)
        elif args.command == "continuous_gait":
            # Loud, because this is the one command whose effect does not stop
            # when you stop talking to the robot. stop/watchdog/exit all zero
            # the velocity, and a marching robot at zero velocity keeps
            # marching.
            print("continuous_gait: the robot will MARCH IN PLACE until you "
                  "halt it.\n"
                  "  Zero velocity does not stop it, and neither does killing "
                  "these processes.\n"
                  "  To stop:  ros2 run g1_walk loco_cli halt")
            reply = client.balance_mode(1)
        elif args.command == "stand_height":
            reply = client.stand_height(args.value)
        elif args.command == "swing_height":
            reply = client.swing_height(args.value)
        else:
            reply = client.fsm(args.command)
            if reply.ok:
                return _wait_for_fsm(client, args.command,
                                     FSM_IDS[args.command])

        if reply.ok:
            print(f"{args.command}: ok")
            return 0
        print(f"{args.command}: server returned "
              f"{describe_error(reply.ret)}", file=sys.stderr)
        return 1

    except LocoTimeout as exc:
        print(f"{exc}", file=sys.stderr)
        return 1
    except LocoError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
