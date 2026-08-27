#!/usr/bin/env python3
"""Check everything that must be true before the robot can walk.

Every failure in this project's first session was a precondition, not a bug in
the walking code: the ethernet link down, the robot unreachable, the FSM in the
wrong state, the loco server not started, the bridge not running. Each one
presented differently and none of them said what was actually wrong.

So: one command that checks them all, in dependency order, and stops at the
first thing that is broken -- because a later check failing is usually just an
echo of the earlier one.

    ros2 run g1_walk preflight
    ros2 run g1_walk preflight --iface=eno1 --robot=192.168.123.164

Exit code is 0 only when the robot is ready to walk.
"""

import argparse
import os
import subprocess
import sys

from g1_walk.loco_protocol import LocoClient, LocoError, LocoTimeout, describe_error

READY_FSM = 200         # main_operation -- see FSM_IDS in loco_protocol.py

OK = "  OK  "
FAIL = " FAIL "
WARN = " WARN "


def line(status, title, detail=""):
    print(f"[{status}] {title}")
    for part in detail.splitlines():
        if part:
            print(f"         {part}")


def check_interface(iface):
    """Link up, with an address. A down link makes DDS fail on its own IP."""
    base = f"/sys/class/net/{iface}"
    if not os.path.isdir(base):
        return False, f"no such interface: {iface}\nCheck the name with: ip -br link"

    try:
        with open(f"{base}/carrier") as handle:
            carrier = handle.read().strip()
    except OSError:
        carrier = "0"       # unreadable when the link is down

    if carrier != "1":
        return False, (
            f"{iface} has no carrier -- the cable is unplugged, or the robot "
            f"is off.\nReseat the cable at both ends, then re-run.")

    result = subprocess.run(["ip", "-4", "-br", "addr", "show", iface],
                            capture_output=True, text=True)
    if "inet" not in result.stdout and "/" not in result.stdout:
        return False, (
            f"{iface} is up but has no IPv4 address.\n"
            f"sudo ip addr add 192.168.123.1/24 dev {iface}")

    return True, result.stdout.strip()


def check_route(robot_ip, iface):
    """Traffic to the robot must leave via the wired link, not WiFi."""
    result = subprocess.run(["ip", "route", "get", robot_ip],
                            capture_output=True, text=True)
    route = result.stdout.strip().splitlines()[0] if result.stdout else ""

    if f"dev {iface}" not in route:
        return False, (
            f"traffic to {robot_ip} is not going via {iface}:\n"
            f"  {route}\n"
            f"That usually means {iface} lost its address and the default "
            f"route (WiFi) is picking it up.")
    return True, route


def check_ping(robot_ip):
    result = subprocess.run(
        ["ping", "-c", "2", "-W", "1", robot_ip],
        capture_output=True, text=True)
    if result.returncode != 0:
        return False, (
            f"{robot_ip} does not answer.\n"
            f"The link is up on this end, so the robot itself is off, still "
            f"booting, or its network is down.")
    return True, f"{robot_ip} responds"


def check_server(address, port):
    client = LocoClient(address=address, port=port, timeout=2.0)
    try:
        client.ping()
        return True, f"g1_loco_server responding on {client.endpoint}", client
    except (LocoError, LocoTimeout):
        client.close()
        return False, (
            "g1_loco_server is not running. Start it in its own terminal:\n"
            "  ros2 run g1_loco_server g1_loco_server --iface=eno1"), None


def check_fsm(client):
    """FSM 200 or the robot accepts velocity commands and ignores them."""
    try:
        reply = client.status()
    except (LocoError, LocoTimeout) as exc:
        return False, f"status call failed: {exc}"

    if not reply.ok and reply.fsm_id < 0:
        return False, (
            f"cannot read the FSM: {describe_error(reply.ret)}\n"
            f"Try: ros2 run g1_walk loco_cli stand_up")

    detail = (f"fsm_id {reply.fsm_id}, fsm_mode {reply.fsm_mode}, "
              f"balance_mode "
              f"{'unavailable' if reply.balance_mode < 0 else reply.balance_mode}")

    if reply.fsm_id == READY_FSM:
        return True, detail + "  <- main operation, ready to walk"

    return False, (
        f"{detail}\n"
        f"The robot is NOT in main operation ({READY_FSM}), so every velocity "
        f"command will be\naccepted and silently ignored. Fix it with:\n"
        f"  ros2 run g1_walk loco_cli stand_up        (if it is not standing)\n"
        f"  ros2 run g1_walk loco_cli main_operation\n"
        f"  ros2 run g1_walk loco_cli status          (confirm fsm_id "
        f"{READY_FSM})")


def check_bridge():
    """Optional: the ROS bridge only exists while walk.launch.py is running."""
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        return None, "rclpy unavailable, skipping"

    started_here = not rclpy.ok()
    if started_here:
        rclpy.init()
    node = Node("g1_preflight")
    try:
        # One short spin so discovery has a chance to populate.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        names = {name for name, _types in node.get_service_names_and_types()}
    finally:
        node.destroy_node()
        if started_here:
            rclpy.shutdown()

    if "/g1_loco_bridge/enable" in names:
        return True, "walk.launch.py is running"
    return None, (
        "the ROS bridge is not running -- /g1_loco_bridge/* services do not "
        "exist.\nThat is fine for loco_cli, which talks to the server "
        "directly. For path\nfollowing, start it in its own terminal:\n"
        "  ros2 launch g1_walk walk.launch.py path:=nudge.yaml")


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="preflight", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="eno1")
    parser.add_argument("--robot", default="192.168.123.164")
    parser.add_argument("--address", default="127.0.0.1",
                        help="host running g1_loco_server")
    parser.add_argument("--port", type=int, default=5558)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    print("G1 preflight\n")

    # Ordered by dependency, and stopping at the first failure on purpose:
    # once the link is down every later check fails too, and three red herrings
    # are harder to read than one real cause.
    for name, (good, detail) in (
        (f"network interface {args.iface}", check_interface(args.iface)),
        (f"route to {args.robot}", check_route(args.robot, args.iface)),
        (f"robot {args.robot} reachable", check_ping(args.robot)),
    ):
        line(OK if good else FAIL, name, "" if good else detail)
        if not good:
            print("\nNot ready. Fix the above and re-run.")
            return 1

    good, detail, client = check_server(args.address, args.port)
    line(OK if good else FAIL, "g1_loco_server", "" if good else detail)
    if not good:
        print("\nNot ready. Fix the above and re-run.")
        return 1

    try:
        fsm_good, fsm_detail = check_fsm(client)
    finally:
        client.close()
    line(OK if fsm_good else FAIL, "robot FSM", fsm_detail)

    bridge_good, bridge_detail = check_bridge()
    line(OK if bridge_good else WARN, "ROS bridge",
         "" if bridge_good else bridge_detail)

    print()
    if not fsm_good:
        print("Not ready to walk. Fix the FSM as shown above.")
        return 1

    print("Ready to walk.")
    if bridge_good:
        print("  ros2 service call /g1_loco_bridge/enable std_srvs/srv/Trigger")
        print("  ros2 service call /path_follower/start  std_srvs/srv/Trigger")
    else:
        print("  ros2 run g1_walk loco_cli move --vx 0.2 --seconds 3")
    return 0


if __name__ == "__main__":
    sys.exit(main())
