#!/usr/bin/env python3
"""Validate a path file and print what it would do, without a robot.

Run this on every path file before you run it on the robot. It applies the same
parser and the same trapezoid integral path_follower uses, so the end pose it
prints is exactly what will be commanded -- which, open loop, is the best
prediction available.

    ros2 run g1_walk check_path config/paths/square_2m.yaml
    ros2 run g1_walk check_path square_2m.yaml --linear-scale 1.1
"""

import argparse
import math
import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from g1_walk.path import MIN_EFFECTIVE_SPEED, PathError, load_path


def _resolve(filename):
    """Accept a bare name, a name under config/paths, or a real path."""
    if os.path.isfile(filename):
        return filename

    try:
        share = get_package_share_directory("g1_walk")
    except PackageNotFoundError:
        return filename

    for candidate in (
        os.path.join(share, filename),
        os.path.join(share, "config", "paths", filename),
        os.path.join(share, "config", "paths", filename + ".yaml"),
    ):
        if os.path.isfile(candidate):
            return candidate
    return filename


def main(argv=None):
    parser = argparse.ArgumentParser(prog="check_path", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path_file")
    parser.add_argument("--linear-scale", type=float, default=1.0)
    parser.add_argument("--angular-scale", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0,
                        help="integration rate, match path_follower's")
    parser.add_argument("--ramp", type=float, default=None,
                        help="override every ramp-in [s]")
    parser.add_argument("--ramp-down", type=float, default=None,
                        help="override every ramp-out [s]")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    filename = _resolve(args.path_file)

    try:
        path = load_path(filename, args.linear_scale, args.angular_scale,
                         args.ramp_down, args.ramp)
    except PathError as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1

    print(path.describe())
    print()

    dt = 1.0 / args.rate
    x = y = yaw = 0.0
    distance = 0.0
    # Time spent commanding a translation too slow for the gait to achieve.
    # The trapezoid credits that time as distance; the robot does not cover it.
    dead_time = 0.0

    print(f"{'#':>3}  {'segment':<28} {'end x':>7} {'end y':>7} "
          f"{'end yaw':>8}")
    for index, segment in enumerate(path.segments):
        steps = max(1, int(round(segment.move_time / dt)))
        for step in range(steps):
            vx, vy, omega = segment.velocity_at(step * dt)
            mid_yaw = yaw + 0.5 * omega * dt
            dx = (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
            dy = (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
            x += dx
            y += dy
            yaw += omega * dt
            distance += math.hypot(dx, dy)
            speed = math.hypot(vx, vy)
            if 0.0 < speed < MIN_EFFECTIVE_SPEED:
                dead_time += dt
        print(f"{index + 1:>3}  {segment.label:<28} {x:>7.2f} {y:>7.2f} "
              f"{math.degrees(yaw):>7.1f}d")

    print()
    print(f"commanded end pose : x={x:.2f} m  y={y:.2f} m  "
          f"yaw={math.degrees(yaw):.1f} deg")
    print(f"ground covered     : {distance:.2f} m")
    print(f"wall time          : {path.total_time:.1f} s")

    if dead_time > 0.05:
        print()
        print(f"[!] {dead_time:.1f} s of this path commands a translation "
              f"below {MIN_EFFECTIVE_SPEED} m/s.")
        print(f"    This G1's gait barely translates that slowly, so that "
              f"time covers little or no")
        print(f"    ground -- but the numbers above count it as distance. "
              f"Most of it is ramping.")
        print(f"    Try:  --ramp 0 --ramp-down 0   (and the matching launch "
              f"arguments)")
    print()
    print("Open loop: the robot will not end exactly here. Measure the real "
          "end pose and\nfeed the ratio back through linear_scale / "
          "angular_scale.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
