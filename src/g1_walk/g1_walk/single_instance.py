"""Refuse to start when another copy of the same node is already running.

Two `walk.launch.py` invocations at once is an easy mistake -- one terminal per
path file feels natural -- and it is a bad one. ROS 2 tolerates duplicate node
names, so you get two loco_bridges commanding the same robot and two
path_followers publishing /cmd_vel. Service calls go to whichever answers
first, so you can start a path you are not looking at. Seen 2026-08-11: a
`nudge.yaml` launch and a `straight_3m.yaml` launch up together, and the run
that happened was not the one the operator had in the front terminal.

Implemented with an abstract AF_UNIX socket rather than a lock file. The
abstract namespace is Linux-only, which is fine here, and the name disappears
when the process does -- no stale lock to clean up after a crash or a kill -9,
which is exactly the failure mode that makes lock files annoying.
"""

import socket


class AlreadyRunning(RuntimeError):
    """Another instance holds the lock."""


def claim(name):
    """Claim an exclusive, process-lifetime lock called `name`.

    Returns the socket. **Keep the returned object alive** -- the lock is held
    by the open socket, so letting it be garbage collected releases it. Callers
    should stash it on the node.
    """
    lock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        # Leading NUL puts this in the abstract namespace: no filesystem entry,
        # released automatically when the process exits by any means.
        lock.bind("\0" + name)
    except OSError as exc:
        lock.close()
        raise AlreadyRunning(
            f"another {name} is already running.\n"
            f"  Two of these commanding one robot is not safe: both publish to "
            f"the same topics,\n"
            f"  and a service call goes to whichever answers first -- so you "
            f"can start a path you\n"
            f"  are not watching.\n"
            f"  Stop the other one (Ctrl-C its terminal), or find it with:\n"
            f"    pgrep -af {name}"
        ) from exc
    return lock
