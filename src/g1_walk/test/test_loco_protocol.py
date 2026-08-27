"""Round-trip the command protocol against a stub server.

The struct formats here and the packed structs in g1_loco_server.cpp are two
definitions of the same thing. The C++ side has static_asserts on both sizes
and these tests pin the Python side to the same numbers, so a change to one
without the other fails somewhere rather than producing a plausible-looking
wrong command.
"""

import struct
import time
import threading

import pytest
import zmq

from g1_walk.loco_protocol import (
    Command, LocoClient, LocoError, LocoTimeout, REPLY_FMT, REPLY_SIZE,
    REP_MAGIC, REQUEST_FMT, REQUEST_SIZE, REQ_MAGIC, VERSION,
)


def test_sizes_match_the_cpp_static_asserts():
    assert REQUEST_SIZE == 40
    assert REPLY_SIZE == 44


class StubServer:
    """A REP socket that records requests and replies with whatever it's told."""

    def __init__(self, ret=0, drop=False, delay=0.0):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.port = self.sock.bind_to_random_port("tcp://127.0.0.1")
        self.requests = []
        self.ret = ret
        self.drop = drop
        self.delay = delay
        self.running = True
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while self.running:
            if self.sock.poll(100) == 0:
                continue
            raw = self.sock.recv()
            self.requests.append(struct.unpack(REQUEST_FMT, raw))
            if self.drop:
                continue        # never reply, to exercise the timeout path
            if self.delay:
                time.sleep(self.delay)      # a robot mid-motion
            seq = self.requests[-1][2]
            self.sock.send(struct.pack(
                REPLY_FMT, REP_MAGIC, VERSION, seq, self.ret,
                4, 1, 0, 0.1, 0.0, 0.0, 0.5))

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.sock.close()
        self.ctx.term()


@pytest.fixture
def server():
    stub = StubServer()
    yield stub
    stub.close()


def test_velocity_round_trip(server):
    client = LocoClient(port=server.port)
    reply = client.velocity(0.3, -0.1, 0.2, 0.5)
    client.close()

    assert reply.ok
    magic, version, _seq, command, vx, vy, omega, duration, _f, _i = \
        server.requests[0]
    assert magic == REQ_MAGIC
    assert version == VERSION
    assert command == Command.VELOCITY
    assert vx == pytest.approx(0.3)
    assert vy == pytest.approx(-0.1)
    assert omega == pytest.approx(0.2)
    assert duration == pytest.approx(0.5)


def test_fsm_sends_the_right_id(server):
    client = LocoClient(port=server.port)
    client.fsm("start")
    client.close()
    assert server.requests[0][3] == Command.FSM
    assert server.requests[0][9] == 500


def test_unknown_fsm_never_reaches_the_wire(server):
    client = LocoClient(port=server.port)
    with pytest.raises(LocoError, match="unknown FSM state"):
        client.fsm("moonwalk")
    client.close()
    assert server.requests == []


def test_sequence_increments(server):
    client = LocoClient(port=server.port)
    client.ping()
    client.ping()
    client.close()
    assert server.requests[1][2] == server.requests[0][2] + 1


def test_nonzero_ret_is_reported_not_raised():
    stub = StubServer(ret=-1001)
    try:
        client = LocoClient(port=stub.port)
        reply = client.stop()
        client.close()
        assert not reply.ok
        assert reply.ret == -1001
    finally:
        stub.close()


def test_timeout_raises_and_leaves_the_client_usable():
    stub = StubServer(drop=True)
    try:
        client = LocoClient(port=stub.port, timeout=0.2)
        with pytest.raises(LocoTimeout, match="no reply"):
            client.ping()
        # A timed-out REQ socket cannot be sent on again; the client is
        # supposed to have replaced it rather than wedged itself.
        with pytest.raises(LocoTimeout):
            client.ping()
        client.close()
    finally:
        stub.close()


def test_slow_commands_get_the_long_timeout():
    """Regression: `stand_up` reported failure while the robot was standing up.

    The client's timeout was 2 s and the server waits up to 10 s for the SDK, so
    any command the robot takes real time to finish looked like a failure. That
    is the worst kind of wrong answer -- it invites re-sending a motion command
    to a robot that is already moving. FSM commands must use `slow_timeout`.
    """
    stub = StubServer(delay=0.8)
    try:
        client = LocoClient(port=stub.port, timeout=0.3, slow_timeout=5.0)

        # A fast command is meant to give up quickly.
        with pytest.raises(LocoTimeout):
            client.ping()

        # The same 0.8 s delay must NOT defeat an FSM command.
        assert client.fsm("stand_up").ok
        assert client.status().ok
        client.close()
    finally:
        stub.close()


def test_timeout_message_warns_the_command_may_have_landed():
    stub = StubServer(drop=True)
    try:
        client = LocoClient(port=stub.port, timeout=0.2)
        with pytest.raises(LocoTimeout, match="may still have been delivered"):
            client.ping()
        client.close()
    finally:
        stub.close()


def test_no_server_at_all_times_out():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    port = sock.bind_to_random_port("tcp://127.0.0.1")
    sock.close()
    ctx.term()

    client = LocoClient(port=port, timeout=0.2)
    with pytest.raises(LocoTimeout):
        client.ping()
    client.close()


# -- halt vs stop ---------------------------------------------------------

def test_halt_and_stop_are_different_commands(server):
    """Regression: the robot marched on after every 'stop' we had.

    balance mode 1 keeps the robot stepping in place at zero velocity, so
    SetVelocity(0,0,0) does not stop it. `stop` stays velocity-only because it
    runs between path segments and must not change the gait; `halt` is the one
    that means stop, and the server pairs it with balance mode 0.
    """
    client = LocoClient(port=server.port)
    client.stop()
    client.halt()
    client.close()

    assert server.requests[0][3] == Command.STOP
    assert server.requests[1][3] == Command.HALT


def test_bridge_segment_stop_does_not_change_balance_mode(server):
    """The per-segment stop must never send BALANCE_MODE.

    If it did, a path with settle periods would drop out of continuous gait at
    every segment boundary and change how the robot walks mid-path.
    """
    client = LocoClient(port=server.port)
    client.stop()
    client.close()
    assert all(r[3] != Command.BALANCE_MODE for r in server.requests)
