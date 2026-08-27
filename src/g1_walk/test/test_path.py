"""Tests for the path parser and the trapezoid integral.

The parser is the only thing standing between a typo in a YAML file and a
humanoid walking somewhere unexpected, so the rejection cases matter at least
as much as the happy path.

    colcon test --packages-select g1_walk && colcon test-result --verbose
"""

import math
import os
import tempfile

import pytest

from g1_walk.path import MAX_SPEED, PathError, Segment, load_path


def write_path(text):
    handle = tempfile.NamedTemporaryFile(
        "w", suffix=".yaml", delete=False, encoding="utf-8")
    handle.write(text)
    handle.close()
    return handle.name


def integrate(segment, dt=0.001):
    """Ground actually covered by the ramped profile."""
    total = 0.0
    steps = int(round(segment.move_time / dt))
    for step in range(steps):
        vx, _vy, _omega = segment.velocity_at(step * dt)
        total += vx * dt
    return total


# -- the ramp must not change the distance --------------------------------

def test_ramp_preserves_distance():
    # 2 m at 0.4 m/s = 5 s nominal. With 0.5 s ramps the plateau shrinks to
    # 4.5 s, so the trapezoid area is still 2 m. Getting this wrong would make
    # every segment quietly undershoot, and open loop nothing would correct it.
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=5.0, ramp=0.5,
                      settle=0.0)
    assert segment.plateau == pytest.approx(4.5)
    assert segment.move_time == pytest.approx(5.5)
    assert integrate(segment) == pytest.approx(2.0, abs=1e-3)


def test_segment_shorter_than_ramp_still_covers_the_distance():
    # 0.3 s nominal with a 0.5 s ramp: the ramp collapses to the whole segment
    # rather than overrunning it.
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=0.3, ramp=0.5,
                      settle=0.0)
    assert segment.ramp == pytest.approx(0.3)
    assert segment.plateau == pytest.approx(0.0)
    assert integrate(segment) == pytest.approx(0.12, abs=1e-3)


def test_velocity_is_zero_while_settling():
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=1.0, ramp=0.0,
                      settle=2.0)
    assert segment.velocity_at(0.5) == (0.4, 0.0, 0.0)
    assert segment.velocity_at(1.5) == (0.0, 0.0, 0.0)
    assert segment.total_time == pytest.approx(3.0)


# -- unit conversion ------------------------------------------------------

def test_forward_and_turn_durations():
    filename = write_path("""
name: t
defaults: {linear_speed: 0.5, angular_speed: 0.5, settle: 0.0, ramp: 0.0,
           ramp_down: 0.0}
segments:
  - {move: forward, distance: 2.0}
  - {move: turn, angle: 90}
""")
    path = load_path(filename)
    os.unlink(filename)

    assert path.segments[0].move_time == pytest.approx(4.0)     # 2.0 / 0.5
    assert path.segments[0].vx == pytest.approx(0.5)
    # 90 deg = pi/2 rad at 0.5 rad/s
    assert path.segments[1].move_time == pytest.approx(math.pi / 2 / 0.5)
    assert path.segments[1].omega == pytest.approx(0.5)


def test_signs_follow_rep103():
    filename = write_path("""
segments:
  - {move: forward, distance: -1.0}
  - {move: strafe,  distance:  1.0}
  - {move: turn,    angle:   -90}
""")
    path = load_path(filename)
    os.unlink(filename)

    assert path.segments[0].vx < 0      # negative distance = backward
    assert path.segments[1].vy > 0      # positive strafe = left
    assert path.segments[2].omega < 0   # negative angle = clockwise


def test_scales_stretch_time_not_speed():
    filename = write_path("""
defaults: {linear_speed: 0.5, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: forward, distance: 1.0}
""")
    plain = load_path(filename)
    scaled = load_path(filename, linear_scale=1.2)
    os.unlink(filename)

    assert scaled.segments[0].vx == pytest.approx(plain.segments[0].vx)
    assert scaled.segments[0].move_time == pytest.approx(
        plain.segments[0].move_time * 1.2)


# -- rejection ------------------------------------------------------------

@pytest.mark.parametrize("body,fragment", [
    ("segments:\n  - {move: forwards, distance: 1.0}", "move must be one of"),
    ("segments:\n  - {move: forward, distnace: 1.0}", "unknown key"),
    ("segments:\n  - {move: forward}", "missing required key 'distance'"),
    ("segments:\n  - {move: forward, distance: 0}", "does nothing"),
    ("segments:\n  - {move: turn, angle: 0}", "does nothing"),
    ("segments:\n  - {move: forward, distance: one}", "must be a number"),
    ("segments:\n  - {move: forward, distance: 1, speed: 0}",
     "must be greater than zero"),
    (f"segments:\n  - {{move: forward, distance: 1, speed: {MAX_SPEED + 1}}}",
     "sanity limit"),
    ("segments:\n  - {move: forward, distance: 1, ramp: -1}",
     "must not be negative"),
    ("segments: []", "non-empty list"),
    ("segments:\n  - {move: forward, distance: 1}\ndefalts: {}",
     "unknown top-level key"),
    ("defaults: {linear_speed: 0.3, wobble: 1}\nsegments:\n"
     "  - {move: forward, distance: 1}", "unknown default"),
    ("segments:\n  - {move: raw, vx: 5.0, duration: 1.0}", "exceeds"),
    ("segments:\n  - {move: forward, distance: 1000}", "sanity limit"),
])
def test_rejects_bad_files(body, fragment):
    filename = write_path(body)
    try:
        with pytest.raises(PathError) as excinfo:
            load_path(filename)
    finally:
        os.unlink(filename)
    assert fragment in str(excinfo.value)


def test_missing_file_is_a_path_error():
    with pytest.raises(PathError, match="no such path file"):
        load_path("/nonexistent/path.yaml")


def test_error_names_the_segment():
    filename = write_path("""
segments:
  - {move: forward, distance: 1.0}
  - {move: forward, distance: 1.0}
  - {move: turn, angle: bad}
""")
    try:
        with pytest.raises(PathError, match="segment 3"):
            load_path(filename)
    finally:
        os.unlink(filename)


# -- the shipped path files must all be valid -----------------------------

def test_packaged_paths_load():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    directory = os.path.join(here, "config", "paths")
    files = sorted(name for name in os.listdir(directory)
                   if name.endswith(".yaml"))
    assert files, "no path files found to check"
    for name in files:
        path = load_path(os.path.join(directory, name))
        assert path.segments
        assert path.total_time > 0.0


def test_square_returns_to_start_in_the_ideal_case():
    """The commanded square closes on paper. On the floor it will not.

    Worth asserting anyway: if the parser ever mangles a turn, this catches it
    before the robot does.
    """
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = load_path(os.path.join(here, "config", "paths", "square_2m.yaml"))

    dt = 0.001
    x = y = yaw = 0.0
    for segment in path.segments:
        steps = int(round(segment.move_time / dt))
        for step in range(steps):
            vx, vy, omega = segment.velocity_at(step * dt)
            mid = yaw + 0.5 * omega * dt
            x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
            y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
            yaw += omega * dt

    assert math.hypot(x, y) < 0.02
    assert abs(math.degrees(yaw) - 360.0) < 1.0


# -- yaw trim -------------------------------------------------------------

class _TrimStub:
    """Just the trim logic from PathFollower, without a ROS context.

    Importing path_follower needs rclpy and claims a process lock, neither of
    which belongs in a unit test. The rule being pinned is small and worth
    pinning: trim applies to straights only.
    """

    def __init__(self, yaw_trim):
        self.yaw_trim = yaw_trim

    _apply_trim = None  # bound below from the real implementation


def _load_apply_trim():
    """Pull the real _apply_trim out of the source, so the test cannot drift.

    Reading it from the module would drag in rclpy; extracting the function
    keeps the test honest about *which* implementation it is checking.
    """
    import ast
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    source = open(os.path.join(here, "g1_walk", "path_follower.py")).read()
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_apply_trim":
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {}
            exec(compile(module, "<extracted>", "exec"), namespace)
            return namespace["_apply_trim"]
    raise AssertionError("_apply_trim not found in path_follower.py")


def test_yaw_trim_applies_to_straights_only():
    apply_trim = _load_apply_trim()
    node = _TrimStub(0.16)

    # A forward segment gets the trim.
    assert apply_trim(node, 0.3, 0.0, 0.0) == pytest.approx(0.16)
    # A strafe does too -- it is still a translation that can veer.
    assert apply_trim(node, 0.0, 0.2, 0.0) == pytest.approx(0.16)
    # A commanded turn must be left alone, or the path's angle is corrupted.
    assert apply_trim(node, 0.0, 0.0, 0.4) == pytest.approx(0.4)
    # Settling must stay still, not creep round.
    assert apply_trim(node, 0.0, 0.0, 0.0) == pytest.approx(0.0)


def test_yaw_trim_of_zero_changes_nothing():
    apply_trim = _load_apply_trim()
    node = _TrimStub(0.0)
    for vx, vy, omega in ((0.3, 0.0, 0.0), (0.0, 0.0, 0.4), (0.0, 0.0, 0.0)):
        assert apply_trim(node, vx, vy, omega) == pytest.approx(omega)


# -- asymmetric ramps -----------------------------------------------------

def test_ramp_down_can_be_longer_and_distance_is_still_exact():
    """Stopping is the hard direction, so ramp_down defaults longer than ramp.

    The area must still equal the distance the file asked for, or every segment
    would quietly undershoot -- and open loop nothing would notice.
    """
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=7.5, ramp=0.5,
                      settle=0.0, ramp_down=1.5)
    assert segment.ramp_up == pytest.approx(0.5)
    assert segment.ramp_down == pytest.approx(1.5)
    # plateau = nominal - (up + down)/2
    assert segment.plateau == pytest.approx(6.5)
    assert segment.move_time == pytest.approx(8.5)
    assert integrate(segment) == pytest.approx(3.0, abs=1e-3)


def test_ramp_down_decays_over_its_own_length():
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=7.5, ramp=0.5,
                      settle=0.0, ramp_down=1.5)
    # Halfway down the 1.5 s tail, speed should be about half.
    midpoint = segment.move_time - 0.75
    assert segment.velocity_at(midpoint)[0] == pytest.approx(0.2, abs=0.01)
    # Just before the end, nearly stopped -- this is the whole point.
    assert segment.velocity_at(segment.move_time - 0.05)[0] < 0.02


def test_ramps_shrink_together_when_the_segment_is_too_short():
    # 0.5 s of motion cannot hold a 0.5 s up ramp plus a 1.5 s down ramp.
    segment = Segment("test", 0.4, 0.0, 0.0, nominal_time=0.5, ramp=0.5,
                      settle=0.0, ramp_down=1.5)
    assert segment.plateau == pytest.approx(0.0)
    # Ratio preserved: down is still 3x up.
    assert segment.ramp_down == pytest.approx(3.0 * segment.ramp_up)
    assert integrate(segment) == pytest.approx(0.2, abs=1e-3)


def test_ramp_down_is_settable_per_segment():
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.5, ramp_down: 1.5, settle: 0.0}
segments:
  - {move: forward, distance: 2.0, ramp_down: 3.0}
""")
    path = load_path(filename)
    os.unlink(filename)
    assert path.segments[0].ramp_down == pytest.approx(3.0)
    assert integrate(path.segments[0]) == pytest.approx(2.0, abs=1e-3)


def test_negative_ramp_down_is_rejected():
    filename = write_path(
        "segments:\n  - {move: forward, distance: 1, ramp_down: -1}")
    try:
        with pytest.raises(PathError, match="ramp_down must not be negative"):
            load_path(filename)
    finally:
        os.unlink(filename)


def test_zero_ramp_override_is_honoured_not_treated_as_unset():
    """Regression: `ramp_down:=0` silently kept the file's value.

    The override used `value or None`, and 0.0 is falsy -- so zero, the one
    value the flag was added to test, was the one it could not express.
    """
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.5, ramp_down: 1.5, settle: 0.0}
segments:
  - {move: forward, distance: 3.0}
""")
    try:
        default = load_path(filename)
        stepped = load_path(filename, ramp_down_override=0.0, ramp_override=0.0)
    finally:
        os.unlink(filename)

    # 3.0 m at 0.4 m/s = 7.5 s of motion; ramps only add time on top.
    assert default.segments[0].move_time == pytest.approx(8.5)
    assert stepped.segments[0].move_time == pytest.approx(7.5)
    assert stepped.segments[0].ramp_up == 0.0
    assert stepped.segments[0].ramp_down == 0.0
    # Distance is unchanged either way -- that is the whole point of the
    # area-preserving trapezoid.
    assert integrate(default.segments[0]) == pytest.approx(3.0, abs=1e-3)
    assert integrate(stepped.segments[0]) == pytest.approx(3.0, abs=1e-3)


def test_ramp_override_of_zero_still_allows_a_long_ramp_down():
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.5, ramp_down: 1.5, settle: 0.0}
segments:
  - {move: forward, distance: 3.0}
""")
    try:
        path = load_path(filename, ramp_override=0.0, ramp_down_override=2.5)
    finally:
        os.unlink(filename)
    assert path.segments[0].ramp_up == 0.0
    assert path.segments[0].ramp_down == pytest.approx(2.5)
    assert integrate(path.segments[0]) == pytest.approx(3.0, abs=1e-3)


# -- distance_offset ------------------------------------------------------

def test_distance_offset_is_fixed_not_proportional():
    """The retreat at the end of a segment does not grow with segment length.

    linear_scale multiplies time and so scales with distance; correcting a
    fixed end effect with it would be right for one segment length and wrong
    for every other. The two knobs must stay independent.
    """
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: forward, distance: 1.0}
  - {move: forward, distance: 4.0}
""")
    try:
        plain = load_path(filename)
        offset = load_path(filename, distance_offset=0.35)
    finally:
        os.unlink(filename)

    # Both segments gain the SAME extra distance, regardless of their length.
    for index in (0, 1):
        extra = (integrate(offset.segments[index])
                 - integrate(plain.segments[index]))
        assert extra == pytest.approx(0.35, abs=1e-3)


def test_distance_offset_composes_with_linear_scale():
    """scale corrects the rate, offset the end effect, and the offset is in
    REAL metres: commanded distance is `(asked + offset) * scale`."""
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: forward, distance: 3.0}
""")
    try:
        path = load_path(filename, linear_scale=2.0, distance_offset=0.35)
    finally:
        os.unlink(filename)
    # (3.0 target + 0.35 given back) * 2.0 efficiency = 6.7 commanded
    assert integrate(path.segments[0]) == pytest.approx(6.7, abs=1e-3)
    assert path.segments[0].move_time == pytest.approx(6.7 / 0.4)


def test_distance_offset_does_not_touch_turns():
    filename = write_path("""
defaults: {angular_speed: 0.5, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: turn, angle: 90}
""")
    try:
        plain = load_path(filename)
        offset = load_path(filename, distance_offset=0.35)
    finally:
        os.unlink(filename)
    assert (offset.segments[0].move_time
            == pytest.approx(plain.segments[0].move_time))


def test_offset_is_in_real_metres_so_scaling_applies_to_it():
    """The offset must survive the efficiency factor.

    Applied after the scale, an 0.35 m offset at linear_scale 2.0 would buy
    only 0.175 m of real ground and the run would land short -- which is
    exactly what the first version of this did.
    """
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: forward, distance: 3.0}
""")
    try:
        path = load_path(filename, linear_scale=2.0, distance_offset=0.35)
    finally:
        os.unlink(filename)

    commanded = integrate(path.segments[0])
    efficiency = 0.5                      # measured: covers half of commanded
    real_forward = commanded * efficiency
    assert real_forward - 0.35 == pytest.approx(3.0, abs=1e-3)


def test_offset_that_cancels_a_segment_is_rejected():
    filename = write_path("""
defaults: {linear_speed: 0.4, ramp: 0.0, ramp_down: 0.0, settle: 0.0}
segments:
  - {move: forward, distance: 0.2}
""")
    try:
        with pytest.raises(PathError, match="cancels a"):
            load_path(filename, distance_offset=-0.5)
    finally:
        os.unlink(filename)


# -- closed-loop helpers --------------------------------------------------
#
# path_follower needs rclpy and claims a process lock, so the pure functions
# are extracted from source rather than imported. Keeps the test honest about
# which implementation it checks.

def _extract(name):
    import ast
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    source = open(os.path.join(here, "g1_walk", "path_follower.py")).read()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {"math": math}
            exec(compile(module, "<extracted>", "exec"), namespace)
            return namespace[name]
    raise AssertionError(f"{name} not found in path_follower.py")


def test_wrap_folds_angles_the_short_way():
    """A 359 deg error must read as -1 deg, or the controller spins the long
    way round to correct a hair's breadth."""
    wrap = _extract("_wrap")
    assert wrap(math.radians(359)) == pytest.approx(math.radians(-1), abs=1e-6)
    assert wrap(math.radians(-359)) == pytest.approx(math.radians(1), abs=1e-6)
    assert wrap(math.radians(180)) == pytest.approx(math.pi, abs=1e-6)
    assert wrap(0.0) == pytest.approx(0.0)


class _SteerStub:
    """Enough of PathFollower for _steer and _measured_progress."""

    _wrap = staticmethod(_extract("_wrap"))

    def __init__(self, start, pose, direction=0.0):
        self.seg_start = start
        self.odom_pose = pose
        self.seg_direction = direction
        self.yaw_gain = 1.2
        self.cross_track_gain = 0.8
        self.max_cross_track_angle = 0.4
        self.max_correction_omega = 0.4


def test_progress_projects_onto_the_segment_direction():
    """Distance must be measured ALONG the intended line, not as straight-line
    displacement -- otherwise sideways drift would count as progress."""
    progress = _extract("_measured_progress")
    # Started at the origin heading +x; now 2 m forward and 0.5 m left.
    node = _SteerStub((0.0, 0.0, 0.0), (2.0, 0.5, 0.0), direction=0.0)
    along, cross = progress(node)
    assert along == pytest.approx(2.0)
    assert cross == pytest.approx(0.5)


def test_progress_handles_a_rotated_segment():
    progress = _extract("_measured_progress")
    # Segment heading north; robot moved 3 m north and 1 m east (right).
    node = _SteerStub((0.0, 0.0, math.pi / 2), (1.0, 3.0, math.pi / 2),
                      direction=math.pi / 2)
    along, cross = progress(node)
    assert along == pytest.approx(3.0)
    assert cross == pytest.approx(-1.0)     # east is right of north


def test_steer_corrects_a_rightward_veer():
    """The measured failure: heading drifts right, so steering must be left
    (positive omega, REP-103)."""
    steer = _extract("_steer")
    node = _SteerStub((0.0, 0.0, 0.0), (1.0, 0.0, math.radians(-20)))
    assert steer(node, cross_track=0.0) > 0.0


def test_steer_is_bounded():
    """A large error must not produce a yaw rate that fights the walk."""
    steer = _extract("_steer")
    node = _SteerStub((0.0, 0.0, 0.0), (1.0, 0.0, math.radians(-170)))
    assert abs(steer(node, cross_track=0.0)) <= node.max_correction_omega
    node = _SteerStub((0.0, 0.0, 0.0), (1.0, 5.0, 0.0))
    assert abs(steer(node, cross_track=5.0)) <= node.max_correction_omega


def test_steer_returns_to_the_line_when_heading_is_already_correct():
    """Heading right but pushed left of the line: steer back right."""
    steer = _extract("_steer")
    node = _SteerStub((0.0, 0.0, 0.0), (1.0, 0.5, 0.0))
    assert steer(node, cross_track=0.5) < 0.0


class _LeadStub:
    def __init__(self, along_at_stop, seen=False, lead=0.0):
        self.along_at_stop = along_at_stop
        self.stop_lead_seen = seen
        self.stop_lead = lead
        self.logged = []

    def get_logger(self):
        stub = self

        class _L:
            @staticmethod
            def info(text):
                stub.logged.append(text)
        return _L()


def test_stop_lead_is_learned_from_the_settle():
    """Regression: without this the loop cannot converge.

    Every approach ends in a stop and every stop costs a retreat, so a
    correction gains the missing distance and immediately gives it back --
    observed as three identical corrections achieving nothing. Learning the
    retreat lets the approach aim past the target.
    """
    learn = _extract("_learn_stop_lead")
    node = _LeadStub(along_at_stop=3.02)
    learn(node, 2.67)                       # settled back to 2.67
    assert node.stop_lead == pytest.approx(0.35, abs=0.01)
    assert node.stop_lead_seen


def test_stop_lead_is_clamped_against_a_wild_measurement():
    """A bad value would send the robot past the target on every later
    segment, which is worse than not correcting."""
    learn = _extract("_learn_stop_lead")
    node = _LeadStub(along_at_stop=50.0)
    learn(node, 0.0)
    assert node.stop_lead <= 1.0

    node = _LeadStub(along_at_stop=-50.0)
    learn(node, 0.0)
    assert node.stop_lead >= -0.5


def test_stop_lead_blends_rather_than_jumping():
    learn = _extract("_learn_stop_lead")
    node = _LeadStub(along_at_stop=1.0, seen=True, lead=0.30)
    learn(node, 0.5)                        # this settle cost 0.50
    # Blended, not replaced: one odd segment must not dominate.
    assert 0.30 < node.stop_lead < 0.50


def test_stop_lead_ignores_a_segment_it_never_saw_stop():
    learn = _extract("_learn_stop_lead")
    node = _LeadStub(along_at_stop=None)
    learn(node, 1.0)
    assert not node.stop_lead_seen
