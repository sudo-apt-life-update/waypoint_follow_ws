"""Turn a path YAML into a list of timed velocity segments.

This project drives the robot **open loop**: nothing measures where the robot
actually is, so a path is executed purely as "hold this velocity for this long".
Everything below is therefore about converting the units a human wants to write
(metres, degrees) into the units the controller wants (m/s, rad/s, seconds),
and about refusing to do that when the file is wrong.

The validation is deliberately strict -- unknown keys are an error, not a
warning. A misspelled `distnace` that silently defaulted to zero would be a
confusing no-op; a misspelled `angle` that silently defaulted to a full turn
would be worse. This file is the last place a typo is still cheap.

Path file format
----------------

    name: square 2m               # optional, for logs
    defaults:
      linear_speed: 0.4           # m/s,    used when a segment omits `speed`
      angular_speed: 0.5          # rad/s,  used when a segment omits `rate`
      settle: 1.0                 # s of standing still after every segment
      ramp: 0.5                   # s to ease IN to each segment
      ramp_down: 1.5              # s to ease OUT -- longer, stopping is harder
    segments:
      - {move: forward, distance: 2.0}          # metres, negative = backward
      - {move: turn,    angle: 90}              # degrees, positive = left/CCW
      - {move: strafe,  distance: 0.5}          # metres, positive = left
      - {move: wait,    duration: 2.0}          # stand still
      - {move: raw, vx: 0.2, vy: 0.0, omega: 0.1, duration: 3.0}

`forward`, `strafe` and `turn` may each override the default rate with
`speed:` (linear) or `rate:` (angular), and any segment may override `settle:`
and `ramp:`. `raw` is the escape hatch for gaits the three named moves cannot
express; its duration is the time the plateau velocity is held, same as the
others.
"""

import math
import os

import yaml

# Sanity ceilings applied while parsing, independent of the hard clamps in
# g1_loco_server. Two layers on purpose: this one produces a readable error
# naming the offending segment, the C++ one is the limit that actually holds
# when something bypasses this code.
MAX_SPEED = 1.0        # m/s
MAX_RATE = 1.5         # rad/s
MAX_SEGMENT_TIME = 120.0  # s -- a single segment longer than this is a typo

# [MEASURED 2026-08-11] Below roughly this speed the G1's gait stomps without
# translating. It matters for more than picking `linear_speed`: a velocity ramp
# spends its whole duration under the target speed, so on this robot the ramp
# is largely dead time that the trapezoid's area still credits as distance.
# Measured: a 3 m segment at 0.4 m/s covered 1.5 m with a 1.5 s ramp_down and
# 1.3 m with 2.5 s, while a constant-velocity 4 m command covered 4.2 m.
# Long ramps are not gentle here; they are lost ground.
MIN_EFFECTIVE_SPEED = 0.4  # m/s

# [MEASURED 2026-08-12, remote off] After a translation segment ends the robot
# settles backwards by a roughly fixed amount -- 0.3, 0.4, 0.4, 0.3 m over four
# runs -- as its balance controller brings the torso back over its feet. It is
# an END EFFECT, not a rate error: it does not grow with segment length.
#
# That distinction matters, because linear_scale multiplies time and therefore
# scales with length. Correcting a fixed loss with a proportional knob works
# for exactly one segment length and is wrong for every other. So the two are
# separate: linear_scale for the rate, distance_offset for the end effect.
#
# With the remote ON the same measurement scattered from 0 to 1.0 m. Run with
# the remote powered off; see INSTRUCTIONS.md section 11.
DEFAULT_DISTANCE_OFFSET = 0.0  # m -- measure it, do not guess

# [MEASURED 2026-08-11] 0.4 m/s, not 0.3. Below roughly 0.4 this G1's gait
# stomps without translating: a commanded 3 m at 0.3 m/s covered about 1 m,
# while 0.4 m/s over 10 s covered 4.2 m of a commanded 4.0. A slower default is
# not a safer one here; it is one that does not walk.
DEFAULTS = {
    "linear_speed": 0.4,
    "angular_speed": 0.5,
    "settle": 1.0,
    "ramp": 0.5,
    # Longer than the up ramp: stopping is the hard direction. See Segment.
    "ramp_down": 1.5,
}

MOVES = ("forward", "strafe", "turn", "wait", "raw")

_MOVE_KEYS = {
    "forward": {"move", "ramp_down", "distance", "speed", "settle", "ramp", "comment"},
    "strafe": {"move", "ramp_down", "distance", "speed", "settle", "ramp", "comment"},
    "turn": {"move", "angle", "rate", "settle", "ramp", "ramp_down",
             "comment"},
    "wait": {"move", "duration", "comment"},
    "raw": {"move", "ramp_down", "vx", "vy", "omega", "duration", "settle", "ramp",
            "comment"},
}


class PathError(ValueError):
    """The path file is malformed. The message names the segment."""


class Segment:
    """One constant-velocity move, with symmetric ease-in/ease-out ramps.

    The ramps exist because a humanoid asked to step from 0 to 0.3 m/s in one
    control tick lurches. They are shaped so the *integral* is unchanged: a
    trapezoid of peak `v`, ramp `ramp` and plateau `plateau` covers exactly
    `v * (plateau + ramp)`, which is set equal to the distance the file asked
    for. Ramping without that correction would quietly undershoot every
    segment, and in open loop nothing would ever correct it.
    """

    def __init__(self, label, vx, vy, omega, nominal_time, ramp, settle,
                 ramp_up=None, ramp_down=None):
        # Asymmetric on purpose. Accelerating a humanoid is easy -- it just
        # starts stepping. Decelerating is not: at 0.4 m/s there is real
        # momentum, and if the velocity command vanishes faster than the robot
        # can shed it, the balance controller arrests it by stepping, usually
        # backwards and off-heading. A longer ramp down gives it time to bleed
        # the speed off while still being told where to go.
        ramp_up = ramp if ramp_up is None else ramp_up
        ramp_down = ramp if ramp_down is None else ramp_down

        # A segment too short for both ramps cannot reach full speed. Scale
        # them down together, preserving their ratio, so the area stays exact.
        total_ramp = ramp_up + ramp_down
        if total_ramp > 2.0 * nominal_time and total_ramp > 0.0:
            shrink = (2.0 * nominal_time) / total_ramp
            ramp_up *= shrink
            ramp_down *= shrink

        self.label = label
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.ramp_up = ramp_up
        self.ramp_down = ramp_down
        # Trapezoid area = v * (plateau + (ramp_up + ramp_down)/2), and we want
        # that to equal v * nominal_time -- the distance the file asked for.
        self.plateau = max(0.0, nominal_time - 0.5 * (ramp_up + ramp_down))
        self.settle = settle

    @property
    def ramp(self):
        """Backwards-compatible view: the up ramp."""
        return self.ramp_up

    @property
    def move_time(self):
        """Seconds of motion, ramps included."""
        return self.plateau + self.ramp_up + self.ramp_down

    @property
    def total_time(self):
        """Seconds this segment occupies, including the settle that follows."""
        return self.move_time + self.settle

    def velocity_at(self, elapsed):
        """Scaled (vx, vy, omega) at `elapsed` seconds into the segment."""
        if elapsed >= self.move_time:
            return 0.0, 0.0, 0.0        # settling

        if elapsed < self.ramp_up:
            scale = elapsed / self.ramp_up if self.ramp_up > 0.0 else 1.0
        elif elapsed < self.ramp_up + self.plateau:
            scale = 1.0
        elif self.ramp_down > 0.0:
            scale = (self.move_time - elapsed) / self.ramp_down
        else:
            scale = 1.0

        scale = min(1.0, max(0.0, scale))
        return self.vx * scale, self.vy * scale, self.omega * scale

    def __repr__(self):
        return (f"Segment({self.label!r}, v=({self.vx:.2f}, {self.vy:.2f}, "
                f"{self.omega:.2f}), move={self.move_time:.1f}s, "
                f"settle={self.settle:.1f}s)")


class Path:
    def __init__(self, name, segments, source):
        self.name = name
        self.segments = segments
        self.source = source

    @property
    def total_time(self):
        return sum(segment.total_time for segment in self.segments)

    def describe(self):
        lines = [f"{self.name} ({len(self.segments)} segments, "
                 f"{self.total_time:.1f} s nominal)"]
        for index, segment in enumerate(self.segments):
            lines.append(f"  {index + 1:2d}. {segment.label:<28} "
                         f"{segment.move_time:5.1f} s move "
                         f"+ {segment.settle:.1f} s settle")
        return "\n".join(lines)


def _number(raw, key, where):
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise PathError(f"{where}: {key} must be a number, got {raw!r}")
    value = float(raw)
    if not math.isfinite(value):
        raise PathError(f"{where}: {key} must be finite, got {raw!r}")
    return value


def _required(spec, key, where):
    if key not in spec:
        raise PathError(f"{where}: missing required key {key!r}")
    return _number(spec[key], key, where)


def _positive(value, key, limit, where):
    if value <= 0.0:
        raise PathError(f"{where}: {key} must be greater than zero, got {value}")
    if value > limit:
        raise PathError(f"{where}: {key} of {value} exceeds the sanity limit "
                        f"of {limit}. Raise it in path.py if you really mean it.")
    return value


def _build_segment(spec, index, defaults, linear_scale, angular_scale,
                   ramp_down_override=None, ramp_override=None,
                   distance_offset=0.0):
    where = f"segment {index + 1}"

    if not isinstance(spec, dict):
        raise PathError(f"{where}: expected a mapping, got {type(spec).__name__}")

    move = spec.get("move")
    if move not in MOVES:
        raise PathError(f"{where}: move must be one of {list(MOVES)}, "
                        f"got {move!r}")

    unknown = set(spec) - _MOVE_KEYS[move]
    if unknown:
        raise PathError(f"{where}: unknown key(s) {sorted(unknown)} for "
                        f"move {move!r}; allowed: "
                        f"{sorted(_MOVE_KEYS[move])}")

    settle = _number(spec.get("settle", defaults["settle"]), "settle", where)
    ramp = _number(spec.get("ramp", defaults["ramp"]), "ramp", where)
    if ramp_override is not None:
        ramp = float(ramp_override)
    # An override beats the per-segment value: it exists to retune the stop
    # across a whole path from the command line without editing the file.
    ramp_down = _number(spec.get("ramp_down", defaults["ramp_down"]),
                        "ramp_down", where)
    if ramp_down_override is not None:
        ramp_down = float(ramp_down_override)
    if settle < 0.0:
        raise PathError(f"{where}: settle must not be negative")
    if ramp < 0.0:
        raise PathError(f"{where}: ramp must not be negative")
    if ramp_down < 0.0:
        raise PathError(f"{where}: ramp_down must not be negative")

    if move == "wait":
        duration = _required(spec, "duration", where)
        _positive(duration, "duration", MAX_SEGMENT_TIME, where)
        return Segment(f"wait {duration:g}s", 0.0, 0.0, 0.0, duration,
                       0.0, 0.0)

    if move == "raw":
        vx = _number(spec.get("vx", 0.0), "vx", where)
        vy = _number(spec.get("vy", 0.0), "vy", where)
        omega = _number(spec.get("omega", 0.0), "omega", where)
        duration = _required(spec, "duration", where)
        _positive(duration, "duration", MAX_SEGMENT_TIME, where)
        if abs(vx) > MAX_SPEED or abs(vy) > MAX_SPEED:
            raise PathError(f"{where}: raw speed exceeds {MAX_SPEED} m/s")
        if abs(omega) > MAX_RATE:
            raise PathError(f"{where}: raw omega exceeds {MAX_RATE} rad/s")
        label = f"raw ({vx:g}, {vy:g}, {omega:g}) {duration:g}s"
        return Segment(label, vx, vy, omega, duration, ramp, settle,
                       ramp_down=ramp_down)

    if move == "turn":
        angle_deg = _required(spec, "angle", where)
        rate = _number(spec.get("rate", defaults["angular_speed"]),
                       "rate", where)
        _positive(rate, "rate", MAX_RATE, where)
        if angle_deg == 0.0:
            raise PathError(f"{where}: angle of zero does nothing; use "
                            f"move: wait if that is the intent")
        # Scale corrects for the robot consistently over- or under-turning.
        # It multiplies time, not rate: the rate is what keeps the gait stable,
        # so calibration should not change it.
        angle_rad = math.radians(abs(angle_deg)) * angular_scale
        nominal_time = angle_rad / rate
        _positive(nominal_time, "turn duration", MAX_SEGMENT_TIME, where)
        omega = rate if angle_deg > 0.0 else -rate
        direction = "left" if angle_deg > 0.0 else "right"
        label = f"turn {abs(angle_deg):g} deg {direction}"
        return Segment(label, 0.0, 0.0, omega, nominal_time, ramp, settle,
                       ramp_down=ramp_down)

    # forward / strafe
    distance = _required(spec, "distance", where)
    speed = _number(spec.get("speed", defaults["linear_speed"]), "speed", where)
    _positive(speed, "speed", MAX_SPEED, where)
    if distance == 0.0:
        raise PathError(f"{where}: distance of zero does nothing; use "
                        f"move: wait if that is the intent")

    # Two independent corrections, applied in the right order.
    #
    # distance_offset is measured in REAL metres -- you watch the robot roll
    # back 0.35 m and type 0.35. So it is added to the target first, and the
    # sum is then scaled by linear_scale, which converts real metres into the
    # commanded metres this robot needs to actually cover them.
    #
    # Adding it after the scale would under-correct by exactly the efficiency
    # factor: at linear_scale 2.0 an offset of 0.35 would buy only 0.175 m of
    # real ground, and the calibration would quietly land short.
    commanded_distance = (abs(distance) + distance_offset) * linear_scale
    if commanded_distance <= 0.0:
        raise PathError(
            f"{where}: distance_offset of {distance_offset} cancels a "
            f"{distance} m segment entirely")
    nominal_time = commanded_distance / speed
    _positive(nominal_time, "segment duration", MAX_SEGMENT_TIME, where)
    signed = speed if distance > 0.0 else -speed

    if move == "forward":
        direction = "forward" if distance > 0.0 else "backward"
        label = f"{direction} {abs(distance):g} m"
        return Segment(label, signed, 0.0, 0.0, nominal_time, ramp, settle,
                       ramp_down=ramp_down)

    direction = "left" if distance > 0.0 else "right"
    label = f"strafe {direction} {abs(distance):g} m"
    return Segment(label, 0.0, signed, 0.0, nominal_time, ramp, settle,
                       ramp_down=ramp_down)


def load_path(filename, linear_scale=1.0, angular_scale=1.0,
              ramp_down_override=None, ramp_override=None,
              distance_offset=0.0):
    """Parse a path YAML into a Path. Raises PathError on anything suspect."""
    if not os.path.isfile(filename):
        raise PathError(f"no such path file: {filename}")

    with open(filename, "r") as handle:
        try:
            document = yaml.safe_load(handle)
        except yaml.YAMLError as exc:
            raise PathError(f"{filename} is not valid YAML: {exc}") from exc

    if not isinstance(document, dict):
        raise PathError(f"{filename}: expected a mapping at the top level")

    unknown = set(document) - {"name", "defaults", "segments"}
    if unknown:
        raise PathError(f"{filename}: unknown top-level key(s) "
                        f"{sorted(unknown)}")

    defaults = dict(DEFAULTS)
    if ramp_down_override is not None:
        defaults["ramp_down"] = float(ramp_down_override)
    if ramp_override is not None:
        defaults["ramp"] = float(ramp_override)
    supplied = document.get("defaults") or {}
    if not isinstance(supplied, dict):
        raise PathError(f"{filename}: defaults must be a mapping")
    unknown = set(supplied) - set(DEFAULTS)
    if unknown:
        raise PathError(f"{filename}: unknown default(s) {sorted(unknown)}; "
                        f"allowed: {sorted(DEFAULTS)}")
    for key, value in supplied.items():
        defaults[key] = _number(value, f"defaults.{key}", filename)

    segments_spec = document.get("segments")
    if not isinstance(segments_spec, list) or not segments_spec:
        raise PathError(f"{filename}: segments must be a non-empty list")

    segments = [
        _build_segment(spec, index, defaults, linear_scale, angular_scale,
                       ramp_down_override, ramp_override, distance_offset)
        for index, spec in enumerate(segments_spec)
    ]

    name = document.get("name") or os.path.splitext(
        os.path.basename(filename))[0]

    return Path(str(name), segments, filename)
