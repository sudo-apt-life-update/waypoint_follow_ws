# INSTRUCTIONS — why this workspace is built the way it is

`README.md` says how to run things. This file says why, so that a decision made
once does not have to be rediscovered by walking a robot into a wall.

---

## 1. The objective, stated narrowly

Walk a G1 along a path that is fixed in advance. Not: plan a path, not: localise
the robot, not: react to the world. Obstacle detection and avoidance are the
next stage and will need a pose estimate; this stage deliberately does not.

Everything below follows from that. When something here looks under-built for
autonomous navigation, it is, on purpose.

---

## 2. Open loop, and what that costs

**The decision.** Each segment is "hold this velocity for this long". Nothing
measures the result. There is no odometry, no IMU feedback on heading, no
correction of any kind.

**Why.** The alternative needs a pose estimate, and the only source available is
visual odometry from the D435i, which means standing up the whole camera stack
before the robot takes its first step. Open loop gets a robot walking a path
today with nothing but the SDK, and it makes the drift *visible* rather than
partially hidden behind a mediocre estimator.

**What it costs, concretely.** Errors compound and are never corrected:

- A humanoid's actual travel per commanded m/s is not 1:1 and varies with stand
  height, swing height, speed, load and floor surface.
- Turns are worse than straights. A 90-degree turn-in-place has heading error
  that carries into every subsequent segment as a lateral error growing with
  distance.
- Nothing detects a foot slipping, a stumble, or the robot being physically
  blocked. The path continues on schedule regardless of what happened.

A 2 m square does not close. Expect to be off by a meaningful fraction of a
metre. `config/paths/square_2m.yaml` exists to make that concrete rather than
theoretical.

**When to abandon this.** The moment a task needs the robot to *be* somewhere,
rather than to *have walked* somewhere. Obstacle avoidance is exactly that
moment: dodging an obstacle means leaving the path, and rejoining it requires
knowing where you are. Section 9 sketches the transition.

**The one mitigation.** `linear_scale` and `angular_scale`. They multiply
commanded *time*, not commanded speed — the speed is what keeps the gait
stable, so calibration must not touch it. They correct a consistent bias; they
do nothing about run-to-run variance.

---

## 3. Why there is a separate C++ process

`unitree_sdk2` and ROS 2 cannot be linked into one binary. Both bundle their own
CycloneDDS, and in one address space they corrupt the heap — the symptom is an
abort inside `dds_create_topic_impl` reading `corrupted size vs. prev_size`.

slam_ws hit this and solved it with `g1_state_server`: a standalone binary that
talks DDS and exposes a plain socket. This workspace needs the mirror image for
*commands*, so `g1_loco_server` is that, with the ZMQ direction reversed.

Two consequences worth knowing:

- The command path has a process boundary in it. That is not incidental
  complexity to be refactored away later; it is load-bearing.
- `g1_loco_server` must never grow a ROS dependency. If it ever needs to publish
  something, publish it from the Python side.

### The CycloneDDS trap

Three separate hazards, all of which produce the same unhelpful heap-corruption
abort. All three are handled; do not undo them.

**a. Library pinning.** The SDK's `libddscxx` was built against the `libddsc`
sitting next to it. colcon builds in a ROS-sourced shell, which puts
`/opt/ros/jazzy/lib/x86_64-linux-gnu` at the head of `LD_LIBRARY_PATH`; the
loader then pairs the SDK's `libddscxx` with ROS's `libddsc`. `CMakeLists.txt`
pins both with `BUILD_RPATH`/`INSTALL_RPATH` plus
`-Wl,--disable-new-dtags` — the flag matters, because it emits `DT_RPATH`
rather than `DT_RUNPATH`, and only `DT_RPATH` outranks `LD_LIBRARY_PATH`.
`setup.sh` checks it after every build:

```bash
ldd install/g1_loco_server/lib/g1_loco_server/g1_loco_server | grep ddsc
# both lines must point into /opt/unitree_robotics/lib
```

**b. ZMQ include paths.** Do not use `pkg_check_modules` for libzmq. It reports
`/usr/include/x86_64-linux-gnu` among its include dirs, which shadows
arch-specific system headers and silently changes struct layouts underneath the
DDS headers. Link `zmq` by bare name; `zmq.hpp` is already on the default path.

**c. Initialisation order.** Inside `g1_loco_server`, `ChannelFactory::Init`
must run before the ZMQ context is created, *and* the `LocoClient` must be
constructed after `ChannelFactory::Init`. The second one bit during
development: `LocoClient` as a value member is constructed in the initialiser
list, i.e. before the constructor body calls `Init`, and it segfaults before
printing anything. It is a `unique_ptr` created in the body for that reason.

---

## 4. Where the safety limits live, and why there

All of them are in `g1_loco_server`, the C++ process, not in the Python that
calls it. A bug or a typo in the ROS layer must not be able to command a 35 kg
humanoid to full speed.

| limit | default | what it protects against |
|---|---|---|
| `--max-vx` / `--max-vy` / `--max-omega` | 0.6 / 0.3 / 0.6 | a wrong number reaching the controller |
| `--max-duration` | 2.0 s | a command that outlives the process that sent it |
| `--watchdog` | 0.5 s | a client still connected but no longer sending |
| unknown FSM ids | rejected | posting an arbitrary integer to the controller |

**Layered on purpose.** There are three independent things that stop the robot
when the software above fails:

1. **The robot's own deadline.** Every `SetVelocity` carries a duration and the
   controller stops when it expires. This is the one that works even if the
   whole PC dies. `SwitchMoveMode(false)` is set explicitly so that continuous
   mode — `864000 s`, i.e. ten days — is unreachable.
2. **The server watchdog.** Covers the case the deadline does not: a client that
   is alive and connected but has stopped sending.
3. **The bridge's staleness check.** A `/cmd_vel` older than `input_timeout`
   counts as no command, so a dead publisher stops the robot within 0.5 s
   rather than at the next tick.

The path parser also has its own sanity ceilings (`MAX_SPEED`, `MAX_RATE`,
`MAX_SEGMENT_TIME`). Those exist to produce a readable error naming the
offending segment; the C++ clamps are the ones that actually hold.

**None of it replaces the remote.** All of it assumes the failure is in
software. A gait that goes unstable is not a software failure.

### The gate

`loco_bridge` drops every `/cmd_vel` until `~/enable` is called, and starts
closed. The reason is a specific failure: launch the stack while some other node
is already publishing `/cmd_vel`, and without a gate the robot walks before
anyone has looked at it. Any FSM service also closes the gate, because changing
FSM while a velocity stream is running means two things are commanding the robot
at once.

This does mean two service calls before the robot walks. That is the intended
cost.

---

## 5. Why the wire format is a packed struct

Following `G1StatePacket` in slam_ws. JSON would be more readable on the wire,
but the peer is a single Python client in this same repo — there is nobody to
negotiate a self-describing format with — and matching the existing project
convention is worth more than debuggability we can get another way.

The C++ has `static_assert`s on both struct sizes and the Python has matching
`assert`s on `struct.calcsize`. Change one without the other and something
fails loudly rather than producing a plausible-looking wrong command.

Debuggability comes from `loco_cli` (poke the server by hand, no ROS) and
`fake_loco_server` (run the whole stack with no robot) instead.

**REQ/REP, not PUB/SUB.** The state stream in slam_ws is PUB/SUB because a
dropped sample is fine — the next one is 5 ms away. A command is not like that:
one that silently vanished is worse than one that returns an error code. The
reply also carries the clamped values back, so the client can see when the
server overrode it.

---

## 6. Why the ramps are shaped the way they are

A humanoid told to step from 0 to 0.3 m/s in one control tick lurches. So every
segment eases in and out over `ramp` seconds.

The non-obvious part is that the ramp must not change the distance travelled. A
trapezoid with peak `v`, ramps `R` and plateau `P` covers `v * (P + R)`. Setting
`P = T_nominal - R` makes that exactly the distance the file asked for. Ramping
without that correction would undershoot every single segment — and open loop,
nothing would ever notice or correct it.

Segments shorter than one ramp collapse the ramp to the whole segment rather
than overrunning it, which keeps the area exact at the cost of a slightly
sharper start. `test_path.py` pins both cases.

**`settle` between segments** is standing still, and it is not padding. A
humanoid that finishes a 2 m straight is still oscillating; turning immediately
adds that oscillation to the turn error. 1.5 s before a turn is cheap compared
to the drift it prevents.

---

## 7. Why turn-in-place instead of arcs

`square_2m.yaml` walks straights and turns in place rather than driving rounded
corners. Two reasons: a humanoid holds a straight line considerably better than
a constant-radius curve, and a turn-then-go path has exactly one error source
per corner instead of one that accumulates continuously through it. Open loop,
fewer error sources is the whole game.

---

## 8. What `commanded_odom` is, and is not

`path_follower` publishes `~/commanded_odom` and a matching `odom → base_footprint`
TF. Both are the integral of our own output. They cannot see slip, drift, a
stumble, or the robot being physically blocked; if the robot is held in place it
will still report walking a clean 2 m.

They exist so RViz has something to draw and so a run can be compared against
its intent. The covariances are set large deliberately. **Nothing should ever
fuse this as a measurement**, and when real odometry arrives this should be
renamed or removed rather than quietly reused.

The same caveat applies to `~/planned_path`: it is the path as written, drawn
with the same integral the follower executes. Useful for catching an
off-by-a-turn path file before anything moves. Not a claim about the robot.

---

## 8b. Closed loop: what it measures and why it converges

The robot publishes its own state estimator on `rt/odommodestate` (and the
same values as `nav_msgs/Odometry` on `rt/dog_odom`). That is undocumented for
the G1 and was found by enumerating DDS discovery, after two guessed leads
produced a dead end and a false negative -- see section 11.

It is not LiDAR-derived, which matters because this project is deliberately
LiDAR-free: ~500 Hz (no LiDAR SLAM publishes pose that fast), `position.z`
tracks pelvis height, and it is the locomotion controller's own leg-plus-IMU
estimate. Validated against a tape measure: over one 7.5 s walk it reported
1.37 m forward and 0.52 m right, matching both the distance and the observed
drift, and it captured the backwards settle.

`g1_loco_server` republishes it on a PUB socket (port 5560, separate from the
REQ/REP command channel) and `odom_bridge` turns it into `/odom` plus TF.

**Two errors, two mechanisms.**

*Heading* is corrected continuously during a segment: hold the heading the
segment started with, biased toward the line by the cross-track error, both
contributions bounded so neither can fight the walk. This is what fixes the
20-35 deg rightward drift that no open-loop knob could touch.

*Distance* is corrected by measuring after the settle rather than by modelling
the robot. Walk until `/odom` says the target is reached, stop, settle, then
measure and top up if short.

**Why the naive version does not converge.** Every approach ends in a stop and
every stop costs a ~0.35 m backwards settle, so a correction gains the missing
distance and immediately gives it back. Observed against the simulator: three
corrections, byte-identical results, 2.67 m every time.

The fix is to aim *past* the target by the stopping distance -- and that
distance is observable, as the difference between where the robot was when
commanded to stop and where it came to rest. It is learned during the run
(logged as `settle cost +0.34 m`), clamped against a wild reading, and blended
so one odd segment cannot dominate. With it, `straight_3m` lands at 3.01 m and
the 2 m square closes to 2 cm.

**What it does not fix.** The odometry drifts and has no loop closure, so it
is good for one segment and not for a map. And a correction smaller than
`min_correction_distance` (0.15 m) is not attempted at all: below that the
gait stomps without translating, so the attempt would be theatre.

---

## 9. What comes next, and what will have to change

**Obstacle detection and avoidance** is the stated next step, and it is the
thing that breaks the open-loop assumption. Detecting an obstacle is additive —
a depth-camera node that pauses the follower is a small change and needs no
pose estimate at all. *Avoiding* one is not: leaving the path and rejoining it
means knowing where you are relative to it.

The likely order:

1. **Detect and stop.** Camera bridge from slam_ws + a depth threshold that
   calls `/path_follower/pause`. Fits the current architecture unchanged, and
   is worth having on its own for safety.
2. **Add a pose estimate.** RTAB-Map visual odometry from the D435i, or IMU yaw
   fused with commanded velocity as a cheaper intermediate. This is the real
   work.
3. **Close the loop.** Replace the time-based follower with pure pursuit on the
   measured pose. `path.py` keeps distances and angles rather than only
   velocities and times, so the same path files describe a geometric path once
   there is something to track it with — that was the point of storing them in
   metres and degrees.
4. **Then, and only then, local avoidance.**

The parts that should survive all of that: `g1_loco_server` and its limits, the
`/cmd_vel` gate, the path file format. The part that gets replaced is
`path_follower`'s timing loop.

---

## 10. Relationship to slam_ws

`~/workspaces/slam_ws` is a separate project (depth-only SLAM and navigation).
This workspace **copies** two things from it and shares nothing else:

- `g1_description` — the URDF and meshes, trimmed to data only.
- `g1_walk/state_bridge.py` — from `g1_perception/g1_state_bridge.py`, unchanged.

Copied rather than overlaid so that this workspace builds and runs on its own,
and so that nothing here can break slam_ws. The cost is that a fix to either
file has to be carried across by hand.

**Source one workspace at a time.** Both define `g1_description`; sourcing both
means one silently shadows the other.

---

## 11. Things that will bite

- **`ros2` CLI hangs.** Usually a wedged daemon, not your code:
  `ros2 daemon stop`, or add `--no-daemon`.
- **`status` returns 7301 "LocoState not available" but commands still work.**
  Seen on first contact with the robot, 2026-08-10. The three getters
  (`GetFsmId`, `GetFsmMode`, `GetBalanceMode`) all read `LocoState`, which the
  controller only publishes once it is actually running; the setters
  (`SetVelocity`, `SetFsmId`) do not. So `status` can fail while `stop`,
  `stand_up` and `start` all succeed.

  **Do not read a 7301 from `status` as "the robot is unreachable."** The
  discriminating test is `loco_cli stop` — it is `SetVelocity(0,0,0)`, touches
  no `LocoState`, and does nothing physically to a robot that is not walking:

  | `status` | `stop` | meaning |
  |---|---|---|
  | 7301 | ok | controller accepts commands, just not in main operation yet. Run `start`, then `status` should work. |
  | 7301 | 7301 / 3104 | the locomotion service really is not running. Fix it on the robot; nothing here will help. |
  | ok | ok | normal. |

- **The robot accepts every command and does nothing.** This one cost an
  afternoon on 2026-08-10, so it gets the full story.

  `LocoClient::Start()` in the SDK hardcodes `SetFsmId(500)`. On this robot,
  500 is **accepted and ignored**: the call returns 0, the FSM stays at 4
  (`stand_up`), and because the robot is not in main operation every subsequent
  `SetVelocity` is silently discarded. Nothing anywhere reports an error. The
  robot just stands there.

  The state that actually enables walking is **FSM 200**, `main_operation`.

  ```bash
  ros2 run g1_walk loco_cli main_operation
  ros2 run g1_walk loco_cli status          # must show fsm_id 200
  ```

  **The general lesson, which is bigger than this one id: a return code of 0
  from `SetFsmId` means "accepted", not "changed".** Always confirm with
  `status`. The server now prints `accepted != changed -- confirm with
  loco_cli status` after every FSM command for exactly this reason.

  If an id outside the known table needs trying, start the server with
  `--allow-any-fsm` and use `loco_cli fsm_id <n>`. It prompts first.

- **The robot covers about half the commanded distance, repeatably.**
  Measured 2026-08-12, eight runs at 0.4 m/s for 7.5 s (commanded 3.0 m),
  rampless, with the bridge verified delivering full speed at 17-20 Hz and no
  clamping:

  | command window | forward distance |
  |---|---|
  | 0.15 s | 1.5, 1.5, 1.5, 1.6 m |
  | 0.5 s | 1.65, 1.5, 1.5, 1.5 m |

  Mean 1.53 m either way -- so the command window makes no difference, and the
  factor is a repeatable **0.51x**. That is exactly what `linear_scale` is for:
  `linear_scale:=2.0`.

  An early single run covering 4.2 m of a commanded 4.0 was an outlier and sent
  four hypotheses (clamping, command rate, ramps, command window) down blind
  alleys before eight repeats contradicted it. **One measurement of this robot
  is not a measurement.** `loco_cli move --repeat 4` exists for that reason.

- **Run with the handheld remote powered OFF.** It was the source of all the
  run-to-run variability, and it is invisible to every diagnostic here.

  | | forward | backward |
  |---|---|---|
  | remote on (8 runs) | 1.5-1.65 m | 0, 0, 0, 0.5, 0.7, 0.7, 1.0, 1.0 m |
  | remote off (4 runs) | 1.4-1.5 m | 0.3, 0.4, 0.4, 0.3 m |

  Forward is unaffected; the scatter in the retreat collapses from 1.0 m to
  0.1 m. That is the difference between "open loop cannot fix this" and "open
  loop can fix this", so it is not an optional precaution.

- **Two different errors, two different knobs.** With the remote off the robot
  reliably covers about half the commanded distance *and* settles ~0.35 m
  backwards once a segment ends. These do not have the same shape:

  | error | scales with | knob |
  |---|---|---|
  | covers 0.2 m/s of a commanded 0.4 | segment length | `linear_scale` (2.0) |
  | rolls back ~0.35 m after stopping | nothing -- fixed per segment | `distance_offset` (0.35) |

  Correcting the fixed one with the proportional knob would be right for
  exactly one segment length and wrong for every other, which is why they are
  separate. `distance_offset` is in REAL metres -- the number you measure with
  a tape -- and is added to the target *before* `linear_scale` is applied, so
  it survives the efficiency factor. Commanded distance is
  `(target + offset) * scale`; with 3.0, 0.35 and 2.0 that is 6.7 m commanded,
  ~3.35 m covered, 0.35 m given back, 3.0 m net.

- **A consistent heading error is still open. The retreat leans right by about
  30 degrees every run.** `yaw_trim` corrects a veer *during* the walk; nothing
  yet corrects a yaw that accrues during the settle. Measure which phase it
  happens in before reaching for a knob.

- **Long ramps LOSE ground on this robot.** The asymmetric-ramp reasoning
  below is sound for a machine that tracks the velocity it is given. This one
  does not below about 0.4 m/s (`MIN_EFFECTIVE_SPEED` in `path.py`), and a
  velocity ramp spends its entire duration under the target speed. The
  trapezoid's area credits that time as distance; the robot covers little or
  none of it.

  Measured 2026-08-11, all at 0.4 m/s, with the bridge verified delivering
  20 Hz at full commanded speed and no clamping:

  | profile | dead time below 0.4 m/s | commanded | actual forward |
  |---|---|---|---|
  | none (constant velocity) | 0.0 s | 4.0 m | 4.2 m |
  | ramp 0.5 / ramp_down 1.5 | 2.0 s | 3.0 m | 1.5 m |
  | ramp 0.5 / ramp_down 2.5 | 3.0 s | 3.0 m | 1.3 m |

  More ramp, less distance, monotonically. The backward lurch also got *worse*
  with the longer stop ramp, which kills the momentum explanation the ramps
  were built on.

  Honest caveat: dead time alone does not fully account for the numbers -- it
  predicts about 2.5 m where 1.5 m was measured -- so something else is
  contributing too. What is established is the direction: on this robot, ramps
  cost ground. `check_path` now reports a path's dead time, and `ramp:=0
  ramp_down:=0` are available on the launch to test a pure step profile.

- **Ramps are asymmetric, and that is deliberate.** Accelerating a humanoid is
  easy; it just starts stepping. Decelerating is not -- at 0.4 m/s there is
  real momentum, and if the velocity command decays faster than the robot can
  shed it, the balance controller arrests the rest by stepping, backwards and
  off-heading. So `ramp` (in) defaults to 0.5 s and `ramp_down` (out) to 1.5 s.

  Both preserve the distance integral: the trapezoid covers
  `v * (plateau + (ramp_up + ramp_down)/2)`, and `plateau` is solved so that
  equals the distance the file asked for. Lengthening `ramp_down` therefore
  makes the stop gentler *without* changing where the robot ends up, which is
  what makes it safe to tune. Tune it from the command line with
  `ramp_down:=2.5` rather than editing path files.

- **The robot stomps and barely moves.** There is a minimum speed below which
  this G1's gait does not translate. Measured 2026-08-11: a commanded 3 m at
  0.3 m/s covered about 1 m, while 0.4 m/s for 10 s covered 4.2 m of a
  commanded 4.0 -- within 5%, essentially calibrated. The shipped path files
  and `DEFAULTS` in `path.py` now use 0.4 for that reason. Do not lower it
  without re-measuring; a slow path is not a cautious one here, it is a broken
  one. If a 3x `linear_scale` seems necessary, the speed is the problem.

- **The robot drifts backwards and off-heading after a move ends.** Not drift
  during the walk -- recovery after it. Cutting a humanoid from 0.4 m/s to zero
  in one control tick leaves real momentum, and the balance controller arrests
  it by stepping, which is motion nobody commanded (~0.5 m back and 20 deg off
  after an unramped stop).

  `path_follower` always ramped for this reason. `loco_cli move` did not, which
  also meant a distance measured with the CLI did not transfer to a path run.
  It now builds a `path.Segment` -- literally the same class and the same
  trapezoid -- and holds commanded zeros through a settle period afterwards, so
  the robot is told to stand still rather than merely hearing nothing.

- **The robot marches in place and nothing stops it.** Balance mode 1
  (`continuous_gait`) is a persistent mode, not a command: the controller keeps
  stepping so it is always ready to move. Every stop this project had was
  `SetVelocity(0,0,0)`, which zeroes the velocity and leaves the mode alone --
  and a marching robot at zero velocity keeps marching. Observed 2026-08-11:
  the robot marched until the mode was changed by hand, and killing
  g1_loco_server made no difference, because its shutdown ran the same
  StopMove.

  Fixed by splitting the two ideas:

  | | does | used by |
  |---|---|---|
  | `stop` | zeroes velocity only | the follower, between path segments |
  | `halt` | zeroes velocity **and** sets balance mode 0 | operators, watchdog, shutdown |

  `stop` stays velocity-only on purpose: it runs at every segment boundary, and
  dropping out of continuous gait there would change the gait mid-path. The
  watchdog and the exit path now halt rather than stop, because losing the
  commander while the robot is marching is exactly the case that needs it.

  **The general lesson: a mode is not a command.** The whole safety design --
  per-command durations, watchdog, stop-on-exit -- assumed everything the robot
  does is driven by a command stream that decays when the stream stops. Modes
  do not decay. Anything else mode-like added later needs the same treatment.

- **The FSM will not leave zero_torque (0).** From zero torque the robot is
  limp and has no holding force; the controller accepts `stand_up` and stays
  put. `damp` first, which takes up the joints, then `stand_up`. Support the
  robot before either -- in zero torque it holds nothing. `loco_cli` now prints
  this when a transition stalls at 0 or 3, rather than leaving you to guess.

- **Two `walk.launch.py` at once.** ROS 2 allows duplicate node names, so you
  get two bridges commanding one robot and two followers publishing `/cmd_vel`;
  a service call lands on whichever answers first. Seen 2026-08-11: a
  `nudge.yaml` launch and a `straight_3m.yaml` launch running together, and the
  path that executed was not the one in the operator's front terminal.

  `loco_bridge` and `path_follower` now claim an abstract AF_UNIX lock at
  startup and refuse to run twice. Abstract sockets vanish with the process, so
  there is no stale lock file to clean up after a kill -9.

- **The FSM ends up in a state that is in no table (801 seen).** Sending two
  FSM commands back to back does it. `SetFsmId` returns on *acceptance*, not on
  completion, so `stand_up` immediately followed by `main_operation` puts the
  second transition into the middle of the first one's motion, and the
  controller lands somewhere unintended. Observed 2026-08-11: fsm_id 801, with
  `balance_mode` unreadable (a reliable tell that the state is not a ready one
  -- at 200 it reads 0).

  `loco_cli` now polls after every FSM command until the state actually
  arrives, up to 12 s, so transitions cannot be chained too fast by accident.
  It prints `main_operation: ok (fsm_id 200)` only when the robot really got
  there, and says `ACCEPTED BUT NOT REACHED` otherwise.

  To recover: let the robot settle and re-send the transition. If it is stuck
  in an unrecognised state, support the robot and walk back through known ones
  -- `damp`, then `stand_up`, then `main_operation`, one at a time, waiting for
  each to report ok.

- **`status` returns 7301 for some fields but not others.** Normal, and
  informative. `GetBalanceMode` is unavailable until the robot reaches main
  operation, while `GetFsmId` and `GetFsmMode` work earlier. Once `fsm_id` is
  200 all three report. Getters were briefly suspected of being unsupported in
  firmware; they are not.

- **Velocity commands return an error code.** The robot is probably not in main
  operation FSM. `ros2 run g1_walk loco_cli status`, then `stand_up`, then
  `main_operation`.

- **A command times out but the robot keeps walking / a walk aborts on one slow
  reply.** The robot occasionally takes longer than the fast timeout to
  acknowledge a velocity command. That is a hiccup, not a fault: `loco_cli move`
  tolerates up to 5 consecutive slow replies before giving up, and the bridge
  logs and continues. Every velocity command carries its own expiry, so a gap in
  the stream makes the robot pause rather than run on.
- **The robot does not move and nothing logs an error.** The gate is closed.
  It starts closed every time.
- **`cannot bind tcp://127.0.0.1:5558`.** A second `g1_loco_server`, or a
  `fake_loco_server` left running. `ss -ltnp | grep 5558`.
- **Distances are consistently wrong.** That is expected; that is what
  `linear_scale` is for. Consistently wrong is the good case.
- **Distances are inconsistently wrong.** Check the floor surface and the
  battery. Open loop has no answer for this one.
