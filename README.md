# waypoint_follow_ws — G1 fixed-path walking

Walk a Unitree G1 along a path written in a YAML file. **Open loop**: nothing
measures where the robot is, so errors accumulate and are never corrected.

This is not autonomous navigation and is not trying to be. There is no map, no
planner, no localisation. Obstacle detection and avoidance come later; this
workspace is the layer they will sit on top of.

**For why anything here is the way it is, read [`INSTRUCTIONS.md`](INSTRUCTIONS.md).**
This file is just how to run things.

## Environment

| | |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| unitree_sdk2 | installed to `/opt/unitree_robotics` |
| Robot | `192.168.123.164` over `eno1` (`192.168.123.1/24`), key-based SSH |

Related but independent: `~/workspaces/slam_ws` is the depth-camera SLAM
project. This workspace copies its URDF and its state bridge and otherwise
shares nothing with it. **Source one workspace at a time** — both define
`g1_description`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
./setup.sh          # checks ROS, the SDK and cppzmq, then colcon build
```

`setup.sh` also verifies the one build detail that fails expensively — see
"The CycloneDDS trap" in `INSTRUCTIONS.md`. Rebuilding a single package:

```bash
colcon build --packages-select g1_walk
```

## Packages

| package | build | what it is |
|---|---|---|
| `g1_loco_server` | plain CMake | Standalone C++ binary. The **only** thing that can move the robot. ZMQ in, Unitree DDS out. Links no ROS. |
| `g1_walk` | ament_python | Everything ROS: the `/cmd_vel` bridge, the path follower, the path files, the tools. |
| `g1_description` | ament_cmake | URDF and meshes, copied from slam_ws. Data only. |

```
path_follower ──/cmd_vel──▶ loco_bridge ──ZMQ REP──▶ g1_loco_server ──DDS──▶ robot
   (path.yaml)                (the gate)               (the limits)
```

## Try it with no robot

The whole stack runs at a desk against a fake server that speaks the same
protocol and integrates the velocities it is given.

```bash
source install/setup.bash

# check a path file first -- prints every segment and the commanded end pose
ros2 run g1_walk check_path square_2m.yaml

# terminal 1
ros2 run g1_walk fake_loco_server

# terminal 2
ros2 launch g1_walk walk.launch.py path:=square_2m.yaml

# terminal 3
ros2 service call /g1_loco_bridge/enable std_srvs/srv/Trigger
ros2 service call /path_follower/start  std_srvs/srv/Trigger
```

The fake server prints the pose it would have reached. For a square that should
come back to `0.00, 0.00` at `360 deg` — on paper. On a floor it will not.

## Running on the real robot — the runbook

Follow these in order. Every step says what to expect, so you can tell a
success from a silent no-op. Keep the remote in hand throughout.

**Two interfaces, and mixing them up is the most common confusion:**

| | needs | use it for |
|---|---|---|
| `ros2 run g1_walk loco_cli ...` | only `g1_loco_server` | FSM, status, manual nudges |
| `ros2 service call /g1_loco_bridge/...` | `walk.launch.py` running too | path following |

`/g1_loco_bridge/*` services do not exist until the launch is up. If a service
call sits at `waiting for service to become available...`, the launch is not
running — that is the whole explanation.

Also: `ros2 service call` always needs the **type**. It is
`std_srvs/srv/Trigger`, and omitting it is an argparse error, not a robot
problem.

### 0. Preflight

```bash
source install/setup.bash
ros2 run g1_walk preflight
```

Checks the link, the route, the robot, the server and the FSM in dependency
order, and tells you the exact command to fix the first thing that is wrong.
Run it whenever something does not behave — it catches most of it.

### 1. Terminal 1 — the command server

```bash
source install/setup.bash
ros2 run g1_loco_server g1_loco_server --iface=eno1
```

Expect: `Listening on tcp://127.0.0.1:5558` and the limits line. Leave it
running. **Nothing moves without this process.**

### 2. Terminal 2 — stand up and enter main operation

```bash
source install/setup.bash
ros2 run g1_walk loco_cli status           # where are we?
ros2 run g1_walk loco_cli damp             # required from fsm_id 0 (zero_torque)
ros2 run g1_walk loco_cli stand_up         # skip if already standing
ros2 run g1_walk loco_cli main_operation   # FSM 200
```

**One at a time.** Each FSM command now waits until the state actually arrives
and prints e.g. `main_operation: ok (fsm_id 200)`. Do not fire the next one
until you see that — `SetFsmId` returns on acceptance, not completion, and
chaining them lands the robot in an undefined state (801 has been seen).
If you get `ACCEPTED BUT NOT REACHED`, let it settle and re-send.

**`fsm_id 200` is the gate to everything else.** At `fsm_id 4` the robot
accepts velocity commands and silently ignores them. A `0` return code from an
FSM command means "accepted", not "changed" — always confirm with `status`.

Do not use `loco_cli start`: that is FSM 500, the SDK's name, and this robot
ignores it. See INSTRUCTIONS.md §11.

### 3. First motion, no ROS

```bash
ros2 run g1_walk loco_cli move --vx 0.2 --seconds 3
```

Type `go`. The robot walks ~0.5 m and stops itself. If this works, everything
below ROS is proven. Occasional `slow reply (n/5)` lines are tolerated hiccups,
not failures.

### 4. Terminal 3 — the ROS stack

**Exactly one of these at a time.** One terminal per path file is a tempting
habit and a dangerous one: both launches publish `/cmd_vel` and answer the same
service names, so you can start a path you are not watching. The nodes now
refuse to start twice, but change the `path:=` argument and relaunch rather
than opening a second terminal.

```bash
source install/setup.bash
ros2 launch g1_walk walk.launch.py path:=nudge.yaml
```

Expect `Gate is CLOSED` and `idle`. Launching never moves the robot.

Optional, for RViz and logging — terminal 4, plus `state:=true` on the launch:
```bash
~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1
```

### 5. Walk the path

Back in terminal 2. **Order matters:** any FSM service closes the gate, so
`main_operation` must come before `enable`.

```bash
ros2 service call /g1_loco_bridge/main_operation std_srvs/srv/Trigger
ros2 service call /g1_loco_bridge/enable         std_srvs/srv/Trigger
ros2 service call /path_follower/start           std_srvs/srv/Trigger
```

Expect, for `nudge.yaml`: 2 s pause, 0.5 m forward, 2 s settle.

### 6. Closed loop (recommended) — or calibrate open loop

The robot publishes its own state estimate, and `walk.launch.py` turns it into
`/odom` by default. With `feedback:=true` each segment runs until the robot has
**measured** its arrival, holding heading as it goes:

```bash
ros2 launch g1_walk walk.launch.py path:=straight_3m.yaml feedback:=true
```

Leave `linear_scale` and `distance_offset` at their defaults — the loop
measures what they were guessing, and setting both double-corrects. The
follower warns if you do.

What it fixes, verified against a simulated robot reproducing this one's
measured misbehaviour (50% of commanded speed, 0.35 m backwards settle,
rightward veer):

| | open loop | closed loop |
|---|---|---|
| `straight_3m` | 1.5 m of 3.0 | **3.01 m**, heading held to 0.1° |
| `square_2m` | does not close | **closes to 2 cm, 1.4°** |

It also learns the robot's stopping distance during the run and aims past the
target by it — printed as `settle cost +0.34 m`. Without that a correction
gains the missing distance and gives it straight back at the next stop, which
is exactly what the first version did.

The loop refuses to start without fresh `/odom`, holds position if odometry
goes stale mid-path, and aborts any segment that runs `timeout_factor` (3x)
over its nominal time.

### 6b. Calibrating open loop instead

**Power the handheld remote OFF.** It is what made earlier measurements
scatter — the retreat after a move varied 0 to 1.0 m with it on, and 0.3 to
0.4 m with it off. None of the diagnostics here can see it.

**Check the gait first.** If the robot stomps heavily and covers far less
ground than commanded, it is in balance-stand mode, taking a step and
re-stabilising rather than walking. No amount of calibration fixes that:

```bash
ros2 run g1_walk loco_cli continuous_gait   # balance mode 1
ros2 run g1_walk loco_cli status            # confirm balance_mode 1
```

**Continuous gait makes the robot march in place until halted.** That is the
mode working as intended — it stays ready to step — but it means the robot is
never still while it is on, including during a path's settle periods. Stop it
with `loco_cli halt`; nothing else will.

Then measure. Tape at the start, run `straight_3m.yaml`, tape at the end:

```bash
ros2 launch g1_walk walk.launch.py path:=straight_3m.yaml
```

Three knobs, all launch arguments, all measured the same way — walk, measure,
divide:

| knob | fixes | how to get it |
|---|---|---|
| `linear_scale` | travels the wrong distance | `3.0 ÷ measured metres`; measured ~2.0 on this robot |
| `angular_scale` | turns the wrong angle | `360 ÷ measured degrees` over four 90° turns |
| `distance_offset` | rolls backwards after a segment ends | measure the roll-back in metres; ~0.35 on this robot |
| `yaw_trim` | veers while walking straight | `radians(heading error) ÷ seconds of motion`, positive steers left |
| `ramp_down` | steps backwards when a segment ends | but see below — on this robot longer ramps made it *worse* |

```bash
ros2 launch g1_walk walk.launch.py path:=straight_3m.yaml \
    ramp:=0 ramp_down:=0 linear_scale:=2.0 distance_offset:=0.35
```

`linear_scale` and `distance_offset` fix different shapes of error and are not
interchangeable: the first is proportional to segment length, the second is a
fixed amount lost at every segment end. Commanded distance is
`(target + distance_offset) × linear_scale`.

`yaw_trim` is applied only on straight segments — never on commanded turns,
which would corrupt the path's angles, and never while settling.

**Measure more than once.** This robot's forward distance is repeatable but
its post-move drift is not, and an early outlier cost a lot of debugging. Use
`ros2 run g1_walk loco_cli move --vx 0.4 --seconds 7.5 --repeat 4` and take the
mean. `move` also refuses to run unless `fsm_id` is 200, so a run cannot
silently produce nothing.

**Ramps cost ground on this robot.** Below ~0.4 m/s its gait barely
translates, and a velocity ramp spends its whole duration under the target
speed — time the distance maths credits but the robot does not cover. Measured
at 0.4 m/s: constant velocity tracked to within 5%, a 1.5 s stop ramp lost
half the distance, a 2.5 s ramp lost more. Run `check_path` to see a path's
dead time, and try a step profile:

```bash
ros2 launch g1_walk walk.launch.py path:=straight_3m.yaml ramp:=0 ramp_down:=0
```

**A `linear_scale` above about 2 is a warning sign, not a calibration.** The
follower logs one if you set it. It means the robot is achieving less than half
its commanded speed, which is a gait problem to fix rather than scale around.

Only once a `straight_3m` run lands near 3 m and straight is `square_2m.yaml`
worth running.

### Stopping

```bash
ros2 run g1_walk loco_cli halt                                # THE stop button
ros2 service call /g1_loco_bridge/stop std_srvs/srv/Trigger   # gate shut + halt
ros2 run g1_walk loco_cli damp                                # limp -- IT WILL SAG
```

**`halt`, not `stop`.** `stop` only zeroes the velocity, and a robot in
continuous gait (balance mode 1) marches in place at zero velocity — so `stop`
does not stop it, and neither does killing these processes. `halt` zeroes the
velocity *and* leaves continuous gait. `stop` exists because the follower uses
it between path segments, where changing the gait would be wrong.

`damp` on a standing robot makes it collapse. Support it first.

Ctrl-C on any process also stops the robot, and if one dies outright the robot
halts when the current command's duration expires. Neither is a stop button.

### When something does not work

| symptom | cause |
|---|---|
| `waiting for service to become available...` | `walk.launch.py` is not running |
| `error: the following arguments are required: service_type` | add `std_srvs/srv/Trigger` |
| commands return `ok`, robot does not move | `fsm_id` is not 200 — see step 2 |
| `DDS cannot transmit` | link down; `ros2 run g1_walk preflight` |
| `7301 LocoState not available` | not in main operation yet; harmless before step 2 |
| `ros2` CLI hangs | stale daemon: `ros2 daemon stop` |
| robot walks nothing, gate log says closed | you called an FSM service after `enable` |
| `another g1_loco_bridge is already running` | a second `walk.launch.py` — Ctrl-C the other one |
| `stand_up` accepted but `fsm_id` stays 0 | zero_torque; send `damp` first, supporting the robot |
| colcon warns `g1_description` is in an underlay | slam_ws is sourced — use a fresh terminal |
| the wrong path ran | you had two launches up; only ever run one |
| `g1_state_server: Address already in use` | one is already running; `pgrep -af g1_state_server` |
| robot marches in place and will not stop | continuous gait; `loco_cli halt` (`stop` will not do it) |
| stomps heavily, covers far less than commanded | speed below ~0.4 m/s — raise `linear_speed`, do not scale |
| drifts back / off-heading *after* a move ends | momentum; raise `ramp_down:=2.5` (launch) or `--ramp-down` (CLI) |
| distance differs between `loco_cli move` and a path | compare the bridge's `commanding: N Hz` line against 20 Hz |

## Path files

`src/g1_walk/config/paths/`:

| file | what it is for |
|---|---|
| `nudge.yaml` | 0.5 m forward. The first thing to run on the real robot — tests plumbing, not paths. |
| `straight_3m.yaml` | The calibration path. Run it, measure, set `linear_scale`. |
| `square_2m.yaml` | 2 m square. The honest demonstration of how far open loop gets you. |

Format, in metres and degrees:

```yaml
name: my path
defaults:
  linear_speed: 0.4       # m/s -- below ~0.4 this robot stomps without moving
  angular_speed: 0.5      # rad/s
  settle: 1.0             # s of standing still after each segment
  ramp: 0.5               # s to ease IN
  ramp_down: 1.5          # s to ease OUT -- longer, stopping is the hard part
segments:
  - {move: forward, distance: 2.0}     # negative = backward
  - {move: turn,    angle: 90}         # degrees, positive = left
  - {move: strafe,  distance: 0.5}     # positive = left
  - {move: wait,    duration: 2.0}
  - {move: raw, vx: 0.2, vy: 0.0, omega: 0.1, duration: 3.0}
```

Any segment can override `speed`/`rate`, `settle` and `ramp`. Unknown keys are
a hard error — validate with `check_path` before going near the robot.

## Calibration

The only defence against drift. Walk `straight_3m.yaml`, measure with a tape:

```bash
# robot covered 2.7 m of a commanded 3.0
ros2 launch g1_walk walk.launch.py path:=square_2m.yaml linear_scale:=1.11
```

Same for turns: walk four 90-degree turns, measure the total, set
`angular_scale`. Redo it after changing speed, stand height, swing height or
floor surface. The numbers are not portable between any of those.

## Commands worth remembering

```bash
ros2 run g1_walk preflight                             # check every precondition
ros2 run g1_walk check_path <file>                     # validate + predict
ros2 run g1_walk loco_cli status                       # FSM state (confirm 200)
ros2 run g1_walk loco_cli move --vx 0.2 --seconds 3    # calibration nudge
ros2 service call /path_follower/pause std_srvs/srv/Trigger
ros2 topic echo /path_follower/status
```

`ros2 launch g1_walk walk.launch.py rviz:=true` shows the robot model, the
planned path (green) and the commanded pose trail (orange). Both are nominal:
RViz is drawing what was commanded, not where the robot is.

## Tests

```bash
colcon test --packages-select g1_walk && colcon test-result --verbose
```

Covers the path parser's rejection cases, the ramp's distance integral, and a
round trip of the command protocol against a stub server.
