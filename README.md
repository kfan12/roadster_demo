# vlm_av_ws

A ROS2 + Gazebo autonomous-driving simulation stack: vision-based lane perception, a curvature-aware local planner, and an OSQP QP model-predictive controller, running a full-size sedan through a generated urban road course.
A second pipeline layers a vision-language model on top: it reads road signs (stop / left / right / winding) from the front camera and turns them into speed caps and a stop, on top of the vision-only lane following.

This project is a continuation of [kfan12/vlm_robot_public](https://github.com/kfan12/vlm_robot_public).
It carries forward the same ROS2/Gazebo simulation approach, now built around a full-size sedan model (Ackermann steering, RGB-D front camera) instead of the original robot, and a more realistic generated road world with lane markings, curves, intersections, and street furniture.

**Status: under active development.**
The core stack runs end to end (perception -> planning -> control -> sim), and the VLM sign-reading pipeline (Qwen2.5-VL-3B) runs alongside it and drives the demo course (straight -> right 90 -> winding S -> left 90 -> STOP).
Tuning and new features are ongoing; expect frequent changes to camera/perception parameters, MPC weights, VLM sign-arbitration tuning, and the road-world generator as the project evolves.

## Roadster stunt demo: the inverted ramp

This branch adds a simulation of the inverted-ramp stunt from Tesla's 2026-10-15 Roadster unveil.
The car drives up a curved ramp past vertical, held on by suction fans, stops upside down at the top and holds there, then drives off the overhang still upside down, kick-flips 180 degrees in the air on cold-gas thrusters, brakes its fall with them, and lands on its wheels.
It is a plausible engineering reproduction built on public reporting, not a replica of Tesla's hardware or controller: the nozzle layout, fan force, mass and ramp size are assumptions (see `docs/plans/roadster_demo_plan.md` for sources, confidence and the full plan).

| Drive-off, upside down (trackside camera) | Kick flip in flight | Landing burn | Mid-flip (onboard camera) |
| --- | --- | --- | --- |
| ![Drive-off](docs/snapshots/roadster_driveoff_trackside.png) | ![Flip](docs/snapshots/roadster_flip_trackside.png) | ![Landing](docs/snapshots/roadster_landing_trackside.png) | ![Onboard](docs/snapshots/roadster_flip_onboard.png) |

### Sequence

1. **Run-in:** accelerate to 6 m/s; the suction fans spin up about 6 m before the ramp.
2. **Climb:** round an 8 m radius loop at 6 m/s, pressed onto the track by the fans.
3. **Stop and hold:** brake on the upper arc and stop just past the top, upside down on a 5 m overhang, and stand still for 1 s on fan suction alone (parking brake on the wheels).
4. **Drive-off:** from standstill, still upside down, accelerate forward with the wheels plus the two rear thrusters and let go of the overhang at about 7 m/s.
5. **Kick flip:** roll 180 degrees about the car's long axis in about 0.8 s while flying forward.
6. **Landing burn:** fire the down-facing thrusters on a constant-deceleration profile to touch down at about 2-3 m/s, then roll out and stop.

### Hardware enablers (as modelled)

- **Cold-gas thrusters:** 10 nozzles fed from a 600 L carbon-overwrapped tank at 400 bar (about 250 kg of air), regulated to 100 bar at the nozzles, with first-order valve lag (20 ms) and a minimum on-time.
  Two rear main nozzles (2 x 2.25 m g, the launch), four down-facing corner nozzles (4 x 1.1 m g, lift and landing) and four roof-corner nozzles pushing down (4 x 0.45 m g).
  A down pair on one side with the roof pair on the other makes a pure roll couple with no net force, which is what lets the car flip without being pushed sideways.
  The tank depletes with use, so gas is the binding budget: the demo uses about 170 of the 250 kg.
- **Suction fans:** up to 1.5 m g of downforce along the body's floor normal, with spin-up lag and a ground-effect falloff with the gap to the track (computed from the track's analytic geometry), enough to hold the car upside down at standstill.
- **Suspension:** a preloaded spring-damper per corner (250 kN/m, damping ratio 0.5, plus or minus 12 cm), sized for the 4-5 g load at the bottom of the loop.
  Without it the rigid car can rest on one diagonal, leaving only two driven wheels.
- **Sensors:** a 1 kHz IMU (sampled every physics step, so the rigid-wheel contact impulses on the ramp are integrated instead of aliased), wheel encoders, force/torque sensors on the four wheel joints (wheel load and contact), and a trackside plus an optional onboard camera.
- **Drive:** torque-controlled wheels on all four corners and position-controlled front steering, through `ros2_control`.

All stunt forces (tank, nozzles, fan, suspension springs) come from a single Gazebo system plugin (`roadster_gz_plugins`), so there is exactly one writer of the car's external forces.

### Control method

- **Mission state machine** (`roadster_control/stunt_mission_node`): run-in, climb, stop, inverted hold, drive-off, release, flip, descent, landing brake, landed, rollout, stopped, with an abort path.
- **Ground phase:** a speed PI with gravity feed-forward along the track and per-wheel traction control (wheel torque capped at k mu N r from the measured wheel load).
  Progress is measured as distance along the whole track (run-in, arc, overhang), the stop onto the hold point is a planned constant-deceleration trajectory, and the hold itself simply stands still.
  Steering is held straight: the track is straight in one vertical plane, so with no disturbance there is nothing to correct.
- **Flight phase:** a quaternion attitude controller (PD on the rotation-vector error with reference rate and acceleration feed-forward and gyroscopic compensation) tracks a minimum-jerk 180 degree roll.
  A thrust allocator solves a small QP with OSQP for the 10 valve openings, weighting forces in world axes (lift up, no sideways push whatever the attitude) and torques in body axes.
- **Landing:** a constant-deceleration velocity profile (1.5 g down to a 1.5 m/s touchdown target) latches on once the car falls faster than it, with the deceleration feed-forward only while descending; touchdown is detected from wheel contact, not height.
- **State estimation** (`roadster_control/state_estimator_node`): an IMU strapdown integration in a 15-state error-state EKF (position, velocity, attitude, accelerometer and gyro biases).
  It is corrected only by what the car can sense itself: wheel speed (encoders plus gyro pitch rate) with no side-slip and no hop, each loaded wheel centre one radius from the known track surface, and zero-velocity, gravity and zero-rate updates when standing still.
  In flight it runs on the IMU alone.
  Wheel-velocity updates are innovation-gated with fault recovery.
  Over a whole run the estimate stays within about 5 cm and 0.3 degrees of Gazebo's ground truth, and the mission flies on it by default.

### Run it

```bash
colcon build --symlink-install
source install/setup.bash

# Gazebo + RViz + the mission; the world starts paused and unpauses after ~14 s
ros2 launch roadster_control demo_a.launch.py

ros2 launch roadster_control demo_a.launch.py state:=truth          # fly on Gazebo ground truth instead of the IMU estimate
ros2 launch roadster_control demo_a.launch.py gui:=false            # RViz only, no Gazebo window
ros2 launch roadster_control demo_a.launch.py onboard_camera:=false # skip the car's camera (saves render time)
ros2 launch roadster_control demo_a.launch.py release_after_hold:=false   # climb and stay upside down
```

To launch the pieces by hand (for example to restart only the mission):

```bash
# 1. the simulation: Gazebo world + roadster + bridges + ros2_control (paused until unpaused)
ros2 launch roadster_description roadster_sim.launch.py world:=roadster_ramp paused:=true gui:=true
# 2. the IMU state estimator + the stunt mission
ros2 launch roadster_control stunt_mission.launch.py mode:=ramp world:=roadster_ramp state:=imu
# 3. unpause the world
ign service -s /world/roadster_ramp/control --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'
```

A recorded and scored run (writes `data/stunt_runs/<session>/run.jsonl` and `metrics.json`):

```bash
scripts/run_stunt_scenario.sh roadster_ramp ramp                              # mission on ground truth
MISSION_ARGS="state:=imu" scripts/run_stunt_scenario.sh roadster_ramp ramp    # mission on the IMU estimate
```

The stunt worlds are generated, with their geometry recorded in `maps/<world>.json` (the single source the fan, estimator and mission read):

```bash
python3 scripts/generate_stunt_world.py --kind ramp --radius 8              # roadster_ramp (5 m overhang)
python3 scripts/generate_stunt_world.py --kind ceiling --height 16 --out roadster_drop16
```

In RViz the car carries orange thrust arrows (one per nozzle, length proportional to thrust), white exhaust puffs and blue fan exhaust trailing in the air, a fan disc that glows with fan speed, IMU arrows (specific force, angular rate), control arrows (requested and allocated thrust, requested torque, per-wheel drive and brake torque), the true and estimated paths, mission, control and IMU panels, and the trackside and onboard camera views.
Gazebo shows the same thrust and fan plumes as particle effects.

### Result

IMU-flown run (`state:=imu`): stands upside down at the top for 1.0 s without moving, all four wheels loaded from run-in to drive-off, release at 7.2 m/s, 180.0 degree flip, touchdown at 2.8 m/s vertical with 0.14 degrees tilt and a peak near 5.6 g, then a rollout to a stop, with no body contact; the whole stunt takes 18 s of sim time.
The simulation is deterministic apart from sensor noise, so repeated runs show repeatability, not margin; robustness testing (noise, wind, parameter sweeps) is still to come.

### Roadster packages

| Package | Role |
| --- | --- |
| `roadster_description` | Roadster URDF derived from the sedan: torque-driven wheels, suspension, nozzle frames, IMU, wheel force/torque sensors, particle effects; sim launch, bridge and test worlds. |
| `roadster_gz_plugins` | The actuation system: compressed-air tank, 10 cold-gas nozzles, suction fan, suspension springs and effect switching, plus unit-tested models. |
| `roadster_control` | Stunt mission node, IMU state estimator, OSQP thrust allocator, attitude and landing control, RViz visualization, demo launch. |

## Screenshots

| Gazebo simulation | RViz (lane debug image + planned path) |
| --- | --- |
| ![Gazebo](docs/snapshots/gazebo.png) | ![RViz](docs/snapshots/rviz.png) |

## Stack overview

| Package | Role |
| --- | --- |
| `sedan_description` | V2 full-size sedan URDF: 2.7 m wheelbase, Ackermann steering, windshield-height RGB-D camera. |
| `urban_gazebo` | Generated urban world/course tooling: road geometry, lane paint, signage, traffic-light control. |
| `robot_bringup` | Top-level launch files, `ros_gz` bridge configs, RViz config. |
| `av_perception_cpp` | `lane_node`: HSV-based lane-paint detection, chain clustering/merging, ground-plane projection, debug image/markers. |
| `av_behavior_cpp` | `local_planner` (curve splicing + speed profiling) and `mpc_tracker_v2` (OSQP QP tracker on a linearized bicycle model). |
| `robotcar_localization` | EKF configuration (`robot_localization`) fusing wheel odometry and IMU. |
| `av_common_cpp` | Shared C++ library: geometry/polyline math, projection, JSON map I/O, debug tap. |
| `vlm_planner_py` | VLM sign pipeline: `vlm_sign_node` (Qwen2.5-VL-3B sign reading) + `sign_maneuver_node` (board detection, sign-to-speed arbitration). Runs under a separate Python venv (torch); see below. |

Perception, planning, and control run as separate ROS2 nodes at their own rates, tied together through `av_stack.launch.py`, which also fans out the map's spawn pose so every node agrees on the same odom-frame origin.
The VLM pipeline runs as two extra nodes launched separately (not part of `av_stack.launch.py`, since it needs the torch venv) and only ever *caps* speed or adds a stop - it never steers, so the stack still drives lane-only with it off.

## VLM sign pipeline

Two nodes, both under `vlm_planner_py`, run in the torch venv alongside (not inside) `av_stack.launch.py`:

- **`vlm_sign_node`** - subscribes to the front camera, classifies the nearest/latched road sign with Qwen2.5-VL-3B-Instruct (4-bit, on its own ~0.5 Hz timer since inference takes ~2 s) and publishes `/vlm/sign` (`{"sign": <label>, "stamp": <capture time>}`, label in `{right,left,winding,stop,none,unparsed}`) plus an annotated `/vlm/sign_image`.
  It loads the model at startup, not on first frame, because the cold load (~2-3 min) pegs the CPU and would otherwise starve the Gazebo camera render and trip the MPC's path-stale safety.
- **`sign_maneuver_node`** - detects candidate sign boards from the depth image (elevated blob + color gate), latches one board by odom position, hands its pixel ROI to `vlm_sign_node` so Qwen reads the latched board instead of the nearest board-shaped thing, and runs the v1 `SignLabelLatch` / `ManeuverStateMachine` arbitration on the result.
  It publishes `/maneuver/target_speed` (absolute setpoint) and `/maneuver/state` (state name); `mpc_tracker_v2` takes `min(planner speed profile, this setpoint)`, so a sign can only lower speed, never steer or raise it.
  A `left`/`right` state also selects the MPC's turn-mode heading lookahead.
  `/vlm/sign_maneuver` publishes the full diagnostic JSON (maneuver, speed cap, stop distance, sign latch state) for the demo's narrative pane.

Tunables (camera geometry, board-detection z/x/y bands, color gate, label band, cruise/turn/winding speed caps, stop creep + standoff) live in `src/vlm_planner_py/config/sign_maneuver_params.yaml`; copy it per tuning cycle rather than editing code.

Requires a Python venv with `torch`, `transformers`, and (for the 4-bit GPU path) `bitsandbytes` - default location `~/venvs/vlm_robot` (override with `VLM_VENV`).
Without a CUDA GPU, Qwen falls back to full-precision CPU inference, which is slow enough to be impractical for the live demo.

Run it manually (two extra terminals, after `av_stack.launch.py` is up):

```bash
source ~/venvs/vlm_robot/bin/activate
python3 -m vlm_planner_py.vlm_sign_node --ros-args -p use_sim_time:=true -p vlm_query_rate_hz:=0.5
# wait for the "VLM ready" log line, then:
python3 -m vlm_planner_py.sign_maneuver_node --ros-args \
    --params-file src/vlm_planner_py/config/sign_maneuver_params.yaml -p use_sim_time:=true
```

Toggle Qwen querying live, without restarting: `ros2 param set /vlm_sign vlm_query_enabled false|true`.

## Running it

The easiest way to see the full demo (sim + both VLM nodes + narrative panes) is the course-demo script, which lays a 3x2 `terminator` grid: VLM pipeline + `av_stack` launch in column 1, live sign/maneuver narrative in column 2, topic-rate health + a free shell in column 3.

```bash
# build
colcon build
source install/setup.bash

# scripts/ can lose the executable bit (e.g. after a Windows-side checkout);
# re-set it if you get "Permission denied" on the next line:
chmod +x ./scripts/start_course_demo.sh

./scripts/start_course_demo.sh
# ./scripts/start_course_demo.sh waits for the "VLM ready" marker before
# starting the sim; skip the wait, or pick a world, with:
STACK_NOWAIT=1 ./scripts/start_course_demo.sh
DEMO_WORLD=urban_tight ./scripts/start_course_demo.sh
```

Teardown is closing the terminator window; each pane's job dies with it.

For the vision-only stack (no VLM, lane-following and turns only, no sign speed caps or stop), or to drive its launch args directly:

```bash
# full stack: Gazebo + sedan + bridge + EKF + perception + planning + control + RViz
ros2 launch robot_bringup av_stack.launch.py

# a tighter-radius course, or without RViz
ros2 launch robot_bringup av_stack.launch.py world:=urban_tight
ros2 launch robot_bringup av_stack.launch.py rviz:=false

# tune the MPC without touching code
ros2 launch robot_bringup av_stack.launch.py mpc_config:=/path/to/mpc_params_tuned.yaml
```

See `src/robot_bringup/launch/av_stack.launch.py` for the full set of launch arguments (lane-detection debug surfaces, the local planner's road-curvature assist, etc), and `scripts/start_course_demo.sh --help`-style comments at the top of that file for the demo script's env vars.

## Roadmap

This is a work in progress.
Planned areas of continued work include further MPC/perception tuning, further VLM sign-arbitration robustness (board latch, ROI hand-off) tuning, additional road-world variety, and expanded sensing/behavior features.
