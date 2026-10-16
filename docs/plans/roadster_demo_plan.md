# Roadster stunt demo on the vlm_av_ws stack - implementation plan

Branch: `tesla-roadster-investigation` (repo `kfan12/vlm_av_ws_roadster`).
Written 2026-10-16, one day after the 2026-10-15 unveil, so every Tesla figure below is early secondary reporting and may be revised.
Source plan: `local/TeslaRoaster.pdf` (gitignored, local only).
This document turns that PDF into concrete work on **this** repository: which packages are added, which existing pieces are reused, which PDF items change, and what each phase must measurably achieve.

## 0. Goal and scope

Reproduce in simulation, as a plausible engineering approximation (not Tesla's controller or hardware):

- **Demo C, straight launch:** rear cold-gas burst, 0-60 mph in about 0.8 s.
- **Demo A, inverted ramp:** drive up a curved ramp past vertical, hold stationary upside down on suction fans, release, flip midair, cushion the landing with thrusters, land on the wheels.
- **Demo B, obstacle hop:** thruster hop over a row of Model 3, Model Y, Cybertruck, Cybercab and Semi (over 4 m tall) while in motion, upright landing.

Out of scope: matching Tesla's real nozzle layout, fan hardware, or control software, none of which is public.

## 1. Platform decision: stay on Humble + Fortress

The PDF proposes a fresh Jazzy + Harmonic workspace.
This plan instead builds on the current stack (ROS 2 Humble, Ignition Gazebo Fortress 6.18.0, verified installed), because the request is to extend this project and everything it already has (OSQP, EKF, launch/bridge conventions, WSL quirks knowledge, avkit tooling) is Humble/Fortress.
Everything the demo needs exists on Fortress, verified on this machine on 2026-10-16:

| Need | Fortress availability (verified) |
| --- | --- |
| Apply force/torque from a system plugin | `Link::AddWorldForce` / `AddWorldWrench` present (in `gz/sim/Link.hh`, which `ignition/gazebo/Link.hh` redirects to; an earlier check grepped only the redirect and wrongly reported them missing). They accumulate into `ExternalWorldWrenchCmd`. `AddWorldForce`'s `position` is an offset from the link COM, verified by the zero-g test. |
| Ready-made wrench plugin for prototyping | `ignition-gazebo-apply-link-wrench-system` present (6.18 backport). |
| Wheel normal-force measurement | `forcetorque-system` + `components/JointTransmittedWrench.hh` present. |
| Body/landing contact | `contact-system` present. |
| Plume visuals | `particle-emitter2-system` present. |
| Wind gusts | `wind-effects-system` present. |
| Physics engines | dartsim, bullet (classic), tpe in `ign-physics-5`. |
| Ray queries from a system | **Not available** (no ray feature in `ign-physics5` headers). |
| ros2_control in Gazebo | `ros-humble-ign-ros2-control` 0.7.20, `effort_controllers`, `forward_command_controller`, `joint_state_broadcaster` installed. |
| Mission FSM library | `behaviortree_cpp_v3` installed. |
| Plotting | `plotjuggler_ros` installed. |
| Array bridge type | `gz.msgs.Float_V` <-> `ros_gz_interfaces/msg/Float32Array` (not `std_msgs/Float32MultiArray`, which the Humble bridge does not map). |

A Harmonic upgrade path is in Appendix B; nothing in this plan blocks it.

## 2. What the latest reporting says (as of 2026-10-16)

| Item | Value | Source | Confidence |
| --- | --- | --- | --- |
| Event | 2026-10-15, SpaceX McGregor TX test site, livestreamed | TNW, Fox Business, Electrek | High |
| Trims | Performance (2 seats, suction fans, track-mode wings), Thrust (2 seats, rear cold-gas thrusters), Rocket (1 seat, perimeter thrusters for upward and rotational motion) | Electrek, Teslarati | High |
| Thrusters | 10 cold-gas thrusters, built with SpaceX, "linked to 600 litres of compressed air" (Lars Moravy) | TNW | High |
| Propellant store | COPVs, refillable; one outlet says 250 kg of air | Electrek, TNW, PDF | Medium |
| Thrust bound | Tesla: thrusters make "4-5x the force the tires can" | Electrek, Teslarati, Fox | Medium (claim) |
| Rear thrust | Over 80 kN | PDF (one outlet) | Medium |
| Fans | "Multi-mode suction system", pulls air from under the car, holds it upside down even when stationary | Electrek, Teslarati | Medium |
| 0-60 mph, spec | Performance under 1.6 s, Thrust under 0.9 s, Rocket under 0.59 s | Electrek, Teslarati | High (claims) |
| 0-60 mph, demo | 0.81 s reported; Teslarati says the demo showed about 1 s; India TV says 0-100 km/h in 0.81 s | PDF, Teslarati, India TV | Conflicting |
| Mass, dimensions, flight time, apex height | Not published | all | - |
| Remote operated, uncrewed prototype | Reported by Electrek per the PDF; not in the articles fetched here | PDF | Low-medium |

### Cross-checks (own arithmetic, assumptions marked)

- **600 L vs 250 kg agree.**
  250 kg / 0.6 m^3 = 417 kg/m^3.
  Ideal gas at 293 K gives P = rho R T = 417 x 287 x 293 = 35 MPa (351 bar).
  Air's compressibility factor at that state is about 1.1-1.15, so the real pressure is about 390-400 bar (5,700-5,800 psi), consistent with the PDF's 6,000 psi concept figure.
  Treat 400 bar / 0.6 m^3 / 250 kg as one coherent nominal point; the tank model must use a real-gas Z(P) correction or it under-predicts pressure by about 12%.
- **Thrust vs tire claim.**
  Assumed m = 1,800 kg, tire mu = 1.2: tire force about 21 kN, so "4-5x" means about 85-105 kN, consistent with "over 80 kN".
- **0-60 in 0.81 s is not "full thrust".**
  Mean acceleration 26.8 / 0.81 = 33 m/s^2 (3.4 g), so mean net force about 60 kN at 1,800 kg.
  With tires saturated at about 21 kN, mean thrust only needs about 39 kN.
  At 80 kN + tires the run would take about 0.5 s, which matches the Rocket's "under 0.59 s" claim instead.
  A single 0-60 time cannot separate mass from thrust level (full 80 kN at 0.81 s would imply about 3.8 t), so mass stays at its nominal value and the demo is reproduced with a rear throttle of about 50%; Demo C is a Thrust-trim run (rear only).
- **Impulse budget is the binding constraint.**
  Assumed exhaust velocity 300-450 m/s (cold air, sea-level nozzle, blowdown cooling).
  Total impulse = 250 x (300-450) = 75-110 kN.s; usable with a floor of about 20-30 bar is about 60-90 kN.s.
  These figures assume thrust stays at nominal while gas remains, which holds only for a **regulated** system (COPV -> regulator at p_reg -> valves, typical of cold-gas systems).
  In pure blowdown, thrust scales with tank pressure: the four down nozzles (80 kN at 400 bar) only exceed m g above about 90 bar, so a drained tank cannot hover or cushion even with mass left.
  The tank model therefore has both modes (Section 5.2), regulated is the default, and the landing reserve is expressed as a pressure/thrust condition, not only as an impulse number.
  `roadster_sizing.py` integrates the tank model and reports every budget below in both modes.
  Demo C (39 kN x 0.81 s) costs about 32 kN.s, i.e. 35-55% of a full tank.
  Demo B hop to 5 m apex (Semi 4 m + 1 m clearance) costs about 25 kN.s up (60 kN burn, 0.42 s, gravity loss included) + about 18 kN.s landing cushion (9.9 -> 3 m/s), so about 43 kN.s, 50-70% of a tank.
  Conclusion: one fill per demo, and the planner must budget impulse explicitly.
- **Flip is cheap, landing is expensive.**
  Assumed roll inertia Ixx = m (w^2 + h^2) / 12 = 1,800 x (1.9^2 + 1.2^2) / 12 = about 760 kg.m^2.
  A 180 deg roll in 1.2 s needs L = I theta / t = 760 x pi / 1.2 = about 2,000 N.m.s; with a 0.9 m lever that is about 2.2 kN.s to spin up and the same to stop.
  Falling from the ramp top dominates.
  With the default ramp (R = 10 m, so the inverted hold is about H = 2R = 20 m up), touchdown speed is 19.8 m/s.
  Cushioning to 3 m/s with a 60 kN burn takes about 0.7 s and about 43 kN.s (30 kN.s of momentum change plus 13 kN.s gravity loss), so Demo A is as impulse-hungry as Demo B.
  Ramp radius is therefore the strongest lever on feasibility and is the first thing to pin down from the video.

## 3. Corrections to the PDF

1. **The flip is 180 deg, not 360 deg.**
   At the top of a 180 deg ramp the car is already inverted, so landing on the wheels needs (N + 1/2) turns.
   The rotation target is a parameter `flip_turns` (default 0.5) and the planner uses L = I x (2 pi x flip_turns) / t_flight, not I x 2 pi / t_flight.
   Phase 5's exit criterion is rewritten accordingly.
2. **Inverted-hold fan force formula has the wrong sign for a loop.**
   On the inside of a concave loop at the top, with fan suction F_fan pulling toward the surface and normal force N pushing away from it, m v^2 / R = m g + N - F_fan, so N >= 0 needs F_fan >= m (g - v^2 / R).
   Speed helps; the worst case is the stationary hold the demo actually showed, which needs F_fan > m g (about 17.7 kN at 1,800 kg), plus margin for traction.
   The PDF's m (g + v^2 / R) is the convex-crest case and would over-size the fans by 2-3x.
   Sizing rule used here: F_fan_max = k_hold x m g with k_hold = 1.5 (about 26.5 kN), and traction margin checked as mu (N + ...) against the drive force needed to creep up the overhang.
   The real high-load case for the contact solver is the **bottom** of the ramp, where N = m g + m v^2 / R reaches about 4-5 g at 20 m/s and R = 10-12 m.
3. **DART is primary; Bullet is not assumed to be a fallback.**
   Fortress's classic bullet plugin is far less complete than the later bullet-featherstone engine, and whether it handles this car's wheel joints and friction is untested; tpe has no real contact dynamics.
   A one-off Bullet smoke test in Phase 1 decides whether it can serve as a comparison at all.
   Contact-stability mitigation is a small step, simple collision primitives, and a logged "stick force" fallback (Section 6), not an engine swap.
4. **No raycasts on Fortress.**
   The fan plugin computes the gap h from the analytic ramp geometry instead (Section 5.4), which is also more deterministic than a ray.
5. **Bridge types.**
   The thruster command is `gz.msgs.Float_V` <-> `ros_gz_interfaces/msg/Float32Array` (Section 1), not `std_msgs/Float32MultiArray`.

## 4. How this maps onto the existing repository

### Reused as-is

- `av_common_cpp` (header-only geometry math, JSON map I/O, debug tap): linked by every new C++ package.
- OSQP / osqp-eigen in `/usr/local`, already used by `mpc_tracker_v2`: reused for the thrust allocator QP.
- `robotcar_localization` EKF config pattern: reused in Phase 7 for a stunt-tuned EKF.
- The `maps/<world>.json` single-owner convention: ramp and obstacle geometry live in `maps/roadster_*.json`, read once by the launch file and fanned out as parameters (same as `map_origin_*` today).
- `tools/avkit` record/analyze pattern (JSONL recorder, no-pandas analyzer, `scripts/*_scenario.sh` wrappers, output under `data/`).
- WSL rules: `LIBGL_ALWAYS_SOFTWARE=1`, `ign gazebo -s -r`, never `--headless-rendering`.

### Deliberately not reused in the stunt worlds

- **`lane_node`**: its ground-plane projection assumes a flat road under the car, which is false on a curved ramp and meaningless when inverted.
  Ramp centering uses ground-truth pose against the analytic ramp centerline (and later the EKF), in ramp-frame coordinates (arc length s, lateral offset d).
- **RGBD camera**: the stunt model ships without it by default (`with_camera:=false`).
  Software-GL rendering plus the known depth-stall quirk would make a 1 ms physics step crawl.
  A camera can be re-enabled for the final video pass only.
- **`mpc_tracker_v2` / `local_planner` / VLM chain / `aeb_node`**: not launched in stunt worlds.
- **`sedan_description` and the AckermannSteering plugin**: untouched.
  `roadster_description` is a separate package whose model is derived from `sedan.urdf.xacro` (same chassis collision box, wheelbase, track, wheels, steering stops, friction path, IMU), with only the changes the stunts need.
  The Ackermann plugin commands wheel joint *velocities* from `/cmd_vel` (capped at 10 m/s and 4 m/s^2), so it would brake the car against the thrusters during a burst.
  The roadster uses `gz_ros2_control` (the Humble package also ships the `ign_ros2_control` name) with **effort** on the four wheels and **position** on the two steer joints, so wheels free-roll or deliver torque and DART friction sets the real traction limit.

### Command authority

During a stunt, `stunt_mission_node` is the only publisher of wheel effort, steering, thruster and fan commands, because the driving chain is simply not launched (`roadster_stunt.launch.py` brings up only roadster nodes).
This sidesteps the unbuilt Day 17 arbiter (`local/plans/day17.md`) for now.
Phase 9 (optional) wires the stunt into the arbiter once Day 17 exists, so the normal driving chain can drive the car to the ramp entry and hand over.

### Zero-edit rule

No changes to `lane_node.cpp`, `local_planner.cpp`, the MPC solver, `aeb_node`, `sedan.urdf.xacro`, or the existing urban worlds.
Everything is new packages, new worlds, a new launch file, and new bridge YAML.

## 5. New packages, files and interfaces

```
src/
  roadster_description/        BUILT 2026-10-16. Derived from sedan.urdf.xacro: body, 4 effort wheels, 2 position
                               steer joints, 10 nozzle frames, gz_ros2_control block, IMU, 3D truth odometry,
                               F/T sensors on wheel joints, body contact sensor, optional camera (rear door: Phase 2)
    urdf/roadster.urdf.xacro             args: chassis_mass (1730 -> 1,800 kg total), tire_mu (1.2), with_camera (false)
    config/nozzles.yaml                  nozzle layout, read by the xacro and later by the thruster plugin
    config/ros2_controllers.yaml
    config/bridge_roadster.yaml
    launch/roadster_sim.launch.py        world / spawn pose / paused / surfaces args (worlds/roadster_*_test.world.sdf)
    config/roadster_params.yaml          (Phase 0) single source of the remaining physical + actuator parameters
  roadster_gz_plugins/         BUILT 2026-10-16. ONE Gazebo Fortress system writes every stunt force
    src/roadster_actuation_system.cpp    tank + 10 nozzles + suction fan (+ rear door later)
    include/roadster_gz_plugins/tank_model.hpp, nozzle_model.hpp, fan_model.hpp   pure math, unit-tested
    test/test_models.cpp
  roadster_control/            BUILT 2026-10-16 (C++ ROS 2)
    src/stunt_mission_node.cpp           mission FSM (modes ramp, drop) + drive / fan / attitude / landing loops
    src/allocator.cpp                    OSQP QP B u = [F; tau], 0 <= u <= 1, forces weighted in world axes
    include/roadster_control/attitude.hpp, landing.hpp, allocator.hpp
    test/test_control.cpp
    config/stunt_params.yaml, launch/stunt_mission.launch.py
    (planned: flight_planner for the hop, stunt_safety_monitor, PWM / min-on-time-aware allocation)
src/av_feature_msgs/msg/StuntStatus.msg  BUILT: state, ground-phase and flight-phase telemetry
src/urban_gazebo/worlds/roadster_hop.world.sdf / maps/roadster_hop.json   (Phase 6)
src/roadster_description/worlds/roadster_{ceiling,drop16,ramp}.world.sdf  BUILT, generated, not hand-edited
maps/roadster_{ceiling,drop16,ramp}.json  BUILT, generated alongside each world (single geometry owner)
scripts/generate_stunt_world.py           BUILT: ceiling and ramp kinds (hop row: Phase 6)
scripts/run_stunt_scenario.sh             BUILT: paused sim + mission + record + analyze, one command
tools/avkit/roadster_sizing.py            (Phase 0) the Section 2 arithmetic as a script
tools/avkit/record_stunt_run.py, analyze_stunt_run.py   BUILT; stunt_monte_carlo.py (Phase 7)
tools/avkit/roadster_actuation_test.py, roadster_hold_test.py, roadster_smoke_drive.py   BUILT
```

Language policy holds: everything in the control loop is C++; Python is only world generation, sizing and offline analysis.

### 5.1 Topics

| Topic (ROS) | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/roadster/thruster_cmd` | `ros_gz_interfaces/Float32Array` (gz `Float_V`) | ROS -> gz | 10 valve commands 0-1, 200 Hz |
| `/roadster/thruster_state` | `ros_gz_interfaces/Float32Array` | gz -> ROS | 10 actual thrusts in N |
| `/roadster/tank/pressure`, `/roadster/tank/mass` | `std_msgs/Float64` (gz `Double`) | gz -> ROS | Pa, kg |
| `/roadster/fan_cmd` | `std_msgs/Float64` | ROS -> gz | 0-1 |
| `/roadster/fan_force` | `std_msgs/Float64` | gz -> ROS | N, actual suction force |
| `/roadster/rear_door_cmd` | `std_msgs/Float64` | ROS -> gz | 0 closed, 1 open |
| `/roadster/wheel_wrench/{fl,fr,rl,rr}` | `geometry_msgs/Wrench` (gz `Wrench`) | gz -> ROS | wheel normal force from F/T sensors |
| `/roadster/body_contact` | `ros_gz_interfaces/Contacts` | gz -> ROS | landing impact, scrape detection |
| `/odom_truth`, `/imu`, `/clock` | as today | gz -> ROS | same names as the sedan bridge |
| `/stunt/status` | `av_feature_msgs/StuntStatus` | ROS | for recorder, RViz text, PlotJuggler |
| `/stunt/start`, `/stunt/abort`, `/roadster/tank/recharge` | `std_srvs/Trigger` | ROS | the recharge service is implemented as a gz service bridged, or by respawn (Phase 2 decides) |

### 5.2 Gas tank model (inside ColdGasThrusterSystem)

- The tank is a model class (`tank_model.hpp`) owned by the thruster system, not a separate system: Fortress has no system execution priority, so two coupled systems would update in an undefined order and introduce a hidden one-step lag.
- State: gas mass m_g, gas temperature T_g; pressure from P = Z(P, T) rho R T with a tabulated Z for air.
- Thermo modes `isothermal` (default) and `polytropic` (n = 1.2-1.4, models blowdown cooling); polytropic drops pressure faster and is the conservative choice for budgeting.
- Feed modes `regulated` (default: nozzles see p_reg while P_tank > p_reg, then blowdown below it) and `blowdown` (nozzles see P_tank throughout).
- dm_g/dt = - sum of nozzle mass flows in the same step.
- Floor `p_min` below which no nozzle can fire; recharge restores `p0`, `T0`.
- Parameters: `volume_m3: 0.6`, `p0_bar: 400`, `T0_K: 293`, `p_reg_bar: 100`, `p_min_bar: 25`, `feed_mode: regulated`.
- Published on `/roadster/tank/{pressure,mass}` so the planner and safety monitor can check the landing condition (Section 5.9).

### 5.3 ColdGasThrusterSystem

- Per nozzle i: body-frame position r_i, unit direction e_i, `f_max_i` at `p_ref`, valve time constants `tau_rise`/`tau_fall` (10-30 ms), `min_on_time`, thrust-curve exponent, optional jitter.
- Thrust: F_i = u_i,actual x f_max_i x (P_feed / p_ref), where P_feed is p_reg while the regulator holds and P_tank otherwise, and p_ref = p_reg (the PDF's F = Cf Pt At with Cf At folded into f_max / p_ref); mass flow mdot_i = F_i / v_e.
- Valve: first-order lag from command to u_actual, with min-on-time enforced in the plugin so the controller cannot cheat it.
- Force on the SDF link (`base_link`, into which sdformat lumps `chassis`) through `Link::AddWorldForce(F_world, offset)`, rotated by the current link pose.
  The offset is measured from the link COM, so the plugin passes `frame_offset + r_i - com_link`; passing link-frame points instead doubled the rear-main pitch lever (0.63 m vs 0.30 m) in the first zero-g run.
- **Single writer:** tank, nozzles and fan live in one system (`RoadsterActuationSystem`), so there is no cross-system ordering or overwrite question.
- **Verified (2026-10-16):** `tools/avkit/roadster_actuation_test.py` in `roadster_zero_g_test` fires every nozzle alone plus three combinations and compares with the rigid-body prediction from the expanded URDF: force within 0.3% and torque within 0.1% on all 13 patterns.
  Fortress's WorldControl reset left the car moving, so the test runs bursts back to back and compares torques through the full Euler equation.
- Rear nozzle door: a prismatic or revolute joint driven by the plugin through `JointPositionCmd`-style position control; the rear main nozzles are masked to 0 until the door reports open.
- Plume: one `particle-emitter2` emitter per nozzle, rate scaled by u_actual; visual only, plus optional ground-impingement force (off by default).

### 5.4 SuctionFanSystem

- F = F_max x w^2 x g(h), with w the normalized fan speed (first-order lag 0.2-0.5 s from `/roadster/fan_cmd`), applied along the body -z axis at the body center.
- g(h): ground-effect falloff, g = 1 for h <= h_seal, decaying to 0 by h_max (defaults 0.15 m, 0.6 m).
- h is computed from the analytic surfaces in the world JSON: the plugin loads `maps/<world>.json` (path passed as an SDF parameter by the launch file, same single-owner rule) and computes the signed distance from the body underside to the nearest ramp/ground/ceiling patch.
- Fault injection: `fan_fail_at_s` and `fan_ramp_down_s` parameters.

### 5.5 World generator (`scripts/generate_stunt_world.py`)

- Ramp: flat run-in, then a circular arc of radius R (default 10 m, tunable) from 0 to `arc_deg` (default 180), with an optional straight lip; extruded to width W as a triangle mesh (`.dae` or `.obj`) written next to the world, with a separate low-poly collision mesh or a chain of box segments (box chain is the default because DART handles boxes better than a concave trimesh).
- Per-patch friction (`mu`, `mu2`) in the SDF surface block.
- `--ceiling`: a flat static slab at a set height for the Phase 4 inverted-hold test.
- `--hop-row`: boxes with real outer dimensions (Model 3 4.72 x 1.85 x 1.44 m, Model Y about 4.79 x 1.92 x 1.62 m, Cybertruck 5.68 x 2.20 x 1.79 m, Semi tractor about 6.5 x 2.6 x 4.1 m, Cybercab about 3.6 x 1.9 x 1.4 m - Cybercab is an estimate) placed along a runway.
- Landing pad with a contact sensor.
- Writes `maps/roadster_<name>.json`: spawn pose, ramp centerline polyline + normals, patch list, obstacle boxes, landing pad - consumed by the fan plugin, `ramp_frame.hpp`, the planner and the analyzer.
- Physics block written explicitly: `<engine><filename>ignition-physics-dartsim-plugin</filename></engine>` on the physics system, `max_step_size` 0.001, no sensors/render systems unless `--with-camera`.

### 5.6 Mission state machine (`stunt_mission_node`)

States: `IDLE -> ACCELERATE -> CLIMB -> HOLD_INVERTED -> RELEASE -> FLIGHT_FLIP -> LAND_BRAKE -> LANDED`, with `ABORT` reachable from any state, plus a `HOP` branch (`ACCELERATE -> HOP_BURN -> BALLISTIC -> LAND_BRAKE`) and a `LAUNCH_C` branch for Demo C.

- **Drive loop (200 Hz):** speed PI on wheel effort following an s-indexed speed profile, lateral PD on ramp-frame d through steering.
- **Fan loop:** the surface normal force is N = m v^2 / R + m g cos(phi) + F_fan, with phi the arc angle (0 at the ramp bottom, 180 deg inverted at the top).
  Feed-forward F_ff = k_hold x (m (-g cos(phi) - v^2 / R))^+ + N_min from the planned profile, plus feedback trim on measured wheel forces keeping each wheel above `n_min` (default 0.25 m g / 4, so N_min = 0.25 m g).
- **Release:** fans cut on a ramp, door opens, upward-relative nozzles push the car off the surface while a roll/pitch couple injects the planned angular momentum.
- **Attitude loop (200 Hz):** quaternion-error PD (LQR as a stretch) on body rates, output body torque and force requests to the allocator.
- **Landing brake:** time-to-impact from height above the pad and vertical speed, start the burn so that v_z <= 3 m/s at touchdown with attitude error under 10 deg.

### 5.7 Thrust allocator

- Effectiveness matrix B (6 x 10) from nozzle r_i, e_i, f_max_i x P_tank / p_ref, rebuilt each tick (pressure changes).
- QP: min ||W (B u - w_des)||^2 + lambda ||u||^2 subject to 0 <= u <= 1, solved with OSQP warm-started, with a pseudo-inverse + clip fallback when OSQP fails (same failure policy idea as `mpc_tracker_v2`).
- Output to PWM at 200 Hz respecting `min_on_time`; residual (unachievable) wrench published on `/stunt/status` so the planner sees saturation.

### 5.8 Flight planner

- Inputs: world JSON, current state, tank state.
- Ramp mode: given release state at the inverted hold (height H = 2R above the pad for a 180 deg arc, near-zero velocity), compute t_flight with the planned push-off, required L = I x 2 pi x `flip_turns` / t_spin, and the landing-burn start.
- Hop mode: from approach speed v_x and obstacle row extents, choose burn start, burn level and duration so the body clears every box by `clearance_m` (default 1.0 m), then plan the landing burn.
- Every plan carries an impulse estimate and is rejected (state `ABORT` before committing) if it exceeds `impulse_reserve_frac` of the remaining usable impulse.

### 5.9 Safety monitor

Watchdogs: tank below the planned-landing impulse, or predicted nozzle pressure at planned touchdown too low for the down nozzles to deliver at least 1.5 m g, tilt beyond envelope while on the ground, unexpected contact loss during CLIMB/HOLD, any body (non-wheel) contact, state timeouts, NaN in odometry.
ABORT behavior depends on state: on the ramp, max fans and creep down; in flight, all remaining impulse to the landing burn.

## 6. Phases

Each phase ends with a recorded run and an analyzer report under `data/stunt_runs/<session>/`, not a by-eye judgment.

| Phase | Goal | Deliverables | Exit criterion |
| --- | --- | --- | --- |
| 0 | Sizing and scaffolding | `tools/avkit/roadster_sizing.py`, empty packages building, `roadster_params.yaml` v0 with a source/confidence column | `colcon build` of the new packages is warning-free; sizing script reproduces the Section 2 numbers in both regulated and blowdown modes and prints which demos stay feasible in each |
| 1 | Car on flat ground | `roadster.urdf.xacro`, `ign_ros2_control` effort/position, `roadster_flat` world, `roadster_stunt.launch.py`, bridge YAML, Bullet smoke test | Tire-only straight run, all-wheel drive: 0-60 mph in 2.2-2.6 s (traction-limited, mu = 1.2 gives about 11.8 m/s^2 and 2.3 s); F/T normal forces sum to m g within 2% at rest; RTF >= 0.5 at 1 ms step. Optional cross-check: Tesla's "Performance under 1.6 s" corresponds to roughly 0.5 m g of fan downforce added to traction |
| 2 | Tank + thruster plugin | `ColdGasThrusterSystem` with the tank model, accumulator, unit tests | Static test (car fixed by a joint to the world): measured wrench matches commanded within 2%; a constant command held for 1,000 steps gives a constant applied force (proves `ExternalWorldWrenchCmd` is cleared each step, so read-and-add accumulation cannot diverge); valve step response matches tau; tank depletion matches the closed-form curve within 2% in both feed modes; torque signs correct for all 10 nozzles |
| 3 | Demo C | Launch branch in mission FSM, rear-door sequencing, rear throttle profile | Mass held at nominal; the plant tracks the commanded rear thrust profile within 5%, and the fitted profile (about 50% rear throttle) reproduces 0-60 mph in 0.81 +- 0.05 s; impulse used reported and within 25% of the 32 kN.s estimate; yaw error under 2 deg |
| 4 | Fans, ceiling hold, ramp climb | `SuctionFanSystem`, `roadster_ceiling` and `roadster_ramp` worlds, fan loop, ramp-frame tracking | Ceiling: car held at rest upside down for 10 s with all four wheel normal forces positive. Ramp: climbs to 180 deg, holds stationary inverted for 2 s (PDF criterion) and 10 s (stretch), lateral error under 0.2 m, no body contact |
| 5 | Release, flip, land | Planner ramp mode, allocator, attitude loop, landing brake | 10/10 runs: total rotation 180 deg +- 15 deg (`flip_turns = 0.5`), touchdown v_z under 3 m/s, attitude error at touchdown under 10 deg, wheels-only contact, impulse under the tank budget |
| 6 | Demo B | `roadster_hop` world, planner hop mode | 10/10 runs: minimum clearance over the 4.1 m Semi box at least 0.5 m (target 1.0 m), upright landing per Phase 5 limits |
| 7 | Robustness | EKF stunt config (IMU + wheel odom, `robotcar_localization` pattern), noise models, wind-effects gusts, `stunt_monte_carlo.py` | At least 90% success over 200 randomized runs (mass +- 10%, COM +- 5 cm, f_max +- 10%, valve tau +- 50%, mu +- 20%, fan force +- 15%, gusts up to 8 m/s), with the controller on EKF state, not ground truth |
| 8 | Calibration and polish | Video calibration (Section 8), plume tuning, RViz config, camera-on video render, report under `docs/reports/` | Fitted flight time and roll rate within 10% of the livestream measurements; full Demo A/B/C reel recorded |
| 9 (optional) | Integration with the driving stack | Requires Day 17 arbiter; stunt node becomes an arbiter source; urban course leg drives to the ramp entry | Lane-following leg hands over to the stunt FSM at the ramp entry with no command gap over 50 ms |

Phase 1 status (2026-10-16): the model part is done and smoke-tested with `tools/avkit/roadster_smoke_drive.py` on `roadster_sim.launch.py` (then named `roadster_smoke.launch.py`) at 1,800 kg total.
The four wheel normal forces sum to the sprung weight within 0.1%, steering settles from a 0.3 rad step in 0.175 s, the full-torque tire-only 0-60 mph takes 2.29 s (1.19 g, mass-independent as expected, inside the 2.2-2.6 s window), yaw drift is 0.01 deg, and RTF is 1.00 at the 1 ms step.
Two findings fed back into the model: Fortress ignores a contact sensor's `<topic>` (the launch file bridges the world-scoped name instead), and the sedan's 400 N.m steering effort stalls under standstill tire scrub at 1,800 kg (raised to 3000 N.m).
Still open for Phase 1: a drive controller with traction control (at full torque the wheels spin to their 200 rad/s limit, slip ratio +1.46), wheel odometry, and the Bullet smoke test.

### Demo A status (2026-10-16): done in simulation, 10/10 runs

Demo A was built ahead of Phases 3 and 6, in the order: zero-g force-point test, ceiling hold, drop-flip-land, ramp climb and hold, full chain.
Run it with `scripts/run_stunt_scenario.sh roadster_ramp ramp`; each run lands in `data/stunt_runs/<session>/` with `run.jsonl` and `metrics.json`.

| Check (plan exit criteria) | Result, all 10 runs |
| --- | --- |
| Climbs to fully inverted, no body contact | yes; minimum wheel load on the ramp 1.71-1.72 kN |
| Holds stationary upside down >= 2 s | 2.00 s at x = -7.00 m, still to 0.7-0.8 cm, wheels >= 2.3 kN, lateral offset 0.000 m |
| Flip rotation 180 deg +- 15 | 180.0 deg (roll, "kick flip") |
| Touchdown vertical speed < 3 m/s | 2.19-2.24 m/s |
| Touchdown attitude error < 10 deg | 0.01 deg |
| Landing spot | 6 cm from directly below the hold point |
| Gas / impulse | 47.9-48.1 kN s, 137 kg of the 250 kg fill (consistent with the Section 2 estimate) |

Ramp: R = 8 m, 180 deg, 10 m overhang, so the drop is about 16 m.
The release, flip and landing were developed first on `roadster_drop16` (a ceiling at 16 m), which reproduces the top-of-ramp state and gives identical flight numbers.

Findings that changed the design along the way:
- `Link::AddWorldForce`'s position argument is an offset from the link COM, not a link-frame point (Section 5.3).
- The first nozzle layout (four in-plane side nozzles) could only roll the car with +z thrust, which pushed it 4.5 m sideways mid-flip.
  Roof-corner nozzles (body -z) make force-free roll couples; drift is now 5.6 cm.
- Tangent-at-midpoint arc boxes left a 2.7 mm step onto the overhang that unloaded a wheel to 0.2 kN at 6 m/s; the arc is now an inscribed polygon.
- A speed-vs-distance stop law demands twice the available deceleration at its crossover; the stop onto the hold point is a planned constant-deceleration trajectory instead.
- Fortress ignores a contact sensor's `<topic>`, `Link::WorldPose` needs a `WorldPose` component, and WorldControl reset leaves the car moving.

Known gaps, not blocking Demo A:
- Peak deceleration at touchdown is about 8.3 g: the car has rigid wheels and no suspension. A suspension model belongs to Phase 7.
- Touchdown speed is 2.2 m/s against the 1.5 m/s profile target; the brake feedback (`brake_kv`) could be raised.
- State is ground truth; the EKF path, sensor noise and Monte Carlo robustness are Phase 7.
- No rear nozzle door, no plume visuals and no video calibration yet (Phase 8); ramp radius and fan force are assumptions until then.
- The zero-g test failed once in 13 runs, unexplained and not reproduced since; it now refuses to run with another thruster publisher alive.

Phase ordering note: Phase 3 comes before fans on purpose, because it validates the thruster plugin under the simplest dynamics and calibrates mass against the 0.81 s figure before any ramp tuning depends on mass.

## 7. Parameters (roadster_params.yaml v0)

Every entry carries `value`, `source` and `confidence` fields in the YAML; the table is the initial set.

| Parameter | Value | Basis |
| --- | --- | --- |
| mass | 1,800 kg | Assumption; demo cars may be stripped shells (PDF), so calibrate in Phase 3 |
| body L x W x H | 4.4 x 1.95 x 1.15 m | Assumption, not published |
| COM | 0.15 m behind wheelbase center, 0.45 m above ground | Assumption, low and slightly rearward per PDF |
| wheelbase, track, wheel radius | 2.6 m, 1.65 m, 0.34 m | Assumption |
| tire mu / mu2 | 1.2 / 1.2 | PDF range 1.0-1.5 |
| nozzles | 10: 2 rear main (+x thrust, behind the plate door), 4 down-facing at corners, 4 lateral/corner canted for roll/yaw | Layout is an assumption; count is reported |
| rear main f_max | 2 x 40 kN at p_ref | "Over 80 kN" |
| corner f_max | 4 x 20 kN down, 4 x 8 kN canted | Assumption sized so 4 down nozzles exceed 3.5 g lift; check against "4-5x tire force" |
| exhaust velocity v_e | 350 m/s (range 300-450) | PDF estimate |
| valve tau_rise / tau_fall | 20 ms / 20 ms | PDF 10-30 ms |
| min_on_time | 20 ms | Assumption |
| tank | 0.6 m^3, 400 bar, 293 K, regulated at 100 bar, floor 25 bar, real-gas Z | Reported 600 L; pressure from the 250 kg cross-check |
| fan F_max | 26.5 kN (1.5 m g) | Section 3 correction; no public data |
| fan spin-up tau | 0.3 s | PDF 0.2-0.5 s |
| ramp R, arc, width | 10 m, 180 deg, 5 m | Assumption until video calibration |
| physics step | 1 ms DART | PDF; control loops 200 Hz |

## 8. Calibrating against the livestream

1. Extract frames from the public livestream at native rate with `tools/avkit/extract_frames.py` (already exists; extend it for a video input if needed).
2. Scale with known sizes: Semi tractor height about 4.1 m, Model 3 length 4.72 m.
3. Measure ramp radius and lip height, time from release to touchdown, rotation angle vs time, hop apex, and the Demo C 0-60 interval.
4. Fit mass, COM height, push-off impulse and rear f_max by least squares on flight time, roll rate and 0-60 time with `analyze_stunt_run.py --fit`.
5. Report the fit error and state clearly that the result is an approximation, not Tesla data.

## 9. Verification

- **Unit tests (gtest, actually built):** thrust model step response and pressure scaling, tank model vs closed-form blowdown, fan g(h), allocator (exact solution when feasible, bounded residual when saturated, sign checks per nozzle), ramp-frame projection, planner impulse estimates.
  This is the first built test suite in the repo; it does not touch the existing unbuilt `test/` directories.
- **Physics sanity:** angular momentum conserved within 1% in a no-thrust free flight; energy bookkeeping in a fan-off ballistic drop.
- **Scenario metrics (analyzer):** release speed and angle, apex height, total rotation, min obstacle clearance, tank mass used, impulse used, touchdown v_z, touchdown attitude error, peak acceleration, min wheel normal force on the ramp, success flag.
- **Sensitivity sweep:** one-at-a-time sweeps of mass, COM, f_max, fan force, mu and valve tau, to report margins next to the Monte Carlo result.
- **Regression:** `scripts/analyze_stunt_run.sh --baseline <session>` compares against a stored baseline, same shape as the AEB tooling.

## 10. Risks

| Risk | Mitigation |
| --- | --- |
| Public data is thin and conflicting (mass, layout, pressure, fan power, 0-60) | Every value in YAML with source and confidence; video calibration in Phase 8 |
| DART contact jitter at 4-5 g on the ramp bottom or when inverted | 1 ms step, sphere wheel collisions, box-chain ramp collision, contact parameters tuned on the ceiling world first; a logged "stick force" fallback (small extra suction, flagged in the report, off for final runs) |
| Wrench overwrite between plugins | Accumulator helper plus a dedicated two-system test (Section 5.3) |
| RTF collapse on WSL software GL | No camera or render systems in stunt worlds, simple collisions; runs use sim time so a low RTF only costs wall-clock time |
| Valve nonlinearity and min-on-time | PWM with min-on-time inside the plugin; MPC-based allocation as a stretch goal |
| Impulse budget too small for the planned maneuver | Planner rejects plans over `impulse_reserve_frac`; safety monitor reserves landing impulse |
| Effort-controlled wheels behave differently from the kinematic sedan | Phase 1 exit criteria check traction-limited acceleration and normal forces before any thruster work |

## 11. First concrete steps

1. Phase 0: write `tools/avkit/roadster_sizing.py` and generate `roadster_params.yaml` v0.
2. Scaffold `roadster_description`, `roadster_gz_plugins`, `roadster_control` and make them build (empty systems, empty nodes).
3. Phase 1: roadster URDF with `ign_ros2_control` on the `roadster_flat` world, then the tire-only 0-60 run.

## Appendix A: Sources

- `local/TeslaRoaster.pdf` (the source plan).
- [Electrek, 2026-10-15: Tesla unveils Cyberfied new Roadster that can kickflip](https://electrek.co/2026/10/15/tesla-unveils-cyberfied-new-roadster-kickflip/)
- [The Next Web: Tesla unveils thruster-equipped Roadster, with production set for 2028](https://thenextweb.com/news/tesla-roadster-reveal-thrusters-2028)
- [Teslarati: Tesla Roadster: three configs, one can fly, 0-60 in 0.8 seconds](https://www.teslarati.com/tesla-roadster-specs-configurations/)
- [Fox Business: Tesla's new Roadster takes flight during dramatic unveiling at SpaceX facility](https://www.foxbusiness.com/technology/teslas-new-roadster-makes-flying-debut-hitting-60-mph-under-second)
- [India TV News: 0-100 km/h in 0.81 seconds](https://www.indiatvnews.com/auto/elon-musk-s-tesla-roadster-can-flip-and-fly-with-spacex-rocket-thrusters-goes-from-0-100kmph-in-0-81-seconds-2026-10-16-1057128)

## Appendix B: Harmonic upgrade path

If the stack later moves to Jazzy + Harmonic: rename `ignition::gazebo` to `gz::sim` in the actuation system (re-run the zero-g test, since `AddWorldForce`'s offset convention must be re-checked), optionally replace the analytic gap with physics ray queries, swap `ign_ros2_control` for `gz_ros2_control`, and re-verify the bridge type mappings.
The pure-math headers, controllers, planner, tooling and parameter files carry over unchanged.
