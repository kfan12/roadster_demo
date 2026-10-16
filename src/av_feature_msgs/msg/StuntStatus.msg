# Roadster stunt mission status (roadster_control/stunt_mission_node, 200 Hz).
# Frames: "world" = Gazebo world (z up); "body" = the roadster chassis frame.
std_msgs/Header header

string state              # HOLD, RELEASE, FLIP, LAND_BRAKE, LANDED, ABORT, ...
float64 t_state           # time in the current state [s]
float64 t_mission         # time since the mission started [s]

float64 height            # COM height above its landed (upright, on ground) height [m]
float64 vz                # COM vertical velocity, world frame [m/s]
float64 roll_progress     # flip rotation achieved about the flip axis [rad]
float64 attitude_error    # angle between the reference and actual attitude [rad]

# ground phase (ramp mode)
float64 speed             # forward speed along the body x axis [m/s]
float64 speed_ref         # [m/s]
float64 arc_angle         # progress round the ramp arc: 0 at its bottom, pi fully inverted [rad]
float64 lateral_offset    # offset across the ramp width from its centreline [m]
float64 min_wheel_normal  # smallest wheel normal force (chassis z, F/T sensors) [N]
float64[4] wheel_torque   # commanded FL FR RL RR [N m], after traction control

float64 tank_mass         # gas remaining [kg]
float64 feed_ratio        # nozzle feed pressure / reference pressure (0 = cannot fire)
float64 impulse_used      # integral of total measured thrust since mission start [N s]
float64 fan_cmd           # 0..1

float64[6] wrench_des     # requested wrench: Fx Fy Fz in WORLD axes [N], Tx Ty Tz body [N m]
float64[6] wrench_alloc   # what the allocated valve commands produce, same axes (no valve lag)
