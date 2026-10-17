#!/bin/bash
# run_stunt_scenario.sh - one recorded + analyzed roadster stunt run.
#
#   scripts/run_stunt_scenario.sh <world> <mode> [session] [-- extra sim launch args]
#   MISSION_ARGS="release_after_hold:=false" scripts/run_stunt_scenario.sh roadster_ramp ramp
#   scripts/run_stunt_scenario.sh roadster_drop16 drop
#   scripts/run_stunt_scenario.sh roadster_hop hop          # Demo B
#   scripts/run_stunt_scenario.sh roadster_launch launch    # Demo C
#
# Starts the sim PAUSED (roadster_sim.launch.py), the stunt mission node, then
# record_stunt_run.py unpauses the world and records until STOPPED/ABORT;
# analyze_stunt_run.py scores it. Output: data/stunt_runs/<session>/
#
# Each run is isolated: its own Gazebo partition and ROS domain (STUNT_DOMAIN_ID
# to choose one), and its processes in their own process groups, which the
# cleanup kills, and only those. A global `pkill -f "ign gazebo"` used to kill
# any other sim on the machine (an interactive demo too), and Gazebo servers it
# missed kept answering /controller_manager in the shared partition: the next
# run's controllers then "failed to configure" (td_hop_imu3, flip_hop_imu1).
WORLD=${1:?world}; MODE=${2:?mode}; SESSION=${3:-$(date +%Y%m%d_%H%M%S)_${WORLD}_${MODE}}
shift $(( $# < 3 ? $# : 3 ))
[ "${1:-}" = "--" ] && shift
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)  # this workspace
OUT=$WS/data/stunt_runs/$SESSION
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PYTHONNOUSERSITE=1
export IGN_PARTITION="stunt_$$"
export ROS_DOMAIN_ID=${STUNT_DOMAIN_ID:-$(( $$ % 100 + 1 ))}  # 1-100: the safe Linux range
echo "run $SESSION: IGN_PARTITION=$IGN_PARTITION ROS_DOMAIN_ID=$ROS_DOMAIN_ID" > "$OUT/env.log"

stop_group() {  # SIGINT (ros2 launch shuts its children down), then SIGKILL what is left
  [ -n "${1:-}" ] || return
  kill -INT -- "-$1" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 -- "-$1" 2>/dev/null || return
    sleep 0.5
  done
  kill -KILL -- "-$1" 2>/dev/null
}
cleanup() {
  stop_group "${MPID:-}"
  stop_group "${SPID:-}"
}
trap cleanup EXIT

FAN0=0.0
[ "$MODE" = "drop" ] && FAN0=1.0   # drop starts held to the ceiling
setsid ros2 launch roadster_description roadster_sim.launch.py world:="$WORLD" paused:=true \
  fan_initial_speed:=$FAN0 "$@" > "$OUT/sim.log" 2>&1 &
SPID=$!
sleep 10
setsid ros2 launch roadster_control stunt_mission.launch.py mode:="$MODE" world:="$WORLD" \
  ${MISSION_ARGS:-} > "$OUT/mission.log" 2>&1 &
MPID=$!
sleep 3
python3 "$WS/tools/avkit/record_stunt_run.py" --world "$WORLD" --out "$OUT" ${RECORD_ARGS:-}
# a sim whose ros2_control controllers failed to come up never drove the car:
# report it as invalid, not as a stunt failure
if grep -q "Failed to configure controller" "$OUT/sim.log"; then
  echo "INVALID RUN: ros2_control controllers failed to configure (see $OUT/sim.log)"
  exit 2
fi
python3 "$WS/tools/avkit/analyze_stunt_run.py" "$OUT" \
  --map "$WS/src/roadster_description/maps/$WORLD.json" ${ANALYZE_ARGS:-}
RC=$?
grep -E "\->" "$OUT/mission.log" | sed 's/.*\] //' | head -20
exit $RC
