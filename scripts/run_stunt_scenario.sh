#!/bin/bash
# run_stunt_scenario.sh - one recorded + analyzed roadster stunt run.
#
#   scripts/run_stunt_scenario.sh <world> <mode> [session] [-- extra launch args]
#   scripts/run_stunt_scenario.sh roadster_drop16 drop
#
# Starts the sim PAUSED (roadster_sim.launch.py), the stunt mission node, then
# record_stunt_run.py unpauses the world and records until LANDED/ABORT;
# analyze_stunt_run.py scores it. Output: data/stunt_runs/<session>/
WORLD=${1:?world}; MODE=${2:?mode}; SESSION=${3:-$(date +%Y%m%d_%H%M%S)_${WORLD}_${MODE}}
shift $(( $# < 3 ? $# : 3 ))
[ "${1:-}" = "--" ] && shift
WS=$HOME/vlm_av_ws
OUT=$WS/data/stunt_runs/$SESSION
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PYTHONNOUSERSITE=1

cleanup() {
  kill "${MPID:-}" "${SPID:-}" 2>/dev/null
  sleep 2
  pkill -f "ign gazebo" ; pkill -f parameter_bridge ; pkill -f robot_state_publisher
  pkill -f stunt_mission_node
}
trap cleanup EXIT

pkill -f "ign gazebo"; sleep 1
FAN0=0.0
[ "$MODE" = "drop" ] && FAN0=1.0   # drop starts held to the ceiling
ros2 launch roadster_description roadster_sim.launch.py world:="$WORLD" paused:=true \
  fan_initial_speed:=$FAN0 "$@" > "$OUT/sim.log" 2>&1 &
SPID=$!
sleep 10
ros2 launch roadster_control stunt_mission.launch.py mode:="$MODE" > "$OUT/mission.log" 2>&1 &
MPID=$!
sleep 3
python3 "$WS/tools/avkit/record_stunt_run.py" --world "$WORLD" --out "$OUT"
python3 "$WS/tools/avkit/analyze_stunt_run.py" "$OUT"
RC=$?
grep -E "\->" "$OUT/mission.log" | sed 's/.*\] //' | head -20
exit $RC
