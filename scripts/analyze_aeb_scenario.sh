#!/bin/bash
# analyze_aeb_scenario.sh <session|latest> [--plots]
# Thin wrapper over tools/avkit/analyze_aeb_scenario.py (no ROS needed).
set -e
export PYTHONNOUSERSITE=1
exec python3 "$HOME/vlm_av_ws/tools/avkit/analyze_aeb_scenario.py" "$@"
