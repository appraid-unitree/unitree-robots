#!/usr/bin/env bash
# Terminal 2 -- the business example: stand up, walk from A (start) to B around
# the obstacles, report the result measured from feedback.
#
#   bash scripts/run_nav_demo.sh                          # default: --goal 5 0
#   bash scripts/run_nav_demo.sh --goal 5 0 --goal 0 0 0  # there and back, final yaw 0
#   bash scripts/run_nav_demo.sh --goal 2 2 --vmax 0.3 --sit
#
# Restart the simulator (or press Backspace in its window) before every run:
# the robot must start lying at the origin.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -eq 0 ]]; then set -- --goal 5 0; fi
exec "$REPO/go2_autonomy/build/go2_nav_app" "$@"
