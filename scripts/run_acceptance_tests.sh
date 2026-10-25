#!/usr/bin/env bash
# Terminal 2 -- the 8 acceptance tests (about 80 s). Needs a FRESH simulator
# started with scripts/run_sim.sh (default scene_nav.xml).
#
#   bash scripts/run_acceptance_tests.sh               # logs to ./test_logs/<date-time>
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${LOG_DIR:-$REPO/test_logs/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$LOG"
echo "logs -> $LOG"
"$REPO/go2_autonomy/build/go2_acceptance_tests" --log "$LOG" "$@" | tee "$LOG/console.log"
