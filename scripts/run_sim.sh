#!/usr/bin/env bash
# Terminal 1 -- start the MuJoCo simulator with the Go2, the obstacle arena and
# the simulated range scanner (DDS domain 1, interface lo; see unitree_mujoco/simulate/config.yaml).
#
#   bash scripts/run_sim.sh                       # obstacle arena (scene_nav.xml)
#   bash scripts/run_sim.sh -s scene.xml          # any other Go2 scene
#   STOCK=1 bash scripts/run_sim.sh               # original unitree_mujoco (no scanner)
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${STOCK:-0}" == "1" ]]; then
  cd "$REPO/unitree_mujoco/simulate/build"          # must run from here: it reads ../config.yaml
  exec ./unitree_mujoco "$@"                         # default scene from config.yaml
fi
cd "$REPO/unitree_mujoco/simulate/build_nav"        # must run from here: it reads ../config.yaml
if [[ $# -eq 0 ]]; then set -- -s scene_nav.xml; fi
exec ./unitree_mujoco_nav "$@"
