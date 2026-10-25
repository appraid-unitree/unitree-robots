#!/usr/bin/env bash
# Step 2 -- build everything from this repository. No sudo, nothing is installed
# outside the repo: unitree_sdk2 is installed into <repo>/.install/unitree_sdk2,
# so an existing /opt/unitree_robotics installation is neither needed nor touched.
#
#   bash scripts/build.sh            # build all
#   JOBS=2 bash scripts/build.sh     # limit parallel jobs (default: number of CPUs)
#   CXX=g++-11 bash scripts/build.sh # choose a compiler
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-$(nproc)}"
SDK_PREFIX="$REPO/.install/unitree_sdk2"
CMAKE_COMMON=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$SDK_PREFIX")

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

step "1/5 unitree_sdk2  ->  $SDK_PREFIX"
cmake -S "$REPO/unitree_sdk2" -B "$REPO/unitree_sdk2/build" \
      -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX" -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/unitree_sdk2/build" -j"$JOBS"
cmake --install "$REPO/unitree_sdk2/build" > /dev/null

step "2/5 unitree_mujoco (stock simulator)"
cmake -S "$REPO/unitree_mujoco/simulate" -B "$REPO/unitree_mujoco/simulate/build" "${CMAKE_COMMON[@]}"
cmake --build "$REPO/unitree_mujoco/simulate/build" -j"$JOBS" --target unitree_mujoco

step "3/5 unitree_mujoco_nav (simulator + range scanner)"
cmake -S "$REPO/unitree_mujoco/simulate/nav_ext" -B "$REPO/unitree_mujoco/simulate/build_nav" "${CMAKE_COMMON[@]}"
cmake --build "$REPO/unitree_mujoco/simulate/build_nav" -j"$JOBS"

step "4/5 go2_autonomy (navigation + locomotion + apps)"
cmake -S "$REPO/go2_autonomy" -B "$REPO/go2_autonomy/build" "${CMAKE_COMMON[@]}" ${GO2_CMAKE_ARGS:-}
cmake --build "$REPO/go2_autonomy/build" -j"$JOBS"

step "5/5 examples/stand_go2_3step"
cmake -S "$REPO/examples/stand_go2_3step" -B "$REPO/examples/stand_go2_3step/build" "${CMAKE_COMMON[@]}"
cmake --build "$REPO/examples/stand_go2_3step/build" -j"$JOBS"

step "Self-test (offline, no simulator needed)"
"$REPO/go2_autonomy/build/test_planner"

cat <<EOF

$(printf '\033[1;32m')Build complete.$(printf '\033[0m')
  Terminal 1:  bash scripts/run_sim.sh
  Terminal 2:  bash scripts/run_nav_demo.sh            # robot walks A -> B around obstacles
          or:  bash scripts/run_acceptance_tests.sh    # the 8 acceptance tests
EOF
