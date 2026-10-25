#!/usr/bin/env bash
# Step 1 -- system packages (the only step that needs sudo).
# Tested on Ubuntu 22.04 (gcc 11, cmake 3.22) and Ubuntu 24.04 (gcc 13, cmake 3.28).
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libyaml-cpp-dev libspdlog-dev libfmt-dev \
  libboost-program-options-dev \
  libglfw3-dev

# Optional: plotting the test logs / rendering replay videos (scripts in go2_autonomy/scripts)
#   sudo apt-get install -y python3-pip ffmpeg && pip3 install mujoco==3.3.6 imageio matplotlib

echo
echo "Dependencies installed. Next:  bash scripts/build.sh"
