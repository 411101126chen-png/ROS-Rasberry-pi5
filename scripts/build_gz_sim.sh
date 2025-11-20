#!/usr/bin/env bash
#
# Build Gazebo Sim (gz-sim8) from source into a separate colcon workspace.
# Usage:
#   ./scripts/build_gz_sim.sh [<workspace_dir>] [<branch/tag>]
# Defaults: workspace_dir=$HOME/gz_ws, branch=gz-sim8

set -euo pipefail

WORKSPACE_DIR=${1:-"$HOME/gz_ws"}
GZ_BRANCH=${2:-"gz-sim8"}
REPOS_FILE="gz-sim.repos"

echo "[gazebo] Workspace: $WORKSPACE_DIR"
mkdir -p "$WORKSPACE_DIR/src"

echo "[gazebo] Installing build dependencies via apt..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    qtbase5-dev

cd "$WORKSPACE_DIR"

if [[ ! -f "$REPOS_FILE" ]]; then
    echo "[gazebo] Fetching $GZ_BRANCH repo list..."
    wget "https://raw.githubusercontent.com/gazebosim/gz-sim/${GZ_BRANCH}/${GZ_BRANCH}.repos" -O "$REPOS_FILE"
fi

echo "[gazebo] Importing repositories..."
vcs import src < "$REPOS_FILE"

echo "[gazebo] Resolving rosdep dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "[gazebo] Building with colcon..."
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "[gazebo] Build finished. Source the workspace with:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
