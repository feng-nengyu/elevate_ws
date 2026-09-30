#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# This shell may already have sourced the other workspace via .bashrc.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"

# Separate outputs prevent stale paths from the other elevate_ws workspace.
colcon --log-base "$workspace/log_wzl" build \
  --base-paths "$workspace/src" \
  --build-base "$workspace/build_wzl" \
  --install-base "$workspace/install_wzl" \
  --symlink-install \
  "$@"
