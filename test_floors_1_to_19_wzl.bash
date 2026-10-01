#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /home/xie/elevate_ws/install/setup.bash
source "$workspace/install_wzl/local_setup.bash"
expected="$workspace/install_wzl/piper_pbvs_control"
actual="$(ros2 pkg prefix piper_pbvs_control)"
if [[ "$actual" != "$expected" ]]; then
  echo "加载了错误的控制包: $actual" >&2
  exit 1
fi
exec python3 "$workspace/scripts/test_floors_1_to_19_wzl.py" "$@"
