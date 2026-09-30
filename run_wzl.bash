#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$workspace/install_wzl/setup.bash" ]]; then
  echo "未找到 wzl 的构建结果；先运行 bash $workspace/build_wzl.bash" >&2
  exit 1
fi

# The camera driver is presently available only in the original workspace.
# Load it as an underlay, then put the newly built wzl packages first.
source /home/xie/elevate_ws/install/setup.bash
source "$workspace/install_wzl/local_setup.bash"
set -u
expected="$workspace/install_wzl/piper_pbvs_control"
actual="$(ros2 pkg prefix piper_pbvs_control)"
if [[ "$actual" != "$expected" ]]; then
  echo "加载了错误的 piper_pbvs_control: $actual" >&2
  echo "请在新终端运行此脚本，避免旧工作区环境残留。" >&2
  exit 1
fi

cd "$workspace"
echo "工作区: $workspace"
echo "控制包: $actual"
echo "默认 dry-run；真机参数需在命令行明确指定。"
exec ros2 launch piper_launch all.launch.py \
  model_path:="$workspace/best.pt" \
  auto_enable:=false \
  enable_motion:=false \
  "$@"
