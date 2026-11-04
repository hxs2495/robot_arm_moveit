#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u

exec ros2 launch simple_arm_gazebo gazebo_moveit.launch.py use_gui:=true "$@"
