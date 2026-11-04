#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace_dir"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+

echo "Build complete. Run: source $workspace_dir/install/setup.bash"
