#!/usr/bin/env bash
# Run from a desktop terminal. This displays the static map; it does not drive Carter.
set -eo pipefail
source /opt/ros/humble/setup.bash
nav_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_LOG_DIR="$nav_dir/logs"
mkdir -p "$ROS_LOG_DIR"
if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  echo 'Missing Nav2. Install from a terminal:'
  echo 'sudo apt update && sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox'
  exit 1
fi
map_pid=''
rviz_pid=''
cleanup() {
  if [[ -n "$rviz_pid" ]]; then kill "$rviz_pid" 2>/dev/null || true; fi
  if [[ -n "$map_pid" ]]; then kill "$map_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=warehouse_preview_map_server \
  -p yaml_filename:="$nav_dir/../warehouse_map__n2duzc9/map.yaml" \
  -p use_sim_time:=false >"$ROS_LOG_DIR/map_server.log" 2>&1 &
map_pid=$!
ready=false
for attempt in {1..20}; do
  if ! kill -0 "$map_pid" 2>/dev/null; then
    cat "$ROS_LOG_DIR/map_server.log"
    exit 1
  fi
  if timeout 2 ros2 lifecycle get /warehouse_preview_map_server 2>/dev/null | grep -q unconfigured; then
    ready=true
    break
  fi
  sleep 0.5
done
if [[ "$ready" != true ]]; then
  echo "Map server did not become ready. See $ROS_LOG_DIR/map_server.log"
  exit 1
fi
ros2 lifecycle set /warehouse_preview_map_server configure
ros2 lifecycle set /warehouse_preview_map_server activate
rviz2 -d "$nav_dir/warehouse_map.rviz" &
rviz_pid=$!
wait "$rviz_pid"
