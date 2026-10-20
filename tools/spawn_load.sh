#!/bin/bash
# Start N load_node processes (one DDS participant each) and stop them all on Ctrl-C.
#
#   tools/spawn_load.sh N [load_node ros-args...]
#   tools/spawn_load.sh 10 -p topics:=5 -p publish:=false
#
# Needs a sourced ROS environment, or run it inside the image:
#   docker run --rm -it --network host --ipc host -v "$PWD/tools:/tools" mesh-ping /tools/spawn_load.sh 10
set -euo pipefail

n=${1:?usage: spawn_load.sh N [-p param:=value ...]}
shift

if command -v ros2 >/dev/null && ros2 pkg prefix mesh_ping >/dev/null 2>&1; then
  bin="$(ros2 pkg prefix mesh_ping)/lib/mesh_ping/load_node"
else
  bin=/ws/install/mesh_ping/lib/mesh_ping/load_node
fi

pids=()
trap 'kill -INT "${pids[@]}" 2>/dev/null; wait' INT TERM EXIT

# Foxy's logger races on creating this directory when processes start together, and the
# losers abort. Create it up front, and stagger the starts.
mkdir -p "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"

host=$(hostname | tr -c 'a-zA-Z0-9\n' _)
for i in $(seq 1 "$n"); do
  sleep 0.2
  # Only the first process logs, so N nodes don't flood the terminal.
  if [ "$i" -eq 1 ]; then
    "$bin" --ros-args -r __node:="load_${host}_$i" "$@" &
  else
    "$bin" --ros-args -r __node:="load_${host}_$i" "$@" >/dev/null 2>&1 &
  fi
  pids+=($!)
done
echo "spawned $n load_node processes; Ctrl-C to stop"
wait
