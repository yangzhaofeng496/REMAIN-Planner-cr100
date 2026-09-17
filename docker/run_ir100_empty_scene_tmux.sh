#!/usr/bin/env bash
set -euo pipefail

# Unified tmux launcher for the IR100 empty-scene whole-body planner.
# The planner runs inside the existing remani_pcd_demo container while tmux
# keeps the ROS master, planner, timing log, and command window organized.

CONTAINER="${IR100_CONTAINER:-ir100_goal_adapter}"
SESSION="${IR100_TMUX_SESSION:-ir100_empty}"
MASTER_URI="${IR100_MASTER_URI:-http://127.0.0.1:11312}"
MASTER_PORT="${MASTER_URI##*:}"
WORKSPACE="${IR100_WORKSPACE:-/opt/remani_ws}"
LOG_FILE="${IR100_LOG_FILE:-/tmp/ir100_empty_scene_tmux.log}"
DISPLAY_VALUE="${DISPLAY:-:0}"

container_bash() {
  docker exec -e ROS_MASTER_URI="$MASTER_URI" -e DISPLAY="$DISPLAY_VALUE" \
    "$CONTAINER" bash -lc "$1"
}

require_container() {
  if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
    echo "Container is not running: $CONTAINER" >&2
    echo "Start it first, or set IR100_CONTAINER to the correct container." >&2
    exit 1
  fi
}

planner_command() {
  printf "export ROS_MASTER_URI='%s'; source /opt/ros/noetic/setup.bash; source '%s/devel/setup.bash'; roslaunch ir100_goal_adapter empty_scene_goal_planning.launch" \
    "$MASTER_URI" "$WORKSPACE"
}

start() {
  require_container

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session already exists: $SESSION"
    echo "Use '$0 attach' or '$0 stop' first."
    exit 1
  fi

  # Clear only the ROS nodes belonging to this test master, if any.
  container_bash "source /opt/ros/noetic/setup.bash; rosnode kill /ir100_goal_adapter /remani_planner_node /mm_controller_node /model_vis /fake_mm /robot_state_publisher /rviz 2>/dev/null || true"
  container_bash ": > '$LOG_FILE'"

  tmux new-session -d -s "$SESSION" -n master
  tmux send-keys -t "$SESSION:master" \
    "docker exec -it -e ROS_MASTER_URI='$MASTER_URI' -e DISPLAY='$DISPLAY_VALUE' '$CONTAINER' bash -lc 'source /opt/ros/noetic/setup.bash; roscore -p '$MASTER_PORT' 2>&1 | tee '$LOG_FILE''" C-m

  tmux new-window -t "$SESSION" -n planner
  tmux send-keys -t "$SESSION:planner" \
    "sleep 3; docker exec -it -e ROS_MASTER_URI='$MASTER_URI' -e DISPLAY='$DISPLAY_VALUE' '$CONTAINER' bash -lc \"$(planner_command) 2>&1 | tee -a '$LOG_FILE'\"" C-m

  tmux new-window -t "$SESSION" -n timing
  tmux send-keys -t "$SESSION:timing" \
    "docker exec -it -e ROS_MASTER_URI='$MASTER_URI' '$CONTAINER' bash -lc \"tail -F '$LOG_FILE' | grep --line-buffered -E 'PLAN_TIME|ee-goal solved|send trajectory|final high-precision|TEST_PASS'\"" C-m

  tmux new-window -t "$SESSION" -n commands
  tmux send-keys -t "$SESSION:commands" \
    "echo '发送点位: $0 send X Y Z'; echo '查看状态: $0 status'; echo '查看完整日志: $0 log'; echo '停止: $0 stop'; echo; exec bash" C-m

  tmux select-window -t "$SESSION:planner"
  echo "Started tmux session: $SESSION"
  echo "Attach with: $0 attach"
}

attach() {
  tmux attach-session -t "$SESSION"
}

stop() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
  fi
  if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
    container_bash "source /opt/ros/noetic/setup.bash; rosnode kill /ir100_goal_adapter /remani_planner_node /mm_controller_node /model_vis /fake_mm /robot_state_publisher /rviz 2>/dev/null || true"
  fi
  echo "Stopped tmux session and IR100 planning nodes."
}

status() {
  require_container
  echo "tmux:"
  tmux list-sessions 2>/dev/null | grep -F "$SESSION:" || echo "  session not running"
  echo "ROS nodes ($MASTER_URI):"
  container_bash "source /opt/ros/noetic/setup.bash; rosnode list 2>/dev/null || true"
}

send_point() {
  require_container
  if [[ $# -ne 3 ]]; then
    echo "Usage: $0 send X Y Z" >&2
    exit 2
  fi
  for value in "$@"; do
    [[ "$value" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]] || { echo "Invalid coordinate: $value" >&2; exit 2; }
  done
  container_bash "source /opt/ros/noetic/setup.bash; rostopic pub -1 /ir100/end_effector_target geometry_msgs/PoseStamped '{header: {frame_id: world}, pose: {position: {x: $1, y: $2, z: $3}, orientation: {w: 1.0}}}'"
}

log() {
  require_container
  container_bash "tail -n 200 '$LOG_FILE'"
}

usage() {
  cat <<EOF
Usage: $0 {start|attach|stop|status|send X Y Z|log}

Environment overrides:
  IR100_CONTAINER=$CONTAINER
  IR100_TMUX_SESSION=$SESSION
  IR100_MASTER_URI=$MASTER_URI
  IR100_WORKSPACE=$WORKSPACE
  IR100_LOG_FILE=$LOG_FILE
EOF
}

case "${1:-}" in
  start) shift; start "$@" ;;
  attach) shift; attach "$@" ;;
  stop) shift; stop "$@" ;;
  status) shift; status "$@" ;;
  send) shift; send_point "$@" ;;
  log) shift; log "$@" ;;
  *) usage; exit 2 ;;
esac
