#!/usr/bin/env bash
set -o pipefail

# Run a single paired A/B bench run with robust cleanup.
# Usage: ./scripts/run_single_bench.sh <label> <install_dir>
#   label: B6, B7, A8, A9, B10
#   install_dir: install_v020 (for A) or install (for B)

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_ROOT="${WS_ROOT:-$HOME/ros2_ws}"
PKG_DIR="$WS_ROOT/src/mpc_controller"
RECORD_SECONDS="${RECORD_SECONDS:-65}"

LABEL="$1"
INSTALL_DIR="$2"

if [[ -z "$LABEL" || -z "$INSTALL_DIR" ]]; then
  echo "Usage: $0 <label> <install_dir>"
  echo "  label: B6, B7, A8, A9, B10"
  echo "  install_dir: install_v020 or install"
  exit 1
fi

OUTPUT_DIR="$PKG_DIR/records/paired_ab"
mkdir -p "$OUTPUT_DIR"

echo "=== Run $LABEL: cleanup ==="
# Nuclear cleanup
pkill -9 -f "gz-server" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz_sim" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "controller_manager" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "trajectory_publisher" 2>/dev/null || true
pkill -9 -f "ros2.*launch" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "ros_gz_sim" 2>/dev/null || true
# Kill ros2 daemon to clear stale service cache
pkill -9 -f "ros2.*daemon" 2>/dev/null || true
sleep 3

# Verify clean
leftovers=$(ps aux | grep -E "gz|spawner|controller_manager|ros2.*launch" | grep -v grep | wc -l)
if [[ "$leftovers" -gt 0 ]]; then
  echo "  WARNING: $leftovers processes still running after cleanup"
  ps aux | grep -E "gz|spawner|controller_manager" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true
  sleep 2
fi

echo "=== Run $LABEL: sourcing $INSTALL_DIR ==="
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$WS_ROOT/$INSTALL_DIR/setup.bash" 2>/dev/null || true
set -u

export PATH="$PATH:/opt/ros/${ROS_DISTRO}/opt/gz_tools_vendor/bin"
export GZ_SIM_RESOURCE_PATH="$PKG_DIR/urdf"

cd "$WS_ROOT"

BAG_NAME="${OUTPUT_DIR}/${LABEL}_bag"
rm -rf "$BAG_NAME"

echo "=== Run $LABEL: launching simulation ==="
ros2 launch mpc_controller rrbot_mpc.launch.py > /tmp/mpc_rrbot_launch.log 2>&1 &
LAUNCH_PID=$!

ACTIVE=0
for i in $(seq 1 60); do
  sleep 1
  CTRL_STATE="$(ros2 control list_controllers 2>/dev/null || true)"
  if echo "$CTRL_STATE" | grep -qE '^mpc_controller[[:space:]].*active'; then
    echo "  MPC controller active after ${i}s"
    ACTIVE=1
    break
  fi
  if [[ $i -eq 30 ]]; then
    echo "  Still waiting... ($i/60)"
  fi
done

if [[ "$ACTIVE" -ne 1 ]]; then
  echo "  ERROR: MPC controller did not become active after 60s"
  cat /tmp/mpc_rrbot_launch.log | tail -20
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

echo "=== Run $LABEL: recording ${RECORD_SECONDS}s ==="
timeout "$((RECORD_SECONDS + 2))" \
  ros2 bag record \
  -o "$BAG_NAME" \
  /mpc_controller/diagnostics \
  > /tmp/mpc_bag_record.log 2>&1 || true

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "=== Run $LABEL: done, bag at $BAG_NAME ==="
