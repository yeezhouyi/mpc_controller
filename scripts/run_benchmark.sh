#!/usr/bin/env bash
# Note: we don't use 'u' flag because ROS2 setup.bash may reference
# unbound variables in certain environments. Use explicit checks instead.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_ROOT="${WS_ROOT:-$HOME/ros2_ws}"
PKG_DIR="$WS_ROOT/src/mpc_controller"
BAG_NAME="${BAG_NAME:-mpc_bench_bag}"
RECORD_SECONDS="${RECORD_SECONDS:-60}"

# Source ROS2 with +u to handle unbound variables in setup scripts
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true
set -u

export PATH="$PATH:/opt/ros/${ROS_DISTRO}/opt/gz_tools_vendor/bin"
export GZ_SIM_RESOURCE_PATH="$PKG_DIR/urdf"

cd "$WS_ROOT"
rm -rf "$BAG_NAME"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  # Kill any lingering Gazebo and controller_manager processes so the
  # next run starts clean (they don't get cleaned up by the launch kill).
  killall -9 gz-server gz_sim robot_state_publisher 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching RRBot MPC simulation..."
ros2 launch mpc_controller rrbot_mpc.launch.py \
  > /tmp/mpc_rrbot_launch.log 2>&1 &
LAUNCH_PID=$!

ACTIVE=0
for i in $(seq 1 45); do
  sleep 1
  CTRL_STATE="$(ros2 control list_controllers 2>/dev/null || true)"

  if echo "$CTRL_STATE" | grep -qE '^mpc_controller[[:space:]].*active'; then
    echo "MPC controller active after ${i}s"
    ACTIVE=1
    break
  fi

  echo "Waiting for controller... ${i}/45"
done

if [[ "$ACTIVE" -ne 1 ]]; then
  echo "ERROR: MPC controller did not become active."
  echo "Inspect /tmp/mpc_rrbot_launch.log"
  exit 1
fi

echo "Checking diagnostics..."
timeout 5 ros2 topic echo /mpc_controller/diagnostics --once \
  > /tmp/mpc_first_diag.log

echo "Measuring diagnostics rate..."
timeout 8 ros2 topic hz /mpc_controller/diagnostics \
  > /tmp/mpc_diag_hz.log 2>&1 || true

TOPICS=(
  /mpc_controller/diagnostics
  /joint_states
  /clock
)

if ros2 topic list | grep -qx '/diagnostics'; then
  TOPICS+=(/diagnostics)
fi

echo "Recording ${RECORD_SECONDS}s rosbag..."
timeout "$((RECORD_SECONDS + 2))" \
  ros2 bag record \
  -o "$BAG_NAME" \
  "${TOPICS[@]}" \
  > /tmp/mpc_bag_record.log 2>&1 || true

echo "=== Bag Info ==="
ros2 bag info "$BAG_NAME"

echo "=== Diagnostics Topic Rate ==="
cat /tmp/mpc_diag_hz.log || true

echo "=== Generate Benchmark Plots ==="
python3 "$PKG_DIR/scripts/benchmark_plot.py" \
  --bags "$BAG_NAME" \
  --plot \
  --output "$PKG_DIR/screenshots"

echo "Benchmark complete."
