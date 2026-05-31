#!/usr/bin/env bash
set -eo pipefail

# Paired A/B benchmark runner for MPC Controller.
# Alternates between v0.2.0 (A) and v0.2.1-rc1 (B) installs.
#
# Usage:
#   ./scripts/run_paired_bench.sh [--dry-run]
#
# Produces:
#   records/paired_ab/  — rosbag files labeled by version + run number
#   records/paired_ab/metrics.csv  — per-run aggregate metrics

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_ROOT="${WS_ROOT:-$HOME/ros2_ws}"
PKG_DIR="$WS_ROOT/src/mpc_controller"
RECORD_SECONDS="${RECORD_SECONDS:-65}"
DRY_RUN="${1:+false}"
[[ "$1" == "--dry-run" ]] && DRY_RUN=true

OUTPUT_DIR="$PKG_DIR/records/paired_ab"

# Sequence: alternating to avoid order bias
# A = v0.2.0 (install_v020), B = v0.2.1-rc1 (install)
SEQUENCE=(
  "A:install_v020"
  "B:install"
  "B:install"
  "A:install_v020"
  "A:install_v020"
  "B:install"
  "B:install"
  "A:install_v020"
  "A:install_v020"
  "B:install"
)

cleanup_all() {
  echo "=== Cleanup ==="
  # Aggressive kill: use pkill -f to match any process with these keywords
  pkill -9 -f "gz-server" 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "gz_sim" 2>/dev/null || true
  pkill -9 -f "robot_state_publisher" 2>/dev/null || true
  pkill -9 -f "controller_manager" 2>/dev/null || true
  pkill -9 -f "spawner" 2>/dev/null || true
  pkill -9 -f "trajectory_publisher" 2>/dev/null || true
  pkill -9 -f "ros2 launch.*rrbot" 2>/dev/null || true
  pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
  sleep 2
  # Verify nothing is left
  local leftovers
  leftovers=$(ps aux | grep -E "gz|spawner|controller_manager|trajectory_publisher" | grep -v grep | wc -l)
  if [[ "$leftovers" -gt 0 ]]; then
    echo "  WARNING: $leftovers processes still running after cleanup"
    ps aux | grep -E "gz|spawner|controller_manager" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true
    sleep 1
  fi
}

run_single() {
  local version="$1"        # "A" or "B"
  local install_dir="$2"    # "install" or "install_v020"
  local run_num="$3"        # 1-10
  local label="${version}${run_num}"

  echo ""
  echo "======================================================"
  echo "  Run $label: v0.2.${version} (install=$install_dir)"
  echo "======================================================"
  echo ""

  # Clean up from previous run
  cleanup_all

  # Source the right ROS2 workspace
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  source "$WS_ROOT/$install_dir/setup.bash" 2>/dev/null || true
  set -u

  export PATH="$PATH:/opt/ros/${ROS_DISTRO}/opt/gz_tools_vendor/bin"
  export GZ_SIM_RESOURCE_PATH="$PKG_DIR/urdf"

  cd "$WS_ROOT"

  BAG_NAME="${OUTPUT_DIR}/${label}_bag"

  if [[ "$DRY_RUN" == true ]]; then
    echo "[DRY RUN] Would:"
    echo "  Source: $WS_ROOT/$install_dir/setup.bash"
    echo "  Launch: ros2 launch mpc_controller rrbot_mpc.launch.py"
    echo "  Record: ros2 bag record -o $BAG_NAME /mpc_controller/diagnostics"
    echo "  Duration: ${RECORD_SECONDS}s"
    return 0
  fi

  rm -rf "$BAG_NAME"

  echo "Launching RRBot MPC simulation (v0.2.${version})..."
  ros2 launch mpc_controller rrbot_mpc.launch.py \
    > /tmp/mpc_rrbot_launch.log 2>&1 &
  LAUNCH_PID=$!

  ACTIVE=0
  for i in $(seq 1 45); do
    sleep 1
    CTRL_STATE="$(ros2 control list_controllers 2>/dev/null || true)"
    if echo "$CTRL_STATE" | grep -qE '^mpc_controller[[:space:]].*active'; then
      echo "  MPC controller active after ${i}s"
      ACTIVE=1
      break
    fi
    echo "  Waiting for controller... ${i}/45"
  done

  if [[ "$ACTIVE" -ne 1 ]]; then
    echo "  ERROR: MPC controller did not become active."
    kill "$LAUNCH_PID" 2>/dev/null || true
    return 1
  fi

  # Measure diagnostics rate
  timeout 8 ros2 topic hz /mpc_controller/diagnostics \
    > /tmp/mpc_diag_hz.log 2>&1 || true

  echo "  Recording ${RECORD_SECONDS}s to $BAG_NAME ..."
  timeout "$((RECORD_SECONDS + 2))" \
    ros2 bag record \
    -o "$BAG_NAME" \
    /mpc_controller/diagnostics \
    > /tmp/mpc_bag_record.log 2>&1 || true

  # Kill launch
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true

  echo "  Bag saved: $BAG_NAME"
}

mkdir -p "$OUTPUT_DIR"

echo "======================================================"
echo "  Paired A/B Benchmark: v0.2.0 (A) vs v0.2.1-rc1 (B)"
echo "  Sequence: ${SEQUENCE[*]}"
echo "  Output: $OUTPUT_DIR"
echo "======================================================"
echo ""

for i in "${!SEQUENCE[@]}"; do
  entry="${SEQUENCE[$i]}"
  version="${entry%%:*}"
  install_dir="${entry##*:}"
  run_num=$((i + 1))

  if ! run_single "$version" "$install_dir" "$run_num"; then
    echo "  Run $version$run_num failed. Continuing..."
  fi
done

echo ""
echo "======================================================"
echo "  All runs complete. Generating comparison report..."
echo "======================================================"

# Generate aggregate metrics from all bags
if [[ "$DRY_RUN" != true ]]; then
  echo "run,label,cycles,solve_mean_us,optimal_rate,deadline_miss_rate,rms_pos,hold_rate" > "$OUTPUT_DIR/metrics.csv"

  for bag_dir in "$OUTPUT_DIR"/*_bag/; do
    [[ -d "$bag_dir" ]] || continue
    label=$(basename "$bag_dir" | sed 's/_bag//')
    echo "  Processing $label ..."

    python3 "$PKG_DIR/scripts/benchmark_plot.py" \
      --bags "$bag_dir" \
      --quiet 2>/dev/null \
      | grep -E '(cycles|solve|optimal|deadline|rms|hold)' \
      | awk -v label="$label" '{print label","$0}' >> "$OUTPUT_DIR/metrics.csv" 2>/dev/null || true
  done

  echo "  Metrics saved to $OUTPUT_DIR/metrics.csv"
fi

# Final cleanup
cleanup_all

echo ""
echo "======================================================"
echo "  Done. Results in: $OUTPUT_DIR"
echo "======================================================"
