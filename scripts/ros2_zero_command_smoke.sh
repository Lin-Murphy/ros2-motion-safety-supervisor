#!/usr/bin/env bash
# Verify the ROS2 adapter's missing-odometry failure path on isolated topics.
# This script never uses /cmd_vel and must not be connected to a motor driver.

set -euo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly CANDIDATE_TOPIC="/cmd_vel_guardrail_smoke_candidate"
readonly SAFE_TOPIC="/cmd_vel_guardrail_smoke_safe"
readonly ODOM_TOPIC="/odom_guardrail_smoke_missing"
readonly OUTPUT_DIR="$PROJECT_ROOT/reports/ros2_zero_command_smoke_$(date +%Y%m%d_%H%M%S)"
readonly EVENT_PATH="$OUTPUT_DIR/runtime_episode.json"
readonly SAFE_CAPTURE="$OUTPUT_DIR/safe_twist.yaml"

if [[ "$SAFE_TOPIC" == "/cmd_vel" || "$CANDIDATE_TOPIC" == "/cmd_vel" ]]; then
  echo "Refusing to use /cmd_vel for a smoke test." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null; then
  echo "ROS2 is not sourced. Source the robot's ROS2 environment first." >&2
  exit 1
fi

if ! python3 -c 'import rclpy; from geometry_msgs.msg import Twist; from nav_msgs.msg import Odometry'; then
  echo "The selected Python cannot import rclpy, geometry_msgs, and nav_msgs." >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"
cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

node_pid=""
echo_pid=""
cleanup() {
  [[ -n "$echo_pid" ]] && kill "$echo_pid" 2>/dev/null || true
  [[ -n "$node_pid" ]] && kill "$node_pid" 2>/dev/null || true
  [[ -n "$echo_pid" ]] && wait "$echo_pid" 2>/dev/null || true
  [[ -n "$node_pid" ]] && wait "$node_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 -m raspbot_guardrail.ros2_node --ros-args \
  -p candidate_topic:="$CANDIDATE_TOPIC" \
  -p safe_topic:="$SAFE_TOPIC" \
  -p odom_topic:="$ODOM_TOPIC" \
  -p event_path:="$EVENT_PATH" &
node_pid="$!"

sleep 2
echo "Check the following graph before continuing:"
echo "- only motion_safety_supervisor should subscribe to $CANDIDATE_TOPIC"
echo "- no motor-driver node may subscribe to $SAFE_TOPIC"
ros2 topic info -v "$CANDIDATE_TOPIC"
ros2 topic info -v "$SAFE_TOPIC"
read -r -p "Type ISOLATED to send one nonzero candidate only to these isolated topics: " confirmation
if [[ "$confirmation" != "ISOLATED" ]]; then
  echo "Stopped before publishing the candidate command."
  exit 1
fi

timeout 10s ros2 topic echo --once "$SAFE_TOPIC" geometry_msgs/msg/Twist > "$SAFE_CAPTURE" &
echo_pid="$!"
sleep 1

# No odometry is published on the configured odom topic. The only valid result
# is RISK_UNKNOWN and a zero Twist on the isolated safe topic.
ros2 topic pub --once "$CANDIDATE_TOPIC" geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
wait "$echo_pid"
echo_pid=""

for _ in $(seq 1 20); do
  [[ -f "$EVENT_PATH" ]] && break
  sleep 0.1
done

python3 - "$EVENT_PATH" "$SAFE_CAPTURE" <<'PY'
import json
import re
import sys
from pathlib import Path

event_path = Path(sys.argv[1])
safe_capture = Path(sys.argv[2])
if not event_path.is_file():
    raise SystemExit(f"missing episode output: {event_path}")

episode = json.loads(event_path.read_text(encoding="utf-8"))
evidence = episode["metadata"]["execution_evidence"]
if evidence["supervisor_decision"] != "RISK_UNKNOWN":
    raise SystemExit(f"expected RISK_UNKNOWN, got {evidence['supervisor_decision']}")
commands = evidence["backend_accepted_commands"]
if len(commands) != 1 or any(commands[0][key] != 0.0 for key in ("linear_x", "linear_y", "angular_z")):
    raise SystemExit(f"expected exactly one accepted zero hold, got {commands!r}")

values = re.findall(r"^\s*[xyz]:\s*([^\s]+)\s*$", safe_capture.read_text(encoding="utf-8"), re.MULTILINE)
if len(values) != 6 or any(float(value) != 0.0 for value in values):
    raise SystemExit(f"expected a zero Twist on safe topic, got {values!r}")

print("PASS: missing odometry produced RISK_UNKNOWN and one isolated zero Twist.")
print(f"episode={event_path}")
print(f"safe_twist={safe_capture}")
PY
