#!/bin/bash
# test_run.sh — Launch sim, vision, and autonomy with startup verification.
# Ensures all nodes are running and topics are publishing before autonomy starts.
#
# Usage:
#   bash scripts/test_run.sh                    # Autonav course, real vision
#   bash scripts/test_run.sh --fake             # Autonav course, fake lanes
#   bash scripts/test_run.sh --oval             # Oval course, real vision
#   bash scripts/test_run.sh --oval --fake      # Oval course, fake lanes

USE_FAKE=false
COURSE=autonav

for arg in "$@"; do
    case $arg in
        --fake)   USE_FAKE=true ;;
        --oval)   COURSE=oval ;;
        --autonav) COURSE=autonav ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

# Colors
RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GRN}[TEST]${NC} $1"; }
warn()  { echo -e "${YEL}[WARN]${NC} $1"; }
fail()  { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }

# ── Cleanup ──────────────────────────────────────────────────────────
cleanup() {
    info "Shutting down..."
    kill $SIM_PID $VISION_PID $AUTO_PID 2>/dev/null
    sleep 1
    pkill -9 -f "gz|ruby" 2>/dev/null
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    info "Done."
}
trap cleanup EXIT INT TERM

# ── Pre-flight ───────────────────────────────────────────────────────
info "Killing stale processes..."
pkill -9 -f "gz|ros2|ruby" 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

info "Sourcing workspace..."
source install/setup.bash

info "Course: $COURSE"

# ── Helper: wait for a topic to have data ────────────────────────────
wait_for_topic() {
    local topic="$1"
    local timeout="$2"
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
            echo -e "  ${GRN}✓${NC} $topic is publishing"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo -e "  ${RED}✗${NC} $topic not found after ${timeout}s"
    return 1
}

# ── Helper: wait for a node to appear ────────────────────────────────
wait_for_node() {
    local node="$1"
    local timeout="$2"
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            echo -e "  ${GRN}✓${NC} $node is running"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo -e "  ${RED}✗${NC} $node not found after ${timeout}s"
    return 1
}

# ══════════════════════════════════════════════════════════════════════
# STAGE 1: Launch Simulation
# ══════════════════════════════════════════════════════════════════════
info "STAGE 1: Launching simulation (course=$COURSE)..."
ros2 launch littleblue_sim sim.launch.py course:=$COURSE > /tmp/sim.log 2>&1 &
SIM_PID=$!

info "Waiting for sim topics..."
wait_for_topic "/scan"      45 || fail "Sim failed — no /scan topic"
wait_for_topic "/odom"      15 || fail "Sim failed — no /odom topic"
wait_for_topic "/image_raw" 45 || fail "Sim failed — no /image_raw topic"
wait_for_topic "/clock"     15 || fail "Sim failed — no /clock topic"
info "Sim is up."

# ══════════════════════════════════════════════════════════════════════
# STAGE 2: Launch Vision (or fake_lanes)
# ══════════════════════════════════════════════════════════════════════
if [ "$USE_FAKE" = "true" ]; then
    if [ "$COURSE" = "autonav" ]; then
        FAKE_SCRIPT="$WS_DIR/scripts/fake_lanes_autonav.py"
    else
        FAKE_SCRIPT="$WS_DIR/scripts/fake_lanes.py"
    fi
    info "STAGE 2: Launching fake_lanes ($FAKE_SCRIPT)..."
    python3 "$FAKE_SCRIPT" > /tmp/vision.log 2>&1 &
    VISION_PID=$!

    info "Waiting for lane topics..."
    wait_for_topic "/lane_points" 15 || fail "fake_lanes failed — no /lane_points"
    wait_for_topic "/world_pose"  10 || fail "fake_lanes failed — no /world_pose"
    info "fake_lanes is up."
else
    info "STAGE 2: Launching vision pipeline..."
    ros2 launch littleblue_vision vision.launch.py > /tmp/vision.log 2>&1 &
    VISION_PID=$!

    info "Waiting for vision nodes..."
    wait_for_node "/lane_detector_node" 15 || fail "Vision failed — lane_detector_node not running"
    wait_for_topic "/lane_points" 20 || fail "Vision failed — no /lane_points"
    info "Vision is up."

    # Check obstacle_points (may not publish until YOLO detections arrive)
    wait_for_topic "/obstacle_points" 5 || warn "/obstacle_points not yet publishing (needs YOLO detections)"
fi

# Verify lane data is actually flowing
info "Verifying lane data flow..."
LANE_HZ=$(timeout 5 ros2 topic hz /lane_points --window 3 2>&1 | grep "average rate" | head -1 | awk '{print $3}' || true)
if [ -z "$LANE_HZ" ]; then
    warn "Could not measure /lane_points rate — continuing anyway"
else
    info "/lane_points rate: ${LANE_HZ} Hz"
fi

# ══════════════════════════════════════════════════════════════════════
# STAGE 3: Launch Autonomy
# ══════════════════════════════════════════════════════════════════════
info "STAGE 3: Launching autonomy..."
ros2 launch littleblue_autonomy autonomy.launch.py > /tmp/autonomy.log 2>&1 &
AUTO_PID=$!

info "Waiting for autonomy node..."
wait_for_topic "/cmd_vel" 30 || fail "Autonomy not publishing /cmd_vel"
info "Autonomy is up."

# ══════════════════════════════════════════════════════════════════════
# STAGE 4: Monitor
# ══════════════════════════════════════════════════════════════════════
echo ""
info "═══════════════════════════════════════════════════════"
info "  ALL NODES RUNNING — Robot is driving ($COURSE course)"
info "═══════════════════════════════════════════════════════"
echo ""
info "Monitoring robot position (Ctrl+C to stop)..."
echo ""

LAP=0
CROSSED_TOP=false

# Lap detection thresholds depend on course
if [ "$COURSE" = "autonav" ]; then
    LAP_TOP_THRESHOLD=20.0
    LAP_BOTTOM_THRESHOLD=4.0
else
    LAP_TOP_THRESHOLD=8.0
    LAP_BOTTOM_THRESHOLD=2.0
fi

while true; do
    # Get position — try world_pose first, fall back to odom
    POSE=$(timeout 3 ros2 topic echo /world_pose --once 2>/dev/null || true)
    if [ -z "$POSE" ]; then
        POSE=$(timeout 3 ros2 topic echo /odom --once 2>/dev/null || true)
    fi
    if [ -z "$POSE" ]; then
        warn "No position data"
        sleep 2
        continue
    fi

    X=$(echo "$POSE" | grep -A1 "position:" | grep "x:" | head -1 | awk '{print $2}' || echo "0")
    Y=$(echo "$POSE" | grep -A2 "position:" | grep "y:" | head -1 | awk '{print $2}' || echo "0")

    if [ -z "$X" ] || [ -z "$Y" ]; then
        sleep 2
        continue
    fi

    # Format position
    printf "[POS] x=%7.2f  y=%7.2f" "$X" "$Y" 2>/dev/null || true

    # Lap detection
    if awk "BEGIN{exit(!($Y > $LAP_TOP_THRESHOLD))}" 2>/dev/null; then
        CROSSED_TOP=true
    fi
    if awk "BEGIN{exit(!($Y < $LAP_BOTTOM_THRESHOLD))}" 2>/dev/null && [ "$CROSSED_TOP" = "true" ]; then
        LAP=$((LAP + 1))
        CROSSED_TOP=false
        printf "  [LAP %d COMPLETE]" "$LAP"
    fi

    # Get cmd_vel
    CMD=$(timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null || true)
    if [ -n "$CMD" ]; then
        VX=$(echo "$CMD" | grep -A1 "linear:" | grep "x:" | head -1 | awk '{print $2}' || echo "0")
        AZ=$(echo "$CMD" | grep -A1 "angular:" | grep "z:" | head -1 | awk '{print $2}' || echo "0")
        printf "  cmd: vx=%.2f az=%.2f" "$VX" "$AZ" 2>/dev/null || true
    fi

    echo ""
    sleep 3
done
