#!/bin/bash
# test_10_approaches.sh — Run all 10 NML navigation approaches and collect results.
#
# Usage:
#   bash scripts/test_10_approaches.sh [approach]   # Run specific approach
#   bash scripts/test_10_approaches.sh all           # Run all 10 sequentially
#   bash scripts/test_10_approaches.sh               # Default: run all

APPROACH="${1:-all}"
DURATION="${2:-300}"  # seconds per run (timeout)
TARGET_LAPS=2
RESULTS_DIR="/tmp/nml_results"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
CYN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GRN}[TEST]${NC} $1"; }
warn()  { echo -e "${YEL}[WARN]${NC} $1"; }
fail()  { echo -e "${RED}[FAIL]${NC} $1"; }
stage() { echo -e "${CYN}[====]${NC} $1"; }

# Clean results directory
mkdir -p "$RESULTS_DIR"

wait_for_topic() {
    local topic="$1"
    local timeout="$2"
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

kill_all() {
    pkill -9 -f "gz|ros2|ruby" 2>/dev/null || true
    sleep 3
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

run_approach() {
    local LABEL="$1"
    stage "═══ Running approach: $LABEL (timeout=${DURATION}s, target=${TARGET_LAPS} laps) ═══"

    # Kill everything from previous run
    kill_all

    source install/setup.bash

    # 1. Launch sim
    info "Launching sim..."
    ros2 launch littleblue_sim sim.launch.py course:=autonav > /tmp/sim_${LABEL}.log 2>&1 &
    SIM_PID=$!

    wait_for_topic "/odom" 60 || { fail "No /odom after 60s"; kill_all; return 1; }
    info "Sim up (odom available)."

    # 2. Launch fake lanes
    info "Launching fake_lanes..."
    python3 scripts/fake_lanes_autonav.py > /tmp/vision_${LABEL}.log 2>&1 &
    VISION_PID=$!

    wait_for_topic "/lane_points" 20 || { fail "No /lane_points after 20s"; kill_all; return 1; }
    wait_for_topic "/world_pose" 10 || { fail "No /world_pose after 10s"; kill_all; return 1; }
    info "Fake lanes up."

    # 3. Launch autonomy with approach
    info "Launching autonomy (approach=$LABEL)..."
    ros2 launch littleblue_autonomy autonomy.launch.py \
        approach:=$LABEL \
        results_dir:=$RESULTS_DIR \
        target_laps:=$TARGET_LAPS \
        rviz:=true \
        > /tmp/autonomy_${LABEL}.log 2>&1 &
    AUTO_PID=$!

    wait_for_topic "/cmd_vel" 30 || { fail "No /cmd_vel after 30s"; kill_all; return 1; }
    info "Autonomy up — robot driving."

    # 4. Monitor for duration or until results file appears
    local TIMER_START=$SECONDS
    local LAP=0
    local CROSSED_TOP=false
    local RESULTS_FILE="${RESULTS_DIR}/${LABEL}.json"

    while true; do
        local ELAPSED=$((SECONDS - TIMER_START))
        if [ $ELAPSED -ge $DURATION ]; then
            warn "Timeout reached (${ELAPSED}s) for approach $LABEL."
            break
        fi

        # Check if results file was written (safety monitor writes after target laps)
        if [ -f "$RESULTS_FILE" ]; then
            info "Results file detected for $LABEL — laps completed."
            sleep 5  # let safety monitor finish writing
            break
        fi

        # Track progress via world_pose
        POSE=$(timeout 3 ros2 topic echo /world_pose --once 2>/dev/null || true)
        if [ -z "$POSE" ]; then
            sleep 3
            continue
        fi

        X=$(echo "$POSE" | grep -A1 "position:" | grep "x:" | head -1 | awk '{print $2}' || echo "0")
        Y=$(echo "$POSE" | grep -A2 "position:" | grep "y:" | head -1 | awk '{print $2}' || echo "0")

        printf "\r[%s] t=%3ds  x=%7.2f  y=%7.2f  laps=%d" "$LABEL" "$ELAPSED" "$X" "$Y" "$LAP" 2>/dev/null || true

        if awk "BEGIN{exit(!($Y > 20.0))}" 2>/dev/null; then
            CROSSED_TOP=true
        fi
        if awk "BEGIN{exit(!($Y < 4.0))}" 2>/dev/null && [ "$CROSSED_TOP" = "true" ]; then
            LAP=$((LAP + 1))
            CROSSED_TOP=false
            printf "  [LAP %d]" "$LAP"
        fi

        sleep 5
    done

    echo ""
    info "Stopping approach $LABEL."

    # Send SIGINT first for clean shutdown (allows safety monitor to write results)
    kill -INT $AUTO_PID 2>/dev/null
    sleep 3

    # Force kill everything
    kill $AUTO_PID $VISION_PID $SIM_PID 2>/dev/null
    sleep 2
    kill_all
}

# ── Main ──────────────────────────────────────────────────────────────
if [ "$APPROACH" = "all" ]; then
    APPROACHES="baseline M1 M2 M3 M4 M5 A1 A2 A3 A4 A5"
else
    APPROACHES="$APPROACH"
fi

info "Approaches to test: $APPROACHES"
info "Results directory: $RESULTS_DIR"
echo ""

for a in $APPROACHES; do
    run_approach "$a"
    echo ""
done

# Analyze results
stage "═══ Results Summary ═══"
python3 scripts/analyze_nml_results.py "$RESULTS_DIR"

info "All done! Results in $RESULTS_DIR"
