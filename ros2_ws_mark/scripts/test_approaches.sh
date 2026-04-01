#!/bin/bash
# test_approaches.sh — Run A/B tests for all 5 obstacle navigation approaches.
#
# Usage:
#   bash scripts/test_approaches.sh [approach]   # Run specific approach (baseline, A, B, C, D)
#   bash scripts/test_approaches.sh all           # Run all 5 approaches sequentially
#   bash scripts/test_approaches.sh               # Default: run all

APPROACH="${1:-all}"
DURATION="${2:-400}"  # seconds per run (2 laps at 0.35 m/s)

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

run_approach() {
    local LABEL="$1"
    stage "═══ Running approach: $LABEL (${DURATION}s) ═══"

    # Kill everything
    pkill -9 -f "gz|ros2|ruby" 2>/dev/null || true
    sleep 3
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

    source install/setup.bash

    # 1. Launch sim
    info "Launching sim..."
    ros2 launch littleblue_sim sim.launch.py course:=autonav > /tmp/sim.log 2>&1 &
    SIM_PID=$!

    wait_for_topic "/scan" 45 || { fail "No /scan"; kill $SIM_PID 2>/dev/null; return 1; }
    wait_for_topic "/image_raw" 45 || { fail "No /image_raw"; kill $SIM_PID 2>/dev/null; return 1; }
    wait_for_topic "/odom" 15 || { fail "No /odom"; kill $SIM_PID 2>/dev/null; return 1; }
    info "Sim up."

    # 2. Launch fake lanes
    info "Launching fake_lanes..."
    python3 scripts/fake_lanes_autonav.py > /tmp/vision.log 2>&1 &
    VISION_PID=$!

    wait_for_topic "/lane_points" 15 || { fail "No /lane_points"; return 1; }
    wait_for_topic "/world_pose" 10 || { fail "No /world_pose"; return 1; }
    info "Fake lanes up."

    # 3. Launch autonomy with approach + recorder
    info "Launching autonomy (approach=$LABEL, record=true)..."
    ros2 launch littleblue_autonomy autonomy.launch.py approach:=$LABEL record:=true rviz:=true > /tmp/autonomy.log 2>&1 &
    AUTO_PID=$!

    wait_for_topic "/cmd_vel" 30 || { fail "No /cmd_vel"; return 1; }
    info "Autonomy up — robot driving."

    # 4. Monitor for duration
    local START_TIME=$(date +%s)
    local LAP=0
    local CROSSED_TOP=false

    while true; do
        local NOW=$(date +%s)
        local ELAPSED=$((NOW - START_TIME))
        if [ $ELAPSED -ge $DURATION ]; then
            info "Time limit reached (${ELAPSED}s)."
            break
        fi

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
            if [ $LAP -ge 2 ]; then
                echo ""
                info "$LABEL completed 2 laps!"
                break
            fi
        fi

        sleep 5
    done

    echo ""
    info "Stopping approach $LABEL (completed $LAP laps)."

    # Kill this run
    kill $AUTO_PID $VISION_PID $SIM_PID 2>/dev/null
    sleep 2
    pkill -9 -f "gz|ruby" 2>/dev/null || true
    sleep 2
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

# ── Main ──────────────────────────────────────────────────────────────
if [ "$APPROACH" = "all" ]; then
    APPROACHES="baseline A B C D"
else
    APPROACHES="$APPROACH"
fi

for a in $APPROACHES; do
    run_approach "$a"
    echo ""
done

# Analyze
stage "═══ Analysis ═══"
python3 scripts/analyze_runs.py

info "All done!"
