#!/bin/bash
# test_lane_detectors.sh — Benchmark all lane detection methods against fake_lanes ground truth.
#
# Usage:
#   bash scripts/test_lane_detectors.sh [method]     # Run specific method
#   bash scripts/test_lane_detectors.sh all           # Run all methods
#   bash scripts/test_lane_detectors.sh               # Default: run all
#
# Each method runs for DURATION seconds while the benchmark scorer compares
# the candidate's /lane_points against ground truth /benchmark/lane_points.

METHOD="${1:-all}"
DURATION="${2:-180}"  # seconds per method
RESULTS_DIR="/tmp/lane_benchmark"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
CYN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GRN}[BENCH]${NC} $1"; }
warn()  { echo -e "${YEL}[WARN]${NC} $1"; }
fail()  { echo -e "${RED}[FAIL]${NC} $1"; }
stage() { echo -e "${CYN}[====]${NC} $1"; }

ALL_METHODS="hsv_threshold brightness canny_hough adaptive lab_lightness saturation_filter sobel_gradient combined_vote morpho_skeleton clahe_enhanced"

rm -rf "$RESULTS_DIR"
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
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "ruby" 2>/dev/null
    pkill -9 -f "ros2" 2>/dev/null
    pkill -9 -f "fake_lanes" 2>/dev/null
    pkill -9 -f "lane_candidate" 2>/dev/null
    pkill -9 -f "lane_benchmark" 2>/dev/null
    sleep 3
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
}

run_method() {
    local LABEL="$1"
    stage "═══ Testing method: $LABEL (${DURATION}s) ═══"

    kill_all
    source install/setup.bash

    # 1. Launch sim
    info "Launching sim..."
    ros2 launch littleblue_sim sim.launch.py course:=autonav > /tmp/sim_bench.log 2>&1 &
    SIM_PID=$!
    wait_for_topic "/odom" 60 || { fail "No /odom"; kill_all; return 1; }
    info "Sim up."

    # 2. Launch fake_lanes (publishes benchmark + normal lane_points + world_pose)
    info "Launching fake_lanes (benchmark publisher)..."
    python3 scripts/fake_lanes_autonav.py > /tmp/fake_lanes_bench.log 2>&1 &
    LANES_PID=$!
    wait_for_topic "/benchmark/lane_points" 15 || { fail "No benchmark topic"; kill_all; return 1; }
    wait_for_topic "/world_pose" 10 || { fail "No /world_pose"; kill_all; return 1; }
    info "Fake lanes up."

    # 3. Launch candidate detector
    info "Launching candidate detector (method=$LABEL)..."
    ros2 run littleblue_vision lane_candidate_node --ros-args \
        -p method:=$LABEL \
        -p use_sim_time:=true \
        -p image_topic:=/left_camera/image_raw \
        -p camera_lateral_offset:=0.28 \
        > /tmp/candidate_${LABEL}.log 2>&1 &
    CAND_PID=$!

    # 4. Launch benchmark scorer
    info "Launching benchmark scorer..."
    ros2 run littleblue_vision lane_benchmark_node --ros-args \
        -p use_sim_time:=true \
        -p detector_name:=$LABEL \
        -p results_dir:=$RESULTS_DIR \
        > /tmp/scorer_${LABEL}.log 2>&1 &
    SCORE_PID=$!

    # Wait for candidate to start publishing
    wait_for_topic "/candidate/lane_points" 30 || { warn "Candidate not publishing — check camera"; }
    wait_for_topic "/benchmark/lane_points" 10 || { warn "Benchmark not publishing"; }

    # 5. Launch autonomy (drives the robot around the track)
    info "Launching autonomy (drives robot)..."
    ros2 launch littleblue_autonomy autonomy.launch.py approach:=A rviz:=true \
        > /tmp/autonomy_bench.log 2>&1 &
    AUTO_PID=$!

    wait_for_topic "/cmd_vel" 30 || { fail "No /cmd_vel"; kill_all; return 1; }
    info "All up. Running for ${DURATION}s..."

    # 6. Wait
    local TIMER_START=$SECONDS
    while true; do
        local ELAPSED=$((SECONDS - TIMER_START))
        if [ $ELAPSED -ge $DURATION ]; then
            break
        fi
        # Progress
        POSE=$(timeout 3 ros2 topic echo /world_pose --once 2>/dev/null || true)
        X=$(echo "$POSE" | grep -A1 "position:" | grep "x:" | head -1 | awk '{print $2}' 2>/dev/null || echo "?")
        Y=$(echo "$POSE" | grep -A2 "position:" | grep "y:" | head -1 | awk '{print $2}' 2>/dev/null || echo "?")
        printf "\r[%s] t=%3ds  x=%s  y=%s" "$LABEL" "$ELAPSED" "$X" "$Y"
        sleep 5
    done
    echo ""
    info "Done with $LABEL."

    # Cleanup
    kill $AUTO_PID $SCORE_PID $CAND_PID $LANES_PID $SIM_PID 2>/dev/null
    sleep 2
    kill_all
}

# ── Main ──────────────────────────────────────────────────────────────
if [ "$METHOD" = "all" ]; then
    METHODS="$ALL_METHODS"
else
    METHODS="$METHOD"
fi

info "Methods to test: $METHODS"
info "Duration per method: ${DURATION}s"
info "Results directory: $RESULTS_DIR"
echo ""

for m in $METHODS; do
    run_method "$m"
    echo ""
done

# Analyze results
stage "═══ Results Summary ═══"
python3 scripts/analyze_lane_benchmark.py "$RESULTS_DIR"

info "All done! Results in $RESULTS_DIR"
