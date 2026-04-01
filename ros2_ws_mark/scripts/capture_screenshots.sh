#!/bin/bash
# capture_screenshots.sh — Capture Gazebo and RViz screenshots.
#
# Usage:
#   bash scripts/capture_screenshots.sh                    # one-shot capture
#   bash scripts/capture_screenshots.sh --interval 30      # capture every 30s
#   bash scripts/capture_screenshots.sh --on-event         # watch safety log for events
#
# Screenshots saved to /tmp/autonomy_diag/screenshots/
#
# Methods tried (in order):
#   1. import (ImageMagick) — X11 window capture
#   2. scrot — X11 fullscreen capture
#   3. PowerShell — WSL interop, Windows-side capture
# Install one: sudo apt install imagemagick-6.q16 (for import)

DIAG_DIR="/tmp/autonomy_diag/screenshots"
mkdir -p "$DIAG_DIR"

INTERVAL=0  # 0 = one-shot
ON_EVENT=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --interval) INTERVAL="$2"; shift 2;;
        --on-event) ON_EVENT=true; shift;;
        *) shift;;
    esac
done

capture_once() {
    local TS=$(date +%Y%m%d_%H%M%S)
    local TAG="${1:-manual}"

    # Try import (ImageMagick)
    if command -v import &>/dev/null; then
        import -window root "${DIAG_DIR}/full_${TAG}_${TS}.png" 2>/dev/null && \
            echo "[SCREENSHOT] Captured full_${TAG}_${TS}.png (import)" && return 0
    fi

    # Try scrot
    if command -v scrot &>/dev/null; then
        scrot "${DIAG_DIR}/full_${TAG}_${TS}.png" 2>/dev/null && \
            echo "[SCREENSHOT] Captured full_${TAG}_${TS}.png (scrot)" && return 0
    fi

    # Try PowerShell (WSL interop) — captures entire screen
    if command -v powershell.exe &>/dev/null; then
        local WIN_PATH=$(wslpath -w "${DIAG_DIR}/full_${TAG}_${TS}.png" 2>/dev/null)
        if [ -n "$WIN_PATH" ]; then
            powershell.exe -NoProfile -Command "
                Add-Type -AssemblyName System.Windows.Forms
                \$screen = [System.Windows.Forms.Screen]::PrimaryScreen
                \$bitmap = New-Object System.Drawing.Bitmap(\$screen.Bounds.Width, \$screen.Bounds.Height)
                \$graphics = [System.Drawing.Graphics]::FromImage(\$bitmap)
                \$graphics.CopyFromScreen(\$screen.Bounds.Location, [System.Drawing.Point]::Empty, \$screen.Bounds.Size)
                \$bitmap.Save('${WIN_PATH}')
                \$graphics.Dispose()
                \$bitmap.Dispose()
            " 2>/dev/null && echo "[SCREENSHOT] Captured full_${TAG}_${TS}.png (powershell)" && return 0
        fi
    fi

    echo "[SCREENSHOT] No capture method available. Install: sudo apt install imagemagick-6.q16"
    return 1
}

if [ "$ON_EVENT" = true ]; then
    echo "Watching for safety events in /tmp/autonomy_diag/safety_events_*.jsonl..."
    # Find the latest safety events file
    EVENTS_FILE=$(ls -t /tmp/autonomy_diag/safety_events_*.jsonl 2>/dev/null | head -1)
    if [ -z "$EVENTS_FILE" ]; then
        echo "No safety events file found. Start the autonomy stack first."
        exit 1
    fi
    echo "Tailing: $EVENTS_FILE"
    tail -f "$EVENTS_FILE" 2>/dev/null | while read -r line; do
        TYPE=$(echo "$line" | python3 -c "import sys,json; print(json.load(sys.stdin).get('type',''))" 2>/dev/null)
        if [ "$TYPE" = "collision" ] || [ "$TYPE" = "stuck" ] || [ "$TYPE" = "lap" ]; then
            capture_once "$TYPE"
        fi
    done
elif [ "$INTERVAL" -gt 0 ] 2>/dev/null; then
    echo "Capturing every ${INTERVAL}s to $DIAG_DIR (Ctrl+C to stop)"
    COUNT=0
    while true; do
        capture_once "periodic_${COUNT}"
        COUNT=$((COUNT + 1))
        sleep "$INTERVAL"
    done
else
    capture_once "manual"
fi
