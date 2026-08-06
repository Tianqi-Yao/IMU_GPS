#!/usr/bin/env bash
# start_bridges.sh — start all 7 bridges (00~06), each in its own tmux window.
set -euo pipefail
SESSION="bridges"
ROOT="$(cd "$(dirname "$0")" && pwd)"

start_window() {
    local window_name="$1"
    local command="$2"
    if [[ -z "${FIRST_WINDOW_STARTED:-}" ]]; then
        tmux new-session -d -s "$SESSION" -n "$window_name" "$command"
        FIRST_WINDOW_STARTED=1
    else
        tmux new-window -t "$SESSION" -n "$window_name" "$command"
    fi
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists — attaching..."
    tmux attach-session -t "$SESSION"
    exit 0
fi

start_window "00_QR"      "cd '$ROOT/00_QR'      && python3 qr_server.py"
start_window "01_IMU"     "cd '$ROOT/01_IMU'     && python3 imu_bridge.py"
start_window "02_RTK"     "cd '$ROOT/02_RTK'     && python3 rtk_bridge.py"
start_window "03_Nav"     "cd '$ROOT/03_Nav'     && python3 nav_bridge.py"
start_window "04_Robot"   "cd '$ROOT/04_Robot'   && python3 robot_bridge.py"
start_window "05_AutoNav" "cd '$ROOT/05_AutoNav' && python3 autonav_bridge.py"
start_window "06_Camera"  "cd '$ROOT/06_Camera'  && python3 camera_bridge.py"

tmux select-window -t "$SESSION:01_IMU"
tmux attach-session -t "$SESSION"
