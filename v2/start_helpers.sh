#!/usr/bin/env bash
# start_helpers.sh — menu-driven launcher for listen_*/replay_*/send_demo
# helper scripts, in a tmux session separate from start_bridges.sh's.
set -euo pipefail
SESSION="helpers"
ROOT="$(cd "$(dirname "$0")" && pwd)"

show_menu() {
    cat <<'EOF'
  1  01_IMU     listen        2  01_IMU     replay
  3  02_RTK     listen        4  02_RTK     replay
  5  03_Nav     listen        6  03_Nav     replay
  7  04_Robot   listen        8  04_Robot   send_demo
  9  04_Robot   replay        A  06_Camera  listen
  B  06_Camera  replay        C  03_Nav     bridge
  E  05_AutoNav listen        F  05_AutoNav replay (imu+rtk)
  D  full replay stack (02_RTK replay + 01_IMU replay + 03_Nav bridge + 03_Nav listen)
  X  Stop helpers session      Q  Quit

  Enter one or more keys (e.g. "234" or "2 3 4") to launch them together
  as tiled panes in one tmux window.
EOF
}

cmd_for_key() {
    local key="$1"
    case "$key" in
        1) echo "cd '$ROOT/01_IMU' && python3 listen_imu_websocket.py" ;;
        2) echo "cd '$ROOT/01_IMU' && python3 replay_imu_websocket.py" ;;
        3) echo "cd '$ROOT/02_RTK' && python3 listen_rtk_websocket.py" ;;
        4) echo "cd '$ROOT/02_RTK' && python3 replay_rtk_websocket.py" ;;
        5) echo "cd '$ROOT/03_Nav' && python3 listen_nav_websocket.py" ;;
        6) echo "cd '$ROOT/03_Nav' && python3 replay_nav_websocket.py" ;;
        7) echo "cd '$ROOT/04_Robot' && python3 listen_robot_websocket.py" ;;
        8) echo "cd '$ROOT/04_Robot' && python3 send_robot_only_demo.py" ;;
        9) echo "cd '$ROOT/04_Robot' && python3 replay_robot_websocket.py" ;;
        A) echo "cd '$ROOT/06_Camera' && python3 listen_camera_websocket.py" ;;
        B) echo "cd '$ROOT/06_Camera' && python3 replay_camera_websocket.py" ;;
        C) echo "cd '$ROOT/03_Nav' && python3 nav_bridge.py" ;;
        E) echo "cd '$ROOT/05_AutoNav' && python3 listen_autonav.py" ;;
        F) echo "cd '$ROOT/05_AutoNav' && python3 replay_imu_rtk.py" ;;
        *) echo "" ;;
    esac
}

start_selected() {
    local selection
    selection="$(echo "$1" | tr '[:lower:]' '[:upper:]' | tr -d '[:space:]')"

    if [[ "$selection" == "D" ]]; then
        selection="24C5"
    fi

    # Starting the 03_Nav bridge (C) needs 01_IMU/02_RTK sources running first.
    if [[ "$selection" == *C* ]]; then
        [[ "$selection" != *2* ]] && selection="2$selection"
        [[ "$selection" != *4* ]] && selection="4$selection"
    fi

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
    fi

    local first=1
    local i key cmd
    for ((i = 0; i < ${#selection}; i++)); do
        key="${selection:$i:1}"
        cmd="$(cmd_for_key "$key")"
        if [[ -z "$cmd" ]]; then
            echo "Skipping unknown key: $key"
            continue
        fi
        if [[ "$first" -eq 1 ]]; then
            tmux new-session -d -s "$SESSION" "$cmd"
            first=0
        else
            tmux split-window -t "$SESSION:0" "$cmd"
        fi
    done

    if [[ "$first" -eq 1 ]]; then
        echo "Nothing started."
        return
    fi

    tmux select-layout -t "$SESSION:0" tiled
    tmux attach-session -t "$SESSION"
}

show_menu
read -r -p "Select: " selection

case "$selection" in
    X|x)
        if tmux has-session -t "$SESSION" 2>/dev/null; then
            tmux kill-session -t "$SESSION"
            echo "Stopped '$SESSION' session."
        else
            echo "No '$SESSION' session running."
        fi
        ;;
    Q|q) ;;
    *) start_selected "$selection" ;;
esac
