#!/usr/bin/env bash
# start_bridges.sh — 在一个 tmux session 中以瓦片布局启动 01~04 bridge
#
# 布局（四格）：
#   ┌─────────────┬─────────────┐
#   │  01_IMU     │  02_RTK     │
#   ├─────────────┼─────────────┤
#   │  03_Nav     │  04_Robot   │
#   └─────────────┴─────────────┘

SESSION="bridges"
ROOT="$(cd "$(dirname "$0")" && pwd)"

# 如果 session 已存在则直接 attach
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists — attaching..."
    tmux attach-session -t "$SESSION"
    exit 0
fi

# 创建 session，第一个 pane = 01_IMU
tmux new-session -d -s "$SESSION" -n "bridges" \
    -x "$(tput cols)" -y "$(tput lines)"

# Pane 0 — 01_IMU（左上，已存在）
tmux send-keys -t "$SESSION:0.0" \
    "cd '$ROOT/01_IMU' && python imu_bridge.py" Enter

# Pane 1 — 02_RTK（右上，水平切割）
tmux split-window -t "$SESSION:0.0" -h \
    "cd '$ROOT/02_RTK' && python rtk_bridge.py"

# Pane 2 — 03_Nav（左下，对左上垂直切割）
tmux split-window -t "$SESSION:0.0" -v \
    "cd '$ROOT/03_Nav' && python nav_bridge.py"

# Pane 3 — 04_Robot（右下，对右上垂直切割）
tmux split-window -t "$SESSION:0.1" -v \
    "cd '$ROOT/04_Robot' && python robot_bridge.py"

# 均匀排列四格
tmux select-layout -t "$SESSION:0" tiled

# 给每个 pane 设置标题（需要终端支持 title escape）
tmux select-pane -t "$SESSION:0.0" -T "01_IMU"
tmux select-pane -t "$SESSION:0.1" -T "02_RTK"
tmux select-pane -t "$SESSION:0.2" -T "03_Nav"
tmux select-pane -t "$SESSION:0.3" -T "04_Robot"

# 聚焦左上角
tmux select-pane -t "$SESSION:0.0"

# attach
tmux attach-session -t "$SESSION"
