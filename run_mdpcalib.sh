#!/usr/bin/env bash

SESSION_NAME="mdpcalib_six_panes"
HELPER_SCRIPT="/tmp/attach_mdpcalib.sh"

# --- Create helper script that waits for the container & runs role-specific commands ---
cat > "$HELPER_SCRIPT" << 'EOF'
#!/usr/bin/env bash
set -euo pipefail

PREFIX="mdpcalib-mdpcalib-run"
ROLE="${1:-shell}"

echo "[attach_mdpcalib] Role: $ROLE"
echo "[attach_mdpcalib] Looking for container starting with: $PREFIX"

# Wait for container to exist
while true; do
    CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep "^${PREFIX}" | head -n1 || true)
    if [[ -n "${CONTAINER_NAME:-}" ]]; then
        echo "[attach_mdpcalib] Attaching to: $CONTAINER_NAME"
        break
    fi
    echo "[attach_mdpcalib] Waiting for container..."
    sleep 2
done

# Wait for roscore (process-based, robust)
WAIT_ROSCORE='echo "[attach_mdpcalib] Waiting for roscore (process)..."; \
while ! pgrep -f "[r]oscore" >/dev/null 2>&1; do sleep 2; done; \
echo "[attach_mdpcalib] roscore process found.";'

ROLE_CMD='exec bash'

case "$ROLE" in
    rviz)
        ROLE_CMD="$WAIT_ROSCORE roscd pose_synchronizer 2>/dev/null || cd /root || true; rviz -d rviz/combined.rviz; exec bash"
        ;;
    cmrnext)
        ROLE_CMD="$WAIT_ROSCORE roslaunch cmrnext cmrnext_kitti.launch; exec bash"
        ;;
    optimizer)
        ROLE_CMD="$WAIT_ROSCORE \
echo \"[attach_mdpcalib] Cleaning data/experiments/*\"; \
rm -rf data/experiments/*; \
roslaunch optimization_utils optimizer.launch; \
exec bash"
        ;;
    sync)
        ROLE_CMD="$WAIT_ROSCORE \
echo \"[attach_mdpcalib] Cleaning /data/cache/*\"; \
rm -rf /data/cache/*; \
roslaunch pose_synchronizer pose_synchronizer_fastlo_kitti.launch; \
exec bash"
        ;;
    play)
        ROLE_CMD="$WAIT_ROSCORE \
echo \"[attach_mdpcalib] Waiting for ORB vocabulary to be loaded...\"; \
read -p \"Press ENTER after ORB vocabulary has been loaded... \" _; \
roslaunch pose_synchronizer play_bag_kitti_left.launch; \
exec bash"
        ;;
esac

# Exec into container with interactive shell (loads ~/.bashrc → ROS env OK)
exec docker exec -it "$CONTAINER_NAME" bash -ic "$ROLE_CMD"
EOF

chmod +x "$HELPER_SCRIPT"

# --- Kill any existing tmux session ---
tmux has-session -t "$SESSION_NAME" 2>/dev/null && tmux kill-session -t "$SESSION_NAME"

# --- Start tmux session ---
tmux new-session -d -s "$SESSION_NAME" -n main

# ---- Build strict 3x2 layout ----

tmux split-window -h -t "$SESSION_NAME":0.0

tmux select-pane -t "$SESSION_NAME":0.0
tmux split-window -v -t "$SESSION_NAME":0.0
tmux select-pane -t "$SESSION_NAME":0.2
tmux split-window -v -t "$SESSION_NAME":0.2

tmux select-pane -t "$SESSION_NAME":0.1
tmux split-window -v -t "$SESSION_NAME":0.1
tmux select-pane -t "$SESSION_NAME":0.4
tmux split-window -v -t "$SESSION_NAME":0.4

# Pane mapping:
# 0: top-left        → roscore
# 1: top-right       → rviz
# 2: middle-left     → cmrnext
# 3: bottom-left     → optimizer (cleans data/experiments)
# 4: middle-right    → synchroniser (cleans /data/cache)
# 5: bottom-right    → play bag (ORB gate)

# ---- Send commands ----

tmux send-keys -t "$SESSION_NAME":0.0 \
  'xhost +local:docker && cd /home/rajitha/aoc/MDPCalib && docker compose run -v ~/aoc/MDPCalib/data/kitti:/data -it mdpcalib bash -ic "roscore; exec bash"' C-m

tmux send-keys -t "$SESSION_NAME":0.1 "bash '$HELPER_SCRIPT' rviz" C-m
tmux send-keys -t "$SESSION_NAME":0.2 "bash '$HELPER_SCRIPT' cmrnext" C-m
tmux send-keys -t "$SESSION_NAME":0.3 "bash '$HELPER_SCRIPT' optimizer" C-m
tmux send-keys -t "$SESSION_NAME":0.4 "bash '$HELPER_SCRIPT' sync" C-m
tmux send-keys -t "$SESSION_NAME":0.5 "bash '$HELPER_SCRIPT' play" C-m

tmux attach -t "$SESSION_NAME"

