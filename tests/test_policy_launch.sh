#!/bin/bash

SESSION_NAME="policy_test"
TARGET_ALTITUDE=${1:-0.5}

# 0. AGGRESSIVE CLEANUP
echo "Cleaning up all old scripts..."
pkill -9 -f sensors.py
pkill -9 -f strategic_agent.py
pkill -9 -f tactical_controller.py
pkill -9 -f test_policy_live.py
pkill -9 -f test_strategic_live.py

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  tmux kill-session -t $SESSION_NAME
fi

# Detect absolute paths
ROOT_DIR=$(dirname $(dirname $(realpath "$0")))
PYTHON_CMD=$(which python3)

echo "ROOT_DIR: $ROOT_DIR"
echo "PYTHON: $PYTHON_CMD"

# 1. Create session
tmux new-session -d -s $SESSION_NAME

# 2. Setup Panes
tmux split-window -h
tmux split-window -v -t 0
tmux split-window -v -t 1

# 3. Start Services with explicit CD and absolute paths
# Pane 0: Sensors (Top-Left)
tmux send-keys -t 0 "cd $ROOT_DIR && $PYTHON_CMD -u sensors.py" C-m

# Pane 1: Strategic Agent (Top-Right)
tmux send-keys -t 1 "cd $ROOT_DIR && $PYTHON_CMD -u strategic_agent.py $TARGET_ALTITUDE" C-m

# Pane 2: Tactical Controller (Bottom-Left)
tmux send-keys -t 2 "cd $ROOT_DIR && $PYTHON_CMD -u tactical_controller.py" C-m

# Pane 3: THE CORRECT MONITOR (Bottom-Right)
# We use the absolute path to be 100% sure we run the right file
tmux send-keys -t 3 "cd $ROOT_DIR && $PYTHON_CMD -u tests/test_policy_live.py" C-m

# 4. Finish
tmux select-layout tiled
tmux attach-session -t $SESSION_NAME
