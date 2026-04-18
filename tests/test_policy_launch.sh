#!/bin/bash

# Enable debug mode to see exactly what bash is doing
set -x

SESSION_NAME="policy_test"
TARGET_ALTITUDE=${1:-0.5}

# 0. Clean up ALL relevant python processes first
echo "--- CLEANING UP ---"
pkill -f sensors.py
pkill -f strategic_agent.py
pkill -f tactical_controller.py
pkill -f test_policy_live.py

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  tmux kill-session -t $SESSION_NAME
fi

echo "--- LAUNCHING TARGET: $TARGET_ALTITUDE ---"

# Get absolute paths
ROOT_DIR=$(dirname $(dirname $(realpath "$0")))
echo "ROOT_DIR detected as: $ROOT_DIR"

# Get the current python path to ensure we use the virtual environment
PYTHON_CMD=$(which python3)
echo "Using Python: $PYTHON_CMD"

# 1. Create session (Window 0, Pane 0)
tmux new-session -d -s $SESSION_NAME -n "PolicyTest"
sleep 1

# 2. Setup Panes and Start Services ONE BY ONE
# Pane 0: Sensors (Top-Left)
tmux send-keys -t $SESSION_NAME:0.0 "cd $ROOT_DIR && clear && $PYTHON_CMD -u sensors.py 2>&1" C-m
sleep 1

# Pane 1: Strategic Agent (Top-Right)
tmux split-window -h -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROOT_DIR && clear && $PYTHON_CMD -u strategic_agent.py $TARGET_ALTITUDE 2>&1" C-m
sleep 1

# Pane 2: Tactical Controller (Bottom-Left)
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 "cd $ROOT_DIR && clear && $PYTHON_CMD -u tactical_controller.py 2>&1" C-m
sleep 1

# Pane 3: Policy Monitor (Bottom-Right)
tmux split-window -v -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0.3 "cd $ROOT_DIR && clear && $PYTHON_CMD -u tests/test_policy_live.py 2>&1" C-m

# 3. Final Polish
tmux select-layout tiled
tmux set-option -g mouse on # Enable mouse to make pane resizing easier

echo "--- SESSION CREATED. ATTACHING... ---"
sleep 2
tmux attach-session -t $SESSION_NAME
