#!/bin/bash

SESSION_NAME="policy_test"
TARGET_ALTITUDE=${1:-0.5}

# 0. Clean up ALL relevant python processes first
echo "Cleaning up..."
pkill -9 -f sensors.py
pkill -9 -f strategic_agent.py
pkill -9 -f tactical_controller.py
pkill -9 -f test_policy_live.py

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  tmux kill-session -t $SESSION_NAME
fi

# Detect the current python path
PYTHON_CMD=$(which python3)
echo "Using Python: $PYTHON_CMD"

# 1. Create session
tmux new-session -d -s $SESSION_NAME

# 2. Setup Panes
tmux split-window -h
tmux split-window -v -t 0
tmux split-window -v -t 1

# 3. Start Services using relative paths from the root
# Pane 0: Sensors (Top-Left)
tmux send-keys -t 0 "$PYTHON_CMD -u sensors.py" C-m

# Pane 1: Strategic Agent (Top-Right)
tmux send-keys -t 1 "$PYTHON_CMD -u strategic_agent.py $TARGET_ALTITUDE" C-m

# Pane 2: Tactical Controller (Bottom-Left)
tmux send-keys -t 2 "$PYTHON_CMD -u tactical_controller.py" C-m

# Pane 3: Policy Monitor (Bottom-Right)
tmux send-keys -t 3 "$PYTHON_CMD -u tests/test_policy_live.py" C-m

# 4. Finish
tmux select-layout tiled
tmux attach-session -t $SESSION_NAME
