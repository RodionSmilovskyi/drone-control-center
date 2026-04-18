#!/bin/bash

SESSION_NAME="policy_test"
TARGET_ALTITUDE=${1:-0.5}

# 0. Clean up ALL relevant python processes first
echo "Cleaning up old processes..."
pkill -f sensors.py
pkill -f strategic_agent.py
pkill -f tactical_controller.py
pkill -f test_policy_live.py

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "Terminating old '$SESSION_NAME' session."
  tmux kill-session -t $SESSION_NAME
fi

echo "Launching Policy Test Setup (Target Altitude: $TARGET_ALTITUDE)..."

# 1. Create session
tmux new-session -d -s $SESSION_NAME -n "PolicyTest"

# 2. Split panes
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME:0.0

sleep 1.0

# 3. Titles
tmux select-pane -t $SESSION_NAME:0.0 -T "Sensors"
tmux select-pane -t $SESSION_NAME:0.1 -T "Strategic Agent"
tmux select-pane -t $SESSION_NAME:0.2 -T "Policy Monitor"
tmux select-pane -t $SESSION_NAME:0.3 -T "Tactical Controller"

# 4. Start the services
ROOT_DIR=$(dirname $(dirname $(realpath "$0")))

# Start services with full logs visible
tmux send-keys -t $SESSION_NAME:0.0 "cd $ROOT_DIR && python3 -u sensors.py 2>&1" C-m
sleep 0.5
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROOT_DIR && python3 -u strategic_agent.py $TARGET_ALTITUDE 2>&1" C-m
sleep 0.5
tmux send-keys -t $SESSION_NAME:0.3 "cd $ROOT_DIR && python3 -u tactical_controller.py 2>&1" C-m

# Wait for hardware and agent to init
echo "Waiting for services to initialize..."
sleep 5

# 5. Start the live monitor
tmux send-keys -t $SESSION_NAME:0.2 "cd $ROOT_DIR && python3 -u tests/test_policy_live.py 2>&1" C-m


# 6. Attach
tmux attach-session -t $SESSION_NAME
