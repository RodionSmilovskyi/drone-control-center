#!/bin/bash

# This script creates a 4-pane layout:
# - One main pane on the left (teleop)
# - Three smaller panes stacked on the right (dashboard, fc, agent)
#
# You can easily swap 'python3 fc_interface.py' for 'python3 mock_fc.py'
# to run the local simulation.

SESSION_NAME="drone_session"

# --- Configuration ---
# Check the first argument. Default to mock if not '--real'.
if [ "$1" == "--real" ]; then
    USE_MOCK_FC=false
else
    USE_MOCK_FC=true
fi
# ---

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "An old '$SESSION_NAME' session was found. Terminating it."
  tmux kill-session -t $SESSION_NAME
fi

echo "Creating new tmux session: '$SESSION_NAME'"

# 1. Create a new, detached session. This creates window 0, pane 0.
tmux new-session -d -s $SESSION_NAME

# 2. Split pane 0 horizontally (a vertical split line). This creates pane 1 to the right.
tmux split-window -h -t $SESSION_NAME:0.0

# 3. Select pane 1 (the new one on the right) and split it vertically.
#    This creates pane 2 below it.
tmux split-window -v -t $SESSION_NAME:0.1

# 4. Select pane 2 (the new middle-right) and split it vertically again.
#    This creates pane 3 below it.
tmux split-window -v -t $SESSION_NAME:0.2


# We now have 4 panes:
# Pane 0.0: Left
# Pane 0.1: Top-Right
# Pane 0.2: Middle-Right
# Pane 0.3: Bottom-Right

# 5. Set the titles for each pane border
tmux select-pane -t $SESSION_NAME:0.0 -T "Monitor Dashboard"
tmux select-pane -t $SESSION_NAME:0.1 -T "Manual Teleop"
tmux select-pane -t $SESSION_NAME:0.3 -T "Background Services (Silent)"


# 6. Send commands to each pane.
# 'C-m' simulates pressing the Enter key.
tmux send-keys -t $SESSION_NAME:0.0 "python3 dashboard.py" C-m
tmux send-keys -t $SESSION_NAME:0.1 "python3 keyboard.py" C-m
tmux send-keys -t $SESSION_NAME:0.3 "python3 strategic_agent.py &" C-m
tmux send-keys -t $SESSION_NAME:0.3 "python3 tactical_controller.py &" C-m

# 7. Launch either the mock or real FC based on the config
if [ "$USE_MOCK_FC" = true ] ; then
    echo "Launching MOCK Flight Controller..."
    tmux select-pane -t $SESSION_NAME:0.2 -T "MOCK FC"
    tmux send-keys -t $SESSION_NAME:0.2 "echo '--- Starting MOCK FC ---'; python3 fc_interface_mock.py" C-m
else
    echo "Launching REAL Flight Controller Interface..."
    tmux select-pane -t $SESSION_NAME:0.2 -T "REAL FC Interface"
    tmux send-keys -t $SESSION_NAME:0.2 "echo '--- Starting REAL FC Interface ---'; python3 fc_interface.py" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "python3 sensors.py" C-m
fi

tmux select-pane -t $SESSION_NAME:0.1
# 8. Attach to the session.
echo "Attaching to session. Use 'Ctrl+b' then 'd' to detach."
tmux attach-session -t $SESSION_NAME


