#!/bin/bash

# This script launches all four components in a 2x2 tiled layout.
# - fc_interface.py (Top-Left)
# - agent.py (Top-Right)
# - monitor_logger.py (Bottom-Left)
# - monitor_dashboard.py (Bottom-Right)

SESSION_NAME="drone_dashboard"

# Check if the session already exists and kill it
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "An old '$SESSION_NAME' session was found. Terminating it."
  tmux kill-session -t $SESSION_NAME
fi

echo "Creating new tmux session: '$SESSION_NAME'"

# Pane 0: Flight Controller Interface
tmux new-session -d -s $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0 "echo '--- Flight Controller Interface ---'; python3 fc_interface.py" C-m

# # Pane 1: AI Agent (Split Pane 0 horizontally)
# tmux split-window -h -t $SESSION_NAME:0
# tmux send-keys -t $SESSION_NAME:1 "echo '--- AI Agent ---'; python3 agent.py" C-m

# # Pane 2: File Logger (Select Pane 0 and split it vertically)
# tmux select-pane -t $SESSION_NAME:0
# tmux split-window -v -t $SESSION_NAME:0
# tmux send-keys -t $SESSION_NAME:2 "echo '--- File Logger ---'; python3 monitor_logger.py" C-m

# Pane 3: Monitor Dashboard (Select Pane 1 and split it vertically)
tmux select-pane -t $SESSION_NAME:1
tmux split-window -v -t $SESSION_NAME:1
tmux send-keys -t $SESSION_NAME:3 "echo '--- Monitor Dashboard ---'; python3 monitor_dashboard.py" C-m

# Optional: You can select the "tiled" layout to try and even them out
tmux select-layout -t $SESSION_NAME:0 tiled

# Attach to the session
echo "Attaching to session. Use 'Ctrl+b' then 'd' to detach."
tmux attach-session -t $SESSION_NAME
