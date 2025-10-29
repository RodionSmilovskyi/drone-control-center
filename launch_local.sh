#!/bin/bash

# This script creates a 3-pane layout:
# - One main pane on the left (fc_interface)
# - Two smaller panes stacked on the right (agent and logger)

SESSION_NAME="drone_session"

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "An old '$SESSION_NAME' session was found. Terminating it."
  tmux kill-session -t $SESSION_NAME
fi

echo "Creating new tmux session: '$SESSION_NAME'"

# 1. Create a new, detached session. This creates window 0, pane 0.
tmux new-session -d -s $SESSION_NAME

# 2. Split pane 0 horizontally (a vertical split line). This creates pane 1 to the right.
#    We explicitly target {session}:{window}.{pane}
tmux split-window -h -t $SESSION_NAME:0.0

# 3. Select pane 1 (the new one on the right) and split it vertically (a horizontal split line).
#    This creates pane 2 below it.
tmux split-window -v -t $SESSION_NAME:0.1

# We now have 3 panes:
# Pane 0.0: Left
# Pane 0.1: Top-Right
# Pane 0.2: Bottom-Right

# 4. Set the titles for each pane border
tmux select-pane -t $SESSION_NAME:0.0 -T "Dashboard"
tmux select-pane -t $SESSION_NAME:0.1 -T "Flight controller"
tmux select-pane -t $SESSION_NAME:0.2 -T "AI Agent"

# 5. Send commands to each pane.
# 'C-m' simulates pressing the Enter key.
tmux send-keys -t $SESSION_NAME:0.0 "python3 monitor_dashboard.py" C-m
tmux send-keys -t $SESSION_NAME:0.1 "python3 mock_fc.py" C-m
tmux send-keys -t $SESSION_NAME:0.2 "python3 monitor_dashboard.py" C-m

# 6. Attach to the session.
echo "Attaching to session. Use 'Ctrl+b' then 'd' to detach."
tmux attach-session -t $SESSION_NAME

