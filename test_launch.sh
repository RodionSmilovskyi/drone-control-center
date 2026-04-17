#!/bin/bash

SESSION_NAME="drone_test"

# Clean up any old session
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
  echo "Terminating old '$SESSION_NAME' session."
  tmux kill-session -t $SESSION_NAME
fi

echo "Launching Test Setup in tmux session: '$SESSION_NAME'"

# 1. Create a new session and name the first window
tmux new-session -d -s $SESSION_NAME -n "StrategicTest"

# 2. Split the window into three panes
# Left pane: Sensors
# Top-right: Strategic Agent
# Bottom-right: Test Trigger/Output
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.1

# 3. Label the panes
tmux select-pane -t $SESSION_NAME:0.0 -T "Sensors"
tmux select-pane -t $SESSION_NAME:0.1 -T "Strategic Agent"
tmux select-pane -t $SESSION_NAME:0.2 -T "Test Output"

# 4. Start the services
tmux send-keys -t $SESSION_NAME:0.0 "python3 sensors.py" C-m
tmux send-keys -t $SESSION_NAME:0.1 "python3 strategic_agent.py" C-m

# Wait a moment for services to initialize
sleep 2

# 5. Start the live test script in the final pane
tmux send-keys -t $SESSION_NAME:0.2 "python3 test_strategic_live.py" C-m

# 6. Attach to the session
tmux attach-session -t $SESSION_NAME
