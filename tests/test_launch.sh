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
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.1

# Allow panes to initialize
sleep 0.5

# 3. Label the panes
tmux select-pane -t $SESSION_NAME:0.0 -T "Sensors"
tmux select-pane -t $SESSION_NAME:0.1 -T "Strategic Agent"
tmux select-pane -t $SESSION_NAME:0.2 -T "Test Output"

# 4. Start the services with explicit directory context and python flush
# We use $(dirname $(dirname $(realpath $0))) to get to the project root
ROOT_DIR=$(dirname $(dirname $(realpath "$0")))

tmux send-keys -t $SESSION_NAME:0.0 "cd $ROOT_DIR && python3 -u sensors.py" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROOT_DIR && python3 -u strategic_agent.py" C-m

# Wait for services to initialize
sleep 3

# 5. Start the live test script in the final pane
tmux send-keys -t $SESSION_NAME:0.2 "cd $ROOT_DIR && python3 -u tests/test_strategic_live.py" C-m

# 6. Attach to the session
tmux attach-session -t $SESSION_NAME
