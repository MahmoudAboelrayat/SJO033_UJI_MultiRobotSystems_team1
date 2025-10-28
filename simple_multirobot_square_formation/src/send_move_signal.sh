
#!/bin/bash

# Check if an argument was provided
if [ $# -ne 1 ]; then
  echo "Usage: $0 <CW|CCW>"
  exit 1
fi

SIGNAL=$1

# Validate the argument
if [[ "$SIGNAL" != "CW" && "$SIGNAL" != "CCW" ]]; then
  echo "Error: Signal must be CW or CCW"
  exit 1
fi

# Publish the signal
ros2 topic pub /move_signal std_msgs/msg/String "{data: '$SIGNAL'}" --once
