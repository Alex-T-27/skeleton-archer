#!/bin/bash
# Skeleton Archer - autonomous 180 sweep / detect / aim / fire demo (P2.3).
# Brings up Gazebo + camera bridge + detector, waits until the perception
# pipeline is actually producing data, then starts the brain.
# Ctrl+C tears everything down.

cd "$(dirname "$0")" || exit 1

source /opt/ros/jazzy/setup.bash
# Uncomment the next line only if you changed code since the last build:
# colcon build && echo "build ok" || { echo "build failed"; exit 1; }
source install/setup.bash

# Clear any leftover brain from a previous run (two brains fight over the joints).
pkill -f archer_brain 2>/dev/null

# Sim + camera bridge + detector + mover + arrow launcher, in the background.
ros2 launch archer_sim archer_vision.launch.py &
LAUNCH_PID=$!

# Ctrl+C (or any exit) kills the launch and any brain node.
trap 'kill "$LAUNCH_PID" 2>/dev/null; pkill -f archer_brain 2>/dev/null' EXIT

# Wait for real perception output, not a blind sleep: this echo only succeeds
# once Gazebo -> image bridge -> detector is fully up and seeing the tag.
echo "Waiting for the perception pipeline..."
until timeout 5 ros2 topic echo /archer/target_angle --once >/dev/null 2>&1; do
  sleep 1
done
echo "Perception live. Starting the brain."

# Autonomous sweep -> detect -> aim -> fire -> repeat (foreground).
ros2 run archer_sim archer_brain
