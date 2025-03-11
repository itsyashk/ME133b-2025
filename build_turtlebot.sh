#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "TurtleBot build complete!"
echo "Don't forget to source install/setup.bash in new terminals"