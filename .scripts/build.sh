#!/bin/bash
set -e

cd ./pingpong_tracker_ws

source /opt/ros/humble/setup.bash

colcon build --cmake-args -GNinja --symlink-install

source install/setup.bash

echo "colcon build finished."