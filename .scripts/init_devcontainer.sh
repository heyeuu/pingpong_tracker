#! /bin/bash
set -e

echo "Initializing and updating rosdep..."
rosdep update

echo "Installing system dependencies with rosdep..."
cd pingpong_tracker_ws
rosdep install -y --from-paths src --ignore-src --rosdistro humble

echo "Initialization complete."
