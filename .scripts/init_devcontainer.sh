#! /bin/bash
set -e

echo "Updating APT package list..."
sudo apt-get update

echo "Adding 'universe' repository and updating again..."
sudo add-apt-repository universe -y
sudo apt-get update

echo "Initializing and updating rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

echo "Installing system dependencies with rosdep..."
cd pingpong_tracker_ws
rosdep install -y --from-paths src --ignore-src --rosdistro humble

echo "Initialization complete."