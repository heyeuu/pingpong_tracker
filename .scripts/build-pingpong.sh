#!/bin/bash
set -e

echo "Starting colcon build for all packages..."

# 显式进入 ROS 工作空间目录
cd ./pingpong_tracker_ws

# source ROS 2 环境
source /opt/ros/humble/setup.bash

# 运行 colcon build 命令
colcon build --cmake-args -GNinja --symlink-install

echo "colcon build finished."sc