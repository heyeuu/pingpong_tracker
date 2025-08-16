#! /bin/bash

source /opt/ros/humble/setup.bash

BASE_OUTPUT_PATH="/tmp/camera_recordings/"

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
OUTPUT_PATH="$BASE_OUTPUT_PATH$TIMESTAMP"

TOPICS_TO_RECORD="/image_raw /camera_info"

echo "Starting to record ROS 2 topics to $OUTPUT_PATH..."
echo "Topics: $TOPICS_TO_RECORD"

ros2 bag record $TOPICS_TO_RECORD -o "$OUTPUT_PATH" --compression-mode file --compression-format zstd
