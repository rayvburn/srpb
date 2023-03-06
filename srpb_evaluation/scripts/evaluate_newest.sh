#!/usr/bin/env bash
#
# Runs srpb_evaluation node with the newest set of log files from the script's directory.
#
# Remember to source your ROS workspace before execution
#
SCRIPT_DIR=$(realpath $(dirname $0))
LOGS_DIR=$SCRIPT_DIR
SAFETY_DIST=0.55

# https://stackoverflow.com/a/54910963
newest_robot=$(ls -t $LOGS_DIR/log_*_robot.txt | head -1)
newest_people=$(ls -t $LOGS_DIR/log_*_people.txt | head -1)
newest_groups=$(ls -t $LOGS_DIR/log_*_groups.txt | head -1)

echo "Evaluating:"
echo ""
echo "robot  data: $newest_robot"
echo "people data: $newest_people"
echo "groups data: $newest_groups"
echo ""

rosrun srpb_evaluation srpb_evaluation $newest_robot $newest_people $newest_groups $SAFETY_DIST
