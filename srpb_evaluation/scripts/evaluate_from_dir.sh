#!/usr/bin/env bash
#
# Runs srpb_evaluation node with the newest set of log files from the given directory
#
# Remember to source your ROS workspace before execution
#
# This script can be executed using:
#
#   rosrun srpb_evaluation evaluate_from_dir.sh  <path>  <safety_distance>
#
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Wrong usage. Script args:"
    echo "  (1) [required] full path to the directory with logs ('*_robot.txt', '*_people.txt', and '*_groups.txt')"
    echo "  (2) [optional] safety distance for m_obs metric, 0.55 by default"
    exit 0
fi

# only the path to logs directory is given
if [ "$#" -eq 1 ]; then
    logs_dir="$1"
    safety_dist=0.55
    echo "Using logs directory: '$logs_dir' and default safety distance of '$safety_dist' m"
fi

if [ "$#" -eq 2 ]; then
    logs_dir="$1"
    safety_dist=$2
    echo "Using logs directory: '$logs_dir' and custom safety distance of '$safety_dist' m"
fi

# https://stackoverflow.com/a/54910963
newest_robot=$(ls -t $logs_dir/log_*_robot.txt | head -1)
newest_people=$(ls -t $logs_dir/log_*_people.txt | head -1)
newest_groups=$(ls -t $logs_dir/log_*_groups.txt | head -1)

echo "Evaluating:"
echo ""
echo "robot  data: $newest_robot"
echo "people data: $newest_people"
echo "groups data: $newest_groups"
echo ""

rosrun srpb_evaluation srpb_evaluation $newest_robot $newest_people $newest_groups $safety_dist
