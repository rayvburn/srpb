#!/usr/bin/env bash
#
# Runs srpb_evaluation node with the newest set of log files from each directory below the script's path
#
# It expects that `evaluate_from_dir.sh` script is available using `rosrun srpb_evaluation evaluate_from_dir.sh`
#
# Remember to source your ROS workspace before execution
#
# This script can be executed using:
#
#   rosrun srpb_evaluation evaluate_all_dirs.sh  <path>  <safety_distance>
#
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Wrong usage. Script args:"
    echo "  (1) [required] full path to the main directory with grouped logs (3 logs in each separate directory)"
    echo "  (2) [optional] safety distance for m_obs metric, 0.55 by default"
    exit 0
fi

# only the path to logs directory is given
if [ "$#" -eq 1 ]; then
    logs_dir="$1"
    # trim trailing / (if exists)
    logs_dir="${logs_dir%/}"
    safety_dist=0.55
    echo "Using logs main directory: '$logs_dir' and default safety distance of '$safety_dist' m"
fi

if [ "$#" -eq 2 ]; then
    logs_dir="$1"
    # trim trailing / (if exists)
    logs_dir="${logs_dir%/}"
    safety_dist=$2
    echo "Using logs main directory: '$logs_dir' and custom safety distance of '$safety_dist' m"
fi

# exclude main directory from the search below
logs_basename=$(basename $logs_dir)

# - ignore directories starting with the underscore
# - for each directory, run the `evaluate_from_dir` script, passing each found directory as an argument
# Ref:
# * https://stackoverflow.com/a/20292636
# * https://stackoverflow.com/a/15736463
find $logs_dir -maxdepth 1 -type d -not -path "$logs_dir/_*" \( ! -name $logs_basename \) -exec bash -c "rosrun srpb_evaluation evaluate_from_dir.sh '{}'" \;

# first part of command above to save number of evaluated dirs, ref: https://stackoverflow.com/a/15663664
count=$(find $logs_dir -maxdepth 1 -type d -not -path "$logs_dir/_*" \( ! -name $logs_basename \) | wc -l)

echo
echo "Evaluated $count directories"
echo
