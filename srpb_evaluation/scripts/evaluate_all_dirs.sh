#!/usr/bin/env bash
#
# Runs srpb_evaluation node with the newest set of log files from each directory below the script's path
#
# It expects that `evaluate_from_dir.sh` script is placed in the same directory
#
# Remember to source your ROS workspace before execution
#
SCRIPT_DIR=$(realpath $(dirname $0))

# - ignore directories starting with the underscore
# - for each directory, run the `evaluate_from_dir` script, passing each found directory as an argument
# Ref:
# * https://stackoverflow.com/a/20292636
# * https://stackoverflow.com/a/15736463
find . -maxdepth 1 -type d -not -path "./_*" \( ! -name . \) -exec bash -c "$SCRIPT_DIR/evaluate_from_dir.sh '{}'" \;

# first part of command above to save number of evaluated dirs, ref: https://stackoverflow.com/a/15663664
count=$(find . -maxdepth 1 -type d -not -path "./_*" \( ! -name . \) | wc -l)

echo
echo "Evaluated $count directories"
echo
