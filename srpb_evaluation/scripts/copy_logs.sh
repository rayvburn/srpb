#!/usr/bin/env bash
#
# Copies 3 newest log files (robot/people/groups) from the one directory to the directory given by argument.
#
# Usage looks like:
#
#   rosrun srpb_evaluation copy_logs.sh  <path_to_the_main_directory_with_logs>  <path_to_the_target_directory>
#
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Wrong usage. Script args:"
    echo "  (1) [required] path to the source directory with logs ('*_robot.txt', '*_people.txt', and '*_groups.txt')"
    echo "  (2) [required] path to the target directory with copied logs; relative paths will be considered as relative to the source directory"
    exit 0
fi

logs_dir="$1"
# trim trailing / (if exists)
logs_dir="${logs_dir%/}"

target_dir="$2"
# trim trailing / (if exists)
target_dir="${target_dir%/}"

if [[ ! $target_dir == /* ]]; then
    echo "Relative path"
    target_dir=$logs_dir/$target_dir
fi

# create a new directory for the copied logs
mkdir -p $target_dir

# https://stackoverflow.com/a/54910963
newest_robot=$(ls -t $logs_dir/log_*_robot.txt | head -1)
newest_people=$(ls -t $logs_dir/log_*_people.txt | head -1)
newest_groups=$(ls -t $logs_dir/log_*_groups.txt | head -1)

# added an escape sequence for newline (file paths are printed in separate lines)
files_to_copy="$newest_robot"$'\n'"$newest_people"$'\n'"$newest_groups"
cp $files_to_copy $target_dir

echo "Copied:"
echo ""
echo "$files_to_copy"
echo ""
echo "to $target_dir"
