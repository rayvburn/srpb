#!/usr/bin/env bash
#
# Copies newest log files (robot/people/groups) from the one directory to the directory given by argument.
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

# Find the newest files matching the pattern
# https://stackoverflow.com/a/54910963
newest_robot=$(ls -t $logs_dir/log_*_robot.txt | head -1)
newest_people=$(ls -t $logs_dir/log_*_people.txt | head -1)
newest_groups=$(ls -t $logs_dir/log_*_groups.txt | head -1)

# Extract the timestamp from the selected file (note that they may differ slightly)
timestamp_robot=$(echo "$newest_robot" | grep -oP '\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}')
timestamp_people=$(echo "$newest_people" | grep -oP '\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}')
timestamp_groups=$(echo "$newest_groups" | grep -oP '\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}')

# Indicates whether logs for a certain session consists of multiple parts
max_part='1'

# Find all files with the same timestamp
while IFS= read -r -d '' file; do
    # Extract filename without path
    filename=$(basename "$file")
    # Extract part number from filename (no "part" suffix means "1")
    part=$(echo "$filename" | grep -oP '_part\K\d+' || echo '1')
    # Add file to respective part number group
    part_files[$part]+="$file "

    # Find the biggest "part" number
    curr_part_num=$(expr "${part}" + 0)
    max_part_num=$(expr "${max_part}" + 0)
    if [ "${curr_part_num}" -gt "${max_part_num}" ]; then
        max_part="${curr_part_num}"
    fi
done < <(find "$logs_dir" -maxdepth 1 -type f \
    \( \
        -name "log_*_${timestamp_robot}_*.txt" \
        -o -name "log_*_${timestamp_people}_*.txt" \
        -o -name "log_*_${timestamp_groups}_*.txt" \
    \) \
    -print0 \
)

# If the logs are not divided into "parts", we'll don't need to create separate directories in the target dir
if [ "${max_part}" -eq "1" ]; then
    # the logging session consisted from only 1 part
    mkdir -p $target_dir
    cp ${part_files[${max_part}]} $target_dir

    # files but each in a new line
    files_nl=$(echo "${part_files[${max_part}]}" | tr ' ' '\n')
    echo "Copied:"
    echo "$files_nl"
    echo "to $(realpath $target_dir)"
    exit 0
fi

# Found multiple parts of logs related to the same session
# Iterate from 1 to the value in max_part
for ((i = 1; i <= max_part; i++)); do
    target_dir_part=$target_dir/part$i
    mkdir -p $target_dir_part
    cp ${part_files[$i]} $target_dir_part

    # files but each in a new line
    files_nl=$(echo "${part_files[$i]}" | tr ' ' '\n')
    echo "Copied:"
    echo "$files_nl"
    echo "to $(realpath $target_dir_part)"
done
exit 0
