#!/bin/bash
#
# This script is handy when one wants to use `create_excel_from_results` but in the same directory one has planners
# named with the same "base name", e.g., `great_planner` and `great_planner_plus`
#
# Usage:
#
#   ./rename_dirs_matching_pattern.sh ~/srpb_logs/ great_planner great_planner_plus great_planner_original
#
# This script was written by the ChatGPT

# Check if all arguments are provided
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <main_dir_with_target_subdirs> <pattern_to_match> <pattern_to_differentiate> <pattern_to_replace>"
    exit 1
fi

main_dir="$1"
pattern_to_match="$2"
pattern_to_differentiate="$3"
pattern_to_replace="$4"

cd "$main_dir" || exit 1

# Initialize the highest_number
highest_number="0"

# Loop through directories matching the provided pattern
for dir in *"$pattern_to_match"*; do
    # Check if the directory does not contain the differentiate pattern
    if [[ $dir != *"$pattern_to_differentiate"* ]]; then
        # Extract the number part using sed
        number=$(echo "$dir" | sed "s/.*${pattern_to_replace}\([0-9]\+\).*/\1/")

        # Check if the extracted string is a number
        if [[ "$number" =~ ^[0-9]+$ ]]; then
            # Compare the numbers
            if [ "$number" -gt "$highest_number" ]; then
                highest_number="$number"
            fi
        fi
    fi
done

# Loop through directories matching the provided pattern
for dir in *"$pattern_to_match"*; do
    # Check if the directory matching the pattern exists
    if [ ! -d "$dir" ]; then
        continue
    fi
    # Check if the directory does not contain the differentiate pattern
    if [[ $dir != *"$pattern_to_differentiate"* ]]; then
        # Check if the directory does not contain the replace pattern
        if [[ $dir != *"$pattern_to_replace"* ]]; then
            # Extract the base name without the pattern and append the replace pattern
            new_dir=$(echo "$dir" | sed "s/$pattern_to_match/$pattern_to_replace/")""

            # trim the number from a directory named with "pattern_to_match" and replace with the "highest"
            # from the previous loop
            number_match=$(echo "$dir" | sed "s/.*${pattern_to_match}\([0-9]*\).*/\1/")

            # interpret as non-octal, ref: https://unix.stackexchange.com/a/406445
            highest_number=$(echo $((10#$highest_number)))
            # increment
            ((highest_number++))
            # adds a leading zero for a digit
            number_replace=$(printf "%02d" $highest_number)

            new_dir_num=$(echo "$new_dir" | sed "s/$number_match/$number_replace/")""

            # Rename the directory
            mv "$dir" "$new_dir_num"

            # Print a message indicating the rename
            echo "Renamed: $dir -> $new_dir_num"
        fi
    fi
done
exit 0
