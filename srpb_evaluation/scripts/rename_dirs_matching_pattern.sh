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

# Loop through directories matching the provided pattern
for dir in *"$pattern_to_match"*; do
    # Check if the directory does not contain the differentiate pattern
    if [[ $dir != *"$pattern_to_differentiate"* ]]; then
        # Extract the base name without the pattern and append the replace pattern
        new_dir=$(echo "$dir" | sed "s/$pattern_to_match/$pattern_to_replace/")""
        
        # Rename the directory
        mv "$dir" "$new_dir"
        
        # Print a message indicating the rename
        echo "Renamed: $dir -> $new_dir"
    fi
done
