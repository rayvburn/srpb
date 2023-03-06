#!/usr/bin/env bash
#
# Copies 3 newest files from this directory to the directory given by argument.
# It is handy to put that script into the logs directory.
#
if [ "$#" -ne 1 ]; then
    echo "What's the target directory?"
    exit 0
fi

SCRIPT_DIR=$(realpath $(dirname $0))
TARGET_DIR="$SCRIPT_DIR/$1"
LOGS_DIR=$SCRIPT_DIR
mkdir -p $TARGET_DIR

# https://stackoverflow.com/a/54910963
files_to_copy=$(ls -t $LOGS_DIR/log_*.txt | head -3)
cp $files_to_copy $TARGET_DIR

echo "Copied:"
echo ""
echo "$files_to_copy"
echo ""
echo "to $TARGET_DIR"
