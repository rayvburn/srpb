#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
cd $SCRIPT_DIR
cd ..

mkdir -p build
cd build
cmake ..
make -j4
