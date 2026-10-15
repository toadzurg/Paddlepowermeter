#!/bin/sh
# Build the log replay tool for this PC.
set -e
cd "$(dirname "$0")"
c++ -std=c++17 -O2 -Wall -Wextra -I../../src replay.cpp \
  ../../src/Kalman.cpp ../../src/Orientation.cpp ../../src/PowerModel.cpp \
  -o replay
echo "built tools/replay/replay"
