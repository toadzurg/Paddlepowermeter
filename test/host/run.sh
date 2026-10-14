#!/bin/sh
# Build and run the host-side (PC) tests for the signal-processing code.
set -e
cd "$(dirname "$0")"
mkdir -p build
c++ -std=c++17 -O2 -Wall -Wextra -Werror -I../../src \
  test_power_model.cpp ../../src/Kalman.cpp ../../src/Orientation.cpp \
  ../../src/PowerModel.cpp -o build/test_power_model
./build/test_power_model
