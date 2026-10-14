#!/bin/sh
# Build and run the host-side (PC) tests for the signal-processing code.
set -e
cd "$(dirname "$0")"
mkdir -p build
CXX="c++ -std=c++17 -O2 -Wall -Wextra -Werror"

$CXX -I../../src \
  test_power_model.cpp ../../src/Kalman.cpp ../../src/Orientation.cpp \
  ../../src/PowerModel.cpp -o build/test_power_model
$CXX -I../../lib/BoatSpeed/src \
  test_impeller.cpp ../../lib/BoatSpeed/src/ImpellerSpeed.cpp \
  -o build/test_impeller

status=0
./build/test_power_model || status=1
./build/test_impeller || status=1
exit $status
