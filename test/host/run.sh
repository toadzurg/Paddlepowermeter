#!/bin/sh
# Build and run the host-side (PC) tests for the signal-processing code.
set -e
cd "$(dirname "$0")"
mkdir -p build
CXX="c++ -std=c++17 -O2 -Wall -Wextra -Werror"
MODEL="../../src/Kalman.cpp ../../src/Orientation.cpp ../../src/PowerModel.cpp"

$CXX -I../../src test_power_model.cpp $MODEL -o build/test_power_model
$CXX -I../../lib/BoatSpeed/src \
  test_impeller.cpp ../../lib/BoatSpeed/src/ImpellerSpeed.cpp \
  -o build/test_impeller
$CXX -I../../src -I../../tools/replay test_log.cpp $MODEL -o build/test_log
$CXX -I../../src test_ble_protocol.cpp -o build/test_ble_protocol

status=0
./build/test_power_model || status=1
./build/test_impeller || status=1
./build/test_log build/test_log.bin || status=1
./build/test_ble_protocol build/ble_samples.json || status=1
# 30 s at 100 Hz = 3000 records
python3 ../../tools/log2csv.py build/test_log.bin -o build/test_log.csv \
  2>/dev/null && python3 check_csv.py build/test_log.csv 3000 || status=1
python3 ../../tools/log2csv.py build/test_log.bin.v1 -o build/test_log_v1.csv 2>/dev/null \
  && [ "$(wc -l < build/test_log_v1.csv)" -eq 6 ] && echo "  [ ok ] log2csv reads a version-1 log" \
  || { echo "  [FAIL] log2csv reads a version-1 log"; status=1; }
exit $status
