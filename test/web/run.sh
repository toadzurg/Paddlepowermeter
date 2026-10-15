#!/bin/sh
# Browser test for the web app against a simulated paddle.
# Needs a C++ compiler, Node and Playwright with Chromium.
set -e
cd "$(dirname "$0")"
mkdir -p build
c++ -std=c++17 -O2 -Wall -Wextra -Werror -I../../src \
  ../host/test_ble_protocol.cpp -o build/test_ble_protocol
./build/test_ble_protocol build/ble_samples.json >/dev/null
status=0
node app.test.cjs build/ble_samples.json build || status=1
node sim.test.cjs build || status=1
exit $status
