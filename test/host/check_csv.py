#!/usr/bin/env python3
"""Checks tools/log2csv.py output for the log written by test_log."""

import csv
import sys

path, expected_rows = sys.argv[1], int(sys.argv[2])
failures = 0


def check(name, ok, detail=""):
    global failures
    print(f"  [{' ok ' if ok else 'FAIL'}] {name} {detail}")
    failures += not ok


print(f"\nlog2csv.py output ({path})")
with open(path, newline="") as f:
    rows = list(csv.DictReader(f))

check("row count", len(rows) == expected_rows, f"{len(rows)} vs {expected_rows}")
times = [float(r["time_s"]) for r in rows]
check("time monotonic across µs wrap",
      all(b > a for a, b in zip(times, times[1:])))
check("duration ≈ 30 s", abs(times[-1] - 29.99) < 0.011, f"{times[-1]:.3f} s")
worst = max(abs(float(r["force_n"]) - float(r["fw_force_n"])) for r in rows)
check("force_n matches firmware force", worst < 1e-2, f"max diff {worst:.2e} N")
speeds = [float(r["boat_speed_mps"]) for r in rows if r["boat_speed_mps"]]
mean = sum(speeds) / len(speeds) if speeds else 0
check("boat speed from pulses/m", abs(mean - 2.5) < 0.05, f"mean {mean:.3f} m/s")

print(f"\n{'FAILED' if failures else 'PASSED'} ({failures} failure"
      f"{'' if failures == 1 else 's'})")
sys.exit(1 if failures else 0)
