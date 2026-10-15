#!/usr/bin/env python3
"""Convert a paddle SD-card log (PPMxxxxx.BIN) to CSV.

    python3 tools/log2csv.py PPM00001.BIN            # writes PPM00001.csv
    python3 tools/log2csv.py PPM00001.BIN -o out.csv
    python3 tools/log2csv.py PPM00001.BIN --header   # print header only

Columns are the raw readings in each IMU's own axes (bias not removed) plus
force in newtons computed with the header's calibration. The header (
geometry, axis maps, gyro bias, calibration) is printed to stderr. The
binary layout is defined in src/LogFormat.h; keep the two in sync.
"""

import argparse
import csv
import struct
import sys

MAGIC = 0x474C5050
VERSION = 1

HEADER = struct.Struct("<IHHHf3f6B6b6fiffI16s")
RECORD = struct.Struct("<IiB6f6ffI4f")
assert HEADER.size == 94 and RECORD.size == 81  # matches LogFormat.h

F_IMU1, F_IMU2, F_FORCE, F_BOAT = 1, 2, 4, 8


def read_header(f):
    raw = f.read(HEADER.size)
    if len(raw) < HEADER.size:
        sys.exit("file too short")
    v = HEADER.unpack(raw)
    h = {
        "magic": v[0], "version": v[1], "header_size": v[2],
        "record_size": v[3], "sample_hz": v[4],
        "imu2_pos_m": v[5], "blade_cop_pos_m": v[6], "pivot_prior_pos_m": v[7],
        "axis": [list(v[8:11]), list(v[11:14])],
        "sign": [list(v[14:17]), list(v[17:20])],
        "gyro_bias": [list(v[20:23]), list(v[23:26])],
        "force_zero_offset": v[26], "force_counts_per_n": v[27],
        "pulses_per_m": v[28], "start_millis": v[29],
        "firmware": v[30].split(b"\0")[0].decode("ascii", "replace"),
    }
    if h["magic"] != MAGIC:
        sys.exit("not a paddle log (bad magic)")
    if h["version"] != VERSION:
        sys.exit(f"unsupported log version {h['version']}")
    if h["record_size"] != RECORD.size:
        sys.exit("record size mismatch")
    f.read(h["header_size"] - HEADER.size)  # skip newer header fields
    return h


COLUMNS = (
    ["time_s", "force_raw", "force_n", "force_fresh",
     "imu1_valid", "imu2_valid", "boat_fresh"]
    + [f"imu{i}_gyro_{a}" for i in (1, 2) for a in "xyz"]
    + [f"imu{i}_accel_{a}" for i in (1, 2) for a in "xyz"]
    + ["boat_pulse_hz", "boat_pulse_count", "boat_speed_mps",
       "fw_force_n", "fw_omega", "fw_pivot_m", "fw_inst_power_w"]
)


def convert(f, out, h):
    w = csv.writer(out)
    w.writerow(COLUMNS)
    cal, zero, ppm = h["force_counts_per_n"], h["force_zero_offset"], h["pulses_per_m"]
    prev = None
    elapsed_us = 0
    n = 0
    while True:
        raw = f.read(RECORD.size)
        if len(raw) < RECORD.size:  # end, or a partial record from a power cut
            break
        v = RECORD.unpack(raw)
        t_us, force_raw, flags = v[0], v[1], v[2]
        if prev is not None:
            elapsed_us += (t_us - prev) & 0xFFFFFFFF  # survives the µs wrap
        prev = t_us
        gyro, accel = v[3:9], v[9:15]
        boat_hz, boat_count = v[15], v[16]
        fw = v[17:21]
        force_n = (force_raw - zero) / cal if cal else ""
        boat_fresh = bool(flags & F_BOAT)
        speed = boat_hz / ppm if (ppm > 0 and boat_fresh) else ""
        w.writerow(
            [f"{elapsed_us / 1e6:.6f}", force_raw,
             f"{force_n:.3f}" if force_n != "" else "",
             int(bool(flags & F_FORCE)), int(bool(flags & F_IMU1)),
             int(bool(flags & F_IMU2)), int(boat_fresh)]
            + [f"{x:.6g}" for x in gyro]
            + [f"{x:.6g}" for x in accel]
            + [f"{boat_hz:.4g}", boat_count,
               f"{speed:.4f}" if speed != "" else ""]
            + [f"{x:.6g}" for x in fw]
        )
        n += 1
    return n, elapsed_us / 1e6


def tidy(v):
    """Round float32 noise (0.30000001...) for display."""
    if isinstance(v, float):
        return float(f"{v:.6g}")
    if isinstance(v, list):
        return [tidy(x) for x in v]
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("-o", "--output")
    ap.add_argument("--header", action="store_true", help="print header only")
    args = ap.parse_args()

    with open(args.log, "rb") as f:
        h = read_header(f)
        for k, v in h.items():
            print(f"# {k}: {tidy(v)}", file=sys.stderr)
        if args.header:
            return
        out_path = args.output or args.log.rsplit(".", 1)[0] + ".csv"
        with open(out_path, "w", newline="") as out:
            n, secs = convert(f, out, h)
    print(f"{n} records ({secs:.1f} s) -> {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
