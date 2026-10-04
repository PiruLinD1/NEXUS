#!/usr/bin/env python3
"""Inspect NXOD lines captured by `pros terminal --output`, without resampling."""

import argparse
import csv
import math
from pathlib import Path
import re
import sys


# Keep the wire order identical to NXOD_HEADER in src/robot.cpp. The regression
# test reads that declaration, so a firmware schema change cannot pass silently.
HEADER = tuple((
    "seq,host_s,f_mm,l_mm,left_mm,right_mm,imu_deg,x_mm,y_mm,heading_deg,bias_dps,"
    "gx_dps,gy_dps,gz_dps,f_before_ms,l_before_ms,g_before_ms,"
    "f_after_ms,l_after_ms,g_after_ms,epoch,dropped,valid,rejected,stationary,heading_source"
).split(","))
UINT32_MAX = (1 << 32) - 1
INTEGER_COLUMNS = {"seq", "epoch", "dropped", "valid", "rejected", "stationary", "heading_source"}
INTEGER_COLUMNS.update(name for name in HEADER if name.endswith("_ms"))
INTEGER_LIMITS = {"valid": 63, "rejected": 31, "stationary": 1, "heading_source": 2}
SOURCES = ("f", "l", "g")
SENSOR_BITS = {"f": 1, "l": 2, "left": 4, "right": 8, "g": 16}
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def unsigned32(text):
    if not re.fullmatch(r"[0-9]+", text):
        raise ValueError("expected an unsigned decimal integer")
    value = int(text)
    if value > UINT32_MAX:
        raise ValueError("integer exceeds uint32")
    return value


def signed_wrap32(left, right):
    """Signed modular difference of packet clocks, not their physical latency."""
    return ((left - right + (1 << 31)) & UINT32_MAX) - (1 << 31)


def parse_frame(fields):
    if len(fields) != len(HEADER):
        raise ValueError(f"expected {len(HEADER)} columns, got {len(fields)}")
    values = {}
    for name, field in zip(HEADER, fields):
        try:
            if name in INTEGER_COLUMNS:
                value = unsigned32(field)
                if value > INTEGER_LIMITS.get(name, UINT32_MAX):
                    raise ValueError("value outside the wire schema")
            else:
                value = float(field)
                if not math.isfinite(value) or (name == "host_s" and value < 0):
                    raise ValueError("expected a finite value (nonnegative for host_s)")
        except ValueError as error:
            raise ValueError(f"{name}: {error}") from error
        values[name] = value
    return values


class Extrema:
    def __init__(self):
        self.count = 0
        self.minimum = self.maximum = None

    def add(self, value):
        self.count += 1
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)


class Summary:
    def __init__(self):
        self.frames = self.headers = self.ignored_lines = self.malformed_lines = 0
        self.errors = []
        self.sequence_gaps = self.missing_sequences = self.duplicates = self.sequence_restarts = 0
        self.host_regressions = self.nonincreasing_host = self.epoch_changes = 0
        self.dropped_first = self.dropped_last = self.dropped_max = None
        self.dropped_increases = self.dropped_decreases = 0
        self.status_records = self.status_decreases = 0
        self.status_last = self.status_max = None
        self.zero_timestamp_frames = self.changed_bracket_frames = self.invalid_packet_frames = 0
        self.zero_timestamps = dict.fromkeys(SOURCES, 0)
        self.changed_brackets = dict.fromkeys(SOURCES, 0)
        self.host_dt_ms = Extrema()
        self.skews_ms = {pair: Extrema() for pair in ("F-L", "F-G", "L-G")}
        self.invalid_sensors = dict.fromkeys(SENSOR_BITS, 0)
        self.rejected_sensors = dict.fromkeys(SENSOR_BITS, 0)
        self.heading_sources = dict.fromkeys(("unavailable", "imu", "drive_encoders"), 0)
        self.previous = None

    def invalid(self, line_number, message):
        self.malformed_lines += 1
        if len(self.errors) < 5:
            self.errors.append((line_number, message))

    def status(self, dropped):
        # A live status may be newer than the next queued NXOD row. Keep it
        # separate; it cannot attribute individual sequence holes to queue loss.
        self.status_records += 1
        if self.status_last is not None and dropped < self.status_last:
            self.status_decreases += 1
        self.status_last = dropped
        self.status_max = dropped if self.status_max is None else max(self.status_max, dropped)

    def accept(self, row):
        self.frames += 1
        # Availability and estimator rejection are different: a readable
        # counter can still be rejected as an implausible increment. Count
        # observations, not milliseconds or inferred hardware failures.
        for sensor, bit in SENSOR_BITS.items():
            self.invalid_sensors[sensor] += not bool(row["valid"] & bit)
            self.rejected_sensors[sensor] += bool(row["rejected"] & bit)
        source = ("unavailable", "imu", "drive_encoders")[row["heading_source"]]
        self.heading_sources[source] += 1
        previous = self.previous
        if previous is not None:
            advance = (row["seq"] - previous["seq"]) & UINT32_MAX
            backwards = advance >= (1 << 31)
            clock_backwards = row["host_s"] < previous["host_s"]
            self.sequence_restarts += backwards
            self.host_regressions += clock_backwards
            self.epoch_changes += row["epoch"] != previous["epoch"]
            if advance == 0:
                self.duplicates += 1
            elif not backwards and not clock_backwards:
                if advance > 1:
                    self.sequence_gaps += 1
                    self.missing_sequences += advance - 1
                interval = row["host_s"] - previous["host_s"]
                if interval > 0:
                    self.host_dt_ms.add(interval * 1000)
                else:
                    self.nonincreasing_host += 1
            if row["dropped"] < previous["dropped"]:
                self.dropped_decreases += 1
            elif not backwards and not clock_backwards:
                self.dropped_increases += row["dropped"] - previous["dropped"]
        if self.dropped_first is None:
            self.dropped_first = row["dropped"]
        self.dropped_last = row["dropped"]
        self.dropped_max = row["dropped"] if self.dropped_max is None else max(self.dropped_max, row["dropped"])
        self.previous = row

        zero = changed = False
        for source in SOURCES:
            before, after = row[f"{source}_before_ms"], row[f"{source}_after_ms"]
            if before == 0 or after == 0:
                self.zero_timestamps[source] += 1
                zero = True
            if before != after:
                self.changed_brackets[source] += 1
                changed = True
        self.zero_timestamp_frames += zero
        self.changed_bracket_frames += changed
        invalid = (row["valid"] & 0x13) != 0x13  # F=1, L=2, IMU=16.
        self.invalid_packet_frames += invalid
        if not zero and not changed and not invalid:
            clocks = [row[f"{source}_before_ms"] for source in SOURCES]
            for pair, left, right in (("F-L", 0, 1), ("F-G", 0, 2), ("L-G", 1, 2)):
                self.skews_ms[pair].add(signed_wrap32(clocks[left], clocks[right]))


def inspect_stream(source, destination=None):
    """Retain valid frames in wire order, including duplicates and new sessions."""
    summary = Summary()
    writer = csv.writer(destination) if destination is not None else None
    if writer:
        writer.writerow(HEADER)
    schema_matches = True  # Capturing after startup may miss the periodic header.
    for number, original in enumerate(source, 1):
        line = ANSI.sub("", original).strip().lstrip("\ufeff")
        if line == "NXOD_HEADER" or line.startswith("NXOD_HEADER,"):
            fields = tuple(field.strip() for field in line.split(",")[1:])
            schema_matches = fields == HEADER
            if schema_matches:
                summary.headers += 1
            else:
                summary.invalid(number, "unsupported NXOD_HEADER; skipping frames until a matching header")
            continue
        if line == "NXOD_STATUS" or line.startswith("NXOD_STATUS,"):
            fields = [field.strip() for field in line.split(",")]
            try:
                if len(fields) != 3 or fields[1] != "dropped":
                    raise ValueError("expected NXOD_STATUS,dropped,<uint32>")
                summary.status(unsigned32(fields[2]))
            except ValueError as error:
                summary.invalid(number, str(error))
            continue
        if line != "NXOD" and not line.startswith("NXOD,"):
            summary.ignored_lines += 1
            continue
        try:
            if not schema_matches:
                raise ValueError("frame belongs to an unsupported header")
            fields = [field.strip() for field in line.split(",")[1:]]
            row = parse_frame(fields)
        except ValueError as error:
            summary.invalid(number, str(error))
            continue
        summary.accept(row)
        if writer:
            writer.writerow(fields)
    return summary


def format_summary(summary):
    def interval(stats):
        return "unavailable" if not stats.count else f"{stats.minimum:.3f} .. {stats.maximum:.3f} ms (n={stats.count})"

    def counters(values):
        return ", ".join(f"{key.upper()}={value}" for key, value in values.items())

    lines = [
        f"Frames: {summary.frames}; headers: {summary.headers}; ignored lines: {summary.ignored_lines}; malformed lines: {summary.malformed_lines}",
        f"Sequence: gap events={summary.sequence_gaps}, missing IDs={summary.missing_sequences}, duplicates={summary.duplicates}, restart/backward events={summary.sequence_restarts}",
        f"Epoch changes: {summary.epoch_changes}; host clock regressions: {summary.host_regressions}; equal host times on advancing IDs: {summary.nonincreasing_host}",
        f"Frame dropped counter: first={summary.dropped_first}, last={summary.dropped_last}, max={summary.dropped_max}, observed increase={summary.dropped_increases}, decreases={summary.dropped_decreases}",
        f"Live dropped status (separate): records={summary.status_records}, last={summary.status_last}, max={summary.status_max}, decreases={summary.status_decreases}",
        f"Host dt, advancing frames: {interval(summary.host_dt_ms)}; gaps may span missing rows",
        f"Unavailable sensor observations: {counters(summary.invalid_sensors)}",
        f"Rejected increment observations (separate from availability): {counters(summary.rejected_sensors)}",
        f"Heading source observations: {counters(summary.heading_sources)}",
        f"Packet exclusions (may overlap): zero timestamp frames={summary.zero_timestamp_frames} ({counters(summary.zero_timestamps)}); changed bracket frames={summary.changed_bracket_frames} ({counters(summary.changed_brackets)}); invalid F/L/G bits={summary.invalid_packet_frames}",
    ]
    for pair, stats in summary.skews_ms.items():
        lines.append(f"Packet-clock {pair}, signed wrap32: {interval(stats)}")
    lines.append("Packet-clock differences are not physical sensor latency. No interpolation or calibration is applied.")
    lines.extend(f"Malformed line {number}: {error}" for number, error in summary.errors)
    if summary.malformed_lines > len(summary.errors):
        lines.append(f"... {summary.malformed_lines - len(summary.errors)} further malformed lines")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="text file captured by pros terminal --output")
    parser.add_argument("--csv", type=Path, help="write validated NXOD rows without other terminal messages")
    args = parser.parse_args(argv)
    if args.csv is not None and args.csv.resolve() == args.input.resolve():
        parser.error("--csv must differ from the input log")
    try:
        with args.input.open(encoding="utf-8-sig", errors="replace") as source:
            if args.csv is None:
                summary = inspect_stream(source)
            else:
                with args.csv.open("w", encoding="utf-8", newline="") as destination:
                    summary = inspect_stream(source, destination)
    except OSError as error:
        parser.error(str(error))
    print(format_summary(summary))
    if not summary.frames:
        print("No valid NXOD frames found.", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
