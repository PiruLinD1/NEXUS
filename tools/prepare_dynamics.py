"""Prepare mm/s motor telemetry as V, mm/s, mm/s^2 CSVs for offline calibration."""
import argparse
import csv
import math
from pathlib import Path

WHEEL_UNITS = "# units: voltage=V,velocity=mm/s,acceleration=mm/s^2"


def prepare(rows, side, window=7):
    """Centred regression derives mm/s^2 from mm/s samples and timestamps in seconds."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if window < 3 or window % 2 == 0:
        raise ValueError("window must be odd and at least 3")
    if any(f"{side}_motor_mmps" not in row for row in rows):
        raise ValueError(f"expected {side}_motor_mmps in mm/s; convert legacy metre logs explicitly")
    radius = window // 2
    output = []
    for index in range(radius, len(rows) - radius):
        span = rows[index - radius:index + radius + 1]
        times = [row["motor_time"] for row in span]
        velocities = [row[f"{side}_motor_mmps"] for row in span]
        voltages = [row[f"{side}_terminal_v"] for row in span]
        if not all(math.isfinite(v) for v in times + velocities + voltages):
            continue
        if any(b <= a or b - a > .12 for a, b in zip(times, times[1:])):
            continue
        if any(not math.isfinite(row["slip"]) or not 0 <= row["slip"] <= .2
               or row["health"] not in (0, 1) for row in span):
            continue
        # Every velocity contributing to the derivative needs contemporaneous
        # estimator health/slip evidence, not just the centre sample.
        if any(not math.isfinite(row["time"])
               or abs(row["motor_time"] - row["time"]) > .05 for row in span):
            continue
        mean_time = sum(times) / window
        mean_velocity = sum(velocities) / window
        denominator = sum((t - mean_time) ** 2 for t in times)
        acceleration = sum((t - mean_time) * (v - mean_velocity)
                           for t, v in zip(times, velocities)) / denominator
        output.append({"voltage": voltages[radius], "velocity": velocities[radius],
                       "acceleration": acceleration})
    return output


def write_wheel_csv(path, rows):
    """Write the mandatory unit declaration so old metre CSVs cannot be confused."""
    with path.open("w", newline="", encoding="utf-8") as destination:
        destination.write(WHEEL_UNITS + "\n")
        writer = csv.DictWriter(destination, fieldnames=["voltage", "velocity", "acceleration"])
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--window", type=int, default=7)
    args = parser.parse_args()
    with args.log.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"motor_time", "time", "left_motor_mmps", "right_motor_mmps",
                    "left_terminal_v", "right_terminal_v", "slip", "health"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            parser.error("missing columns: " + ", ".join(sorted(missing)) +
                         "; input motor velocities must be in mm/s")
        rows = [{key: float(value) for key, value in row.items()}
                for row in reader]
    args.output_directory.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        output = prepare(rows, side, args.window)
        path = args.output_directory / f"{side}.csv"
        write_wheel_csv(path, output)
        print(f"{path}: {len(output)} samples (V, mm/s, mm/s^2)")
    print("Review slip, synchronization and excitation before fitting. Encoder speed assumes wheel contact.")


if __name__ == "__main__":
    main()
