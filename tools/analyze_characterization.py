"""Offline analysis of deliberate NXCHAR trials; never commands or tunes a robot.

Outputs are candidates for independent validation, not an automatic calibration.
Only complete, contiguous, fresh trials contribute to any fitted coefficient.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


WHEEL_UNITS = "# units: voltage=V,velocity=mm/s,acceleration=mm/s^2"
FIELDS = ("version,sequence,run,trial,stage,failure,t_s,sensor_s,cmd_left_v,cmd_right_v,"
          "applied_left_v,applied_right_v,battery_v,x_m,y_m,heading_rad,v_mps,lateral_mps,"
          "omega_radps,forward_m,lateral_m,gyro_rad,left_wheel_mps,right_wheel_mps,"
          "accel_x_g,accel_y_g,accel_z_g,valid_mask,dropped,imu_scale,forward_scale,"
          "lateral_scale,forward_offset_m,lateral_offset_m,track_width_m,motor_s").split(",")
INTEGER_FIELDS = {"version", "sequence", "run", "trial", "stage", "failure", "valid_mask", "dropped"}
KINDS = ("straight_forward", "straight_reverse", "spin_clockwise", "spin_counterclockwise",
         "curve_left", "curve_right")
METADATA = ("imu_scale", "forward_scale", "lateral_scale", "forward_offset_m",
            "lateral_offset_m", "track_width_m")


def parse_console(text):
    """Read repeated headers among arbitrary console messages, preserving sessions."""
    runs, warnings = {}, []
    header = None
    session, previous_sequence = 0, None
    for line_number, line in enumerate(text.splitlines(), 1):
        start = line.find("NXCHAR")
        if start < 0:
            continue
        columns = next(csv.reader([line[start:]]))
        if columns[0] == "NXCHAR_HEADER":
            header = columns[1:]
            if header != FIELDS:
                raise ValueError(f"line {line_number}: unsupported NXCHAR header")
            continue
        if columns[0] != "NXCHAR":
            continue
        if header is None:
            raise ValueError(f"line {line_number}: NXCHAR sample before header")
        if len(columns) - 1 != len(FIELDS):
            # Refuse this capture rather than accidentally using a trial whose
            # corrupted sample was an abort or otherwise important boundary.
            raise ValueError(f"line {line_number}: truncated NXCHAR sample")
        try:
            row = {name: int(value) if name in INTEGER_FIELDS else float(value)
                   for name, value in zip(FIELDS, columns[1:])}
        except ValueError as error:
            raise ValueError(f"line {line_number}: invalid NXCHAR number") from error
        if row["version"] != 1:
            raise ValueError(f"line {line_number}: unsupported NXCHAR version")
        if previous_sequence is not None and row["sequence"] < previous_sequence:
            session += 1
            warnings.append(f"line {line_number}: sequence restarted; separate boot/session")
        previous_sequence = row["sequence"]
        if row["stage"] == 0:
            continue
        key = session, row["run"], row["trial"]
        runs.setdefault(key, []).append(row)
    return runs, warnings


def validate_trial(rows):
    reasons = []
    if not rows or rows[0]["stage"] != 1:
        reasons.append("missing settling/start record")
    if not rows or rows[-1]["stage"] != 4:
        reasons.append("trial did not complete")
    if any(row["stage"] == 5 or row["failure"] != 0 for row in rows):
        reasons.append("aborted or faulted trial")
    if rows and not 0 <= rows[0]["trial"] < 18:
        reasons.append("unknown trial id")
    if not any(row["stage"] == 2 for row in rows) or not any(row["stage"] == 3 for row in rows):
        reasons.append("missing powered or coast phase")
    if any(row["stage"] not in (1, 2, 3, 4, 5) for row in rows):
        reasons.append("unknown trial stage")
    if any(b["stage"] < a["stage"] for a, b in zip(rows, rows[1:])):
        reasons.append("trial stages moved backwards")
    if any(b["sequence"] != a["sequence"] + 1 for a, b in zip(rows, rows[1:])):
        reasons.append("missing or duplicate sequence")
    if rows and any(row["dropped"] != rows[0]["dropped"] for row in rows):
        reasons.append("dropped log frames during trial")
    finite_fields = [name for name in FIELDS if name not in INTEGER_FIELDS
                     and not name.startswith("accel_")]
    active = [row for row in rows if row["stage"] in (1, 2, 3)]
    for row in active:
        if not all(math.isfinite(row[name]) for name in finite_fields):
            reasons.append("non-finite measurement")
            break
        if row["valid_mask"] & 119 != 119:
            reasons.append("missing required sensor/motor/battery flags")
            break
        if abs(row["t_s"] - row["sensor_s"]) > .06 or abs(row["t_s"] - row["motor_s"]) > .06:
            reasons.append("stale sensor or motor observation over 60 ms")
            break
        if not 7 <= row["battery_v"] <= 15:
            reasons.append("invalid battery voltage")
            break
        if any(row[name] <= 0 for name in ("imu_scale", "forward_scale", "lateral_scale", "track_width_m")):
            reasons.append("invalid geometric scale")
            break
        if any(abs(row[name] - rows[0][name]) > 1e-9 for name in METADATA):
            reasons.append("geometry changed within trial")
            break
    # The final complete/abort record may repeat the final acquisition. It is
    # deliberately absent from derivative chronology checks.
    for clock in ("t_s", "sensor_s", "motor_s"):
        if any(not 0 < b[clock] - a[clock] <= .12 for a, b in zip(active, active[1:])):
            reasons.append(f"nonmonotonic or gapped {clock}")
    return list(dict.fromkeys(reasons))


def phase_rows(rows, stage):
    return [row for row in rows if row["stage"] == stage]


def motion_rows(rows, window=7):
    """Differentiate calibrated pod centre-travel and IMU angle, not estimator speed."""
    working = []
    for row in rows:
        item = dict(row)
        angle = row["gyro_rad"] * row["imu_scale"]
        item.update(time_s=row["sensor_s"], angle=angle,
                    forward_mm=(row["forward_m"] * row["forward_scale"] +
                                row["forward_offset_m"] * angle) * 1000,
                    lateral_mm=(row["lateral_m"] * row["lateral_scale"] -
                                row["lateral_offset_m"] * angle) * 1000)
        working.append(item)
    derivatives = [list(differentiated(working, column, window))
                   for column in ("forward_mm", "lateral_mm", "angle")]
    output = []
    for forward, lateral, angle in zip(*derivatives):
        item = dict(forward[0])
        item.update(raw_v_mmps=forward[2], raw_lateral_mmps=lateral[2], raw_omega_radps=angle[2])
        output.append(item)
    return output


def powered_samples(rows, side, window=7):
    """Exclude phase transitions, turns, stiction and large wheel/body mismatch."""
    powered = phase_rows(rows, 2)
    if not powered:
        return []
    # Raw pod derivatives provide an independent straight/slip check. Match by
    # sequence rather than pretending motor and sensor samples are simultaneous.
    motion = {row["sequence"]: row for row in motion_rows(powered, window)}
    working = []
    for row in powered:
        item = dict(row)
        item.update(time_s=row["motor_s"], speed_mmps=row[f"{side}_wheel_mps"] * 1000)
        working.append(item)
    output = []
    for centre, velocity, acceleration in differentiated(working, "speed_mmps", window):
        raw = motion.get(centre["sequence"])
        if raw is None or centre["t_s"] - powered[0]["t_s"] < .15:
            continue
        if abs(velocity) < 70 or abs(raw["raw_omega_radps"]) > .25 or abs(raw["raw_lateral_mmps"]) > 100:
            continue
        body = raw["raw_v_mmps"]
        mean_wheel = (centre["left_wheel_mps"] + centre["right_wheel_mps"]) * 500
        if abs(body - mean_wheel) > max(120, .2 * abs(body)):
            continue
        voltage = centre[f"applied_{side}_v"]
        command = centre[f"cmd_{side}_v"]
        if abs(command) < .5 or abs(voltage) < .5 or voltage * command <= 0:
            continue
        output.append({"voltage": voltage, "velocity": velocity, "acceleration": acceleration})
    return output


def least_squares(design, values, maximum_condition=100.0):
    """Column-scaled, twice-reorthogonalized QR for these small offline fits."""
    if not design or len(design) != len(values):
        raise ValueError("empty or inconsistent regression")
    width = len(design[0])
    if len(design) < width or any(len(row) != width for row in design):
        raise ValueError("insufficient regression rows")
    if not all(math.isfinite(value) for row in design for value in row) or not all(
            math.isfinite(value) for value in values):
        raise ValueError("non-finite regression")
    scales = [math.sqrt(sum(row[j] ** 2 for row in design)) for j in range(width)]
    if any(scale <= 1e-12 for scale in scales):
        raise ValueError("rank deficient regression")
    q, r = [], [[0.0] * width for _ in range(width)]
    for j in range(width):
        column = [row[j] / scales[j] for row in design]
        for _ in range(2):
            for i, basis in enumerate(q):
                projection = sum(a * b for a, b in zip(column, basis))
                r[i][j] += projection
                column = [a - projection * b for a, b in zip(column, basis)]
        length = math.sqrt(sum(value ** 2 for value in column))
        if length < 1e-10:
            raise ValueError("rank deficient regression")
        r[j][j] = length
        q.append([value / length for value in column])
    # Infinity condition of the normalized design's triangular factor; the
    # normalization makes the test independent of mm vs m unit choices.
    inverse = [[0.0] * width for _ in range(width)]
    for column in range(width):
        for i in reversed(range(width)):
            inverse[i][column] = ((1.0 if i == column else 0.0) - sum(
                r[i][j] * inverse[j][column] for j in range(i + 1, width))) / r[i][i]
    condition = max(sum(abs(v) for v in row) for row in r) * max(
        sum(abs(v) for v in row) for row in inverse)
    if condition > maximum_condition:
        raise ValueError("poorly conditioned regression")
    rhs = [sum(a * b for a, b in zip(basis, values)) for basis in q]
    coefficients = [0.0] * width
    for i in reversed(range(width)):
        coefficients[i] = (rhs[i] - sum(r[i][j] * coefficients[j]
                                      for j in range(i + 1, width))) / r[i][i]
    coefficients = [value / scale for value, scale in zip(coefficients, scales)]
    residuals = [actual - sum(x * c for x, c in zip(row, coefficients))
                 for row, actual in zip(design, values)]
    return coefficients, math.sqrt(sum(r * r for r in residuals) / len(values)), condition


def differentiated(rows, column, window=7):
    """Centred quadratic fit; caller supplies one uninterrupted trial phase."""
    if window < 5 or window % 2 != 1:
        raise ValueError("window must be odd and at least five")
    radius = window // 2
    for index in range(radius, len(rows) - radius):
        centre = rows[index]
        neighbours = rows[index - radius:index + radius + 1]
        times = [row["time_s"] - centre["time_s"] for row in neighbours]
        # Scale local time before QR to keep the polynomial well conditioned.
        duration = max(abs(value) for value in times)
        if duration <= 0:
            continue
        design = [[1, t / duration, (t / duration) ** 2] for t in times]
        coefficients, _, _ = least_squares(design, [row[column] for row in neighbours])
        yield centre, coefficients[0], coefficients[1] / duration


def fit_drive(samples):
    """Samples contain measured V, wheel mm/s and differentiated mm/s^2."""
    report = {"accepted": False, "samples": len(samples), "reasons": []}
    reasons = report["reasons"]
    if len(samples) < 40:
        reasons.append("at least 40 usable powered samples are required")
    for direction in (-1, 1):
        if sum(sample["velocity"] * direction > 0 for sample in samples) < 10:
            reasons.append("at least 10 samples in each direction are required")
    speeds = [abs(sample["velocity"]) for sample in samples]
    accelerations = [sample["acceleration"] for sample in samples]
    if not speeds or max(speeds) - min(speeds) < 150:
        reasons.append("absolute speed span below 150 mm/s")
    if not accelerations or math.sqrt(sum(a * a for a in accelerations) /
                                    len(accelerations)) < 150:
        reasons.append("acceleration excitation below 150 mm/s^2 RMS")
    if reasons:
        return report
    design = [[math.copysign(1, sample["velocity"]), sample["velocity"],
               sample["acceleration"]] for sample in samples]
    values = [sample["voltage"] for sample in samples]
    try:
        coefficients, rms, condition = least_squares(design, values)
    except ValueError as error:
        reasons.append(str(error))
        return report
    report.update(rms_voltage_v=rms, normalized_condition=condition)
    ks, kv, ka = coefficients
    if not (0 < ks < 4 and 0 < kv < .1 and 0 < ka < .05):
        reasons.append("coefficients are not positive plausible drive parameters")
    voltage_rms = math.sqrt(sum(value * value for value in values) / len(values))
    if rms > .6 or rms / max(.1, voltage_rms) > .15:
        reasons.append("voltage residual above 0.6 V or 15 percent RMS")
    if not reasons:
        report.update(accepted=True, candidate={"kS_V": ks, "kV_V_per_mmps": kv,
                                               "kA_V_per_mmps2": ka})
    return report


def coast_response(rows, column, minimum_speed, unit, mass_kg=None, window=7):
    """Fit dissipative coast decay only; zero drive voltage is not powered braking."""
    samples = []
    for row, speed, acceleration in differentiated(rows, column, window):
        if abs(speed) >= minimum_speed:
            samples.append((speed, acceleration))
    report = {"samples": len(samples), "speed_unit": unit, "accepted": False}
    if len(samples) < 8:
        report["reason"] = "insufficient moving coast samples"
        return report
    speeds = [abs(speed) for speed, _ in samples]
    decelerations = [-math.copysign(1, speed) * acceleration for speed, acceleration in samples]
    report.update(initial_speed=speeds[0], final_speed=speeds[-1],
                  median_deceleration=statistics.median(decelerations))
    if max(speeds) - min(speeds) < minimum_speed or statistics.median(decelerations) <= 0:
        report["reason"] = "coast does not show sufficient resolved decay"
        return report
    try:
        coefficients, rms, condition = least_squares([[1, speed] for speed in speeds], decelerations)
    except ValueError as error:
        report["reason"] = str(error)
        return report
    constant, rate = coefficients
    report.update(deceleration_rms=rms, normalized_condition=condition,
                  constant_deceleration=constant, velocity_decay_rate_per_s=rate)
    # Some real coast decays have near-zero constant or velocity terms. Keep
    # these diagnostic coefficients, but never identify drivetrain kA from them.
    if constant < -max(1e-5, .1 * statistics.median(decelerations)) or rate < -1e-5:
        report["reason"] = "coast decay is inconsistent with a passive linear model"
        return report
    if rms > .3 * max(minimum_speed, statistics.median(decelerations)):
        report["reason"] = "coast decay residual is too large"
        return report
    report["accepted"] = True
    if rate > .02:
        report["decay_time_constant_s"] = 1 / rate
    if mass_kg is not None and unit == "mm/s":
        report["median_effective_resisting_force_N"] = mass_kg * statistics.median(decelerations) / 1000
    return report


def analyze_coast(rows, mass_kg=None, window=7):
    coast = phase_rows(rows, 3)
    report = {"mode": "unpowered_coast", "accepted": False}
    if any(abs(row[f"cmd_{side}_v"]) > .02 for row in coast for side in ("left", "right")):
        report["reason"] = "nonzero command during coast"
        return report
    if not coast:
        report["reason"] = "missing coast"
        return report
    # Avoid electrical/kinematic switching transients. Measured terminal V can
    # include a lagged sample, so do not treat it as deliberate braking voltage.
    settled = [row for row in coast if row["t_s"] - coast[0]["t_s"] >= .10]
    motion = motion_rows(settled, window)
    kind = rows[0]["trial"] % 6
    if kind in (0, 1):
        response = coast_response(motion, "raw_v_mmps", 50, "mm/s", mass_kg, window)
        report["translation"] = response
    elif kind in (2, 3):
        response = coast_response(motion, "raw_omega_radps", .2, "rad/s", None, window)
        report["rotation"] = response
        report["yaw_inertia_kg_m2"] = None
        report["limitation"] = "Absolute yaw inertia cannot be identified from unpowered decay without known torque."
    else:
        # Forward component in a curve changes with body rotation. It is not
        # the derivative of inertial speed and must not be reported as drag force.
        report["reason"] = "curve coast retained for lateral response; not interpreted as straight drag"
        return report
    report["accepted"] = response["accepted"]
    return report


def lateral_response(trials, window=7):
    samples, lateral_speeds, drift = [], [], 0.0
    for rows in trials:
        for stage in (2, 3):
            motion = motion_rows(phase_rows(rows, stage), window)
            lateral_speeds.extend(row["raw_lateral_mmps"] for row in motion)
            for a, b in zip(motion, motion[1:]):
                drift += .5 * (abs(a["raw_lateral_mmps"]) + abs(b["raw_lateral_mmps"])) * (
                    b["time_s"] - a["time_s"])
            for row, side_speed, derivative in differentiated(motion, "raw_lateral_mmps", window):
                demand = row["raw_v_mmps"] * row["raw_omega_radps"]
                samples.append((side_speed, demand, derivative))
    report = {"accepted": False, "samples": len(samples),
              "model": "d(lateral_mmps)/dt = -decay_rate*lateral_mmps + demand_gain*v_mmps*omega_radps",
              "limitation": "Effective response on this surface/load, not absolute inertia or a tyre friction coefficient.",
              "absolute_lateral_travel_mm": drift}
    if lateral_speeds:
        report.update(peak_lateral_mmps=max(map(abs, lateral_speeds)),
                      rms_lateral_mmps=math.sqrt(sum(v * v for v in lateral_speeds) / len(lateral_speeds)))
    if len(samples) < 20 or not lateral_speeds or max(lateral_speeds) - min(lateral_speeds) < 30:
        report["reason"] = "insufficient lateral response excitation"
        return report
    try:
        coefficients, rms, condition = least_squares([[u, demand] for u, demand, _ in samples],
                                                    [derivative for _, _, derivative in samples])
    except ValueError as error:
        report["reason"] = str(error)
        return report
    decay, gain = -coefficients[0], coefficients[1]
    report.update(acceleration_rms_mmps2=rms, normalized_condition=condition)
    signal_rms = math.sqrt(sum(derivative ** 2 for _, _, derivative in samples) / len(samples))
    if not .05 < decay < 50 or abs(gain) > 10 or rms > max(40, .35 * signal_rms):
        report["reason"] = "first-order lateral model is unsupported by these measurements"
        return report
    report.update(accepted=True, decay_rate_per_s=decay, persistence_s=1 / decay, demand_gain=gain)
    return report


def analyze(text, mass_kg=None, window=7):
    if mass_kg is not None and (not math.isfinite(mass_kg) or mass_kg <= 0):
        raise ValueError("mass must be a finite positive value in kg")
    if window < 5 or window % 2 != 1:
        raise ValueError("window must be odd and at least five")
    trials, warnings = parse_console(text)
    report = {"format_version": 1, "trial_count": len(trials), "warnings": warnings,
              "mass_kg": mass_kg, "trials": [], "drive": {}, "physical_validation_required": True,
              "notes": ["Candidates are never applied automatically; validate on independent trials.",
                        "Powered response, passive coast and lateral motion are analyzed separately.",
                        "Derivative smoothing and sensor delay can bias fitted dynamics; inspect residuals.",
                        "Coast cannot identify absolute yaw inertia without an independent torque measurement."]}
    drive_samples = {"left": [], "right": []}
    curve_trials = []
    configurations = set()
    for key, rows in trials.items():
        reasons = validate_trial(rows)
        trial = {"session": key[0], "run": key[1], "trial": key[2],
                 "kind": KINDS[key[2] % 6], "tier_voltage_v": 3 + 1.5 * (key[2] // 6),
                 "accepted": not reasons, "reasons": reasons, "frames": len(rows)}
        report["trials"].append(trial)
        if reasons:
            continue
        trial["configuration"] = {name: rows[0][name] for name in METADATA}
        configurations.add(tuple(rows[0][name] for name in METADATA))
        kind = key[2] % 6
        if kind in (0, 1):
            for side in drive_samples:
                values = powered_samples(rows, side, window)
                drive_samples[side].extend(values)
                trial[f"{side}_powered_fit_samples"] = len(values)
        trial["coast"] = analyze_coast(rows, mass_kg, window)
        if kind in (4, 5):
            curve_trials.append(rows)
    report["accepted_trials"] = sum(trial["accepted"] for trial in report["trials"])
    for side, samples in drive_samples.items():
        report["drive"][side] = fit_drive(samples)
        if len(configurations) > 1:
            report["drive"][side].update(accepted=False)
            report["drive"][side].pop("candidate", None)
            report["drive"][side]["reasons"].append("geometry differs between trials; analyze each configuration separately")
    report["lateral_response"] = lateral_response(curve_trials, window) if len(configurations) <= 1 else {
        "accepted": False, "reason": "geometry differs between trials; do not pool lateral responses"}
    report["drive_candidates_available"] = all(fit["accepted"] for fit in report["drive"].values())
    return report, drive_samples


def write_outputs(directory, report, samples):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    artifacts = ("report.json", "candidates.csv", "left.csv", "right.csv")
    if any((directory / name).exists() for name in artifacts):
        raise ValueError("output already contains analysis artifacts; choose a new directory")
    (directory / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if not report["drive_candidates_available"]:
        return
    with (directory / "candidates.csv").open("w", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(("parameter", "candidate", "unit"))
        for side in ("left", "right"):
            fit = report["drive"][side]["candidate"]
            for parameter, key, unit in (("kS", "kS_V", "V"), ("kV", "kV_V_per_mmps", "V/(mm/s)"),
                                         ("kA", "kA_V_per_mmps2", "V/(mm/s^2)")):
                writer.writerow((parameter + side.capitalize(), format(fit[key], ".10g"), unit))
            with (directory / f"{side}.csv").open("w", newline="", encoding="utf-8") as wheel:
                wheel.write(WHEEL_UNITS + "\n")
                rows = csv.DictWriter(wheel, fieldnames=("voltage", "velocity", "acceleration"))
                rows.writeheader()
                rows.writerows(samples[side])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="mixed console capture containing NXCHAR_HEADER and NXCHAR")
    parser.add_argument("--output", type=Path, required=True, help="new analysis output directory")
    parser.add_argument("--mass-kg", type=float, help="optional robot plus payload mass, only for straight coast force")
    parser.add_argument("--window", type=int, default=7, help="odd local derivative window, at least five")
    arguments = parser.parse_args()
    try:
        report, samples = analyze(arguments.log.read_text(encoding="utf-8-sig"), arguments.mass_kg, arguments.window)
        write_outputs(arguments.output, report, samples)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f"{arguments.output / 'report.json'}: {report['accepted_trials']}/{report['trial_count']} complete clean trials")
    if report["drive_candidates_available"]:
        print("Candidate drive coefficients and wheel CSVs written; independent physical validation required.")
    else:
        print("No drive candidate CSV: see fit rejection reasons in report.json.")


if __name__ == "__main__":
    main()
