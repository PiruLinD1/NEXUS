"""Independent synthetic plants and malformed captures for offline NXCHAR analysis."""
import copy
import csv
import importlib.util
import io
import json
import math
from pathlib import Path
import shutil
import unittest
import uuid
from contextlib import contextmanager


spec = importlib.util.spec_from_file_location("analyze_characterization", Path(__file__).parents[1] /
                                           "tools/analyze_characterization.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


@contextmanager
def temporary_directory():
    workspace = Path(__file__).resolve().parents[1]
    # Python 3.14's private Windows mkdtemp ACL can deny a restricted token.
    # A normal workspace directory inherits the repository's writable ACL.
    target = (workspace / "tests" / "_scratch" / ("characterization-" + uuid.uuid4().hex)).resolve()
    if not target.is_relative_to(workspace):
        raise RuntimeError("temporary test directory escaped workspace")
    target.mkdir(parents=True)
    try:
        yield target
    finally:
        shutil.rmtree(target)


def base_row(sequence, trial, stage, time):
    row = {name: 0.0 for name in analysis.FIELDS}
    row.update(version=1, sequence=sequence, run=1, trial=trial, stage=stage, failure=0,
               t_s=time, sensor_s=time, motor_s=time, battery_v=12, valid_mask=127, dropped=0,
               imu_scale=1.01, forward_scale=1.02, lateral_scale=.99,
               forward_offset_m=.020, lateral_offset_m=-.070, track_width_m=.287)
    return row


def position(row, forward_mm, lateral_mm, angle):
    row.update(forward_m=(forward_mm / 1000 - row["forward_offset_m"] * angle) / row["forward_scale"],
               lateral_m=(lateral_mm / 1000 + row["lateral_offset_m"] * angle) / row["lateral_scale"],
               gyro_rad=angle / row["imu_scale"], heading_rad=angle)


def synthetic_trial(trial, start=0, sequence=0, ks=.6, kv=.006, ka=.0018, coast_rate=2.4):
    """Analytic first-order powered drive, independently different passive coast."""
    rows = []
    direction = 1 if trial % 6 in (0, 2) else -1
    voltage = 3 + 1.5 * (trial // 6)
    steady = direction * (voltage - ks) / kv
    powered_rate = kv / ka
    powered_end = 1.6
    end_speed = steady * (1 - math.exp(-powered_rate * powered_end))
    end_position = steady * (powered_end - (1 - math.exp(-powered_rate * powered_end)) / powered_rate)
    for stage, count in ((1, 20), (2, 80), (3, 85)):
        for index in range(count):
            time = start + (len(rows) + 1) * .02
            local = (index + 1) * .02
            row = base_row(sequence + len(rows), trial, stage, time)
            speed = travel = 0.0
            if stage == 2:
                speed = steady * (1 - math.exp(-powered_rate * local))
                travel = steady * (local - (1 - math.exp(-powered_rate * local)) / powered_rate)
                row.update(cmd_left_v=direction * voltage, cmd_right_v=direction * voltage,
                           applied_left_v=direction * voltage, applied_right_v=direction * voltage)
            elif stage == 3:
                speed = end_speed * math.exp(-coast_rate * local)
                travel = end_position + end_speed * (1 - math.exp(-coast_rate * local)) / coast_rate
            if trial % 6 in (2, 3):
                # The spin body's angular velocity is deliberately unrelated to
                # motor voltage parameters; it only exercises measured decay.
                angle, omega = travel / 200, speed / 200
                position(row, 0, 0, angle)
                row.update(left_wheel_mps=omega * .287 / 2, right_wheel_mps=-omega * .287 / 2,
                           omega_radps=omega)
                row["cmd_right_v"] *= -1
                row["applied_right_v"] *= -1
            else:
                position(row, travel, 0, 0)
                row.update(left_wheel_mps=speed / 1000, right_wheel_mps=speed / 1000, v_mps=speed / 1000)
            rows.append(row)
    final = dict(rows[-1])
    final.update(stage=4, sequence=rows[-1]["sequence"] + 1)
    rows.append(final)
    return rows


def capture(trials):
    destination = io.StringIO()
    writer = csv.writer(destination)
    destination.write("unrelated startup message\n")
    for rows in trials:
        writer.writerow(["NXCHAR_HEADER", *analysis.FIELDS])
        for row in rows:
            writer.writerow(["NXCHAR", *(row[name] for name in analysis.FIELDS)])
    return destination.getvalue()


def straight_trials():
    result, sequence, start = [], 0, 0.0
    for trial in (0, 1, 6, 7, 12, 13):
        rows = synthetic_trial(trial, start, sequence)
        result.append(rows)
        sequence = rows[-1]["sequence"] + 1
        start = rows[-1]["t_s"] + 1
    return result


class CharacterizationAnalysisTest(unittest.TestCase):
    def test_recovers_powered_parameters_without_fitting_coast_as_drive(self):
        report, samples = analysis.analyze(capture(straight_trials()), mass_kg=8)
        self.assertEqual(report["accepted_trials"], 6)
        self.assertTrue(report["drive_candidates_available"], report["drive"])
        for side in ("left", "right"):
            candidate = report["drive"][side]["candidate"]
            self.assertAlmostEqual(candidate["kS_V"], .6, delta=.025)
            self.assertAlmostEqual(candidate["kV_V_per_mmps"], .006, delta=.00006)
            self.assertAlmostEqual(candidate["kA_V_per_mmps2"], .0018, delta=.00006)
            self.assertTrue(all(abs(row["voltage"]) >= 3 for row in samples[side]))
        coast = report["trials"][0]["coast"]["translation"]
        self.assertTrue(coast["accepted"], coast)
        self.assertAlmostEqual(coast["decay_time_constant_s"], 1 / 2.4, delta=.02)
        self.assertGreater(coast["median_effective_resisting_force_N"], 0)

    def test_angular_coast_does_not_claim_yaw_inertia(self):
        report, _ = analysis.analyze(capture([synthetic_trial(2)]), mass_kg=8)
        coast = report["trials"][0]["coast"]
        self.assertTrue(coast["accepted"], coast)
        self.assertIsNone(coast["yaw_inertia_kg_m2"])
        self.assertNotIn("median_effective_resisting_force_N", coast["rotation"])
        self.assertFalse(report["drive_candidates_available"])

    def test_rejects_aborted_incomplete_gapped_stale_and_invalid_trials(self):
        variants = {}
        rows = synthetic_trial(0)
        altered = copy.deepcopy(rows)
        altered[-1]["stage"] = 5
        altered[-1]["failure"] = 2
        variants["abort"] = altered
        variants["incomplete"] = copy.deepcopy(rows[:-1])
        altered = copy.deepcopy(rows)
        del altered[45]
        variants["sequence"] = altered
        for label, field, value in (("drops", "dropped", 1), ("stale", "sensor_s", -100),
                                    ("invalid", "valid_mask", 1), ("nonfinite", "left_wheel_mps", float("nan")),
                                    ("geometry", "forward_scale", 2)):
            altered = copy.deepcopy(rows)
            altered[45][field] = value
            variants[label] = altered
        altered = copy.deepcopy(rows)
        for row in altered[45:]:
            for clock in ("t_s", "sensor_s", "motor_s"):
                row[clock] += .2
        variants["gap"] = altered
        for label, altered in variants.items():
            with self.subTest(label=label):
                report, samples = analysis.analyze(capture([altered]))
                self.assertEqual(report["accepted_trials"], 0)
                self.assertEqual(samples, {"left": [], "right": []})
                self.assertFalse(report["drive_candidates_available"])

    def test_complete_terminal_duplicate_and_missing_optional_accel_are_valid(self):
        rows = synthetic_trial(0)
        for row in rows:
            row["valid_mask"] &= ~8
            row["accel_x_g"] = float("nan")
        report, _ = analysis.analyze(capture([rows]))
        self.assertEqual(report["accepted_trials"], 1)
        json.dumps(report, allow_nan=False)

    def test_rejects_rank_deficiency_negative_coefficients_and_bad_residuals(self):
        deficient = [{"velocity": direction * (100 + i * 20), "acceleration": direction * (200 + i * 40),
                      "voltage": direction * 4} for direction in (-1, 1) for i in range(30)]
        self.assertFalse(analysis.fit_drive(deficient)["accepted"])
        for variant in ("negative", "residual"):
            samples = []
            for direction in (-1, 1):
                for index in range(80):
                    speed = direction * (100 + index * 12)
                    acceleration = 900 * math.sin(index * .7)
                    voltage = .6 * direction + .006 * speed + (.0018 if variant == "residual" else -.0018) * acceleration
                    if variant == "residual":
                        voltage += 2 * math.cos(index * 1.3)
                    samples.append({"velocity": speed, "acceleration": acceleration, "voltage": voltage})
            self.assertFalse(analysis.fit_drive(samples)["accepted"], variant)

    def test_rejects_nonzero_coast_without_tainting_powered_fit(self):
        trials = straight_trials()
        for rows in trials:
            for row in rows:
                if row["stage"] == 3:
                    row["cmd_left_v"] = .8
        report, _ = analysis.analyze(capture(trials))
        self.assertTrue(report["drive_candidates_available"])
        self.assertTrue(all(not trial["coast"]["accepted"] for trial in report["trials"]))

    def test_recovers_effective_lateral_response_from_raw_pods(self):
        curves = []
        for direction in (-1, 1):
            rows = []
            for index in range(120):
                t = (index + 1) * .02
                row = base_row(index, 4, 2, t)
                v, omega, decay = 700, direction * .8, 2.0
                side_terminal = -v * omega / decay
                side_travel = side_terminal * (t - (1 - math.exp(-decay * t)) / decay)
                position(row, v * t, side_travel, omega * t)
                # Intentionally inaccurate estimator fields must not influence
                # raw pod-derived identification.
                row.update(lateral_mps=9, v_mps=9, omega_radps=9)
                rows.append(row)
            curves.append(rows)
        result = analysis.lateral_response(curves)
        self.assertTrue(result["accepted"], result)
        self.assertAlmostEqual(result["persistence_s"], .5, delta=.025)
        self.assertAlmostEqual(result["demand_gain"], -1, delta=.03)

    def test_reports_only_without_candidate_csv_when_excitation_is_insufficient(self):
        report, samples = analysis.analyze(capture([synthetic_trial(0)]))
        with temporary_directory() as temporary:
            analysis.write_outputs(temporary, report, samples)
            self.assertTrue((Path(temporary) / "report.json").is_file())
            self.assertFalse((Path(temporary) / "candidates.csv").exists())
            with self.assertRaisesRegex(ValueError, "already contains"):
                analysis.write_outputs(temporary, report, samples)

    def test_candidate_units_and_wheel_csv_match_existing_calibration_tool(self):
        report, samples = analysis.analyze(capture(straight_trials()))
        with temporary_directory() as temporary:
            analysis.write_outputs(temporary, report, samples)
            folder = Path(temporary)
            candidates = list(csv.DictReader((folder / "candidates.csv").read_text().splitlines()))
            self.assertEqual(len(candidates), 6)
            kv = next(row for row in candidates if row["parameter"] == "kVLeft")
            self.assertEqual(kv["unit"], "V/(mm/s)")
            self.assertAlmostEqual(float(kv["candidate"]), .006, delta=.00006)
            self.assertEqual((folder / "left.csv").read_text().splitlines()[0], analysis.WHEEL_UNITS)

    def test_parser_refuses_truncated_or_unknown_schema(self):
        valid = capture([synthetic_trial(0)])
        with self.assertRaisesRegex(ValueError, "truncated"):
            analysis.analyze(valid + "NXCHAR,1,5\n")
        with self.assertRaisesRegex(ValueError, "unsupported NXCHAR header"):
            analysis.analyze(valid.replace("NXCHAR_HEADER,version", "NXCHAR_HEADER,format"))

    def test_rejects_wheel_pod_mismatch_and_mixed_geometry(self):
        trials = straight_trials()
        for rows in trials:
            for row in rows:
                row["left_wheel_mps"] *= 2
                row["right_wheel_mps"] *= 2
        report, samples = analysis.analyze(capture(trials))
        self.assertFalse(report["drive_candidates_available"])
        self.assertEqual(samples, {"left": [], "right": []})
        trials = straight_trials()
        for row in trials[-1]:
            row["track_width_m"] *= 1.05
        report, _ = analysis.analyze(capture(trials))
        self.assertFalse(report["drive_candidates_available"])
        self.assertNotIn("candidate", report["drive"]["left"])

    def test_accepts_final_record_without_newline_and_separates_restarts(self):
        text = capture([synthetic_trial(0), synthetic_trial(0)]).rstrip("\r\n")
        report, _ = analysis.analyze(text)
        self.assertEqual(report["accepted_trials"], 2)
        self.assertEqual([trial["session"] for trial in report["trials"]], [0, 1])


if __name__ == "__main__":
    unittest.main()
