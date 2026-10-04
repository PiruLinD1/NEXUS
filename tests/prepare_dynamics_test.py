import argparse
import csv
import importlib.util
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("prepare_dynamics", Path(__file__).parents[1] / "tools/prepare_dynamics.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
CALIBRATE = None


class LogPreparationTest(unittest.TestCase):
    def rows(self):
        return [{"motor_time": i * .05, "time": i * .05, "left_motor_mmps": 100 + i * 25,
                 "left_terminal_v": 4.0, "slip": 0.0, "health": 0} for i in range(20)]

    def test_recovers_known_acceleration(self):
        result = module.prepare(self.rows(), "left")
        self.assertEqual(len(result), 14)
        self.assertTrue(all(abs(row["acceleration"] - 500) < 1e-8 for row in result))
        self.assertEqual(result[0]["velocity"], 175)

    def test_drops_slip_and_missing_data(self):
        rows = self.rows()
        rows[8]["slip"] = .8
        rows[14]["left_motor_mmps"] = float("nan")
        result = module.prepare(rows, "left")
        self.assertLess(len(result), 14)
        self.assertTrue(all(abs(row["acceleration"] - 500) < 1e-8 for row in result))

    def test_rejects_nonmonotonic_clock(self):
        rows = self.rows()[:7]
        rows[3]["motor_time"] = -1
        self.assertEqual(module.prepare(rows, "left"), [])

    def test_rejects_invalid_or_unsynchronized_quality_evidence(self):
        for column, value in (("time", float("nan")), ("time", float("inf")),
                              ("time", -1), ("health", float("nan")),
                              ("health", 3), ("slip", -.1)):
            with self.subTest(column=column, value=value):
                rows = self.rows()[:7]
                # The edge contributes to acceleration just like the centre.
                rows[0][column] = value
                self.assertEqual(module.prepare(rows, "left"), [])

    def test_rejects_legacy_metre_headers(self):
        rows = self.rows()
        for row in rows:
            row["left_motor_mps"] = row.pop("left_motor_mmps") / 1000
        with self.assertRaisesRegex(ValueError, "expected left_motor_mmps"):
            module.prepare(rows, "left")

    def test_output_declares_units(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "wheel.csv"
            module.write_wheel_csv(path, module.prepare(self.rows(), "left"))
            with path.open() as source:
                self.assertEqual(source.readline().strip(), module.WHEEL_UNITS)
                rows = list(csv.DictReader(source))
            self.assertEqual(float(rows[0]["velocity"]), 175)
            self.assertAlmostEqual(float(rows[0]["acceleration"]), 500)


class CalibrationCliTest(unittest.TestCase):
    def setUp(self):
        if CALIBRATE is None:
            self.skipTest("pass --calibrate path/to/nexus_calibrate to exercise the compiled CLI")
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.profile = self.directory / "nexus.cfg"

    def run_cli(self, *arguments, success=True):
        result = subprocess.run([str(CALIBRATE), *map(str, arguments)],
                                capture_output=True, text=True, timeout=15)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0)
        return result

    def write_csv(self, name, header, rows):
        path = self.directory / name
        with path.open("w", newline="") as destination:
            writer = csv.writer(destination)
            writer.writerow(header.split(","))
            writer.writerows(rows)
        return path

    def test_wheel_mm_inputs_report_mm_coefficients_and_store_si(self):
        paths = []
        for side, ks, kv, ka in (("left", .61, .0058, .00072), ("right", .47, .0062, .00084)):
            rows = []
            for direction in (-1, 1):
                for speed in range(1, 19):
                    for acceleration in range(-4, 5):
                        velocity = direction * (100 + speed * 65)
                        derivative = acceleration * 420
                        rows.append({"voltage": ks * direction + kv * velocity + ka * derivative,
                                     "velocity": velocity, "acceleration": derivative})
            path = self.directory / f"{side}.csv"
            module.write_wheel_csv(path, rows)
            paths.append(path)
        result = self.run_cli("fit-wheel", *paths, self.profile, "123")
        for side, ks, kv, ka in (("left", .61, .0058, .00072), ("right", .47, .0062, .00084)):
            match = re.search(side + r" kS=([^ ]+) V kV=([^ ]+) V/\(mm/s\) kA=([^ ]+)", result.stdout)
            self.assertIsNotNone(match, result.stdout)
            for actual, expected in zip(map(float, match.groups()), (ks, kv, ka)):
                self.assertAlmostEqual(actual, expected, places=8)
        # Published v1 binary layout keeps SI coefficients for firmware compatibility.
        values = struct.unpack_from("<12d", self.profile.read_bytes(), 24)
        for actual, expected in zip(values[6:], (.61, 5.8, .72, .47, 6.2, .84)):
            self.assertAlmostEqual(actual, expected, places=8)

    def test_travel_and_rotation_mm_inputs_report_mm_and_store_si(self):
        for axis, scale in (("forward", 1.031), ("lateral", .982)):
            rows = [(direction * (500 + i * 110), direction * (500 + i * 110) * scale)
                    for direction in (-1, 1) for i in range(10)]
            path = self.write_csv(f"{axis}.csv", "measured_mm,reference_mm", rows)
            baseline = [self.profile] if self.profile.exists() else []
            self.run_cli("fit-travel", axis, path, self.profile, "123", *baseline)
        rows = []
        for direction in (-1, 1):
            for i in range(9):
                angle = direction * (.7 + .3 * i)
                rows.append((angle, angle / 1.012, -29 * angle / 1.031,
                             54 * angle / .982, 302 * angle / 2, -302 * angle / 2))
        path = self.write_csv("rotation.csv", "reference_angle_rad,gyro_angle_rad,forward_travel_mm,"
                              "lateral_travel_mm,left_travel_mm,right_travel_mm", rows)
        result = self.run_cli("fit-rotation", path, self.profile, "123", self.profile)
        for key, expected in (("track_width_mm", 302), ("forward_offset_mm", 29), ("lateral_offset_mm", 54)):
            match = re.search(key + r"=([^\s]+)", result.stdout)
            self.assertIsNotNone(match, result.stdout)
            self.assertAlmostEqual(float(match.group(1)), expected, places=7)
        values = struct.unpack_from("<12d", self.profile.read_bytes(), 24)
        for actual, expected in zip(values[:6], (1.031, .982, 1.012, .029, .054, .302)):
            self.assertAlmostEqual(actual, expected, places=8)

    def test_rejects_legacy_wheel_csv_without_unit_declaration(self):
        path = self.write_csv("legacy.csv", "voltage,velocity,acceleration", [(4, .5, .8)])
        result = self.run_cli("fit-wheel", path, path, self.profile, "123", success=False)
        self.assertIn("expected unit declaration", result.stderr)
        self.assertFalse(self.profile.exists())

    def test_new_gyro_scale_invalidates_geometry_that_was_not_remeasured(self):
        travel = self.write_csv("travel.csv", "measured_mm,reference_mm",
                                [(direction * (500 + i * 100), direction * (500 + i * 100))
                                 for direction in (-1, 1) for i in range(6)])
        self.run_cli("fit-travel", "forward", travel, self.profile, "123")
        self.run_cli("fit-travel", "lateral", travel, self.profile, "123", self.profile)
        header = ("reference_angle_rad,gyro_angle_rad,forward_travel_mm,"
                  "lateral_travel_mm,left_travel_mm,right_travel_mm")
        angles = [direction * (.7 + .3 * i) for direction in (-1, 1) for i in range(9)]
        rotation = self.write_csv("rotation.csv", header,
                                  [(a, a, -29 * a, 54 * a, 151 * a, -151 * a) for a in angles])
        self.run_cli("fit-rotation", rotation, self.profile, "123", self.profile)
        self.assertEqual(struct.unpack_from("<I", self.profile.read_bytes(), 20)[0], 63)

        # Gyro-only data cannot validate geometry previously obtained relative
        # to a gyro scale of 1.0 (including an automatic Brain calibration).
        gyro = self.write_csv("gyro.csv", header,
                              [(a, a / 1.05, "nan", "nan", "nan", "nan") for a in angles])
        self.run_cli("fit-rotation", gyro, self.profile, "123", self.profile)
        self.assertEqual(struct.unpack_from("<I", self.profile.read_bytes(), 20)[0], 7)
        self.assertAlmostEqual(struct.unpack_from("<12d", self.profile.read_bytes(), 24)[2], 1.05)

    def test_rejects_legacy_travel_header(self):
        path = self.write_csv("legacy.csv", "measured,reference", [(1, 1.024)])
        result = self.run_cli("fit-travel", "forward", path, self.profile, "123", success=False)
        self.assertIn("measured_mm,reference_mm", result.stderr)
        self.assertFalse(self.profile.exists())

    def test_distance_mm_fit_reports_explicit_sensor_config_without_profile(self):
        rows = [(100 + distance * 180, 1.035 * (100 + distance * 180) - 14
                 + (150 if repeat == 0 and distance % 2 == 0 else 0))
                for distance in range(9) for repeat in range(6)]
        path = self.write_csv("distance.csv", "measured_mm,reference_mm", rows)
        # This command must neither overwrite nor extend the v1 binary profile.
        self.profile.write_bytes(b"existing profile")
        result = self.run_cli("fit-distance", path)
        for key, expected, tolerance in (("distance_scale", 1.035, .001),
                                         ("distance_offset_mm", -14, 1),
                                         ("measured_min_mm", 100, 1e-8),
                                         ("measured_max_mm", 1540, 1e-8)):
            match = re.search(key + r"=([^\s]+)", result.stdout)
            self.assertIsNotNone(match, result.stdout)
            self.assertAlmostEqual(float(match.group(1)), expected, delta=tolerance)
        self.assertIn(".distanceScale = ", result.stdout)
        self.assertIn(".distanceOffset = ", result.stdout)
        self.assertIn(".minimumDistance = ", result.stdout)
        self.assertIn(".maximumDistance = ", result.stdout)
        self.assertIn("Fit residuals do not establish runtime uncertainty", result.stdout)
        self.assertEqual(self.profile.read_bytes(), b"existing profile")
        self.assertEqual(set(self.directory.iterdir()), {path, self.profile})

    def test_distance_fit_rejects_degenerate_data_and_old_headers(self):
        path = self.write_csv("fixed.csv", "measured_mm,reference_mm", [(500, 515)] * 20)
        result = self.run_cli("fit-distance", path, success=False)
        self.assertIn("insufficient excitation", result.stdout)
        self.assertNotIn("distance_scale=", result.stdout)
        legacy = self.write_csv("legacy.csv", "measured,reference", [(0.5, .515)] * 20)
        result = self.run_cli("fit-distance", legacy, success=False)
        self.assertIn("measured_mm,reference_mm", result.stderr)
        self.assertFalse(self.profile.exists())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--calibrate", type=Path)
    options, remaining = parser.parse_known_args()
    CALIBRATE = options.calibrate
    unittest.main(argv=[sys.argv[0], *remaining])
