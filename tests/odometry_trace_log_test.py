import csv
import importlib.util
import io
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("inspect_odometry_trace", ROOT / "tools/inspect_odometry_trace.py")
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


def frame(seq=1, host_s=.01, **changes):
    values = dict.fromkeys(trace.HEADER, 0)
    values.update(seq=seq, host_s=host_s, valid=63, heading_source=1,
                  f_before_ms=100, f_after_ms=100, l_before_ms=102, l_after_ms=102,
                  g_before_ms=98, g_after_ms=98)
    values.update(changes)
    return "NXOD," + ",".join(str(values[key]) for key in trace.HEADER) + "\n"


HEADER = "NXOD_HEADER," + ",".join(trace.HEADER) + "\n"


class OdometryTraceTest(unittest.TestCase):
    def inspect(self, *lines):
        return trace.inspect_stream(io.StringIO("".join(lines)))

    def test_schema_matches_firmware_exactly(self):
        firmware = (ROOT / "src/robot.cpp").read_text(encoding="utf-8")
        match = re.search(r'"NXOD_HEADER,([^"\n]+)\\n"', firmware)
        self.assertIsNotNone(match, "NXOD_HEADER must remain visible in robot.cpp")
        self.assertEqual(tuple(match.group(1).split(",")), trace.HEADER)

    def test_headers_interleaving_and_csv_preserve_records(self):
        output = io.StringIO()
        text = "other program message\n" + HEADER + frame(f_mm=1.2345) + "NXOD_CONFIG,startup,gyro_scale=1\n" + HEADER + frame(2, .02)
        result = trace.inspect_stream(io.StringIO(text), output)
        self.assertEqual((result.frames, result.headers, result.ignored_lines), (2, 2, 2))
        self.assertEqual(result.malformed_lines, 0)
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(tuple(rows[0]), trace.HEADER)
        self.assertEqual(rows[0]["f_mm"], "1.2345")
        self.assertEqual([row["seq"] for row in rows], ["1", "2"])
        self.assertEqual(result.skews_ms["F-L"].minimum, -2)
        self.assertEqual(result.skews_ms["F-G"].maximum, 2)

    def test_signed_packet_wrap_and_zero_exclusion(self):
        result = self.inspect(frame(f_before_ms=3, f_after_ms=3,
                                    l_before_ms=trace.UINT32_MAX - 2, l_after_ms=trace.UINT32_MAX - 2,
                                    g_before_ms=1, g_after_ms=1),
                              frame(2, .02, f_before_ms=0, f_after_ms=0))
        self.assertEqual(result.skews_ms["F-L"].minimum, 6)
        self.assertEqual(result.skews_ms["F-G"].minimum, 2)
        self.assertEqual(result.skews_ms["L-G"].minimum, -4)
        self.assertEqual(result.skews_ms["F-L"].count, 1)
        self.assertEqual(result.zero_timestamp_frames, 1)
        self.assertEqual(result.zero_timestamps, {"f": 1, "l": 0, "g": 0})

    def test_sequence_gaps_duplicates_wrap_restart_and_epochs(self):
        output = io.StringIO()
        text = "".join((frame(trace.UINT32_MAX, 10), frame(0, 10.01), frame(3, 10.04, dropped=2),
                        frame(3, 10.04, dropped=2), frame(1, 10.05, epoch=1), frame(2, 10.06, epoch=2)))
        result = trace.inspect_stream(io.StringIO(text), output)
        self.assertEqual((result.sequence_gaps, result.missing_sequences, result.duplicates, result.sequence_restarts), (1, 2, 1, 1))
        self.assertEqual(result.epoch_changes, 2)
        self.assertEqual(result.dropped_max, 2)
        self.assertEqual((result.dropped_increases, result.dropped_decreases), (2, 1))
        self.assertEqual(result.host_dt_ms.count, 3)
        self.assertAlmostEqual(result.host_dt_ms.minimum, 10)
        self.assertAlmostEqual(result.host_dt_ms.maximum, 30)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(output.getvalue())))), 6)

    def test_live_status_is_separate_from_queued_drop_counters(self):
        result = self.inspect(frame(), "NXOD_STATUS,dropped,8\n", frame(2, .02, dropped=2),
                              "NXOD_STATUS,dropped,0\n", "NXOD_STATUS,dropped,10\n")
        self.assertEqual((result.status_records, result.status_last, result.status_max, result.status_decreases), (3, 10, 10, 1))
        self.assertEqual((result.dropped_last, result.dropped_max, result.dropped_increases), (2, 2, 2))
        self.assertEqual(result.missing_sequences, 0)

    def test_invalid_packets_and_mutated_brackets_are_separate(self):
        result = self.inspect(frame(), frame(2, .02, valid=15), frame(3, .03, g_after_ms=99),
                              frame(4, .04, l_before_ms=0, l_after_ms=1, valid=0))
        self.assertEqual(result.frames, 4)
        self.assertEqual(result.invalid_packet_frames, 2)
        self.assertEqual(result.changed_bracket_frames, 2)
        self.assertEqual(result.changed_brackets, {"f": 0, "l": 1, "g": 1})
        self.assertEqual(result.zero_timestamp_frames, 1)
        # One bad third channel invalidates every pair, even F-L when only G
        # changes: no selectively optimistic interpretation of a partial frame.
        self.assertTrue(all(stats.count == 1 for stats in result.skews_ms.values()))

    def test_malformed_rows_are_counted_without_repair(self):
        malformed = ["NXOD\n", "NXOD,1,.01\n", frame()[:-1] + ",extra\n",
                     frame(f_mm="nan"), frame(imu_deg="inf"), frame(seq=-1),
                     frame(host_s=-1), frame(stationary=2), frame(valid=64),
                     frame(g_before_ms=1 << 32), "NXOD_STATUS,dropped,nope\n"]
        result = self.inspect(*malformed, frame())
        self.assertEqual(result.frames, 1)
        self.assertEqual(result.malformed_lines, len(malformed))
        self.assertEqual(len(result.errors), 5)
        report = trace.format_summary(result)
        self.assertIn("Malformed line 1", report)
        self.assertIn("further malformed lines", report)

    def test_readable_rejection_and_heading_fallback_are_not_hidden(self):
        result = self.inspect(frame(rejected=2), frame(2, .02, valid=47, heading_source=2),
                              frame(3, .03, valid=0, rejected=17, heading_source=0))
        self.assertEqual(result.invalid_sensors, {"f": 1, "l": 1, "left": 1, "right": 1, "g": 2})
        self.assertEqual(result.rejected_sensors, {"f": 1, "l": 1, "left": 0, "right": 0, "g": 1})
        self.assertEqual(result.heading_sources, {"unavailable": 1, "imu": 1, "drive_encoders": 1})
        report = trace.format_summary(result)
        self.assertIn("Rejected increment observations", report)
        self.assertIn("DRIVE_ENCODERS=1", report)

    def test_unknown_header_blocks_mislabelled_rows_until_valid_header(self):
        wrong = list(trace.HEADER)
        wrong[2], wrong[3] = wrong[3], wrong[2]
        result = self.inspect("NXOD_HEADER," + ",".join(wrong) + "\n", frame(), HEADER, frame(2, .02))
        self.assertEqual(result.frames, 1)
        self.assertEqual(result.malformed_lines, 2)
        self.assertEqual(result.headers, 1)

    def test_clock_restart_does_not_manufacture_a_dt_or_gap(self):
        result = self.inspect(frame(100, 9), frame(105, .01), frame(106, .02), frame(107, .02))
        self.assertEqual(result.host_regressions, 1)
        self.assertEqual(result.missing_sequences, 0)
        self.assertEqual(result.nonincreasing_host, 1)
        self.assertEqual(result.host_dt_ms.count, 1)
        self.assertAlmostEqual(result.host_dt_ms.minimum, 10)

    def test_cli_writes_clean_csv_and_rejects_input_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "terminal.log"
            output = Path(temporary) / "trace.csv"
            source.write_text(HEADER + frame(), encoding="utf-8")
            command = [sys.executable, str(ROOT / "tools/inspect_odometry_trace.py"), str(source)]
            completed = subprocess.run(command + ["--csv", str(output)], capture_output=True, text=True, timeout=10)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("Frames: 1", completed.stdout)
            self.assertIn("not physical sensor latency", completed.stdout)
            with output.open() as cleaned:
                self.assertEqual(len(list(csv.DictReader(cleaned))), 1)
            before = source.read_bytes()
            rejected = subprocess.run(command + ["--csv", str(source)], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(rejected.returncode, 0)
            self.assertEqual(source.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
