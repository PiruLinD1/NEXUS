"""Summarize actual USBODOM records; sensor agreement is not ground truth."""
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def summarize(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        rows = [{k: float(v) for k, v in row.items() if v is not None}
                for row in csv.DictReader(stream)]
    if not rows:
        return {"file": str(path), "rows": 0}
    active = [r for r in rows if r["active"]]
    before = [r for r in rows if not active or r["board_ms"] < active[0]["board_ms"]][-50:]
    after = rows[-50:]
    def avg(records, key):
        return statistics.mean(r[key] for r in records)
    def change(key):
        return avg(after,key) - avg(before,key)
    result = {
        "file": str(path), "rows": len(rows),
        "max_cycle_ms": max(r["cycle_ms"] for r in rows),
        "max_drops": max(r["drops"] for r in rows),
        "active_rows": len(active),
        "final_stop_reason": rows[-1]["stop_reason"],
        "final_session_mm": rows[-1]["session_mm"],
    }
    if active:
        off = next((r for r in rows if r["board_ms"] > active[-1]["board_ms"]), None)
        result["powered_ms"] = off["board_ms"] - active[0]["board_ms"] if off else None
        result["settled_change"] = {k: change(k) for k in (
            "forward_mm", "lateral_right_mm", "imu_deg", "nexus_x_mm", "nexus_y_mm",
            "nexus_h_deg", "lemlib_x_mm", "lemlib_y_mm", "lemlib_h_deg",
            "motor_l1_mm", "motor_l2_mm", "motor_l3_mm", "motor_r1_mm", "motor_r2_mm", "motor_r3_mm")}
        result["settled_delta_disagreement_mm"] = math.hypot(
            change("nexus_x_mm")-change("lemlib_x_mm"),
            change("nexus_y_mm")-change("lemlib_y_mm"))
    return result


if __name__ == "__main__":
    print(json.dumps([summarize(p) for p in sys.argv[1:]], indent=2))
