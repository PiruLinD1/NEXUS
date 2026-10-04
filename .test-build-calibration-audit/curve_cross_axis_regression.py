"""Diagnostic cross-validation of constant F/L coupling; not calibration.

df = a*dDriveCenter + b*dL + c*dGyroRadians. c has units mm/radian.
Blocks are nonoverlapping original acquisitions, with no interpolation. The
reference motor encoders are not physical ground truth. No intercept is fitted
because these are counter increments. Train/test split is fixed at host148s.
"""
import csv
import math
from pathlib import Path
import numpy as np

source = Path(__file__).resolve().parents[1] / "docs/odometry-usb-01.csv"
with source.open(newline="") as stream:
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream) if 130 <= float(row["host_s"]) <= 166]
data = {key: np.array([row[key] for row in rows]) for key in rows[0]}

for stride in (4, 10):
    indices = np.arange(0, len(rows), stride)
    starts, ends = indices[:-1], indices[1:]
    middle = (data["host_s"][starts] + data["host_s"][ends]) / 2
    delta = {key: values[ends] - values[starts] for key, values in data.items()}
    x = np.array([.5 * (delta["left_mm"] + delta["right_mm"]), delta["l_mm"],
                  delta["imu_deg"] * math.pi / 180]).T
    y = delta["f_mm"]
    active = (np.maximum.reduce([abs(y), abs(x[:, 0]), abs(x[:, 1])]) > .5) | (abs(x[:, 2]) > .002)
    first, second = middle < 148, middle >= 148
    everyone = np.ones(len(middle), dtype=bool)
    print("\nBLOCK", stride * 10, "ms; count", len(middle), "active_count", sum(active))
    for train_name, train, test in (("all", everyone, everyone),
                                    ("first->second", first, second),
                                    ("second->first", second, first)):
        for name, columns in (("no_L", [0, 2]), ("with_L", [0, 1, 2])):
            design = x[train][:, columns]
            coefficients = np.linalg.lstsq(design, y[train], rcond=None)[0]
            errors = x[test][:, columns] @ coefficients - y[test]
            normalized = design / np.sqrt(np.mean(design**2, axis=0))
            print(train_name, name, "coef", coefficients,
                  "test_RMSE_mm", np.sqrt(np.mean(errors**2)),
                  "test_active_RMSE_mm", np.sqrt(np.mean(errors[active[test]]**2)),
                  "column_normalized_condition", np.linalg.cond(normalized))
    for name, mask in (("first", first), ("second", second)):
        print(name, "input_RMS C_mm/L_mm/G_deg",
              np.sqrt(np.mean(x[mask]**2, axis=0)) * [1, 1, 180 / math.pi])
