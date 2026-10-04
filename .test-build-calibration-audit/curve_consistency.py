"""Compare redundant measurements in old ODOM6 trace; no physical ground truth.

Motor counters may slip/backlash and raw gyro rate is not a second IMU.
The lateral channel has no independent redundant measurement in this log.
This script changes neither production code nor calibration parameters.
"""
import csv
import math
from pathlib import Path
import numpy as np

source = Path(__file__).resolve().parents[1] / "docs/odometry-usb-01.csv"
with source.open(newline="") as stream:
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)]
a = {key: np.array([row[key] for row in rows]) for key in rows[0]}
t = a["host_s"]
gyro = a["imu_deg"] * math.pi / 180
center = (a["left_mm"] + a["right_mm"]) / 2
forward_residual = a["f_mm"] + 20 * gyro - center
yaw_residual = (a["left_mm"] - a["right_mm"]) / 287 - gyro
print("start_s end_s dF_mm dL_mm dIMU_deg dDriveCenter_mm dFConstraint_mm dYawConstraint_deg")
for start, end in ((139, 143), (143, 144.35), (144.35, 145.5), (145.5, 149.5),
                   (149.5, 153), (153, 155.05), (155.05, 157.2), (157.2, 158.25),
                   (158.25, 162.5)):
    i, j = np.searchsorted(t, [start, end])
    values = [t[i], t[j], a["f_mm"][j] - a["f_mm"][i],
              a["l_mm"][j] - a["l_mm"][i], a["imu_deg"][j] - a["imu_deg"][i],
              center[j] - center[i], forward_residual[j] - forward_residual[i],
              (yaw_residual[j] - yaw_residual[i]) * 180 / math.pi]
    print(" ".join(f"{value:.6f}" for value in values))

selection = (t >= 130) & (t <= 166)
data = {key: value[selection] for key, value in a.items()}
dt = np.diff(data["host_s"])
angle = data["imu_deg"] - data["imu_deg"][0]
# On this mounted device, raw Z and get_rotation have opposite sign.
# useRawGyroRate is false in the robot configuration.
rate_angle = np.r_[0, np.cumsum(-.5 * (data["gz_dps"][1:] + data["gz_dps"][:-1]) * dt)]
error = rate_angle - angle
print("Integrated -rawZ minus get_rotation, end/min/max/RMS deg:",
      error[-1], min(error), max(error), np.sqrt(np.mean(error**2)))

def endpoint(theta_degrees, forward_counter=None):
    theta = (theta_degrees + data["heading_deg"][0]) * math.pi / 180
    dtheta = np.diff(theta)
    middle = (theta[:-1] + theta[1:]) / 2
    sinc = np.sinc(dtheta / (2 * math.pi))
    forward = np.diff(data["f_mm"] if forward_counter is None else forward_counter) + 20 * dtheta
    lateral = np.diff(data["l_mm"]) + 70 * dtheta
    return np.array([np.sum(sinc * (lateral * np.cos(middle) + forward * np.sin(middle))),
                     np.sum(sinc * (forward * np.cos(middle) - lateral * np.sin(middle)))])

drive_yaw = ((data["left_mm"] - data["left_mm"][0]) -
             (data["right_mm"] - data["right_mm"][0])) / 287 * 180 / math.pi
drive_center = (data["left_mm"] + data["right_mm"]) / 2
print("Endpoint using get_rotation:", endpoint(angle))
print("Endpoint using integrated -rawZ:", endpoint(rate_angle))
print("Endpoint using drive yaw:", endpoint(drive_yaw))
print("Endpoint using drive center and get_rotation:", endpoint(angle, drive_center - 20 * angle * math.pi / 180))
print("Endpoint using drive center and drive yaw:", endpoint(drive_yaw, drive_center - 20 * drive_yaw * math.pi / 180))

for start, end in ((144.631154, 145.191153), (152.801169, 154.991174), (161.281185, 161.951187)):
    mask = (t >= start) & (t <= end)
    indices = np.flatnonzero(mask)
    i, j = indices[0], indices[-1]
    print("Observed interval", t[i], t[j], "samples", len(indices))
    for key in ("f_mm", "l_mm", "left_mm", "right_mm", "imu_deg"):
        print(key, "net", a[key][j] - a[key][i], "range", np.ptp(a[key][mask]))
    print("Implied center lateral displacement from L and gyro, mm:",
          a["l_mm"][j] - a["l_mm"][i] + 70 * (gyro[j] - gyro[i]))
