"""Offline sensitivity of the recorded 130--166 s movement, not a calibration.

Requires numpy. Positive time shift reads a future sample of that stream.
Interpolation is only a hypothetical constant-delay model; packet reception
clocks are not physical acquisition times. A zero endpoint is conditional on
the user-established physical return, not an independently measured trajectory.
No firmware/configuration is modified.
"""
import csv
import math
from pathlib import Path
import numpy as np

source = Path(__file__).resolve().parents[1] / "docs/odometry-usb-01.csv"
with source.open(newline="") as stream:
    rows = list(csv.DictReader(stream))
t = np.array([float(row["host_s"]) for row in rows])
data = {key: np.array([float(row[key]) for row in rows]) for key in
        ("f_mm", "l_mm", "imu_deg", "left_mm", "right_mm")}
selection = (t >= 130) & (t <= 166)
times = t[selection]
initial = rows[np.flatnonzero(selection)[0]]
theta0 = math.radians(float(initial["heading_deg"]))


def integrate(f_delay=0, l_delay=0, g_delay=0, f_scale=1, l_scale=1,
              g_scale=1, f_offset=20, l_offset=-70, components=False):
    f = np.interp(times + f_delay, t, data["f_mm"]) * f_scale
    l = np.interp(times + l_delay, t, data["l_mm"]) * l_scale
    g = np.interp(times + g_delay, t, data["imu_deg"]) * math.pi / 180 * g_scale
    theta = g - g[0] + theta0
    angle = np.diff(theta)
    middle = (theta[1:] + theta[:-1]) / 2
    sinc = np.sinc(angle / (2 * math.pi))
    forward = np.array([np.sum(sinc * np.sin(middle) * np.diff(f)),
                        np.sum(sinc * np.cos(middle) * np.diff(f))])
    lateral = np.array([np.sum(sinc * np.cos(middle) * np.diff(l)),
                       -np.sum(sinc * np.sin(middle) * np.diff(l))])
    offset = np.array([
        np.sum(sinc * (f_offset * np.sin(middle) - l_offset * np.cos(middle)) * angle),
        np.sum(sinc * (f_offset * np.cos(middle) + l_offset * np.sin(middle)) * angle)])
    return (forward + lateral + offset, forward, lateral, offset) if components else forward + lateral + offset


baseline, forward, lateral, offset = integrate(components=True)
print("Input:", source.name, len(times), "samples", times[0], times[-1])
print("Relative endpoint / norm mm:", baseline, np.linalg.norm(baseline))
print("F / L / offset components mm:", forward, lateral, offset)
print("Gyro endpoint change deg:", data["imu_deg"][selection][-1] - data["imu_deg"][selection][0])
for name in ("f_delay", "l_delay", "g_delay"):
    for delay in (-0.010, 0.010):
        endpoint = integrate(**{name: delay})
        print(name, delay, "endpoint", endpoint, "change_mm", np.linalg.norm(endpoint - baseline))
for span, step in ((.01, .001), (.05, .0025), (.1, .005)):
    best = (float("inf"), None)
    largest_change = 0
    for l_delay in np.arange(-span, span + step / 2, step):
        for g_delay in np.arange(-span, span + step / 2, step):
            endpoint = integrate(l_delay=l_delay, g_delay=g_delay)
            norm = float(np.linalg.norm(endpoint))
            if norm < best[0]:
                best = (norm, (float(l_delay), float(g_delay), endpoint.tolist()))
            largest_change = max(largest_change, float(np.linalg.norm(endpoint - baseline)))
    print("Relative L/G lag grid, F fixed, span_s", span, "step_s", step,
          "best_norm/lags/endpoint", best, "max_endpoint_change_mm", largest_change)
for f_offset, l_offset in ((-20, -70), (20, 70), (0, 0)):
    print("offsets_mm", f_offset, l_offset, "endpoint", integrate(f_offset=f_offset, l_offset=l_offset))
for name, vector in (("f_scale", forward), ("l_scale", lateral)):
    best_scale = 1 - np.dot(baseline, vector) / np.dot(vector, vector)
    print("Single scale LS", name, best_scale, integrate(**{name: best_scale}))
    print("2.125 vs 2.0", name, integrate(**{name: 1.0625}))
print("Exact two-pod scales for zero endpoint with fixed gyro/offsets (not admissible):",
      np.linalg.solve(np.array([forward, lateral]).T, -offset))
best_gyro = min((float(np.linalg.norm(integrate(g_scale=scale))), float(scale))
                for scale in np.linspace(.5, 1.5, 1001))
print("Gyro scale grid0.5..1.5 step0.001, best norm/scale:", best_gyro)
g = (data["imu_deg"][selection] - data["imu_deg"][selection][0]) * math.pi / 180
left, right = data["left_mm"][selection], data["right_mm"][selection]
drive_yaw = ((left - left[0]) - (right - right[0])) / 287
print("Drive yaw vs IMU LS scale / RMS discrepancy deg:",
      np.dot(g, drive_yaw) / np.dot(g, g), np.sqrt(np.mean((drive_yaw - g)**2)) * 180 / math.pi)
f = data["f_mm"][selection] - data["f_mm"][selection][0] + 20 * g
center = ((left - left[0]) + (right - right[0])) / 2
print("Drive center vs corrected F LS scale / endpoint discrepancy mm:",
      np.dot(f, center) / np.dot(f, f), f[-1] - center[-1])
