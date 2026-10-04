"""Compare independent recorded channels; drive wheels are NOT ground truth.

Ports 4 and 9 (l2/r2) have different cartridges/gearing and are excluded from
all drive-distance and yaw calculations. No fit is applied to the firmware.
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np
from fit_usb_checkpoint_offsets import replay_basis


def inspect(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    if np.any(d['valid_mask'] != 31) or np.any(np.diff(d['sample_seq']) != 1):
        raise ValueError('Invalid or missing samples')
    marks = [int(np.flatnonzero(d['marker'] == m)[0]) for m in (1, 2, 3)]
    windows = [slice(i-50, i) for i in marks]
    h = np.unwrap(np.deg2rad(d['imu_deg']))
    h -= np.mean(h[windows[0]])
    f, l = d['forward_mm'], d['lateral_right_mm']
    mm_per_rev = np.pi * 82.55 * .6
    left = (d['motor_l1_rev'] + d['motor_l3_rev']) * .5 * mm_per_rev
    right = (d['motor_r1_rev'] + d['motor_r3_rev']) * .5 * mm_per_rev
    drive = (left + right) * .5
    drive_heading = (left-right)/287.0
    drive_heading -= np.mean(drive_heading[windows[0]])
    target = np.array([[600, 600], [0, 0]])

    def evaluate(ff, ll, hh=h, fo=20, lo=-70):
        b = replay_basis(ff, ll, hh)
        xy = b[:, :2] + fo*b[:, 2:4] + lo*b[:, 4:6]
        origin = np.mean(xy[windows[0]], axis=0)
        pts = np.array([np.mean(xy[w], axis=0)-origin for w in windows[1:]])
        return dict(points_mm=pts.tolist(), errors_mm=(pts-target).tolist(),
                    error_norms_mm=np.linalg.norm(pts-target, axis=1).tolist())

    comparisons = {
        'pods_native_imu': evaluate(f, l),
        'blue_drive_forward_lateral_pod_native_imu': evaluate(drive, l, fo=0),
        'blue_drive_forward_assume_no_side_slip_native_imu': evaluate(drive, np.zeros_like(l), fo=0, lo=0),
        'forward_pod_assume_no_side_slip_native_imu': evaluate(f, np.zeros_like(l), lo=0),
        'pods_blue_drive_heading_nominal_track_287mm': evaluate(f, l, drive_heading),
    }
    # Non-overlapping 200 ms windows reduce the influence of staggered caches.
    idx = np.arange(marks[0], marks[-1], 20)
    df, dl, dd, dh = [np.diff(a[idx]) for a in (f, l, drive, h)]
    active = (np.abs(dd) > 2) | (np.abs(dh) > np.deg2rad(.5))
    fits = {}
    for name, select in {
        'all_moving': active,
        'mostly_straight': active & (np.abs(dh) < np.deg2rad(.5)),
        'clockwise': active & (dh > np.deg2rad(1)),
        'counterclockwise': active & (dh < -np.deg2rad(1)),
    }.items():
        # df = scale * drive_mm - apparent_forward_offset * dh.
        a = np.column_stack((dd[select], -dh[select]))
        x, _, rank, _ = np.linalg.lstsq(a, df[select], rcond=None)
        residual = df[select] - a@x
        fits[name] = dict(windows=int(np.sum(select)), rank=int(rank),
                         pod_per_drive_scale=float(x[0]), apparent_forward_offset_mm=float(x[1]),
                         residual_rms_mm=float(np.sqrt(np.mean(residual**2))) if len(residual) else None)
    intervals = []
    for i, w in enumerate(windows[1:]):
        def change(a): return float(np.mean(a[w])-np.mean(a[windows[i]]))
        intervals.append(dict(drive_center_mm=change(drive), forward_pod_mm=change(f),
                              forward_center_with_configured_offset_mm=change(f+20*h),
                              lateral_center_with_configured_offset_mm=change(l+70*h),
                              heading_deg=change(np.rad2deg(h))))
    agreement = {}
    for side in ('l', 'r'):
        a, b, green = [d[f'motor_{side}{n}_rev'] for n in (1, 3, 2)]
        delta = (a-b)-(a[marks[0]]-b[marks[0]])
        s = slice(marks[0], marks[-1]+1)
        blue = (a+b)*.5
        blue -= np.mean(blue[windows[0]])
        g = green - np.mean(green[windows[0]])
        ratio = float(blue[s]@g[s]/(g[s]@g[s]))
        agreement[side] = dict(blue_pair_max_difference_mm=float(np.max(np.abs(delta[s]))*mm_per_rev),
                              blue_revs_per_green_reported_rev=ratio,
                              blue_minus_3_green_max_mm=float(np.max(np.abs(blue[s]-3*g[s]))*mm_per_rev))
    s = slice(marks[0], marks[-1]+1)
    heading_difference = np.rad2deg(drive_heading-h)
    drive_yaw = dict(
        difference_from_imu_degrees_min_max=[float(heading_difference[s].min()),float(heading_difference[s].max())],
        at_markers_deg=[float(np.mean(heading_difference[w])) for w in windows[1:]],
        effective_track_from_final_turn_mm=float(np.mean((left-right)[windows[-1]])-np.mean((left-right)[windows[0]]))/float(np.mean(h[windows[-1]])))
    ranges = {k: [float(d[k][s].min()), float(d[k][s].max())] for k in
              ('pitch_deg','roll_deg','gyro_x_dps','gyro_y_dps','gyro_z_dps','accel_x_g','accel_y_g','accel_z_g')}
    return dict(file=str(path), excluded_ports=[4, 9], comparisons=comparisons,
                interval_totals=intervals, blue_motor_agreement=agreement,
                blue_drive_heading_comparison=drive_yaw,
                forward_pod_vs_drive_regressions=fits, inertial_ranges=ranges,
                caveat='Drive encoder travel can slip; fitted offset includes unequal effective wheel travel. No calibration applied.')


if __name__ == '__main__':
    result = inspect(sys.argv[1])
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    else:
        print(json.dumps(result, indent=2))
