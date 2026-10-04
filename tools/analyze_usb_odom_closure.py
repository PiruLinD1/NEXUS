"""Offline USBODOM replay and timing sensitivity; wheel yaw is NOT ground truth.

Requires numpy. No hardware access and no calibration fitted into firmware.
Positive shift samples a sensor later in the recording (advances its signal).
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np


def analyze(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    # Interrupted host processes can leave one incomplete final disk write.
    # Account for it explicitly, rather than treating it as a complete sample.
    incomplete_tail = bool(rows and any(v is None or v == "" for v in rows[-1].values()))
    if incomplete_tail:
        rows.pop()
    if len(rows) < 2 or any(any(v is None or v == "" for v in r.values()) for r in rows):
        raise ValueError("Insufficient data or incomplete record inside the trace")
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    if any(not np.isfinite(v).all() for v in d.values()) or np.any(np.diff(d["board_ms"]) <= 0):
        raise ValueError("Non-finite data or non-increasing board clock")
    t = (d["board_ms"] - d["board_ms"][0]) / 1000
    h = np.unwrap(np.deg2rad(d["imu_deg"]))
    f, l = d["forward_mm"], d["lateral_right_mm"]

    def integrate(hh=h, ff=f, ll=l, forward_offset=20, lateral_offset=-70):
        dh = np.diff(hh)
        mid = (hh[1:] + hh[:-1]) / 2
        factor = np.sinc(dh / (2 * np.pi))
        # Clockwise heading, forward +Y, lateral +X, offsets +20/-70 mm.
        forward = np.diff(ff) + forward_offset * dh
        lateral = np.diff(ll) - lateral_offset * dh
        dx = factor * (np.sin(mid) * forward + np.cos(mid) * lateral)
        dy = factor * (np.cos(mid) * forward - np.sin(mid) * lateral)
        return np.column_stack((np.r_[0, np.cumsum(dx)], np.r_[0, np.cumsum(dy)]))

    replay = integrate()
    candidate_offsets=[]
    for fo,lo in [(20,-70),(15,-36),(15,-29),(15,-32.5),(0,0)]:
        trajectory=integrate(forward_offset=fo,lateral_offset=lo)
        candidate_offsets.append({"forward_offset_mm":fo,"lateral_offset_mm":lo,
                                  "closure_mm":trajectory[-1].tolist(),"norm_mm":float(np.linalg.norm(trajectory[-1])),
                                  "change_from_configured_mm":float(np.linalg.norm(trajectory[-1]-replay[-1]))})
    observed = np.column_stack((d["nexus_x_mm"], d["nexus_y_mm"]))
    observed -= observed[0]
    active = np.flatnonzero(d["active"])
    outbound_end = min(len(t)-1, active[-1]+101) if len(active) else len(t)-1
    stages = {"start": 0, "outbound_settled": int(outbound_end), "final": len(t)-1}
    # Only the four 600 RPM motors: the two geared 200 RPM motors have another scale.
    left = (d["motor_l1_mm"] + d["motor_l3_mm"]) / 2
    right = (d["motor_r1_mm"] + d["motor_r3_mm"]) / 2
    wheel_h = (left-right)/287
    wheel_f = (left+right)/2

    shifts = []
    for ms in range(-200, 201, 10):
        shifted = np.interp(t + ms/1000, t, h)
        end = integrate(hh=shifted)[-1]
        shifts.append({"imu_shift_ms": ms, "closure_mm": end.tolist(), "norm_mm": float(np.linalg.norm(end))})

    # Fit yaw-rate scale at each lag only over powered motion + coast. This is a
    # correlation check, not proof of sensor latency (tyre slip also changes yaw).
    delta = .1
    sample_t = np.arange(t[active[0]] if len(active) else .3, t[outbound_end]-.3, .01)
    wheel_rate = (np.interp(sample_t+delta/2,t,wheel_h)-np.interp(sample_t-delta/2,t,wheel_h))/delta
    lag_fit = []
    for ms in range(-200,201,5):
        tr = sample_t + ms/1000
        imu_rate = (np.interp(tr+delta/2,t,h)-np.interp(tr-delta/2,t,h))/delta
        scale = float(imu_rate @ wheel_rate / (wheel_rate @ wheel_rate))
        rms = float(np.sqrt(np.mean((imu_rate-scale*wheel_rate)**2)))
        lag_fit.append({"imu_shift_ms":ms,"wheel_yaw_scale":scale,"rms_deg_s":float(np.rad2deg(rms))})

    components = {}
    for key, ff, ll in [("forward",f,l*0),("lateral",f*0,l)]:
        dh=np.diff(h); mid=(h[1:]+h[:-1])/2; factor=np.sinc(dh/(2*np.pi))
        components[key]=[float(np.sum(factor*(np.sin(mid)*np.diff(ff)+np.cos(mid)*np.diff(ll)))),
                         float(np.sum(factor*(np.cos(mid)*np.diff(ff)-np.sin(mid)*np.diff(ll))))]
    components["offset"]=(replay[-1]-np.array(components["forward"])-np.array(components["lateral"])).tolist()
    return {
        "file":str(path),"rows":len(rows),"incomplete_final_record_omitted":incomplete_tail,
        "cycle_ms_min_max":[float(d["cycle_ms"].min()),float(d["cycle_ms"].max())],
        "board_interval_ms_min_max":[float(np.diff(d["board_ms"]).min()),float(np.diff(d["board_ms"]).max())],
        "drops_max":float(d["drops"].max()),
        "stages":{name:{"t_s":float(t[i]),"raw_f_l_h":[float(f[i]),float(l[i]),float(np.rad2deg(h[i]))],
                        "replay_xy_mm":replay[i].tolist(),"nexus_xy_from_start_mm":observed[i].tolist(),
                        "blue_wheel_forward_mm":float(wheel_f[i]-wheel_f[0])} for name,i in stages.items()},
        "replay_max_difference_mm":float(np.linalg.norm(replay-observed,axis=1).max()),
        "closure_components_mm":components,
        "candidate_offset_replays":candidate_offsets,
        "imu_shift_sensitivity":shifts,
        "best_powered_yaw_correlation":min(lag_fit,key=lambda r:r["rms_deg_s"]),
        "zero_shift_powered_yaw_correlation":next(r for r in lag_fit if r["imu_shift_ms"]==0),
        "caution":"Fitting closure or wheel yaw does not establish physical ground truth or justify applying a time shift."
    }


if __name__ == "__main__":
    print(json.dumps([analyze(p) for p in sys.argv[1:]], indent=2))
