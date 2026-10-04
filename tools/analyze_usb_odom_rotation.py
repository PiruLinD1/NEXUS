"""Estimate pod offsets from a supervised fixed-centre manual rotation.

These are candidates only: centre translation and pod scale errors confound
the fit. No firmware, profile or robot connection is modified.
"""
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def summarize(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        records = list(csv.DictReader(stream))
    incomplete_tail = bool(records and any(v in (None, "") for v in records[-1].values()))
    if incomplete_tail:
        records.pop()
    rows = [{k: float(v) for k, v in r.items()} for r in records]
    if len(rows) < 100 or any(not math.isfinite(v) for r in rows for v in r.values()):
        raise ValueError("Need at least one second of finite telemetry")
    if any(b["board_ms"]-a["board_ms"] != 10 for a,b in zip(rows,rows[1:])):
        raise ValueError("Non-contiguous recording: inspect gaps before fitting")
    initial = rows[:50]
    def avg(rs,key):
        return statistics.mean(r[key] for r in rs)
    h0,f0,l0 = (avg(initial,k) for k in ("imu_deg","forward_mm","lateral_right_mm"))
    if max(r["imu_deg"] for r in initial)-min(r["imu_deg"] for r in initial) > .2:
        raise ValueError("Initial half-second is not stationary")
    plateaus=[]
    # Search stationary half-second windows near the cardinal headings.
    # Ignore initial heading (0) so the prior translation is not mistaken for a turn.
    for quarter in range(-4,5):
        if not quarter:
            continue
        windows=[]
        for i in range(50,len(rows)-49,10):
            window=rows[i:i+50]
            angle=avg(window,"imu_deg")-h0
            if abs(angle-90*quarter)>12:
                continue
            if max(r["imu_deg"] for r in window)-min(r["imu_deg"] for r in window)>.15:
                continue
            if any(max(r[k] for r in window)-min(r[k] for r in window)>.3
                   for k in ("forward_mm","lateral_right_mm")):
                continue
            windows.append(window)
        if not windows:
            continue
        # Earliest stable plateau avoids folding later manual repositioning into it.
        w=windows[0]
        angle=avg(w,"imu_deg")-h0
        f=avg(w,"forward_mm")-f0
        l=avg(w,"lateral_right_mm")-l0
        plateaus.append({"nominal_deg":90*quarter,"measured_deg":angle,
                         "board_ms":avg(w,"board_ms"),"delta_forward_mm":f,"delta_lateral_mm":l,
                         "candidate_forward_offset_mm":-f/math.radians(angle),
                         "candidate_lateral_offset_mm":l/math.radians(angle),
                         "nexus_x_mm":avg(w,"nexus_x_mm"),"nexus_y_mm":avg(w,"nexus_y_mm")})
    powered_stops=[]
    sequences=sorted({r["accepted"] for r in rows if r["active"]})
    previous=initial
    for seq in sequences:
        indices=[i for i,r in enumerate(rows) if r["active"] and r["accepted"]==seq]
        end=indices[-1]+1
        window=rows[end+70:end+120]
        if len(window)<50 or any(r["active"] for r in window):
            continue
        angle=avg(window,"imu_deg")-avg(previous,"imu_deg")
        if abs(angle)<10:
            continue
        df=avg(window,"forward_mm")-avg(previous,"forward_mm")
        dl=avg(window,"lateral_right_mm")-avg(previous,"lateral_right_mm")
        blue_keys=("motor_l1_mm","motor_l3_mm","motor_r1_mm","motor_r3_mm")
        drive_forward=sum(avg(window,k)-avg(previous,k) for k in blue_keys)/4
        rad=math.radians(angle)
        powered_stops.append({"sequence":int(seq),"stop_reason":int(rows[end]["stop_reason"]),
                              "powered_ms":rows[end]["board_ms"]-rows[indices[0]]["board_ms"],
                              "delta_heading_deg":angle,"heading_deg":avg(window,"imu_deg"),
                              "delta_forward_mm":df,"delta_lateral_mm":dl,
                              "blue_drive_forward_mm":drive_forward,
                              "candidate_forward_offset_mm":-df/rad,
                              "drive_corrected_forward_candidate_mm":(drive_forward-df)/rad,
                              "candidate_lateral_offset_mm":dl/rad,
                              "nexus_x_mm":avg(window,"nexus_x_mm"),"nexus_y_mm":avg(window,"nexus_y_mm"),
                              "settled_heading_span_deg":max(r["imu_deg"] for r in window)-min(r["imu_deg"] for r in window)})
        previous=window
    return {"file":str(path),"rows":len(rows),"incomplete_final_record_omitted":incomplete_tail,
            "baseline":{"imu_deg":h0,"forward_mm":f0,"lateral_right_mm":l0},
            "plateaus":plateaus,"powered_quarter_stops":powered_stops,
            "caution":"Valid offsets require a physically fixed, known centre and independently checked pod scales. These candidates alone cannot establish that condition."}


if __name__=="__main__":
    print(json.dumps([summarize(p) for p in sys.argv[1:]],indent=2))
