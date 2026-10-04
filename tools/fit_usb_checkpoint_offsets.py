"""Replay every offset pair in a stated physical box against marked references.

Also solve the continuous linear least-squares problem exactly. A bounded
grid search is not proof of calibration: reference-heading excitation is
reported separately to expose same-heading unobservability.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def replay_basis(forward, lateral, heading):
    dh=np.diff(heading); mid=(heading[1:]+heading[:-1])/2
    k=np.sinc(dh/(2*np.pi)); s=np.sin(mid); c=np.cos(mid)
    df=np.diff(forward); dl=np.diff(lateral)
    # Columns: raw X/Y, X/Y per millimetre of F offset, X/Y per mm of L offset.
    increments=np.column_stack((k*(s*df+c*dl),k*(c*df-s*dl),k*s*dh,k*c*dh,-k*c*dh,k*s*dh))
    return np.vstack((np.zeros(6),np.cumsum(increments,axis=0)))


def bounded_fit(matrix, target, lo, hi):
    raw,_,rank,singular=np.linalg.lstsq(matrix,target,rcond=None)
    candidates=[np.clip(raw,lo,hi)]
    for fixed in (0,1):
        free=1-fixed
        for bound in (lo,hi):
            column=matrix[:,free];denom=column@column
            value=(column@(target-matrix[:,fixed]*bound))/denom if denom>1e-20 else 0
            point=np.zeros(2);point[fixed]=bound;point[free]=np.clip(value,lo,hi);candidates.append(point)
    best=min(candidates,key=lambda p:float(np.sum((matrix@p-target)**2)))
    return raw,best,int(rank),singular


def analyze(csv_path, plan_path):
    plan=json.loads(Path(plan_path).read_text(encoding="utf-8-sig"))
    with Path(csv_path).open(encoding="utf-8-sig",newline="") as stream:
        records=list(csv.DictReader(stream))
    if not records or any(any(v in (None,"") for v in r.values()) for r in records):
        raise ValueError("Empty or incomplete capture; finish recording before fitting")
    d={key:np.array([float(r[key]) for r in records]) for key in records[0]}
    if any(not np.isfinite(values).all() for values in d.values()):raise ValueError("Non-finite sensor data")
    if np.any(np.diff(d["sample_seq"])!=1) or np.any(np.diff(d["board_ms"])<=0):raise ValueError("Lost samples or non-increasing board time")
    if np.any(d["valid_mask"]!=31) or np.any(np.diff(d["drops"])!=0):raise ValueError("Invalid sensors or dropped records")
    if set(d["marker"].astype(int)) != {0,1,2,3}:raise ValueError("Need exactly start/checkpoint/return markers 1/2/3")
    h=np.unwrap(np.deg2rad(d["imu_deg"]))
    windows=[]
    for point in plan["markers"]:
        index=int(np.flatnonzero(d["marker"]==point["id"])[0])
        if index<50:raise ValueError("Insufficient pre-marker stationary baseline")
        sl=slice(index-50,index)
        spans={k:float(np.ptp(d[k][sl])) for k in ("forward_mm","lateral_right_mm","imu_deg")}
        if spans["forward_mm"]>.3 or spans["lateral_right_mm"]>.3 or spans["imu_deg"]>.15:
            raise ValueError(f"Marker {point['id']} was not stationary: {spans}")
        windows.append((point,sl,index,spans))
    h0=float(np.mean(h[windows[0][1]]));h=h-h0
    basis=replay_basis(d["forward_mm"],d["lateral_right_mm"],h)
    origin=np.mean(basis[windows[0][1]],axis=0)
    points=[];matrices=[];targets=[]
    for point,sl,index,spans in windows[1:]:
        b=np.mean(basis[sl],axis=0)-origin
        matrix=np.column_stack((b[2:4],b[4:6]))
        target=np.array([point["x_mm"],point["y_mm"]])-b[:2]
        matrices.append(matrix);targets.append(target)
        observed_angle=float(np.rad2deg(np.mean(h[sl])))
        points.append({"name":point["name"],"marker":point["id"],"board_ms":float(d["board_ms"][index]),
                       "reference_xy_mm":[point["x_mm"],point["y_mm"]],"reference_heading_deg":point["heading_deg"],
                       "measured_heading_change_deg":observed_angle,
                       "heading_error_wrapped_deg":(observed_angle-point["heading_deg"]+180)%360-180,
                       "stationary_spans":spans,"raw_integrated_xy_mm":b[:2].tolist(),"offset_sensitivity":matrix.tolist()})
    A=np.vstack(matrices);b=np.concatenate(targets)
    scan=plan["offset_scan"];lo=scan["minimum_mm"];hi=scan["maximum_mm"];step=scan["step_mm"]
    raw,best,rank,singular=bounded_fit(A,b,lo,hi)
    axis=np.arange(lo,hi+step/2,step,dtype=float)
    grid_best=None;grid_score=float("inf")
    for forward in axis:
        residual=A[:,0,None]*forward+A[:,1,None]*axis[None,:]-b[:,None]
        scores=np.sum(residual*residual,axis=0);i=int(np.argmin(scores))
        if scores[i]<grid_score:grid_score=float(scores[i]);grid_best=np.array([forward,axis[i]])
    def evaluated(offsets):
        residual=(A@offsets-b).reshape(-1,2)
        return {"offsets_mm":offsets.tolist(),"errors_xy_mm":residual.tolist(),
                "errors_norm_mm":np.linalg.norm(residual,axis=1).tolist(),"sum_squared_error_mm2":float(np.sum(residual**2))}
    external_angles=np.deg2rad([p["heading_deg"]-plan["markers"][0]["heading_deg"] for p in plan["markers"][1:]])
    excitation=2*np.abs(np.sin(external_angles/2))
    return {"file":str(csv_path),"plan":str(plan_path),"rows":len(records),"points":points,
            "cycle_ms_min_max":[float(d["cycle_ms"].min()),float(d["cycle_ms"].max())],
            "reference_heading_excitation":excitation.tolist(),"designed_offsets_identifiable":bool(np.any(excitation>.1)),
            "recorded_matrix_rank":rank,"recorded_singular_values":singular.tolist(),
            "configured":evaluated(np.array(plan["configured_offsets_mm"],dtype=float)),
            "unconstrained_continuous_fit":evaluated(raw),"bounded_continuous_fit":evaluated(best),
            "grid":{"minimum_mm":lo,"maximum_mm":hi,"step_mm":step,"pairs_tested":int(len(axis)**2),"best":evaluated(grid_best)},
            "apply_calibration":False,
            "interpretation":"Coincident externally specified headings give no geometric offset excitation. Small measured heading errors can yield a numerical fit but do not make this a valid offset calibration. Ground truth alignment/measurement uncertainty is not known."}


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("capture");parser.add_argument("plan");parser.add_argument("--output")
    args=parser.parse_args();result=analyze(args.capture,args.plan);text=json.dumps(result,indent=2)
    if args.output:Path(args.output).write_text(text+"\n",encoding="utf-8")
    else:print(text)
