"""Diagnostic sensor substitutions on a marked capture, never firmware tuning."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
from fit_usb_checkpoint_offsets import replay_basis


def inspect(path):
    with Path(path).open(encoding="utf-8-sig",newline="") as f:rs=list(csv.DictReader(f))
    d={k:np.array([float(r[k]) for r in rs]) for k in rs[0]}
    windows=[]
    for marker in (1,2,3):
        i=int(np.flatnonzero(d['marker']==marker)[0]);windows.append(slice(i-50,i))
    t=d['board_ms']/1000;h=np.deg2rad(d['imu_deg']);f=d['forward_mm'];l=d['lateral_right_mm']
    target=np.array([[600,600],[0,0]])
    def evaluate(hh,ff=f,ll=l):
        hh=hh-np.mean(hh[windows[0]])
        b=replay_basis(ff,ll,hh);xy=b[:,:2]+20*b[:,2:4]-70*b[:,4:6]
        origin=np.mean(xy[windows[0]],axis=0)
        points=np.array([np.mean(xy[w],axis=0)-origin for w in windows[1:]])
        return {'points_mm':points.tolist(),'errors_mm':(points-target).tolist(),
                'error_norms_mm':np.linalg.norm(points-target,axis=1).tolist(),
                'sse_mm2':float(np.sum((points-target)**2))}
    gyro=-np.deg2rad(d['gyro_z_dps'])
    integrated=np.r_[0,np.cumsum((gyro[1:]+gyro[:-1])*.5*np.diff(t))]
    changes=[float(np.mean(h[w])-np.mean(h[windows[0]])) for w in windows[1:]]
    angles=[float(np.mean(integrated[w])-np.mean(integrated[windows[0]])) for w in windows[1:]]
    shifts=[]
    for ms in range(-200,201,5):
        shifted=np.interp(t+ms/1000,t,h);result=evaluate(shifted);result['imu_shift_ms']=ms;shifts.append(result)
    # Separate F/IMU and L/IMU timing errors can have different signs. Scan
    # channel shifts independently; API call timing alone cannot rule them out.
    def paired_shift_scan(bound, step):
        axis=np.arange(-bound,bound+1,step)
        hh=h-np.mean(h[windows[0]])
        offset_basis=replay_basis(np.zeros_like(f),np.zeros_like(l),hh)
        offsets=20*offset_basis[:,2:4]-70*offset_basis[:,4:6]
        def endpoints(xy):
            origin=np.mean(xy[windows[0]],axis=0)
            return np.array([np.mean(xy[w],axis=0)-origin for w in windows[1:]])
        ff=[];ll=[]
        for ms in axis:
            ff.append(endpoints(replay_basis(np.interp(t+ms/1000,t,f),np.zeros_like(l),hh)[:,:2]))
            ll.append(endpoints(replay_basis(np.zeros_like(f),np.interp(t+ms/1000,t,l),hh)[:,:2]))
        xy=np.array(ff)[:,None,:,:]+np.array(ll)[None,:,:,:]+endpoints(offsets)
        scores=np.sum((xy-target)**2,axis=(2,3))
        i,j=np.unravel_index(np.argmin(scores),scores.shape)
        return dict(forward_shift_ms=int(axis[i]),lateral_shift_ms=int(axis[j]),
                    points_mm=xy[i,j].tolist(),error_norms_mm=np.linalg.norm(xy[i,j]-target,axis=1).tolist(),
                    note='Constant interpolation shifts are hypotheses, not measured device latencies.')
    # Purely diagnostic scale inferred from assuming exactly one full final turn.
    expected_turn=round(changes[-1]/(2*np.pi))*2*np.pi
    scale=expected_turn/changes[-1] if abs(changes[-1])>np.pi else None
    durations={name:{'max_us':float(np.max(d[end]-d[start])),'median_us':float(np.median(d[end]-d[start]))}
               for name,start,end in [('forward','f_before_us','f_after_us'),('lateral','l_before_us','l_after_us'),
                                      ('imu','g_before_us','g_after_us'),('drive','drive_before_us','drive_after_us'),
                                      ('whole_acquisition','f_before_us','drive_after_us')]}
    return {'file':str(path),'baseline':evaluate(h),'gyro_integrated':evaluate(integrated),
            'native_heading_changes_deg':np.rad2deg(changes).tolist(),'integrated_gyro_changes_deg':np.rad2deg(angles).tolist(),
            'best_constant_imu_shift':min(shifts,key=lambda r:r['sse_mm2']),
            'best_independent_pod_shifts_within_20ms':paired_shift_scan(20,1),
            'best_independent_pod_shifts_within_200ms':paired_shift_scan(200,5),
            'diagnostic_full_turn_gyro_scale':scale,'scaled_native':evaluate(h*scale) if scale else None,
            'call_durations':durations,
            'microsecond_sample_interval_us_min_max':[float(np.min(np.diff(d['f_before_us']))),float(np.max(np.diff(d['f_before_us'])))],
            'note':'Neither gyro integration, fitted lag nor full-turn scaling is independent ground truth; no parameter is applied.'}


if __name__=='__main__':print(json.dumps(inspect(sys.argv[1]),indent=2))
