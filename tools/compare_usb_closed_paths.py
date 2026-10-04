"""Cross-check fixed sensor-model hypotheses on separate recorded paths.

Manual alignment supplies endpoint references, not intermediate ground truth.
No result from this script is written to a robot calibration.
"""
import csv
import json
from pathlib import Path
import numpy as np
from fit_usb_checkpoint_offsets import replay_basis


def load(path, marked=False):
    with Path(path).open(encoding='utf-8-sig',newline='') as stream:
        rows=list(csv.DictReader(stream))
    if any(any(v is None or v=='' for v in r.values()) for r in rows):
        raise ValueError('Incomplete trace: '+str(path))
    d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    # UD1's seq is a command sequence, not an acquisition sequence. Its board
    # timestamp and cycle field independently establish contiguous samples.
    discontinuity=(np.any(np.diff(d['sample_seq'])!=1) if marked else
                   np.any(np.diff(d['board_ms'])!=d['cycle_ms'][1:]))
    if discontinuity or np.any(d['drops']!=0):
        raise ValueError('Dropped frames: '+str(path))
    windows=[slice(0,50),slice(len(rows)-50,len(rows))]
    if marked:
        windows=[slice(int(np.flatnonzero(d['marker']==m)[0])-50,int(np.flatnonzero(d['marker']==m)[0])) for m in (1,2,3)]
    h=np.unwrap(np.deg2rad(d['imu_deg']))
    h-=np.mean(h[windows[0]])
    return dict(name=path.name,f=d['forward_mm'],l=d['lateral_right_mm'],h=h,windows=windows,
                targets=np.array([[600,600],[0,0]]) if marked else np.array([[0,0]]))


def terms(record, gyro_scale=1):
    f,l,h=record['f'],record['l'],record['h']*gyro_scale
    # Separate forward/lateral scale, F leakage into lateral, and offsets.
    z=np.zeros_like(f)
    forward=replay_basis(f,z,h)[:,:2]
    lateral=replay_basis(z,l,h)[:,:2]
    cross=replay_basis(z,f,h)[:,:2]
    b=replay_basis(z,z,h)
    offset=20*b[:,2:4]-70*b[:,4:6]
    def points(a):
        w=record['windows']
        return np.array([np.mean(a[v],axis=0)-np.mean(a[w[0]],axis=0) for v in w[1:]])
    return np.stack((points(forward),points(lateral),points(cross)),axis=-1),points(offset)


def evaluate(records, coefficients, gyro_scale=1):
    results=[]
    for r in records:
        a,o=terms(r,gyro_scale)
        p=a@coefficients+o
        results.append(dict(file=r['name'],points_mm=p.tolist(),errors_mm=(p-r['targets']).tolist(),
                            norms_mm=np.linalg.norm(p-r['targets'],axis=1).tolist()))
    return results


def main():
    paths=[('docs/usb-odom-20261003-181754-035.csv',False),
           ('docs/usb-turn-20261003-185527-596.csv',False),
           ('docs/usb-turn-20261003-185846-589.csv',False),
           ('docs/usb-passive-20261003-192152-873.csv',True)]
    rs=[load(Path(p),m) for p,m in paths]
    output=dict(baseline=evaluate(rs,np.array([1,1,0])),fits=[],
                warning='Endpoint fits are diagnostic only. Manual endpoint alignment and heading errors remain unknown. No calibration applied.')
    for name,train,test in [('all_paths',rs,rs),('earlier_paths_predict_manual',rs[:3],rs[3:]),('manual_predict_earlier',rs[3:],rs[:3])]:
        for columns in ([0,1],[0,1,2]):
            matrices=[];targets=[]
            for r in train:
                a,o=terms(r);matrices.append(a.reshape(-1,3)[:,columns]);targets.append((r['targets']-o).ravel())
            matrix=np.vstack(matrices);target=np.concatenate(targets)
            x,_,rank,singular=np.linalg.lstsq(matrix,target,rcond=None)
            full=np.zeros(3);full[columns]=x
            output['fits'].append(dict(name=name,model='scales' if len(columns)==2 else 'scales_and_forward_leak_into_lateral',
                                      has_nonzero_reference=bool(any(np.any(r['targets']!=0) for r in train)),
                                      apply_calibration=False,
                                      forward_scale=float(full[0]),lateral_scale=float(full[1]),lateral_leak_per_forward=float(full[2]),
                                      rank=int(rank),singular_values=singular.tolist(),test=evaluate(test,full)))
    return output


if __name__=='__main__':
    result=main()
    Path('docs/usb-closed-path-models-20261003.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for fit in result['fits']:
        print(fit['name'],fit['model'],[round(fit[k],4) for k in ('forward_scale','lateral_scale','lateral_leak_per_forward')],
              [r['norms_mm'] for r in fit['test']])
