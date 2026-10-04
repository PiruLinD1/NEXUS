"""Test a fixed 2x2 pod calibration against independent marked paths.

Uses recorded references only. This cannot distinguish physical reference
errors from sensors, and never changes firmware or a calibration profile.
"""
import json
from pathlib import Path
import numpy as np
from compare_usb_closed_paths import load
from fit_usb_checkpoint_offsets import replay_basis


def design(record):
    f, l, h = (record[k] for k in ('f', 'l', 'h'))
    z = np.zeros_like(f)
    # f_corrected = a*f+b*l; l_corrected = c*f+d*l.
    raw = [replay_basis(ff, ll, h)[:, :2]
           for ff, ll in ((f, z), (l, z), (z, f), (z, l))]
    b = replay_basis(z, z, h)
    offset = 20*b[:, 2:4]-70*b[:, 4:6]
    w = record['windows']
    def endpoints(xy):
        return np.array([np.mean(xy[v], axis=0)-np.mean(xy[w[0]], axis=0)
                         for v in w[1:]])
    return np.stack([endpoints(x) for x in raw], axis=-1), endpoints(offset)


def main():
    records = [load(Path(p), marked) for p, marked in (
        ('docs/usb-odom-20261003-181754-035.csv', False),
        ('docs/usb-turn-20261003-185527-596.csv', False),
        ('docs/usb-turn-20261003-185846-589.csv', False),
        ('docs/usb-passive-20261003-192152-873.csv', True))]
    terms = [design(r) for r in records]
    def evaluate(x):
        return [dict(file=r['name'], points_mm=(a@x+o).tolist(),
                     errors_mm=(a@x+o-r['targets']).tolist(),
                     norms_mm=np.linalg.norm(a@x+o-r['targets'], axis=1).tolist())
                for r, (a, o) in zip(records, terms)]
    # Cross-check identity against the ordinary replay before fitting.
    for r, (a, o) in zip(records, terms):
        b = replay_basis(r['f'], r['l'], r['h'])
        p = b[:, :2]+20*b[:, 2:4]-70*b[:, 4:6]
        w = r['windows']
        e = np.array([np.mean(p[v], axis=0)-np.mean(p[w[0]], axis=0) for v in w[1:]])
        np.testing.assert_allclose(a@np.array([1, 0, 0, 1])+o, e, atol=1e-8)
    fits = []
    for name, indices in (('manual_only_predict_earlier', [3]), ('all_paths', range(4))):
        a = np.vstack([terms[i][0].reshape(-1, 4) for i in indices])
        target = np.concatenate([(records[i]['targets']-terms[i][1]).ravel() for i in indices])
        x, _, rank, singular = np.linalg.lstsq(a, target, rcond=None)
        fits.append(dict(name=name, matrix=x.reshape(2, 2).tolist(), rank=int(rank),
                         singular_values=singular.tolist(), evaluation=evaluate(x)))
    result = dict(model='Fixed arbitrary 2x2 linear conversion of raw F/L, native heading, fixed offsets',
                  baseline=evaluate(np.array([1, 0, 0, 1])), fits=fits,
                  apply_calibration=False,
                  caveat='No intermediate ground truth. Equal equation weighting. Fits cannot establish mechanical or sensor failure.')
    Path('docs/usb-axis-model-20261003.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    for fit in fits:
        print(fit['name'], 'rank', fit['rank'], 'matrix', fit['matrix'])
        print([e['norms_mm'] for e in fit['evaluation']])


if __name__ == '__main__':
    main()
