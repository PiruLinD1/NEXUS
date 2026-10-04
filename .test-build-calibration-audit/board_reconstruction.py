"""Independently reconstruct recorded Brain translation using its recorded yaw.

This checks actual ARM output, not another host run of Estimator. Heading is
deliberately conditioned on the recorded fused heading: sensor yaw accuracy is
not tested. Input quantization is 0.0001 mm and 0.000001 degree. No interpolation,
calibration or adjustment of the source data is performed.
"""
import csv
import math
from pathlib import Path

source = Path(__file__).resolve().parents[1] / 'docs/odometry-usb-01.csv'
with source.open(newline='') as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 27838

initial = rows[0]
x, y = float(initial['x_mm']), float(initial['y_mm'])
max_path = (0.0, 0)
max_step = (0.0, 0)
sum_abs_step = 0.0
for a, b in zip(rows, rows[1:]):
    assert int(b['seq']) == int(a['seq']) + 1
    assert b['epoch'] == a['epoch']
    assert (int(b['valid']) & 19) == 19 and int(b['rejected']) == 0
    assert float(b['host_s']) > float(a['host_s'])
    h0, h1 = math.radians(float(a['heading_deg'])), math.radians(float(b['heading_deg']))
    turn = math.remainder(h1 - h0, 2 * math.pi)
    mid = h0 + turn / 2
    chord = 1 if turn == 0 else math.sin(turn / 2) / (turn / 2)
    f = float(b['f_mm']) - float(a['f_mm']) + 20 * turn
    l = float(b['l_mm']) - float(a['l_mm']) + 70 * turn
    dx = chord * (f * math.sin(mid) + l * math.cos(mid))
    dy = chord * (f * math.cos(mid) - l * math.sin(mid))
    x += dx
    y += dy
    board_dx = float(b['x_mm']) - float(a['x_mm'])
    board_dy = float(b['y_mm']) - float(a['y_mm'])
    step_error = math.hypot(dx - board_dx, dy - board_dy)
    path_error = math.hypot(x - float(b['x_mm']), y - float(b['y_mm']))
    max_step = max(max_step, (step_error, int(b['seq'])))
    max_path = max(max_path, (path_error, int(b['seq'])))
    sum_abs_step += step_error

print('File:', source.name, 'frames:', len(rows))
print('Recorded heading used directly; no test of physical yaw accuracy.')
print('Maximum single-step discrepancy mm / sequence:', max_step)
print('Maximum accumulated path discrepancy mm / sequence:', max_path)
print('Independent final reconstruction mm:', x, y)
print('Actual Brain final output mm:', rows[-1]['x_mm'], rows[-1]['y_mm'])
print('Final discrepancy mm:', math.hypot(x - float(rows[-1]['x_mm']), y - float(rows[-1]['y_mm'])))
assert max_path[0] < 0.002, 'Recorded Brain output differs beyond text-quantization tolerance'
