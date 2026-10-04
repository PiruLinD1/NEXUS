"""Adapt passive raw readings for the pinned upstream C++ comparison harness.

x/y are zero initial-pose placeholders, NOT measured poses. The entire-file
replay is used; never interpret those columns as robot ground truth.
"""
import csv
import math
import sys
from pathlib import Path

source,target=sys.argv[1:]
with Path(source).open(encoding='utf-8-sig',newline='') as stream:
    rows=list(csv.DictReader(stream))
start=next(i for i,r in enumerate(rows) if r['marker']=='1')
end=next(i for i,r in enumerate(rows) if r['marker']=='3')
start-=25
origin=float(rows[start]['imu_deg'])
wheel_mm=82.55*math.pi*.6
with Path(target).open('x',newline='',encoding='utf-8') as stream:
    writer=csv.DictWriter(stream,fieldnames=['host_s','seq','epoch','valid','f_mm','l_mm','imu_deg','left_mm','right_mm','x_mm','y_mm','heading_deg'])
    writer.writeheader()
    for r in rows[start:end+1]:
        writer.writerow(dict(host_s=float(r['board_ms'])/1000,seq=r['sample_seq'],epoch=0,valid=31,
            f_mm=r['forward_mm'],l_mm=r['lateral_right_mm'],imu_deg=float(r['imu_deg'])-origin,
            left_mm=(float(r['motor_l1_rev'])+float(r['motor_l3_rev']))*.5*wheel_mm,
            right_mm=(float(r['motor_r1_rev'])+float(r['motor_r3_rev']))*.5*wheel_mm,
            x_mm=0,y_mm=0,heading_deg=0))
print(f'Wrote {end-start+1} raw-input rows; pose columns are placeholders, not observations.')
