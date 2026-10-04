import csv
import math
from pathlib import Path

rows = list(csv.DictReader(Path('docs/odometry-usb-01.csv').open(newline='')))
rows = [{k: float(v) for k, v in r.items()} for r in rows if 130 <= float(r['host_s']) <= 166]

def replay(x_offset=20, y_offset=-70, gyro_sign=1, angle_origin=None, fscale=1, lscale=1):
    x = y = 0.0
    angle = rows[0]['heading_deg'] if angle_origin is None else angle_origin
    angle = math.radians(angle)
    for a, b in zip(rows, rows[1:]):
        d = gyro_sign * math.radians(b['imu_deg'] - a['imu_deg'])
        half = d / 2
        k = math.sin(half) / half if half else 1
        f = fscale*(b['f_mm'] - a['f_mm']) + x_offset*d
        l = lscale*(b['l_mm'] - a['l_mm']) - y_offset*d
        mid = angle + half
        x += k*(l*math.cos(mid) + f*math.sin(mid))
        y += k*(f*math.cos(mid) - l*math.sin(mid))
        angle += d
    return x, y, math.degrees(angle)

print('frames', len(rows), 'time', rows[0]['host_s'], rows[-1]['host_s'])
print('raw endpoint deltas', {k: rows[-1][k]-rows[0][k] for k in ['f_mm','l_mm','left_mm','right_mm','imu_deg']})
print('raw IMU min/max', min(r['imu_deg'] for r in rows), max(r['imu_deg'] for r in rows))
print('native physical convention', replay())
print('zero offsets', replay(0,0))
print('diagnostic opposite yaw sign, NOT a proposed calibration', replay(20,-70,-1))
normal = replay()
pf=replay(0,0,lscale=0)
pl=replay(0,0,fscale=0)
print('separate projected F,L and offset terms mm',pf[:2],pl[:2],(normal[0]-pf[0]-pl[0],normal[1]-pf[1]-pl[1]))
for ox, oy in [(120,-70),(20,30),(120,30)]:
    altered = replay(ox,oy)
    measured = math.hypot(altered[0]-normal[0], altered[1]-normal[1])
    bound = 2*abs(math.sin(math.radians(rows[-1]['imu_deg']-rows[0]['imu_deg'])/2))*math.hypot(ox-20,oy+70)
    print('offset perturbation',ox,oy,'endpoint difference',measured,'exact chord bound',bound)

for n in [10,20,50]:
    pairs=[]
    for i in range(n,len(rows),n):
        a,b=rows[i-n],rows[i]
        g=b['imu_deg']-a['imu_deg']
        w=math.degrees(((b['left_mm']-a['left_mm'])-(b['right_mm']-a['right_mm']))/287)
        if abs(g)>.03 or abs(w)>.03:pairs.append((g,w))
    mg=sum(g for g,w in pairs)/len(pairs);mw=sum(w for g,w in pairs)/len(pairs)
    covariance=sum((g-mg)*(w-mw) for g,w in pairs)
    correlation=covariance/math.sqrt(sum((g-mg)**2 for g,w in pairs)*sum((w-mw)**2 for g,w in pairs))
    slope=sum(g*w for g,w in pairs)/sum(g*g for g,w in pairs)
    print('yaw IMU/drive blocks',n,'pairs',len(pairs),'correlation',correlation,'drive_yaw/imu_yaw slope',slope)
