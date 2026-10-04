"""Conditional timing sensitivity of the recorded path, NOT a latency estimator.

The CSV's F/L/IMU values define piecewise-linear source signals. We resample
them causally with nondecreasing acquisition times, never tune them to closure,
and flush held endpoint values. Actual inter-sample motion and physical sample
ages are unknown; packet reception clocks do not establish those ages.
"""
import bisect
import csv
import math
import random
from pathlib import Path

records = list(csv.DictReader(Path('docs/odometry-usb-01.csv').open(newline='')))
records = [{k: float(v) for k, v in r.items()} for r in records if 130 <= float(r['host_s']) <= 166]
t = [r['host_s'] for r in records]
signals = [[r[key] for r in records] for key in ['f_mm','l_mm','imu_deg']]
signals[2] = [math.radians(v) for v in signals[2]]
origin = math.radians(records[0]['heading_deg'])
poll = t + [t[-1]+.01, t[-1]+.02, t[-1]+.03]

def interpolate(channel, timestamp):
    if timestamp <= t[0]: return signals[channel][0]
    if timestamp >= t[-1]: return signals[channel][-1]
    i = bisect.bisect_right(t, timestamp)-1
    part = (timestamp-t[i])/(t[i+1]-t[i])
    return signals[channel][i]*(1-part)+signals[channel][i+1]*part

def replay(schedule):
    previous = [v[0] for v in signals]
    prior_acquired = [t[0]-.1]*3
    angle = origin
    terms = [0.0]*6  # forwardX/Y, lateralX/Y, offsetX/Y, in mm
    max_age = 0
    for i, now in enumerate(poll):
        ages = schedule(i, now)
        acquired = [max(prior_acquired[c], min(now, now-ages[c])) for c in range(3)]
        assert all(prior_acquired[c] <= acquired[c] <= now for c in range(3))
        max_age = max(max_age, *(now-a for a in acquired))
        sample = [interpolate(c, acquired[c]) for c in range(3)]
        df, dl, da = [b-a for a,b in zip(previous,sample)]
        half=da/2; scale=math.sin(half)/half if half else 1
        mid=angle+half; sn=scale*math.sin(mid); cs=scale*math.cos(mid)
        terms[0]+=df*sn;terms[1]+=df*cs
        terms[2]+=dl*cs;terms[3]-=dl*sn
        terms[4]+=(20*sn+70*cs)*da;terms[5]+=(20*cs-70*sn)*da
        angle+=da;previous=sample;prior_acquired=acquired
    assert all(abs(previous[c]-signals[c][-1])<1e-10 for c in range(3))
    return (sum(terms[::2]),sum(terms[1::2])),terms,max_age

baseline,base_terms,_=replay(lambda i,now:[0,0,0])
results=[]
def run(name,schedule,maximum_age):
    end,terms,age=replay(schedule)
    assert age<=maximum_age+1e-8
    delta=(end[0]-baseline[0],end[1]-baseline[1])
    offset_error=math.hypot(terms[4]-base_terms[4],terms[5]-base_terms[5])
    assert offset_error<1e-8 # Offset integrals telescope when endpoints align.
    result=(math.hypot(*delta),name,delta,age,[terms[i]-base_terms[i] for i in range(6)])
    results.append(result)
    return result

for c in range(3):
    for age in [.005,.010]:
        run(f'constant {"FLG"[c]} {age*1000:.0f}ms',lambda i,now,c=c,age=age:[age if j==c else 0 for j in range(3)],age)
run('all common 10ms',lambda i,now:[.010]*3,.010)
for phase in [(0,1,0),(0,0,1),(0,1,1),(1,0,1)]:
    run('alternating 0/10ms '+str(phase),lambda i,now,p=phase:[.01*((i+v)%2) for v in p],.010)
for period in [.005,.010,.020]:
    for phase in [(0,.25,.5),(0,.5,0),(0,.5,.75)]:
        run(f'held device {period*1000:.0f}ms phases{phase}',
            lambda i,now,p=phase,T=period:[now-(math.floor((now-v*T)/T)*T+v*T) for v in p],period)
for seed in range(128):
    rng=random.Random(seed)
    run(f'independent uniform 0..10ms seed{seed}',lambda i,now,r=rng:[r.random()*.01 for c in range(3)],.010)
for seed in range(128):
    rng=random.Random(seed)
    phases=[0,.00125,.0025]
    def quantized(i,now,r=rng):
        return [now-(math.floor((now-p)/.005)*.005+p-.005*r.randrange(2)) for p in phases]
    run(f'5ms sample-grid independent ages<10ms seed{seed}',quantized,.010)
for frequency in [.5,2,10,25]:
    run(f'smooth 0..10ms jitter {frequency}Hz',lambda i,now,f=frequency:[.005*(1+math.sin(2*math.pi*f*now+p)) for p in [0,2,4]],.010)

print('REFERENCE SOURCE MODEL: linear F/L/IMU between CSV samples; endpoints held and flushed.')
print('frames',len(records),'baseline relative endpoint mm',baseline)
print('All acquisition times causal/nondecreasing. No sequence chosen to make the endpoint zero.')
print('Sample schedule examples:')
for size,name,delta,age,terms in results[:20]:
    print(f'{name}: endpoint change {size:.6f}mm ({delta[0]:+.6f},{delta[1]:+.6f}), max age {age*1000:.3f}ms')
within10=[r for r in results if r[3]<=.010+1e-8]
for label,subset in [('all <=10ms cases',within10),('continuous independent random',results[20:148]),
                     ('5ms-grid independent random',results[148:276]),('all including <=20ms stress',results)]:
    worst=max(subset)
    print(f'MAX OBSERVED {label}: {worst[0]:.6f}mm; {worst[1]}; deltaXY={worst[2]}; term deltas={worst[4]}')

# Conditional universal upper bound for this source model, not just the trials.
# Each resampled encoder interval spans source times within [pollPrevious-D,pollNow].
# The assigned gyro angle is a convex combination of values in that same interval.
# An encoder differential can therefore be paired with an angle no farther than
# maxPollInterval+D away. Bound each rotated differential by 2*sin(epsilon/2)*|ds|.
# Monotone acquisition times partition the source trajectory, so total variation
# is counted once. Offset terms cancel exactly at the common endpoints.
max_dt=max(b-a for a,b in zip(t,t[1:]))
max_omega=max(abs(b-a)/(tb-ta) for a,b,ta,tb in zip(signals[2],signals[2][1:],t,t[1:]))
tv=[sum(abs(b-a) for a,b in zip(v,v[1:])) for v in signals[:2]]
print('source TV F/L mm',tv,'source max |omega| deg/s',math.degrees(max_omega),'max poll ms',max_dt*1000)
for D in [.005,.010,.020]:
    W=max_dt+D
    global_bound=2*math.sin(min(math.pi,max_omega*W)/2)*sum(tv)
    local_bound=0
    for i in range(len(t)-1):
        a,b=t[i],t[i+1]
        lo=bisect.bisect_left(t,a-W);hi=bisect.bisect_right(t,b+W)
        values=signals[2][lo:hi]+[interpolate(2,a-W),interpolate(2,b+W)]
        segment_min=min(signals[2][i:i+2]);segment_max=max(signals[2][i:i+2])
        eps=min(max_omega*W,max(max(values)-segment_min,segment_max-min(values)),math.pi)
        variation=abs(signals[0][i+1]-signals[0][i])+abs(signals[1][i+1]-signals[1][i])
        local_bound+=2*math.sin(eps/2)*variation
    print(f'CONDITIONAL UPPER BOUND arbitrary monotone ages0..{D*1000:.0f}ms: local {local_bound:.6f}mm; global {global_bound:.6f}mm; includes sampled-integration interval ambiguity')

# Tighter discrete bound which is zero at D=0. Write the encoder timing change
# as F'_i=F_i-d_i. Summation by parts makes its position error sum d_i(A_next-A_i),
# where A_i is the original chord projection coefficient. Bound |d_i| by the
# source's maximum past-D excursion. Then account separately for the changed
# angle coefficient: |A'_i-A_i| <= (|e_{i-1}|+|e_i|)/2, with e_i the angle's
# past-D excursion. Monotone encoder acquisition partitions its total variation;
# a source differential can belong only to host intervals ending between its
# source time and source time+D+max_dt. This avoids counting its travel repeatedly.
reference_angles=[interpolate(2,now)-signals[2][0]+origin for now in poll]
coefficients=[(0.0,0.0)]
for a,b in zip(reference_angles,reference_angles[1:]):
    half=(b-a)/2;k=math.sin(half)/half if half else 1
    coefficients.append((k*math.sin((a+b)/2),k*math.cos((a+b)/2)))
for D in [0,.005,.010,.020]:
    deviations=[]
    for now in poll:
        lo=bisect.bisect_left(t,now-D);hi=bisect.bisect_right(t,now)
        deviations.append([max(abs(interpolate(c,now)-v) for v in
            signals[c][lo:hi]+[interpolate(c,now-D),interpolate(c,now)]) for c in range(3)])
    counter_bound=0
    for i in range(1,len(poll)-1):
        coefficient_change=math.dist(coefficients[i],coefficients[i+1])
        counter_bound+=(deviations[i][0]+deviations[i][1])*coefficient_change
    weights=[0]+[(deviations[i-1][2]+deviations[i][2])/2 for i in range(1,len(poll))]
    angle_bound=0
    for i in range(len(t)-1):
        first=max(1,bisect.bisect_left(poll,t[i]))
        last=bisect.bisect_right(poll,t[i+1]+D+max_dt)
        weight=max(weights[first:last],default=0)
        variation=abs(signals[0][i+1]-signals[0][i])+abs(signals[1][i+1]-signals[1][i])
        angle_bound+=weight*variation
    print(f'TIGHTER CONDITIONAL DISCRETE BOUND age0..{D*1000:.0f}ms: {counter_bound+angle_bound:.6f}mm (counter {counter_bound:.6f}, angle {angle_bound:.6f}); offset zero')

# Raw gyro rates are a different SDK signal; compare dynamics without declaring
# either one a physical ground-truth clock. Best correlation does not establish
# the acquisition age of the angle or the tracking wheels.
derivative=[(signals[2][i]-signals[2][i-1])/(t[i]-t[i-1])*180/math.pi for i in range(1,len(t))]
for key in ['gx_dps','gy_dps','gz_dps']:
    scores=[]
    for shift in range(-10,11):
        pairs=[(derivative[i-1],records[i+shift][key]) for i in range(11,len(t)-11)]
        ma=sum(a for a,b in pairs)/len(pairs);mb=sum(b for a,b in pairs)/len(pairs)
        aa=sum((a-ma)**2 for a,b in pairs);bb=sum((b-mb)**2 for a,b in pairs)
        corr=sum((a-ma)*(b-mb) for a,b in pairs)/math.sqrt(aa*bb) if aa*bb else 0
        scores.append((abs(corr),corr,shift))
    print('raw gyro dynamics',key,'best |correlation|, signed correlation, raw-rate index shift',max(scores),'zero-shift',scores[10])
