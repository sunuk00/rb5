"""What jnt_ref is: compare the sent TCP target, tcp_ref, jnt_ref and jnt_ang of a teleop log (log only).

The sent target (position + start rpy with rz + yaw) is solved with our DH model (IK marched from the
previous solution) and compared with jnt_ref and jnt_ang on a time axis: targets at their send time
(t_s + work_ms), RB5 states at their receive time (t_s + leader_read_ms - state_age_ms). The delay is the
shift with the smallest RMS while moving. A model-free check compares the Cartesian sent target with
tcp_ref and the FK of both joint signals. Results for 051339 / 045859: docs/decisions.md 2026-10-17.

Usage:
    python scripts/26_jnt_ref_check.py data/teleop/teleop_real_20261016_051339.csv
"""
import sys, csv
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
import yaml
from teleop.kinematics import RB5Model, rot_zyx, zyx_from_rot
path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
s = yaml.safe_load(open(path[:-4] + '_settings.yaml'))
M = RB5Model(s['tcp']['expected_offset_mm'])
col = lambda k: np.array([float(r[k]) if r[k] != '' else np.nan for r in rows])
t = col('t_s'); work = col('work_ms') / 1000; age = col('state_age_ms') / 1000; lread = col('leader_read_ms') / 1000
eng = col('clutch_engaged') == 1
tgt = np.c_[col('target_x'), col('target_y'), col('target_z')]
yaw = col('yaw_target_deg')
ref = np.c_[[col(f'jnt_ref_{i}') for i in range(6)]].T
ang = np.c_[[col(f'jnt_ang_{i}') for i in range(6)]].T
tcp_ref = np.c_[col('tcp_ref_x'), col('tcp_ref_y'), col('tcp_ref_z')]
t_cmd = t + work                       # target sent at the end of the cycle's work
t_smp = t + lread - age                # when the RB5 state sample was received

# 0. DH model vs the controller: FK(jnt_ref) vs tcp_ref (same packet)
fk = np.array([M.tcp(q)[0] for q in ref])
err = np.linalg.norm(fk - tcp_ref, axis=1)
print(f'DH model: |FK(jnt_ref) - tcp_ref| p50 {np.median(err):.3f}, max {err.max():.3f} mm')

# 1. IK of every sent target (position + rpy = start rpy with rz + yaw), marched from the previous solution
rpy0 = zyx_from_rot(M.tcp(ref[0])[1])
def ik(q, p_t, R_t, iters=60):
    for _ in range(iters):
        p, R = M.tcp(q); dr = R_t @ R.T
        e = np.r_[p_t - p, np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2]
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-8:
            break
        J = M.jacobian(q)
        q = q + np.degrees(np.linalg.solve(J, e))
    return q
q = ref[0].copy(); qik = np.zeros_like(ref); worst = 0.0
for i in range(len(rows)):
    R_t = rot_zyx(rpy0[0], rpy0[1], rpy0[2] + yaw[i])
    q = ik(q, tgt[i], R_t)
    qik[i] = q
    worst = max(worst, np.linalg.norm(M.tcp(q)[0] - tgt[i]))
print(f'IK: largest position error of the solutions {worst:.2e} mm; rpy0 {np.round(rpy0, 2).tolist()}')

# unique state samples (the same packet can be read by two cycles)
new = np.r_[True, np.abs(np.diff(t_smp)) > 1e-4] & np.isfinite(t_smp)
ts, R_, A_, E_ = t_smp[new], ref[new], ang[new], eng[new]
order = np.argsort(t_cmd)
def at(tq, x_t, x):                   # x(t) interpolated at times tq, per joint
    return np.c_[[np.interp(tq, x_t, x[:, j]) for j in range(6)]].T
vel = np.r_[np.zeros((1, 6)), np.abs(np.diff(R_, axis=0)) / np.maximum(np.diff(ts), 1e-3)[:, None]]
moving = E_ & (vel.max(axis=1) > 2.0)  # engaged and some joint above 2 deg/s
print(f'samples {len(ts)}, moving {moving.sum()} ({moving.sum() / len(ts) * 100:.0f} %)')

def best_delay(y, x_t, x, mask, name):
    ds = np.arange(-60, 201, 2) / 1000
    rms = [np.sqrt(np.mean((y[mask] - at(ts[mask] - d, x_t, x)[...])**2)) if True else 0 for d in ds]
    k = int(np.argmin(rms)); d = ds[k]
    diff = y[mask] - at(ts[mask] - d, x_t, x)
    d0 = y[mask] - at(ts[mask], x_t, x)
    print(f'{name}: best delay {d * 1000:+.0f} ms; RMS over joints {rms[k]:.3f} deg (at 0 ms {np.sqrt(np.mean(d0**2)):.3f}); '
          f'per joint RMS {np.round(np.sqrt(np.mean(diff**2, axis=0)), 3).tolist()}, max |diff| {np.abs(diff).max():.3f}')
    return d
best_delay(R_, t_cmd[order], qik[order], moving, 'jnt_ref vs IK(target)')
best_delay(A_, t_cmd[order], qik[order], moving, 'jnt_ang vs IK(target)')
best_delay(A_, ts, R_, moving, 'jnt_ang vs jnt_ref')
# at rest (released, after 1 s): static differences
rest = ~E_
if rest.any():
    print(f'released: |jnt_ref - IK(target)| max {np.abs(R_[rest] - at(ts[rest], t_cmd[order], qik[order])).max():.4f} deg; '
          f'|jnt_ang - jnt_ref| p50 {np.median(np.abs(A_[rest] - R_[rest])):.4f}, max {np.abs(A_[rest] - R_[rest]).max():.4f} deg')

print('\n--- checks')
spd = np.r_[0, np.linalg.norm(np.diff(tcp_ref, axis=0), axis=1)] / 0.02
still = spd < 1.0
print(f'FK(jnt_ref) - tcp_ref: robot still (< 1 mm/s) p50 {np.median(err[still]):.3f} max {err[still].max():.3f} mm ({still.sum()} rows); '
      f'moving p50 {np.median(err[~still]):.3f} max {err[~still].max():.3f} mm')
# is tcp_ref a few ms off jnt_ref? FK(jnt_ref(t)) vs tcp_ref(t + d)
newr = np.r_[True, np.abs(np.diff(t_smp)) > 1e-4]
tt, fkk, tcr = t_smp[newr], fk[newr], tcp_ref[newr]
for d in (-0.04, -0.02, -0.01, 0.0, 0.01, 0.02, 0.04):
    shifted = np.c_[[np.interp(tt + d, tt, tcr[:, j]) for j in range(3)]].T
    print(f'   tcp_ref shifted {d * 1000:+.0f} ms: |FK(jnt_ref) - tcp_ref| p50 {np.median(np.linalg.norm(fkk - shifted, axis=1)):.3f} mm')
# released cycles: where does the 2.6 deg come from (rest after moving vs. the very start)
rel = ~E_
dd = np.abs(R_ - at(ts, t_cmd[order], qik[order])).max(axis=1)
print(f'released: |jnt_ref - IK(target)| p50 {np.median(dd[rel]):.4f}, p95 {np.percentile(dd[rel], 95):.4f}, max {dd[rel].max():.3f} deg; '
      f'released and still (> 0.5 s after the release): max {dd[rel & np.r_[np.zeros(25, bool), rel[:-25]]].max():.4f} deg')

print('\n--- model-free (Cartesian): sent target vs tcp_ref, and FK of both joint signals')
tcr_u = tcp_ref[newr]; mov = moving
ds = np.arange(-60, 201, 2) / 1000
def lag_xyz(y, y_t, x_t, x, mask, name):
    rms = [np.sqrt(np.mean(np.sum((y[mask] - np.c_[[np.interp(y_t[mask] - d, x_t, x[:, j]) for j in range(3)]].T)**2, axis=1))) for d in ds]
    k = int(np.argmin(rms))
    print(f'{name}: best delay {ds[k] * 1000:+.0f} ms, RMS {rms[k]:.2f} mm (at 0 ms {rms[list(ds).index(0.0)]:.2f} mm)')
lag_xyz(tcr_u, ts, t_cmd[order], tgt[order], mov, 'tcp_ref vs sent target')
fk_ref = fk[newr]; fk_ang = np.array([M.tcp(qq)[0] for qq in A_])
lag_xyz(fk_ref, ts, t_cmd[order], tgt[order], mov, 'FK(jnt_ref) vs sent target')
lag_xyz(fk_ang, ts, t_cmd[order], tgt[order], mov, 'FK(jnt_ang) vs sent target')
lag_xyz(fk_ang, ts, ts, fk_ref, mov, 'FK(jnt_ang) vs FK(jnt_ref)')
print(f'released and still: |tcp_ref - target| max {np.linalg.norm(tcr_u[~E_] - np.c_[[np.interp(ts[~E_], t_cmd[order], tgt[order][:, j]) for j in range(3)]].T, axis=1).max():.3f} mm')
settled = ~E_ & np.r_[np.zeros(25, bool), ~E_[:-25]]   # released for at least 25 samples (about 0.5 s)
tq = np.c_[[np.interp(ts[settled], t_cmd[order], tgt[order][:, j]) for j in range(3)]].T
print(f'released for > 0.5 s ({settled.sum()} samples): |tcp_ref - target| max {np.linalg.norm(tcr_u[settled] - tq, axis=1).max():.4f} mm, '
      f'|jnt_ang - jnt_ref| max {np.abs(A_[settled] - R_[settled]).max():.4f} deg, p50 {np.median(np.abs(A_[settled] - R_[settled])):.4f} deg')
