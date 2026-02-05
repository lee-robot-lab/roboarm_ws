import numpy as np

def _quintic_scaling(t, T):
    s = t / T
    s2, s3, s4, s5 = s*s, s**3, s**4, s**5
    a = 10*s3 - 15*s4 + 6*s5
    adot = (30*s2 - 60*s3 + 30*s4) / T
    return a, adot

def make_quintic_joint_traj(q0, q1, T=3.0, dt=0.01):
    q0 = np.asarray(q0)
    q1 = np.asarray(q1)
    ts = np.arange(0.0, T + 1e-12, dt)
    qs = []
    dqs = []
    dq = (q1 - q0)
    for t in ts:
        a, adot = _quintic_scaling(t, T)
        qs.append(q0 + a * dq)
        dqs.append(adot * dq)
    return ts, np.array(qs), np.array(dqs)
