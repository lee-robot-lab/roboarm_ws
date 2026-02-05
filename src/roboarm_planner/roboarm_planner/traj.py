"""
traj.py

[역할]
- 관절 공간에서 q0 -> q1로 부드럽게 이동하는 joint trajectory를 생성한다.
- 현재는 Quintic time-scaling(5차 스케일링) 기반으로 q(t), dq(t) 시퀀스를 만든다.

[입력]
- q0: 시작 관절각
- q1: 목표 관절각
- T: 이동 시간(초)
- dt: 샘플링 간격(초)

[출력]
- ts: 시간 배열
- qs: 목표 관절각 시퀀스
- dqs: 목표 관절속도 시퀀스

[연관]
- plan_node.py에서 /joint_trajectory(trajectory_msgs/JointTrajectory)로 publish하기 위한 데이터 생성에 사용
"""

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
