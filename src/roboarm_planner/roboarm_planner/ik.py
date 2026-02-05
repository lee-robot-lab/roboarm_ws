"""
ik.py

[역할]
- position-only Inverse Kinematics(IK) 해결 모듈.
- 목표 end-effector 위치 p_des(base 기준)에 도달하는 관절각 q를 수치 최적화로 찾는다.
- 여러 초기값(multi-start)을 사용하고, 비용함수로 '최적 자세'를 선택한다.

[최적화 기준(현재 프로젝트)]
1) 위치 오차 최소화: || FK(q) - p_des ||
2) 현재 자세에서 덜 움직이기: || q - q_now ||
3) 관절 제한 회피: joint limit 근처 페널티(soft barrier)

[입력]
- robot: RobotModel (fk_pos 제공)
- p_des: 목표 위치 (x,y,z)
- q_now: 현재 관절각 (최적해 선택 기준)

[출력]
- q_star: 선택된 최적 관절각
- info: cost, 반복횟수 등 디버깅 정보

[확장 포인트]
- 특이점 회피(자코비안 SVD 기반), 충돌 회피, 토크/에너지 최소 등의 항 추가 가능
"""

import numpy as np
from scipy.optimize import least_squares

def _finite_bounds(q_now, q_min, q_max, span=2*np.pi):
    lb = q_min.copy()
    ub = q_max.copy()
    for i in range(len(lb)):
        if not np.isfinite(lb[i]) or not np.isfinite(ub[i]) or lb[i] >= ub[i]:
            # continuous / invalid limits -> 현재각 기준으로 ±span
            lb[i] = q_now[i] - span
            ub[i] = q_now[i] + span
    return lb, ub

def _soft_limit_residual(q, q_min, q_max, margin=0.15):
    # 한계에서 margin(라디안) 안쪽이면 급격히 커지는 residual
    r = np.zeros_like(q)
    eps = 1e-6
    for i in range(len(q)):
        if not (np.isfinite(q_min[i]) and np.isfinite(q_max[i]) and q_min[i] < q_max[i]):
            continue
        dmin = q[i] - q_min[i]
        dmax = q_max[i] - q[i]
        if dmin < margin:
            r[i] += (margin / (dmin + eps) - 1.0)
        if dmax < margin:
            r[i] += (margin / (dmax + eps) - 1.0)
    return r

def solve_ik_position_only(robot, p_des, q_now,
                           w_pos=1.0, w_move=0.08, w_lim=0.03,
                           n_restarts=10, seed=0):
    rng = np.random.default_rng(seed)
    nq = robot.model.nq

    q_now = np.asarray(q_now).reshape(nq)
    p_des = np.asarray(p_des).reshape(3)

    # bounds (continuous joint 대응)
    lb, ub = _finite_bounds(q_now, robot.q_min, robot.q_max, span=2*np.pi)

    def residual(q):
        p = robot.fk_pos(q)
        r_pos = (p - p_des) * np.sqrt(w_pos)
        r_move = (q - q_now) * np.sqrt(w_move)
        r_lim = _soft_limit_residual(q, robot.q_min, robot.q_max) * np.sqrt(w_lim)
        return np.concatenate([r_pos, r_move, r_lim])

    # multi-start: q_now 주변 + 랜덤 섞기
    guesses = [np.clip(q_now, lb, ub)]
    for _ in range(n_restarts - 1):
        g = rng.uniform(lb, ub)
        g = 0.6 * g + 0.4 * q_now
        guesses.append(np.clip(g, lb, ub))

    best_q = None
    best_cost = np.inf
    best_sol = None

    for q0 in guesses:
        sol = least_squares(
            residual, q0, bounds=(lb, ub),
            xtol=1e-8, ftol=1e-8, gtol=1e-8,
            max_nfev=250
        )
        if sol.success and sol.cost < best_cost:
            best_cost = sol.cost
            best_q = sol.x
            best_sol = sol

    if best_q is None:
        raise RuntimeError("IK failed for all initial guesses.")

    return best_q, {
        "cost": float(best_cost),
        "nfev": int(best_sol.nfev),
        "message": str(best_sol.message),
    }
