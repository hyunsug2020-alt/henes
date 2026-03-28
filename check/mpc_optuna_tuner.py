#!/usr/bin/env python3
"""
mpc_optuna_tuner.py
====================
실제 주행 CSV 데이터 기반 LTV-MPC 파라미터 Optuna 최적화

사용법:
  python3 mpc_optuna_tuner.py --csv 20260328_034113_mpc.csv --path 3_filtered.txt --trials 300
"""
import argparse
import math
import os
import sys
import warnings
import numpy as np
import pandas as pd
import scipy.sparse as sp

warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────────────────────
# LTV-MPC Python 구현 (C++ ltv_mpc.cpp 이식)
# ─────────────────────────────────────────────────────────────
def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def solve_ltv_mpc(x0, kappas, v_refs, params):
    """
    x0      : [e_y, e_psi, v]
    kappas  : list[N]
    v_refs  : list[N]
    params  : dict with N, dt, L, Q_ey, Q_epsi, Q_v, QN_ey, QN_epsi, QN_v,
                         R_delta, R_a, delta_max, a_max, a_min
    returns : (steer_rad, accel) or None
    """
    try:
        import osqp
    except ImportError:
        return None

    N, dt, L = params['N'], params['dt'], params['L']
    n, m = 3, 2

    A_list, B_list, f_list = [], [], []
    X_ref = np.zeros(N * n)

    for k in range(N):
        vr = v_refs[k]
        kp = kappas[k]
        A = np.eye(n)
        A[0, 1] = vr * dt
        B = np.zeros((n, m))
        B[1, 0] = vr * dt / max(L, 0.1)
        B[2, 1] = dt
        f = np.zeros(n)
        f[1] = -vr * kp * dt
        A_list.append(A)
        B_list.append(B)
        f_list.append(f)
        X_ref[k*n:k*n+n] = [0.0, 0.0, vr]

    Nn, Nm = N * n, N * m

    # Phi_x0 (Nn x n)
    Phi = np.zeros((Nn, n))
    Psi = np.eye(n)
    for k in range(N):
        Psi = A_list[k] @ Psi
        Phi[k*n:(k+1)*n, :] = Psi

    # Theta (Nn x Nm)
    Theta = np.zeros((Nn, Nm))
    for j in range(N):
        Theta[j*n:(j+1)*n, j*m:(j+1)*m] = B_list[j]
        for i in range(j+1, N):
            Theta[i*n:(i+1)*n, j*m:(j+1)*m] = A_list[i] @ Theta[(i-1)*n:i*n, j*m:(j+1)*m]

    # FF
    FF = np.zeros(Nn)
    FF[0:n] = f_list[0]
    for i in range(1, N):
        FF[i*n:(i+1)*n] = A_list[i] @ FF[(i-1)*n:i*n] + f_list[i]

    E = Phi @ x0 + FF - X_ref

    # Q_bar, R_bar
    Q = np.diag([params['Q_ey'], params['Q_epsi'], params['Q_v']])
    QN = np.diag([params['QN_ey'], params['QN_epsi'], params['QN_v']])
    Q_bar = np.zeros((Nn, Nn))
    for k in range(N - 1):
        Q_bar[k*n:(k+1)*n, k*n:(k+1)*n] = Q
    Q_bar[(N-1)*n:N*n, (N-1)*n:N*n] = QN

    R = np.diag([params['R_delta'], params['R_a']])
    R_bar = np.block([[R if i==j else np.zeros((m,m)) for j in range(N)] for i in range(N)])

    H = Theta.T @ Q_bar @ Theta + R_bar
    H = 0.5 * (H + H.T)
    f_cost = Theta.T @ Q_bar @ E

    P_mat = 2.0 * H
    q_vec = 2.0 * f_cost

    # 입력 제약
    dm = params['delta_max']
    l_vec = np.tile([[-dm], [params['a_min']]], N).flatten()
    u_vec = np.tile([[dm], [params['a_max']]], N).flatten()

    prob = osqp.OSQP()
    P_sp = sp.csc_matrix(np.triu(P_mat))
    A_sp = sp.eye(Nm, format='csc')
    prob.setup(P_sp, q_vec, A_sp, l_vec, u_vec,
               warm_starting=True, verbose=False,
               eps_abs=1e-4, eps_rel=1e-4, max_iter=2000)
    res = prob.solve()

    if res.info.status_val not in (1, 2):
        return None

    steer = float(np.clip(res.x[0], -dm, dm))
    accel = float(np.clip(res.x[1], params['a_min'], params['a_max']))
    return steer, accel


# ─────────────────────────────────────────────────────────────
# 데이터 로딩
# ─────────────────────────────────────────────────────────────
def load_path(path_file):
    rows = []
    with open(path_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                rows.append([float(p) for p in parts[:5]])
            except ValueError:
                continue
    arr = np.array(rows)
    return arr  # [x_abs, y_abs, z, yaw, kappa]


def load_csv(csv_file):
    df = pd.read_csv(csv_file, comment='#')
    return df


def get_utm_origin(csv_file):
    """CSV 옆 경로 파일로부터 UTM origin 추정 (path abs - odom rel)"""
    return None   # 아래에서 직접 계산


# ─────────────────────────────────────────────────────────────
# 목적함수
# ─────────────────────────────────────────────────────────────
def objective(trial_or_params, df_moving, path_rel, is_trial=True):
    if is_trial:
        trial = trial_or_params
        p = {
            'N':          trial.suggest_int   ('N',          8,   20),
            'dt':         trial.suggest_float ('dt',         0.05, 0.2),
            'Q_ey':       trial.suggest_float ('Q_ey',       1.0,  50.0, log=True),
            'Q_epsi':     trial.suggest_float ('Q_epsi',     1.0,  50.0, log=True),
            'Q_v':        trial.suggest_float ('Q_v',        0.1,  10.0, log=True),
            'QN_ey':      trial.suggest_float ('QN_ey',      5.0, 100.0, log=True),
            'QN_epsi':    trial.suggest_float ('QN_epsi',    5.0, 100.0, log=True),
            'QN_v':       trial.suggest_float ('QN_v',       0.5,  20.0, log=True),
            'R_delta':    trial.suggest_float ('R_delta',    0.1,  10.0, log=True),
            'R_a':        trial.suggest_float ('R_a',        0.01,  5.0, log=True),
            'kappa_speed_factor': trial.suggest_float('kappa_speed_factor', 0.5, 8.0),
        }
    else:
        p = trial_or_params

    p['L']        = 1.04
    p['delta_max'] = 55.0 * math.pi / 180.0
    p['a_max']    = 2.0
    p['a_min']    = -3.0

    v_cmd = 3.0 / 3.6   # 3 km/h → m/s
    costs = []

    rows = df_moving.to_dict('records')
    step = max(1, len(rows) // 300)   # 최대 300 포인트 샘플링

    for row in rows[::step]:
        veh_x   = row['x_m']
        veh_y   = row['y_m']
        veh_yaw = math.radians(row['yaw_deg'])
        veh_vx  = row['vx_mps']
        veh_vy  = row['vy_mps']

        # nearest index (heading-aware)
        best_i, best_d = 0, 1e9
        cos_v, sin_v = math.cos(veh_yaw), math.sin(veh_yaw)
        for i, pt in enumerate(path_rel):
            dx = veh_x - pt[0]; dy = veh_y - pt[1]
            d2 = dx*dx + dy*dy
            cos_p = math.cos(pt[3]); sin_p = math.sin(pt[3])
            if cos_v*cos_p + sin_v*sin_p < 0.0:
                continue
            if d2 < best_d:
                best_d = d2; best_i = i

        ref = path_rel[best_i]
        psi_ref = ref[3]
        dx = veh_x - ref[0]; dy = veh_y - ref[1]
        e_y   = -math.sin(psi_ref)*dx + math.cos(psi_ref)*dy
        e_psi = normalize_angle(veh_yaw - psi_ref)
        v_now = math.hypot(veh_vx, veh_vy)
        if v_now < 0.05:
            v_now = v_cmd

        x0 = np.array([e_y, e_psi, v_now])

        N = p['N']
        kappas, v_refs = [], []
        kfac = p['kappa_speed_factor']
        for k in range(N):
            ki = min(best_i + k, len(path_rel) - 1)
            kp = path_rel[ki][4]
            kappas.append(kp)
            v_refs.append(max(0.3, v_cmd / (1.0 + kfac * abs(kp))))

        res = solve_ltv_mpc(x0, kappas, v_refs, p)
        if res is None:
            costs.append(10.0)
            continue

        steer, _ = res
        # 비용: CTE^2 + heading_err^2 (라디안) + steer 크기
        cost = e_y**2 + (2.0 * e_psi)**2 + 0.3 * steer**2
        costs.append(cost)

    if not costs:
        return 1e6
    return float(np.mean(costs))


# ─────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv',    default='20260328_034113_mpc.csv')
    ap.add_argument('--path',   default='3_filtered.txt')
    ap.add_argument('--trials', type=int, default=200)
    ap.add_argument('--check_dir', default='/home/coss/henes_ws_ros2/check')
    ap.add_argument('--path_dir',  default='/home/coss/henes_ws_ros2/paths')
    args = ap.parse_args()

    csv_file  = os.path.join(args.check_dir, args.csv)
    path_file = os.path.join(args.path_dir,  args.path)

    print(f'CSV : {csv_file}')
    print(f'Path: {path_file}')

    df  = load_csv(csv_file)
    df_moving = df[df['vx_mps'].abs() > 0.15].copy()
    print(f'전체 {len(df)}행, 이동 중 {len(df_moving)}행 사용')

    path_abs = load_path(path_file)
    print(f'경로 포인트: {len(path_abs)}')

    # UTM origin: path_abs[nearest_idx] - ref_x/y_m (여러 행 중앙값으로 안정화)
    valid = df.dropna(subset=['nearest_idx', 'ref_x_m', 'ref_y_m'])
    ox_samples, oy_samples = [], []
    for _, row in valid.iterrows():
        ni = int(row['nearest_idx'])
        if ni < len(path_abs):
            ox_samples.append(path_abs[ni, 0] - row['ref_x_m'])
            oy_samples.append(path_abs[ni, 1] - row['ref_y_m'])
    origin_x = float(np.median(ox_samples))
    origin_y = float(np.median(oy_samples))
    print(f'UTM origin 추정: ({origin_x:.3f}, {origin_y:.3f})')

    path_rel = path_abs.copy()
    path_rel[:, 0] -= origin_x
    path_rel[:, 1] -= origin_y

    # 기준 성능 (현재 파라미터)
    base_params = {
        'N': 15, 'dt': 0.1, 'L': 1.04,
        'Q_ey': 10.0, 'Q_epsi': 8.0, 'Q_v': 1.0,
        'QN_ey': 20.0, 'QN_epsi': 16.0, 'QN_v': 2.0,
        'R_delta': 0.5, 'R_a': 0.1,
        'kappa_speed_factor': 3.0,
        'delta_max': 55.0*math.pi/180, 'a_max': 2.0, 'a_min': -3.0,
    }
    base_cost = objective(base_params, df_moving, path_rel, is_trial=False)
    print(f'\n현재 파라미터 비용: {base_cost:.4f}')

    # Optuna 최적화
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    study = optuna.create_study(
        direction='minimize',
        sampler=optuna.samplers.TPESampler(seed=42),
    )

    # 현재 파라미터를 시작점으로
    study.enqueue_trial({
        'N': 15, 'dt': 0.1,
        'Q_ey': 10.0, 'Q_epsi': 8.0, 'Q_v': 1.0,
        'QN_ey': 20.0, 'QN_epsi': 16.0, 'QN_v': 2.0,
        'R_delta': 0.5, 'R_a': 0.1, 'kappa_speed_factor': 3.0,
    })

    def _obj(trial):
        return objective(trial, df_moving, path_rel, is_trial=True)

    print(f'Optuna 최적화 시작 ({args.trials} trials)...')
    study.optimize(_obj, n_trials=args.trials, show_progress_bar=True)

    best = study.best_trial
    print(f'\n✓ 최적 비용: {best.value:.4f}  (기존 대비 {(base_cost-best.value)/base_cost*100:.1f}% 개선)')
    print('\n─── 최적 파라미터 ───────────────────────────────────')
    bp = best.params
    for k, v in bp.items():
        print(f'  {k}: {v:.4g}')

    print('\n─── params.yaml 적용값 ──────────────────────────────')
    print('mpc_path_follower_node:')
    print('  ros__parameters:')
    print(f'    prediction_horizon: {bp["N"]}')
    print(f'    dt:                 {bp["dt"]:.3f}')
    print(f'    Q_ey:               {bp["Q_ey"]:.2f}')
    print(f'    Q_epsi:             {bp["Q_epsi"]:.2f}')
    print(f'    Q_v:                {bp["Q_v"]:.2f}')
    print(f'    QN_ey:              {bp["QN_ey"]:.2f}')
    print(f'    QN_epsi:            {bp["QN_epsi"]:.2f}')
    print(f'    QN_v:               {bp["QN_v"]:.2f}')
    print(f'    R_delta:            {bp["R_delta"]:.3f}')
    print(f'    R_a:                {bp["R_a"]:.3f}')
    print(f'    kappa_speed_factor: {bp["kappa_speed_factor"]:.2f}')

    # params.yaml 자동 업데이트
    params_yaml = '/home/coss/henes_ws_ros2/src/jeju/config/params.yaml'
    update = input('\nparams.yaml에 바로 적용할까요? [y/N] ').strip().lower()
    if update == 'y':
        with open(params_yaml) as f:
            content = f.read()

        import re
        def replace_val(text, key, val):
            if isinstance(val, int):
                return re.sub(rf'({key}:\s*)\S+', rf'\g<1>{val}', text)
            else:
                return re.sub(rf'({key}:\s*)\S+', rf'\g<1>{val:.4g}', text)

        for key, val in [
            ('prediction_horizon', bp['N']),
            ('dt',                 bp['dt']),
            ('Q_ey',               bp['Q_ey']),
            ('Q_epsi',             bp['Q_epsi']),
            ('Q_v',                bp['Q_v']),
            ('QN_ey',              bp['QN_ey']),
            ('QN_epsi',            bp['QN_epsi']),
            ('QN_v',               bp['QN_v']),
            ('R_delta',            bp['R_delta']),
            ('R_a',                bp['R_a']),
            ('kappa_speed_factor', bp['kappa_speed_factor']),
        ]:
            content = replace_val(content, key, val)

        with open(params_yaml, 'w') as f:
            f.write(content)
        print(f'✓ {params_yaml} 업데이트 완료')
    else:
        print('params.yaml 변경 없음')


if __name__ == '__main__':
    main()
