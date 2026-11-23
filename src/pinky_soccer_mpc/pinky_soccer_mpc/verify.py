"""
acados installation / integration check with the Pinky wheel model (not the controller).

Builds a small fixed-horizon OCP from pinky_soccer_dynamics (wheel-speed inputs, wheel speed
bounds, wheel acceleration path constraint), generates and compiles it with acados, solves
it with full SQP and checks:
  * solver status == 0 and KKT residuals below tolerance
  * input bounds and the wheel acceleration constraint hold on the solution
  * the optimal inputs replayed through the NumPy model reproduce the predicted trajectory
The real NMPC (stage 3) is designed separately; this only proves the toolchain end to end.

    ros2 run pinky_soccer_mpc check_acados          # or: python -m pinky_soccer_mpc.verify
"""

import hashlib
import json
import sys
import time

import numpy as np

from .acados_setup import acados, cache_dir, info


def build_ocp(params, target, horizon=2.0, steps=40, name='pinky_verify'):
    """Return an AcadosOcp for a point-to-point move from the model parameters."""
    at = acados()
    from pinky_soccer_dynamics.model import casadi_model
    import casadi as ca
    x, u, xdot, f_expl, acc = casadi_model(params)
    digest = hashlib.sha1(json.dumps(params.as_dict()['model'], sort_keys=True,
                                     default=str).encode()).hexdigest()[:8]
    model = at.AcadosModel()
    model.name = f'{name}_{digest}'
    model.x, model.u, model.xdot = x, u, xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    # acados applies con_h_expr on stages 1..N-1 only; the first stage (whose input is the
    # one sent to the robot) needs its own copy or the limit is silently ignored there.
    model.con_h_expr_0 = acc
    model.con_h_expr = acc
    ocp = at.AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = steps
    ocp.solver_options.tf = horizon
    # Least squares on pose, wheel speeds and inputs; terminal pose weighted strongly.
    ny, ny_e = 7, 5
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = ca.vertcat(x, u)
    ocp.model.cost_y_expr_e = x
    ocp.cost.W = np.diag([5.0, 5.0, 1.0, 1e-3, 1e-3, 1e-3, 1e-3])
    ocp.cost.W_e = np.diag([500.0, 500.0, 50.0, 0.1, 0.1])
    ocp.cost.yref = np.r_[target, 0.0, 0.0, 0.0, 0.0]
    ocp.cost.yref_e = np.r_[target, 0.0, 0.0]
    assert ocp.cost.yref.shape[0] == ny and ocp.cost.yref_e.shape[0] == ny_e
    wmax = params.m('wheel_speed_max')
    amax = params.m('wheel_accel_max')
    ocp.constraints.lbu = np.array([-wmax, -wmax])
    ocp.constraints.ubu = np.array([wmax, wmax])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lh = np.array([-amax, -amax])
    ocp.constraints.uh = np.array([amax, amax])
    ocp.constraints.lh_0 = np.array([-amax, -amax])
    ocp.constraints.uh_0 = np.array([amax, amax])
    ocp.constraints.x0 = np.zeros(5)
    o = ocp.solver_options
    o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    o.hessian_approx = 'GAUSS_NEWTON'
    o.integrator_type = 'ERK'
    o.sim_method_num_stages = 4
    o.sim_method_num_steps = 2
    o.nlp_solver_type = 'SQP'
    o.nlp_solver_max_iter = 200
    o.tol = 1e-6
    ocp.code_export_directory = str(cache_dir(f'c_generated_code_{model.name}'))
    return ocp


def run(params=None, target=(0.4, 0.15, 0.6), tol=1e-5):
    """Generate, compile and solve; return a result dict with pass/fail checks."""
    from pinky_soccer_dynamics import model as dyn
    from pinky_soccer_dynamics import params as dyn_params
    params = params or dyn_params.load('mujoco')
    at = acados()
    ocp = build_ocp(params, np.asarray(target, float))
    json_file = str(cache_dir() / f'{ocp.model.name}.json')
    t0 = time.perf_counter()
    solver = at.AcadosOcpSolver(ocp, json_file=json_file, verbose=False)
    build_s = time.perf_counter() - t0
    t1 = time.perf_counter()
    status = solver.solve()
    solve_s = time.perf_counter() - t1
    n = ocp.solver_options.N_horizon
    xs = np.array([solver.get(k, 'x') for k in range(n + 1)])
    us = np.array([solver.get(k, 'u') for k in range(n)])
    res = solver.get_residuals()
    dt = ocp.solver_options.tf / n
    replay = dyn.predict(params, xs[0], us, dt, accel_limit=False)
    acc = np.array([[(us[k][i] - xs[k][3 + i]) / params.m('tau_left' if i == 0 else 'tau_right')
                     for i in range(2)] for k in range(n)])
    wmax, amax = params.m('wheel_speed_max'), params.m('wheel_accel_max')
    checks = {
        'status_zero': status == 0,
        'residuals_below_tol': bool(np.max(res) < tol),
        'input_bounds': bool(np.max(np.abs(us)) <= wmax + 1e-6),
        'accel_constraint': bool(np.max(np.abs(acc)) <= amax + 1e-3),
        'model_replay_matches': bool(np.max(np.abs(replay[:, :2] - xs[:, :2])) < 1e-3),
        'reaches_target': bool(np.hypot(*(xs[-1][:2] - np.asarray(target[:2]))) < 0.02),
    }
    return {'acados': info(), 'status': int(status), 'sqp_iter': int(solver.get_stats('sqp_iter')),
            'residuals': [float(r) for r in res], 'build_s': round(build_s, 2),
            'solve_ms': round(solve_s * 1e3, 2), 'final_state': xs[-1].round(4).tolist(),
            'max_wheel_speed': float(np.max(np.abs(us))),
            'max_wheel_accel': float(np.max(np.abs(acc))),
            'checks': checks, 'passed': all(checks.values())}


def main(argv=None):
    """CLI: print the verification result and exit non-zero on failure."""
    result = run()
    print(json.dumps(result, indent=1))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
