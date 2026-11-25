"""Left/right effective wheel radii from the optional straight-run heading observations.

Review only: the shared effective radius result ('wheels') is never replaced.

Linear model per trial (encoders normalised to + forward exactly like the shared fit):
  straight distance     (rL thL + rR thR) / 2           = s
  straight heading      rR thR - rL thL - b dpsi         = 0     (dpsi = measured final
                                                                   heading change, CCW +)
  rotation angle        rR thR - rL thL - b angle        = 0
Unknowns rL, rR, b. Without measured straight headings rL - rR is unobservable (a radius
difference and a wheel-speed difference produce the same encoder counts), so the result is
'insufficient_observations'. The left-out validation predicts each straight trial's distance
and heading from the others. Wheel slip and floor effects are not separated from a radius
difference by this test; accepted results are still only candidates for a separate review.
"""
import math

import numpy as np

EXPECTED = {'forward': (1, 1), 'backward': (-1, -1), 'left': (-1, 1), 'right': (1, -1)}


def _rows(trials):
    rows, rhs, kinds, used = [], [], [], []
    for t in trials:
        kind = t.get('kind')
        if kind not in EXPECTED:
            continue
        raw = np.asarray([t['left_rad'], t['right_rad']], float)
        if not np.isfinite(raw).all() or np.min(np.abs(raw)) < 1e-6:
            continue
        left, right = raw * np.asarray(EXPECTED[kind]) * np.sign(raw)
        if kind in ('forward', 'backward'):
            extra = t.get('extra_observations') or {}
            if 'final_heading_change_deg' not in extra:
                continue
            rows.append([left / 2, right / 2, 0.])
            rhs.append(float(t['distance_m']))
            rows.append([-left, right, -math.radians(extra['final_heading_change_deg'])])
            rhs.append(0.)
            kinds += [kind, kind]
        else:
            rows.append([-left, right, -float(t['angle_rad'])])
            rhs.append(0.)
            kinds.append(kind)
        used.append(t.get('id'))
    return np.asarray(rows), np.asarray(rhs), kinds, used


def fit_individual(trials, shared=None, min_straight=4):
    """Return a review result; 'identified' only with enough independent observations."""
    a, b, kinds, used = _rows(trials)
    straight = sum(1 for t in trials if t.get('kind') in ('forward', 'backward')
                   and 'final_heading_change_deg' in (t.get('extra_observations') or {}))
    out = {'model': 'individual_effective_radii_v1', 'trials_used': used,
           'straight_trials_with_heading': straight, 'applied': False,
           'shared_result_kept': True}
    directions = {k for k in kinds if k in ('forward', 'backward')}
    if straight < min_straight or directions != {'forward', 'backward'} or \
            not {'left', 'right'} & set(kinds):
        out['status'] = ('insufficient_observations: need >= %d straight runs (forward and '
                         'backward) with a measured final heading, plus rotation runs' % min_straight)
        return out
    rank = int(np.linalg.matrix_rank(a))
    cond = float(np.linalg.cond(a))
    out.update(rank=rank, condition_number=cond)
    if rank < 3:
        out['status'] = 'insufficient_observations: rank < 3'
        return out
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    res = a @ x - b
    dof = max(1, len(b) - 3)
    cov = np.linalg.inv(a.T @ a) * float(res @ res) / dof
    std = np.sqrt(np.clip(np.diag(cov), 0, None))
    diff = x[0] - x[1]
    diff_std = math.sqrt(max(cov[0, 0] + cov[1, 1] - 2 * cov[0, 1], 0.))
    # leave-one-trial-out prediction of each straight trial's two equations
    loo = []
    straight_ids = [t.get('id') for t in trials if t.get('kind') in ('forward', 'backward')
                    and 'final_heading_change_deg' in (t.get('extra_observations') or {})]
    for tid in straight_ids:
        rest = [t for t in trials if t.get('id') != tid]
        ra, rb, _, _ = _rows(rest)
        if np.linalg.matrix_rank(ra) < 3:
            continue
        xr, *_ = np.linalg.lstsq(ra, rb, rcond=None)
        ta, tb, _, _ = _rows([t for t in trials if t.get('id') == tid])
        loo.append(np.abs(ta @ xr - tb).tolist())
    out.update(left_radius_m=float(x[0]), right_radius_m=float(x[1]), separation_m=float(x[2]),
               std_m={'left': float(std[0]), 'right': float(std[1]), 'separation': float(std[2])},
               radius_difference_m=float(diff), radius_difference_std_m=diff_std,
               equation_residual_max=float(np.max(np.abs(res))), leave_one_out=loo)
    significant = abs(diff) > 2 * diff_std
    out['status'] = ('identified_candidate (difference > 2 sigma)' if significant else
                     'not_significant: keep the shared effective radius')
    if shared:
        out['shared_reference'] = {k: shared.get(k) for k in
                                   ('effective_radius_m', 'separation_m')}
    out['caveat'] = 'slip/floor effects are not separated from a radius difference'
    return out
