"""
Optimization examples using the booster simulation.

Run:  python optimize.py
"""
import math
from dataclasses import replace
from scipy.optimize import minimize_scalar, differential_evolution

from sim.simulate import simulate
from sim.physics import Params


# ---------------------------------------------------------------------------
# Objective functions
# ---------------------------------------------------------------------------

def miss_distance(params: Params, **overrides) -> float:
    """Miss distance (m) for the given params (with optional overrides)."""
    p = replace(params, **overrides) if overrides else params
    return simulate(p)['summary']['miss_distance_m']


def total_control_effort(params: Params, **overrides) -> float:
    """Sum of |aLat| over the flight (proxy for control energy use)."""
    p = replace(params, **overrides) if overrides else params
    return sum(simulate(p)['series']['aLat'])


# ---------------------------------------------------------------------------
# Example 1 — Sweep launch angle to minimise miss distance
# ---------------------------------------------------------------------------

def sweep_launch_angle(base: Params, angles=None) -> list[dict]:
    """
    Try a range of launch angles and return miss distance for each.
    Useful to visualise sensitivity before running optimisation.
    """
    if angles is None:
        angles = [a for a in range(30, 91, 5)]
    results = []
    for angle in angles:
        p = replace(base, launch_angle=float(angle))
        r = simulate(p)
        results.append({
            'launch_angle_deg': angle,
            'miss_m':           r['summary']['miss_distance_m'],
            'impact_x_km':      r['summary']['impact_x_km'],
            'impact_y_km':      r['summary']['impact_y_km'],
        })
    return results


# ---------------------------------------------------------------------------
# Example 2 — Find the launch angle that minimises miss distance (1-D)
# ---------------------------------------------------------------------------

def optimise_launch_angle(base: Params) -> dict:
    """
    Scalar optimisation over launch angle (30–89°) to minimise miss distance.
    Uses Brent's method (golden-section + parabolic interpolation).
    """
    def objective(angle_deg):
        return miss_distance(base, launch_angle=angle_deg)

    result = minimize_scalar(
        objective,
        bounds=(30.0, 89.0),
        method='bounded',
        options={'xatol': 0.1},   # 0.1° tolerance
    )
    best_params = replace(base, launch_angle=result.x)
    best_result = simulate(best_params)
    return {
        'optimal_launch_angle_deg': round(result.x, 2),
        'miss_distance_m':          best_result['summary']['miss_distance_m'],
        'summary':                  best_result['summary'],
    }


# ---------------------------------------------------------------------------
# Example 3 — Joint optimisation of launch angle + burn duration (2-D)
# ---------------------------------------------------------------------------

def optimise_launch_and_burn(base: Params) -> dict:
    """
    Global optimisation over (launch_angle, burn_max) to minimise miss distance.
    Uses differential evolution (population-based, derivative-free).
    """
    def objective(x):
        angle_deg, burn_s = x
        return miss_distance(base, launch_angle=angle_deg, burn_max=burn_s)

    bounds = [
        (30.0, 89.0),    # launch angle (deg)
        (10.0, 200.0),   # burn duration (s)
    ]
    result = differential_evolution(
        objective,
        bounds,
        seed=42,
        tol=1.0,         # stop when miss < 1 m improvement
        maxiter=200,
        workers=1,
    )
    best_params = replace(base, launch_angle=result.x[0], burn_max=result.x[1])
    best_result = simulate(best_params)
    return {
        'optimal_launch_angle_deg': round(result.x[0], 2),
        'optimal_burn_max_s':       round(result.x[1], 2),
        'miss_distance_m':          best_result['summary']['miss_distance_m'],
        'summary':                  best_result['summary'],
    }


# ---------------------------------------------------------------------------
# Main — run the examples
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    base = Params(x_target=120.0, y_target=20.0, a_lat_max=1.5)

    print('=== Baseline simulation ===')
    r = simulate(base)
    for k, v in r['summary'].items():
        print(f'  {k}: {v}')

    print('\n=== Launch angle sweep ===')
    sweep = sweep_launch_angle(base)
    print(f"  {'angle':>6}  {'miss_m':>10}  {'impact_x':>10}  {'impact_y':>10}")
    for row in sweep:
        print(f"  {row['launch_angle_deg']:>6.0f}°  {row['miss_m']:>10.1f}  "
              f"{row['impact_x_km']:>10.1f}  {row['impact_y_km']:>10.1f}")

    print('\n=== 1-D optimisation (launch angle) ===')
    opt1 = optimise_launch_angle(base)
    print(f"  Best angle: {opt1['optimal_launch_angle_deg']}°  "
          f"miss: {opt1['miss_distance_m']:.1f} m")

    print('\n=== 2-D optimisation (angle + burn duration) ===')
    opt2 = optimise_launch_and_burn(base)
    print(f"  Best angle: {opt2['optimal_launch_angle_deg']}°  "
          f"burn: {opt2['optimal_burn_max_s']} s  "
          f"miss: {opt2['miss_distance_m']:.1f} m")
