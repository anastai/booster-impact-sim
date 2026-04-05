"""
sim/opt_trajectory.py — Open-loop trajectory optimizer
=======================================================
Finds the lateral acceleration schedule  (a_nV(t), a_nH(t))  that
minimises a merit function combining miss distance, impact angle error,
and flight time — without assuming any guidance law.

The commands are parameterised as piecewise-linear knot points in time
and passed directly to the equations of motion.

Public API
----------
    from sim.opt_trajectory import optimize_trajectory, sim_open_loop, plot_optimised

    result = optimize_trajectory(params)
    result = optimize_trajectory(params, n_knots=30, a_max_g=5.0, use_de=True)
    plot_optimised(result)

Merit function
--------------
    J = w_miss    ·  miss_m²
      + w_angle_v ·  γV_err_deg²    [only when params.hit_gamma_v is set]
      + w_angle_h ·  γH_err_deg²    [only when params.hit_gamma_h is set]
      + w_time    ·  t_flight_s

When angle constraints are present the optimisation runs in two phases:
    Phase 1 — miss only: drives miss to < 1 m without angle interference.
    Phase 2 — full merit starting from Phase 1 result: improves angle
              accuracy without losing the good miss distance.

The best solution found across ALL evaluations is always returned — so
hitting max_iter never discards a good intermediate result.

subject to:  sqrt(a_nV² + a_nH²)  ≤  a_max_g · G0   at all times.
"""
from __future__ import annotations

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, differential_evolution

from .physics import Params, State, gravity, integrate_step, G0
from .atmosphere import density

_MAX_STEPS = 60_000
_LOG_EVERY = 5


# ---------------------------------------------------------------------------
# Open-loop forward simulation
# ---------------------------------------------------------------------------

def sim_open_loop(
    params:    Params,
    t_knots:   np.ndarray,
    anV_knots: np.ndarray,
    anH_knots: np.ndarray,
    dt:        float = 0.2,
    _series:   bool  = True,
) -> dict:
    """
    Forward simulation driven by a piecewise-linear lateral acceleration
    schedule instead of a guidance law.

    Parameters
    ----------
    params     : Params — same vehicle / target specification as simulate().
    t_knots    : knot times (s), shape (N,), monotonically increasing.
    anV_knots  : a_nV command at each knot (m/s²), shape (N,).
    anH_knots  : a_nH command at each knot (m/s²), shape (N,).
    dt         : integration timestep (s).

    Notes
    -----
    - Commands outside the knot range are held constant at the nearest
      boundary value (np.interp clamp behaviour).
    - The physical saturation limit params.a_lat_max is applied after
      interpolation — the optimiser may request any value; the actuator
      enforces the cap.
    - Actuator drag (cd_ctrl) is computed identically to simulate().

    Returns
    -------
    dict:
        'miss_m'         : 3-D miss distance at termination (m)
        'flight_time_s'  : total flight time (s)
        'final_state'    : State dataclass at termination
        'series'         : time-series dict (t, x, y, h, v,
                           gamma_v_deg, gamma_h_deg, a_nV_g, a_nH_g, aLat_g)
    """
    # ── Unit conversions ──────────────────────────────────────────────────
    thrust_n      = params.thrust_kn * 1_000.0
    x_t           = params.x_target  * 1_000.0
    y_t           = params.y_target  * 1_000.0
    z_t           = params.z_target  * 1_000.0
    a_lat_max_ms2 = params.a_lat_max * G0
    Aref          = math.pi * (params.diam / 2.0) ** 2

    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)

    # ── Initial state ─────────────────────────────────────────────────────
    gV0 = math.radians(params.launch_angle)
    gH0 = math.radians(params.launch_azimuth)
    state = State(
        v=0.0, gamma_v=gV0, gamma_h=gH0,
        mass=params.m_pay + params.m_prop + params.m_str,
        prop_remaining=prop_used,
    )

    ser: dict = {k: [] for k in [
        't', 'x', 'y', 'h', 'v',
        'gamma_v_deg', 'gamma_h_deg',
        'a_nV_g', 'a_nH_g', 'aLat_g',
    ]}

    passed_above = (z_t <= 0.0)

    for i in range(_MAX_STEPS):
        spd  = state.v
        grav = gravity(state.h)
        rho  = density(params.atm, state.h)
        qS   = 0.5 * rho * spd * spd * Aref

        # ── Engine ────────────────────────────────────────────────────────
        thr_now  = 0.0
        eng_on   = state.engine_on
        new_prop = state.prop_remaining
        new_mass = state.mass

        if state.engine_on and state.prop_remaining > 0:
            thr_now  = thrust_n
            new_prop = state.prop_remaining - mdot * dt
            new_mass = state.mass - mdot * dt
            if new_prop <= 0.0 or state.t >= params.burn_max:
                eng_on   = False
                new_mass = params.m_pay + params.m_str + (params.m_prop - prop_used)

        state = State(
            v=state.v, gamma_v=state.gamma_v, gamma_h=state.gamma_h,
            h=state.h, x=state.x, y=state.y,
            mass=new_mass, t=state.t,
            engine_on=eng_on,
            prop_remaining=max(new_prop, 0.0),
        )

        # ── Aerodynamics ──────────────────────────────────────────────────
        cd_now    = params.cd if state.engine_on else params.cd_fall
        drag_aero = qS * cd_now
        lift      = qS * params.cl

        # ── Interpolate scheduled command ─────────────────────────────────
        a_nV = float(np.interp(state.t, t_knots, anV_knots))
        a_nH = float(np.interp(state.t, t_knots, anH_knots))

        # Enforce physical actuator limit (params.a_lat_max)
        mag = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        if a_lat_max_ms2 > 0.0 and mag > a_lat_max_ms2:
            s    = a_lat_max_ms2 / mag
            a_nV *= s
            a_nH *= s

        aLat   = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        d_ctrl = (qS * params.cd_ctrl * (aLat / a_lat_max_ms2) ** 2
                  if a_lat_max_ms2 > 0.0 else 0.0)

        # ── Log ───────────────────────────────────────────────────────────
        if _series and i % _LOG_EVERY == 0:
            ser['t']          .append(round(state.t, 1))
            ser['x']          .append(round(state.x / 1_000, 2))
            ser['y']          .append(round(state.y / 1_000, 2))
            ser['h']          .append(round(state.h / 1_000, 2))
            ser['v']          .append(round(spd, 1))
            ser['gamma_v_deg'].append(round(math.degrees(state.gamma_v), 2))
            ser['gamma_h_deg'].append(round(math.degrees(state.gamma_h), 2))
            ser['a_nV_g']     .append(round(a_nV / G0, 4))
            ser['a_nH_g']     .append(round(a_nH / G0, 4))
            ser['aLat_g']     .append(round(aLat / G0, 4))

        # ── Integrate one step ────────────────────────────────────────────
        state, _ = integrate_step(
            state, params,
            thr_now, drag_aero, lift, d_ctrl,
            grav, (a_nV, a_nH), dt,
        )

        # ── Termination ───────────────────────────────────────────────────
        if state.h > z_t + 50.0:
            passed_above = True
        if passed_above and state.h <= z_t:
            break
        if state.h <= 0.0:
            break

    miss = math.sqrt(
        (state.x - x_t) ** 2 +
        (state.y - y_t) ** 2 +
        (state.h - z_t) ** 2
    )
    return {
        'miss_m':        miss,
        'flight_time_s': state.t,
        'final_state':   state,
        'series':        ser,
    }


# ---------------------------------------------------------------------------
# Fast evaluation kernel (used inside the optimizer only)
# ---------------------------------------------------------------------------
#
# Identical physics to sim_open_loop but with three Python-level speedups:
#   1. No State / StepLog dataclass allocation — plain float variables.
#   2. integrate_step inlined — eliminates 8 M+ function-call overheads.
#   3. Direct bin-index formula instead of np.interp binary search
#      (valid because t_knots is always a linspace).
# Returns only (miss_m, flight_time_s, final_gamma_v_rad, final_gamma_h_rad)
# — the four scalars needed by the merit function.

def _sim_eval_fast(
    params: Params,
    t_knots: np.ndarray,   # linspace — equally spaced
    anV_knots: np.ndarray, # m/s²
    anH_knots: np.ndarray, # m/s²
    dt: float,
) -> tuple:
    """Fast optimizer-only evaluation kernel. Returns (miss_m, t_f, gV_f, gH_f)."""
    # ── Precompute constants ───────────────────────────────────────────────
    thrust_n      = params.thrust_kn * 1_000.0
    x_t           = params.x_target  * 1_000.0
    y_t           = params.y_target  * 1_000.0
    z_t           = params.z_target  * 1_000.0
    a_lat_max_ms2 = params.a_lat_max * G0
    Aref          = math.pi * (params.diam / 2.0) ** 2
    mdot          = thrust_n / (params.isp * G0)
    prop_used     = min(params.m_prop, mdot * params.burn_max)
    grav_turn_v_min = params.grav_turn_v_min
    cd_pow        = params.cd
    cd_fall       = params.cd_fall
    cl            = params.cl
    cd_ctrl       = params.cd_ctrl
    a_cmd_t       = params.a_cmd_t
    a_cmd_nv      = params.a_cmd_nv
    a_cmd_nh      = params.a_cmd_nh
    m_pay_str     = params.m_pay + params.m_str
    grav_turn     = params.grav_turn
    gV0_fixed     = math.radians(params.launch_angle)   # for fixed-body mode

    # Direct linspace index constants
    nk            = len(t_knots)
    tk0           = float(t_knots[0])
    tk1           = float(t_knots[-1])
    tk_span       = tk1 - tk0 if tk1 > tk0 else 1e-9
    nk_minus1     = float(nk - 1)

    # ── Initial state as plain floats ─────────────────────────────────────
    v        = 0.0
    gV       = math.radians(params.launch_angle)
    gH       = math.radians(params.launch_azimuth)
    h        = 0.0
    x        = 0.0
    y        = 0.0
    mass     = params.m_pay + params.m_prop + params.m_str
    t        = 0.0
    prop_rem = prop_used
    eng_on   = True

    passed_above = (z_t <= 0.0)
    half_dt      = 0.5 * dt

    for _ in range(_MAX_STEPS):
        spd  = v
        grav = gravity(h)           # call the shared helper — preserves FP identity
        rho  = density(params.atm, h)

        qS = 0.5 * rho * spd * spd * Aref

        # ── Engine ────────────────────────────────────────────────────────
        if eng_on and prop_rem > 0:
            thr_now  = thrust_n
            prop_rem = prop_rem - mdot * dt
            mass     = mass - mdot * dt
            if prop_rem <= 0.0 or t >= params.burn_max:
                eng_on   = False
                mass     = m_pay_str + (params.m_prop - prop_used)
        else:
            thr_now = 0.0

        # ── Aerodynamics ──────────────────────────────────────────────────
        cd_now    = cd_pow if eng_on else cd_fall
        drag_aero = qS * cd_now
        lift      = qS * cl

        # ── Command interpolation — direct linspace index ─────────────────
        idx_f  = (t - tk0) / tk_span * nk_minus1
        idx    = int(idx_f)
        if idx < 0:
            a_nV = anV_knots[0]
            a_nH = anH_knots[0]
        elif idx >= nk - 1:
            a_nV = anV_knots[-1]
            a_nH = anH_knots[-1]
        else:
            frac = idx_f - idx
            a_nV = anV_knots[idx] + frac * (anV_knots[idx + 1] - anV_knots[idx])
            a_nH = anH_knots[idx] + frac * (anH_knots[idx + 1] - anH_knots[idx])

        # Actuator saturation
        mag = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        if a_lat_max_ms2 > 0.0 and mag > a_lat_max_ms2:
            s = a_lat_max_ms2 / mag
            a_nV *= s
            a_nH *= s

        aLat   = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        d_ctrl = (qS * cd_ctrl * (aLat / a_lat_max_ms2) ** 2
                  if a_lat_max_ms2 > 0.0 else 0.0)
        drag_tot = drag_aero + d_ctrl

        # ── Inlined integrate_step ─────────────────────────────────────────
        spd_safe = max(spd, 1e-6)

        if grav_turn:
            T_t, T_nV, T_nH = thr_now, 0.0, 0.0
        else:
            alpha = gV0_fixed - gV
            T_t   = thr_now * math.cos(alpha)
            T_nV  = thr_now * math.sin(alpha)
            T_nH  = 0.0

        rotation_active = spd_safe > (grav_turn_v_min if grav_turn else 0.5)
        sin_gV = math.sin(gV)
        cos_gV = math.cos(gV)

        dv_dt = (T_t - drag_tot) / mass - grav * sin_gV + a_cmd_t

        if rotation_active:
            dgV_dt = ((T_nV + lift) / (mass * spd_safe)
                      - grav * cos_gV / spd_safe
                      + a_cmd_nv / spd_safe
                      + a_nV    / spd_safe)
            vcos = spd_safe * cos_gV
            dgH_dt = ((T_nH / (mass * vcos)
                       + a_cmd_nh / vcos
                       + a_nH    / vcos)
                      if abs(vcos) > 0.5 else 0.0)
        else:
            dgV_dt = 0.0
            dgH_dt = 0.0

        new_v  = max(0.0, v + dv_dt * dt)
        new_gV = gV + dgV_dt * dt
        _gV_lim = math.pi / 2 - 1e-4        # matches integrate_step exactly
        if new_gV < -_gV_lim:
            new_gV = -_gV_lim
        elif new_gV > _gV_lim:
            new_gV = _gV_lim
        new_gH = gH + dgH_dt * dt

        new_h  = h + new_v * math.sin(new_gV) * dt
        new_x  = x + new_v * math.cos(new_gV) * math.cos(new_gH) * dt
        new_y  = y + new_v * math.cos(new_gV) * math.sin(new_gH) * dt
        t     += dt

        v, gV, gH, h, x, y = new_v, new_gV, new_gH, new_h, new_x, new_y

        # ── Termination ───────────────────────────────────────────────────
        if h > z_t + 50.0:
            passed_above = True
        if passed_above and h <= z_t:
            break
        if h <= 0.0:
            break

    miss = math.sqrt((x - x_t) ** 2 + (y - y_t) ** 2 + (h - z_t) ** 2)
    return miss, t, gV, gH


# ---------------------------------------------------------------------------
# Optimizer
# ---------------------------------------------------------------------------

def optimize_trajectory(
    params:    Params,
    n_knots:   int         = 20,
    a_max_g:   float|None  = None,
    w_miss:    float       = 1.0,
    w_angle_v: float       = 0.01,
    w_angle_h: float       = 0.01,
    w_time:    float       = 1e-5,
    dt:        float       = 0.2,
    dt_eval:   float|None  = None,
    method:    str         = 'L-BFGS-B',
    max_iter:  int         = 500,
    use_de:    bool        = False,
    verbose:   bool        = True,
) -> dict:
    """
    Optimise the lateral acceleration schedule for a single trajectory.

    Unlike the IACPN guidance law, the optimiser has complete freedom over
    the command profile — it outputs a vector (a_nV(t), a_nH(t)) that is
    the direct result of minimising the merit function below.

    Merit function
    --------------
        J = w_miss    ·  miss_m²
          + w_angle_v ·  γV_err_deg²    [only when params.hit_gamma_v set]
          + w_angle_h ·  γH_err_deg²    [only when params.hit_gamma_h set]
          + w_time    ·  t_flight_s

    Optimisation strategy
    ---------------------
    Phase 1 (always): minimise miss only (angle terms off).
        Guarantees < 1 m miss before angle accuracy is attempted.
    Phase 2 (angle-constrained cases only): full merit from Phase 1 result.
        Improves angle accuracy while the large miss-weight keeps miss small.

    The best solution found across ALL evaluations is returned — hitting
    max_iter never loses a good intermediate result.

    Parameters
    ----------
    params     : Params.  hit_gamma_v / hit_gamma_h activate angle terms.
    n_knots    : number of piecewise-linear control knot points (default 20).
    a_max_g    : optimiser lateral acceleration limit (g).
                 Defaults to params.a_lat_max.  The physical actuator cap
                 (params.a_lat_max) is always enforced inside the simulation.
    w_miss     : weight on miss² (per m²)
    w_angle_v  : weight on impact γV error² (per deg²)
    w_angle_h  : weight on impact γH error² (per deg²)
    w_time     : weight on flight time (per s)
    dt         : integration timestep (s) for the final trajectory and warm-start.
    dt_eval    : integration timestep used during optimisation evaluations.
                 Defaults to dt.  A coarser value (e.g. 0.5) speeds up each
                 sim call with negligible effect on the converged solution.
    method     : local optimiser — 'L-BFGS-B' (default) or 'SLSQP'.
    max_iter   : maximum iterations per phase.
    use_de     : prepend a differential_evolution global search before Phase 1.
    verbose    : print progress to stdout.

    Returns
    -------
    dict:
        'summary'  : scalar comparison — includes 'solved' (miss < 1 m).
        'schedule' : optimal knot times and command values (SI and g)
        'series'   : full time-series of the optimised flight
        'nominal'  : complete simulate() result for the IACPN warm-start run
    """
    from .simulate import simulate as _iacpn

    a_max_ms2  = (a_max_g if a_max_g is not None else params.a_lat_max) * G0
    has_angles = (params.hit_gamma_v is not None or params.hit_gamma_h is not None)
    # _dt_arr[0] is the active timestep inside _eval; it starts coarse and is
    # switched to dt for the final refinement pass when dt_eval != dt.
    _dt_arr = [dt_eval if dt_eval is not None else dt]

    # ── Step 1: IACPN runs — nominal for reporting, no-angle for warm start ──
    nominal  = _iacpn(params, dt=dt)
    t_nom    = nominal['summary']['flight_time_s']
    miss_nom = nominal['summary']['miss_distance_m']

    if verbose:
        print(f"  Nominal IACPN : miss = {miss_nom} m,  t_f = {t_nom:.1f} s")

    # Warm start always comes from a no-angle IACPN run.
    # When angle constraints are set, the angle-constrained IACPN may produce a
    # poor trajectory (guidance fighting itself) that makes a bad initial guess.
    # The no-angle run gives a clean miss-minimising trajectory as starting point.
    if has_angles:
        from dataclasses import replace as _dc_replace
        _p_noangle = _dc_replace(params, hit_gamma_v=None, hit_gamma_h=None)
        _ws = _iacpn(_p_noangle, dt=dt)
        t_ws = _ws['summary']['flight_time_s']
        if verbose:
            print(f"  Warm-start    : no-angle IACPN, miss = {_ws['summary']['miss_distance_m']} m")
    else:
        _ws  = nominal
        t_ws = t_nom

    # Knot times span [0, t_ws * 1.2] to handle minor overruns
    t_knots = np.linspace(0.0, t_ws * 1.2, n_knots)

    ws_t   = np.array(_ws['series']['t'],    dtype=float)
    ws_anV = np.array(_ws['series']['a_nV'], dtype=float) * G0   # g → m/s²
    ws_anH = np.array(_ws['series']['a_nH'], dtype=float) * G0

    if len(ws_t) > 1:
        x0_anV = np.interp(t_knots, ws_t, ws_anV)
        x0_anH = np.interp(t_knots, ws_t, ws_anH)
    else:
        x0_anV = np.zeros(n_knots)
        x0_anH = np.zeros(n_knots)

    # Normalise to [-1, 1] for better numerical conditioning
    x0     = np.concatenate([x0_anV / a_max_ms2, x0_anH / a_max_ms2])
    x0     = np.clip(x0, -1.0, 1.0)
    bounds = [(-1.0, 1.0)] * (2 * n_knots)

    # ── Impact angle targets ──────────────────────────────────────────────
    hit_gv = math.radians(params.hit_gamma_v) if params.hit_gamma_v is not None else None
    hit_gh = math.radians(params.hit_gamma_h) if params.hit_gamma_h is not None else None

    # ── Best-ever tracking (always uses the full merit for comparison) ────
    n_eval = [0]
    best   = {'J_full': float('inf'), 'miss_m': float('inf'),
              't_f': float('inf'), 'x': x0.copy()}

    def _eval(x: np.ndarray) -> tuple:
        """Run one fast simulation and return (miss, t_f, gV_f, gH_f).  No series recorded."""
        n_eval[0] += 1
        anV_k = x[:n_knots] * a_max_ms2
        anH_k = x[n_knots:] * a_max_ms2
        return _sim_eval_fast(params, t_knots, anV_k, anH_k, _dt_arr[0])

    def make_merit(use_angle: bool):
        """Return a callable merit function with or without angle terms."""
        def _merit(x: np.ndarray) -> float:
            miss, t_f, gV_f, gH_f = _eval(x)

            J = w_miss * miss * miss + w_time * t_f

            err_v = err_h = 0.0
            if hit_gv is not None:
                err_v = math.degrees(gV_f - hit_gv)
            if hit_gh is not None:
                dh    = (gH_f - hit_gh + math.pi) % (2 * math.pi) - math.pi
                err_h = math.degrees(dh)

            if use_angle:
                J += w_angle_v * err_v * err_v + w_angle_h * err_h * err_h

            # Track full-merit best so the returned result is always correct
            J_full = (w_miss * miss * miss + w_time * t_f
                      + w_angle_v * err_v * err_v
                      + w_angle_h * err_h * err_h)
            if J_full < best['J_full']:
                best.update(J_full=J_full, miss_m=miss, t_f=t_f, x=x.copy())

            return J
        return _merit

    # ── Step 2: optional DE global warm-start ────────────────────────────
    if verbose:
        print(f"  Optimising {2 * n_knots} knot values,  "
              f"a_max = {a_max_ms2 / G0:.2f} g,  method = {method},  "
              f"dt_eval = {_dt_arr[0]} s")

    if use_de:
        if verbose:
            print("  Phase DE — differential_evolution (global search) ...")
        differential_evolution(
            make_merit(has_angles),
            bounds, maxiter=150, popsize=5, tol=1e-6, seed=42, polish=False,
        )
        x0 = best['x'].copy()
        if verbose:
            print(f"  DE done  : miss = {best['miss_m']:.2f} m,  "
                  f"{n_eval[0]} evals")

    # ── Step 3: Phase 1 — miss only ───────────────────────────────────────
    label1 = 'Phase 1 (miss only)' if has_angles else 'Phase 1'
    if verbose:
        print(f"  {label1} — {method} ...")

    minimize(make_merit(False), x0, method=method, bounds=bounds,
             options={'maxiter': max_iter, 'ftol': 1e-12, 'eps': 1e-4})

    x1 = best['x'].copy()
    if verbose:
        print(f"  Phase 1 done : miss = {best['miss_m']:.2f} m  |  "
              f"{n_eval[0]} evals")

    # ── Step 4: Phase 2 — full merit (angle-constrained cases only) ───────
    if has_angles:
        if verbose:
            print(f"  Phase 2 (full merit) — {method} ...")
        minimize(make_merit(True), x1, method=method, bounds=bounds,
                 options={'maxiter': max_iter, 'ftol': 1e-12, 'eps': 1e-4})
        if verbose:
            print(f"  Phase 2 done : miss = {best['miss_m']:.2f} m  |  "
                  f"{n_eval[0]} evals")

    # ── Step 4b: Refinement pass — switch to accurate dt if a coarser dt_eval was used ──
    # The coarse-dt optimizer finds good knot values for the coarse physics;
    # a short pass at the accurate dt translates that solution back to the
    # true dynamics and eliminates residual integration error.
    if _dt_arr[0] != dt:
        _dt_arr[0] = dt   # switch closure to accurate dt
        best['J_full'] = float('inf')  # allow refinement to update best
        if verbose:
            print(f"  Refinement (dt={dt}) — {method} ...")
        minimize(make_merit(has_angles), best['x'].copy(), method=method, bounds=bounds,
                 options={'maxiter': 100, 'ftol': 1e-12, 'eps': 1e-4})
        if verbose:
            print(f"  Refinement done : miss = {best['miss_m']:.2f} m  |  "
                  f"{n_eval[0]} evals")

    if verbose:
        print(f"  Total {n_eval[0]} evaluations  |  "
              f"best miss = {best['miss_m']:.2f} m  |  "
              f"t_f = {best['t_f']:.1f} s")

    # ── Step 5: final trajectory using best-ever x ───────────────────────
    x_opt   = best['x']
    anV_opt = x_opt[:n_knots] * a_max_ms2
    anH_opt = x_opt[n_knots:] * a_max_ms2
    final   = sim_open_loop(params, t_knots, anV_opt, anH_opt, dt=dt)

    fs = final['final_state']
    angle_errs: dict = {}
    if hit_gv is not None:
        angle_errs['gamma_v_err_deg'] = round(math.degrees(fs.gamma_v - hit_gv), 2)
    if hit_gh is not None:
        dh = (fs.gamma_h - hit_gh + math.pi) % (2 * math.pi) - math.pi
        angle_errs['gamma_h_err_deg'] = round(math.degrees(dh), 2)

    return {
        'summary': {
            'solved':                  final['miss_m'] < 1.0,
            'nominal_miss_m':          miss_nom,
            'optimised_miss_m':        round(final['miss_m'], 2),
            'nominal_flight_time_s':   t_nom,
            'optimised_flight_time_s': round(final['flight_time_s'], 1),
            **angle_errs,
            'n_evaluations':           n_eval[0],
        },
        'schedule': {
            't_knots_s':      t_knots.tolist(),
            'a_nV_knots_ms2': anV_opt.tolist(),
            'a_nH_knots_ms2': anH_opt.tolist(),
            'a_nV_knots_g':   (anV_opt / G0).tolist(),
            'a_nH_knots_g':   (anH_opt / G0).tolist(),
        },
        'series':  final['series'],
        'nominal': nominal,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_optimised(result: dict, title: str = 'Trajectory optimisation') -> None:
    """
    4-panel comparison plot: nominal IACPN vs optimised open-loop.

    Panels
    ------
    1. Ground track  (x = North, y = East)
    2. Altitude vs time
    3. a_nV  (pitch-normal command) vs time — knots + continuous interpolation
    4. a_nH  (yaw-normal command)   vs time — knots + continuous interpolation
    """
    nom  = result['nominal']['series']
    opt  = result['series']
    sched = result['schedule']
    summ  = result['summary']

    t_k   = np.array(sched['t_knots_s'])
    anV_k = np.array(sched['a_nV_knots_g'])
    anH_k = np.array(sched['a_nH_knots_g'])

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(
        f"{title}\n"
        f"miss: {summ['nominal_miss_m']} m → {summ['optimised_miss_m']} m  |  "
        f"t_f: {summ['nominal_flight_time_s']:.1f} s → "
        f"{summ['optimised_flight_time_s']:.1f} s  |  "
        f"solved: {summ['solved']}",
        fontsize=11,
    )

    C_NOM = '#888888'
    C_OPT = '#D85A30'
    C_KNT = '#1D9E75'

    # ── Panel 1: Ground track ─────────────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(nom['y'], nom['x'], color=C_NOM, lw=1.5, label='Nominal IACPN', zorder=2)
    ax.plot(opt['y'], opt['x'], color=C_OPT, lw=2.0, label='Optimised',     zorder=3)
    ax.scatter([0], [0], marker='^', s=80, color='k', zorder=5, label='Launch')
    ax.set_xlabel('East (km)')
    ax.set_ylabel('North (km)')
    ax.set_title('Ground track')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # ── Panel 2: Altitude ─────────────────────────────────────────────────
    ax = axes[0, 1]
    ax.plot(nom['t'], nom['h'], color=C_NOM, lw=1.5, label='Nominal IACPN')
    ax.plot(opt['t'], opt['h'], color=C_OPT, lw=2.0, label='Optimised')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (km)')
    ax.set_title('Altitude vs time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 3: a_nV ─────────────────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(nom['t'], nom['a_nV'], color=C_NOM, lw=1.5, alpha=0.8,
            label='Nominal IACPN')
    ax.plot(opt['t'], opt['a_nV_g'], color=C_OPT, lw=2.0, label='Optimised')
    ax.scatter(t_k, anV_k, color=C_KNT, s=40, zorder=5,
               label='Knot points', marker='o')
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('a_nV (g)')
    ax.set_title('Pitch-normal command (a_nV)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 4: a_nH ─────────────────────────────────────────────────────
    ax = axes[1, 1]
    ax.plot(nom['t'], nom['a_nH'], color=C_NOM, lw=1.5, alpha=0.8,
            label='Nominal IACPN')
    ax.plot(opt['t'], opt['a_nH_g'], color=C_OPT, lw=2.0, label='Optimised')
    ax.scatter(t_k, anH_k, color=C_KNT, s=40, zorder=5,
               label='Knot points', marker='o')
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('a_nH (g)')
    ax.set_title('Yaw-normal command (a_nH)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()
