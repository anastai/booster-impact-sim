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
        if i % _LOG_EVERY == 0:
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

    Default weights make miss distance the dominant term; angle accuracy
    becomes significant once miss < ~1 m, and flight-time minimisation
    provides a gentle nudge throughout.

    Parameters
    ----------
    params     : Params.  hit_gamma_v / hit_gamma_h activate angle terms.
    n_knots    : number of piecewise-linear control knot points.
                 Higher values give richer profiles; 15–30 is usually enough.
    a_max_g    : optimiser lateral acceleration limit (g).
                 Defaults to params.a_lat_max.  The physical actuator cap
                 (params.a_lat_max) is always enforced inside the simulation
                 regardless of this setting.
    w_miss     : weight on miss² (units: per m²)
    w_angle_v  : weight on impact γV error² (per deg²)
    w_angle_h  : weight on impact γH error² (per deg²)
    w_time     : weight on flight time (per s)
    dt         : integration timestep (s)
    method     : local optimiser — 'L-BFGS-B' (default, fast) or 'SLSQP'.
                 Gradients are computed by finite differences (eps=1e-4 s).
    max_iter   : maximum local-optimiser iterations.
    use_de     : if True, run differential_evolution first for a global
                 warm start, then refine with `method`.  Slower but more
                 robust for difficult angle-constrained cases.
    verbose    : print progress to stdout.

    Returns
    -------
    dict:
        'summary'  : scalar comparison — nominal IACPN vs optimised
        'schedule' : optimal knot times and command values (SI and g)
        'series'   : full time-series of the optimised flight
        'nominal'  : complete simulate() result for the IACPN warm-start run
    """
    from .simulate import simulate as _iacpn

    a_max_ms2 = (a_max_g if a_max_g is not None else params.a_lat_max) * G0

    # ── Step 1: IACPN run — estimate flight time, build warm start ────────
    nominal   = _iacpn(params, dt=dt)
    t_nom     = nominal['summary']['flight_time_s']
    miss_nom  = nominal['summary']['miss_distance_m']

    if verbose:
        print(f"  Nominal IACPN : miss = {miss_nom} m,  t_f = {t_nom:.1f} s")

    # Knot times span [0, t_nom * 1.2] to handle minor overruns
    t_knots = np.linspace(0.0, t_nom * 1.2, n_knots)

    nom_t   = np.array(nominal['series']['t'],     dtype=float)
    nom_anV = np.array(nominal['series']['a_nV'],  dtype=float) * G0   # g → m/s²
    nom_anH = np.array(nominal['series']['a_nH'],  dtype=float) * G0

    if len(nom_t) > 1:
        x0_anV = np.interp(t_knots, nom_t, nom_anV)
        x0_anH = np.interp(t_knots, nom_t, nom_anH)
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

    n_eval = [0]
    best   = {'J': float('inf'), 'miss_m': float('inf'), 't_f': float('inf')}

    def merit(x: np.ndarray) -> float:
        n_eval[0] += 1
        anV_k = x[:n_knots] * a_max_ms2
        anH_k = x[n_knots:] * a_max_ms2

        r    = sim_open_loop(params, t_knots, anV_k, anH_k, dt=dt)
        fs   = r['final_state']
        miss = r['miss_m']
        t_f  = r['flight_time_s']

        J = w_miss * miss * miss + w_time * t_f

        if hit_gv is not None:
            err_v = math.degrees(fs.gamma_v - hit_gv)
            J += w_angle_v * err_v * err_v

        if hit_gh is not None:
            dh    = (fs.gamma_h - hit_gh + math.pi) % (2 * math.pi) - math.pi
            err_h = math.degrees(dh)
            J += w_angle_h * err_h * err_h

        if J < best['J']:
            best.update(J=J, miss_m=miss, t_f=t_f)

        return J

    # ── Step 2: optimise ─────────────────────────────────────────────────
    if verbose:
        print(f"  Optimising {2 * n_knots} knot values,  "
              f"a_max = {a_max_ms2 / G0:.2f} g,  method = {method}")

    if use_de:
        if verbose:
            print("  Phase 1 — differential_evolution (global search) ...")
        de = differential_evolution(
            merit, bounds,
            maxiter=150, popsize=5, tol=1e-6,
            seed=42, polish=False,
        )
        x0 = de.x
        if verbose:
            print(f"  DE done : miss = {best['miss_m']:.2f} m,  "
                  f"{n_eval[0]} evals")

    phase = '2' if use_de else '1'
    if verbose:
        print(f"  Phase {phase} — {method} local refinement ...")

    opt = minimize(
        merit, x0, method=method, bounds=bounds,
        options={'maxiter': max_iter, 'ftol': 1e-12, 'eps': 1e-4},
    )

    if verbose:
        print(f"  Done : {n_eval[0]} evaluations  |  "
              f"miss = {best['miss_m']:.2f} m  |  "
              f"t_f = {best['t_f']:.1f} s  |  "
              f"converged = {opt.success}")

    # ── Step 3: final trajectory with optimal schedule ────────────────────
    x_opt    = opt.x
    anV_opt  = x_opt[:n_knots] * a_max_ms2
    anH_opt  = x_opt[n_knots:] * a_max_ms2
    final    = sim_open_loop(params, t_knots, anV_opt, anH_opt, dt=dt)

    fs = final['final_state']
    angle_errs: dict = {}
    if hit_gv is not None:
        angle_errs['gamma_v_err_deg'] = round(math.degrees(fs.gamma_v - hit_gv), 2)
    if hit_gh is not None:
        dh = (fs.gamma_h - hit_gh + math.pi) % (2 * math.pi) - math.pi
        angle_errs['gamma_h_err_deg'] = round(math.degrees(dh), 2)

    return {
        'summary': {
            'nominal_miss_m':          miss_nom,
            'optimised_miss_m':        round(final['miss_m'], 2),
            'nominal_flight_time_s':   t_nom,
            'optimised_flight_time_s': round(final['flight_time_s'], 1),
            **angle_errs,
            'n_evaluations':           n_eval[0],
            'optimiser_success':       opt.success,
            'optimiser_message':       opt.message,
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
        f"{summ['optimised_flight_time_s']:.1f} s",
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
