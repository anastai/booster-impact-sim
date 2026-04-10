"""
sim/opt_trajectory.py — Direct-collocation trajectory optimizer  (v3.0)
=======================================================================
Finds the lateral acceleration schedule (a_nV(t), a_nH(t)) that maximises
RangeOnLine and minimises control effort and flight time, subject to
dynamics and terminal-position equality constraints.

Merit function  (single stage, minimise J)
------------------------------------------
    J = −w_range  · RangeOnLine
      + w_effort  · Σ |u_k|² · dt_coll  (normalised to a_max²·T)
      + w_time    · T_f

RangeOnLine: arc length of the final trajectory segment where BOTH γV
and γH are within `angle_tol_deg` of the required approach angles.
The segment is contiguous from the terminal node backward — it ends
(going backward) as soon as either angle exceeds the tolerance.

Optimisation method: direct collocation (forward-Euler transcription)
----------------------------------------------------------------------
Decision variables  (8N + 1 total):
    T_f       — total flight time           (scalar)
    u_nV[k]   — pitch-normal command        (N values, normalised −1…+1)
    u_nH[k]   — yaw-normal command          (N values, normalised −1…+1)
    s[k]      — [v, γV, γH, h, x, y]       (N nodes, nodes 1…N)

Equality constraints  (6N + 3):
    s[k+1] = Euler(s[k], u[k], dt)   k = 0…N−1   (dynamics defects)
    x_N = x_target,  y_N = y_target,  h_N = z_target   (terminal)

Initial state s[0] is fixed.  Solver: SLSQP (scipy).

Public API
----------
    from sim.opt_trajectory import optimize_trajectory, sim_open_loop, plot_optimised

    result = optimize_trajectory(params)
    result = optimize_trajectory(params, N=100, a_max_g=5.0)
    plot_optimised(result)
"""
from __future__ import annotations

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from .physics import Params, State, gravity, integrate_step, G0
from .atmosphere import density

_MAX_STEPS = 60_000
_LOG_EVERY = 5


# ---------------------------------------------------------------------------
# Open-loop forward simulation  (unchanged from v2.x)
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
    params     : Params
    t_knots    : knot times (s), shape (N,)
    anV_knots  : a_nV at each knot (m/s²), shape (N,)
    anH_knots  : a_nH at each knot (m/s²), shape (N,)
    dt         : integration timestep (s)

    Returns
    -------
    dict: miss_m, flight_time_s, final_state, series
    """
    thrust_n      = params.thrust_kn * 1_000.0
    x_t           = params.x_target  * 1_000.0
    y_t           = params.y_target  * 1_000.0
    z_t           = params.z_target  * 1_000.0
    a_lat_max_ms2 = params.a_lat_max * G0
    Aref          = math.pi * (params.diam / 2.0) ** 2

    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)

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

        cd_now    = params.cd if state.engine_on else params.cd_fall
        drag_aero = qS * cd_now
        lift      = qS * params.cl

        a_nV = float(np.interp(state.t, t_knots, anV_knots))
        a_nH = float(np.interp(state.t, t_knots, anH_knots))

        mag = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        if a_lat_max_ms2 > 0.0 and mag > a_lat_max_ms2:
            s    = a_lat_max_ms2 / mag
            a_nV *= s
            a_nH *= s

        aLat   = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        d_ctrl = (qS * params.cd_ctrl * (aLat / a_lat_max_ms2) ** 2
                  if a_lat_max_ms2 > 0.0 else 0.0)

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

        state, _ = integrate_step(
            state, params,
            thr_now, drag_aero, lift, d_ctrl,
            grav, (a_nV, a_nH), dt,
        )

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
# Fast evaluation kernel  (kept for backward compatibility)
# ---------------------------------------------------------------------------

def _sim_eval_fast(
    params: Params,
    t_knots: np.ndarray,
    anV_knots: np.ndarray,
    anH_knots: np.ndarray,
    dt: float,
) -> tuple:
    """Fast optimizer-only evaluation kernel. Returns (miss_m, t_f, gV_f, gH_f, ctrl_effort, x_f, y_f, h_f)."""
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
    gV0_fixed     = math.radians(params.launch_angle)

    nk        = len(t_knots)
    tk0       = float(t_knots[0])
    tk1       = float(t_knots[-1])
    tk_span   = tk1 - tk0 if tk1 > tk0 else 1e-9
    nk_minus1 = float(nk - 1)

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
    ctrl_effort  = 0.0
    prev_u2      = 0.0

    for _ in range(_MAX_STEPS):
        spd  = v
        grav = gravity(h)
        rho  = density(params.atm, h)
        qS   = 0.5 * rho * spd * spd * Aref

        if eng_on and prop_rem > 0:
            thr_now  = thrust_n
            prop_rem = prop_rem - mdot * dt
            mass     = mass - mdot * dt
            if prop_rem <= 0.0 or t >= params.burn_max:
                eng_on = False
                mass   = m_pay_str + (params.m_prop - prop_used)
        else:
            thr_now = 0.0

        cd_now    = cd_pow if eng_on else cd_fall
        drag_aero = qS * cd_now
        lift      = qS * cl

        idx_f = (t - tk0) / tk_span * nk_minus1
        idx   = int(idx_f)
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

        mag = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        if a_lat_max_ms2 > 0.0 and mag > a_lat_max_ms2:
            s = a_lat_max_ms2 / mag
            a_nV *= s
            a_nH *= s

        aLat   = math.sqrt(a_nV * a_nV + a_nH * a_nH)
        d_ctrl = (qS * cd_ctrl * (aLat / a_lat_max_ms2) ** 2
                  if a_lat_max_ms2 > 0.0 else 0.0)
        drag_tot = drag_aero + d_ctrl

        curr_u2      = a_nV * a_nV + a_nH * a_nH
        ctrl_effort += 0.5 * (prev_u2 + curr_u2) * dt
        prev_u2      = curr_u2

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
            vcos   = spd_safe * cos_gV
            dgH_dt = ((T_nH / (mass * vcos)
                       + a_cmd_nh / vcos
                       + a_nH    / vcos)
                      if abs(vcos) > 0.5 else 0.0)
        else:
            dgV_dt = 0.0
            dgH_dt = 0.0

        new_v  = max(0.0, v + dv_dt * dt)
        new_gV = gV + dgV_dt * dt
        _gV_lim = math.pi / 2 - 1e-4
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

        if h > z_t + 50.0:
            passed_above = True
        if passed_above and h <= z_t:
            break
        if h <= 0.0:
            break

    miss = math.sqrt((x - x_t) ** 2 + (y - y_t) ** 2 + (h - z_t) ** 2)
    return miss, t, gV, gH, ctrl_effort, x, y, h


# ---------------------------------------------------------------------------
# Direct-collocation helpers
# ---------------------------------------------------------------------------

def _make_precomp(params: Params, a_max_ms2: float) -> dict:
    """Precompute physics constants used in every collocation step."""
    thrust_n  = params.thrust_kn * 1_000.0
    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)
    burn_dur  = min(prop_used / mdot if mdot > 0 else 0.0, params.burn_max)
    m0        = params.m_pay + params.m_prop + params.m_str
    m_coast   = params.m_pay + params.m_str + (params.m_prop - prop_used)
    Aref      = math.pi * (params.diam / 2.0) ** 2
    return dict(
        thrust_n=thrust_n, mdot=mdot, prop_used=prop_used,
        burn_dur=burn_dur, m0=m0, m_coast=m_coast, Aref=Aref,
        a_max=a_max_ms2,
        cd=params.cd, cd_fall=params.cd_fall, cl=params.cl, cd_ctrl=params.cd_ctrl,
        atm=params.atm,
        grav_turn=params.grav_turn,
        grav_turn_v_min=params.grav_turn_v_min,
        gV0_fixed=math.radians(params.launch_angle),
    )


def _euler_node(
    s:     np.ndarray,   # [v, gV, gH, h, x, y]
    u_nV:  float,
    u_nH:  float,
    dt:    float,
    t_k:   float,        # time at the start of this interval
    Pc:    dict,
) -> np.ndarray:
    """
    One forward-Euler collocation step.
    Applies the same physics as integrate_step/sim_open_loop but operates
    directly on flat numpy arrays for speed inside the constraint loop.
    Returns next state as a (6,) array [v, gV, gH, h, x, y].
    """
    v   = float(s[0])
    gV  = float(s[1])
    gH  = float(s[2])
    h   = max(float(s[3]), 0.0)   # safety clamp
    x   = float(s[4])
    y   = float(s[5])

    grav = gravity(h)
    rho  = density(Pc['atm'], h)
    spd  = max(v, 1e-6)
    qS   = 0.5 * rho * v * v * Pc['Aref']

    # Mass and thrust
    if t_k < Pc['burn_dur']:
        mass = max(Pc['m0'] - Pc['mdot'] * t_k, Pc['m_coast'])
        thr  = Pc['thrust_n']
        cd_now = Pc['cd']
    else:
        mass   = Pc['m_coast']
        thr    = 0.0
        cd_now = Pc['cd_fall']

    # Aerodynamics
    drag_aero = qS * cd_now
    lift      = qS * Pc['cl']

    # Actuator saturation (vector limit)
    mag = math.sqrt(u_nV * u_nV + u_nH * u_nH)
    if Pc['a_max'] > 0.0 and mag > Pc['a_max']:
        sc   = Pc['a_max'] / mag
        u_nV *= sc
        u_nH *= sc

    aLat   = math.sqrt(u_nV * u_nV + u_nH * u_nH)
    d_ctrl = (qS * Pc['cd_ctrl'] * (aLat / Pc['a_max']) ** 2
              if Pc['a_max'] > 0.0 else 0.0)
    drag_tot = drag_aero + d_ctrl

    # Thrust decomposition
    if Pc['grav_turn']:
        T_t, T_nV = thr, 0.0
    else:
        alpha = Pc['gV0_fixed'] - gV
        T_t   = thr * math.cos(alpha)
        T_nV  = thr * math.sin(alpha)

    sin_gV = math.sin(gV)
    cos_gV = math.cos(gV)

    dv_dt = (T_t - drag_tot) / mass - grav * sin_gV

    rotation_active = spd > (Pc['grav_turn_v_min'] if Pc['grav_turn'] else 0.5)
    if rotation_active:
        dgV_dt = (T_nV + lift) / (mass * spd) - grav * cos_gV / spd + u_nV / spd
        vcos   = spd * cos_gV
        dgH_dt = (u_nH / vcos if abs(vcos) > 0.5 else 0.0)
    else:
        dgV_dt = 0.0
        dgH_dt = 0.0

    new_v  = max(0.0, v + dv_dt * dt)
    new_gV = gV + dgV_dt * dt
    _lim   = math.pi / 2 - 1e-4
    new_gV = max(-_lim, min(_lim, new_gV))
    new_gH = gH + dgH_dt * dt
    new_h  = h  + new_v * math.sin(new_gV) * dt
    new_x  = x  + new_v * math.cos(new_gV) * math.cos(new_gH) * dt
    new_y  = y  + new_v * math.cos(new_gV) * math.sin(new_gH) * dt

    return np.array([new_v, new_gV, new_gH, new_h, new_x, new_y])


def _smooth_range_on_line(
    gV_arr:    np.ndarray,   # (N+1,) flight-path angles at all nodes (rad)
    gH_arr:    np.ndarray,   # (N+1,) headings at all nodes (rad)
    v_arr:     np.ndarray,   # (N+1,) speeds at all nodes (m/s)
    hit_gV:    float,        # required impact γV (rad)
    hit_gH:    float,        # required impact γH (rad)
    tol_rad:   float,        # angle tolerance (rad)
    dt:        float,        # collocation time step (s)
    sharpness: float = 20.0, # sigmoid steepness (rad⁻¹)
) -> float:
    """
    Smooth differentiable approximation of RangeOnLine.

    Uses a cumulative-product window from the terminal node:
        P[k] = Π_{j=k}^{N} σ(tol − err_j)
    so P[k] → 1 only when every node from k to N is within tolerance.

    RangeOnLine ≈ Σ_{k=1}^{N} P[k] · v[k] · dt
    """
    err_v   = np.abs(gV_arr - hit_gV)
    dh_arr  = ((gH_arr - hit_gH + np.pi) % (2.0 * np.pi)) - np.pi
    err_h   = np.abs(dh_arr)
    max_err = np.maximum(err_v, err_h)          # both channels must comply

    # Sigmoid compliance indicator (1 = in tolerance, 0 = outside)
    ind     = 1.0 / (1.0 + np.exp(sharpness * (max_err - tol_rad)))

    # Cumulative product from terminal: P[k] = exp(Σ_{j=k}^{N} log ind[j])
    log_ind = np.log(np.maximum(ind, 1e-30))
    cum_log = np.cumsum(log_ind[::-1])[::-1]    # running sum from right
    P       = np.exp(cum_log)

    # Arc length weighted by compliance window
    return float(np.sum(P[1:] * v_arr[1:] * dt))


# ---------------------------------------------------------------------------
# Direct-collocation optimizer
# ---------------------------------------------------------------------------

def optimize_trajectory(
    params:          Params,
    N:               int        = 80,
    n_knots:         int | None = None,    # alias for N (backward compat)
    a_max_g:         float|None = None,
    # ── Merit function weights ──────────────────────────────────────────────
    w_range:         float      = 1.0,     # RangeOnLine  (maximise)
    w_effort:        float      = 1e-3,    # control effort ∫|u|²dt / (a_max² T)
    w_time:          float      = 1e-4,    # flight time T_f
    angle_tol_deg:   float      = 5.0,     # RangeOnLine angle tolerance (deg)
    angle_sharpness: float      = 20.0,    # sigmoid sharpness for smooth RoL
    # ── Solver ──────────────────────────────────────────────────────────────
    method:          str        = 'SLSQP',
    max_iter:        int        = 300,
    verbose:         bool       = True,
    miss_threshold:  float      = 1.0,
    dt:              float      = 0.2,     # final high-res sim timestep (s)
    # ── Absorb legacy keyword args silently ─────────────────────────────────
    **_legacy,
) -> dict:
    """
    Direct-collocation trajectory optimizer.

    Optimises a_nV(t), a_nH(t) to maximise RangeOnLine and minimise
    control effort and flight time in a single optimisation stage.

    RangeOnLine is the arc length of the terminal approach segment where
    both γV and γH are within `angle_tol_deg` of (params.hit_gamma_v,
    params.hit_gamma_h).  Requires both angle constraints to be set.

    Parameters
    ----------
    params          : Params.  hit_gamma_v and hit_gamma_h must be set.
    N               : collocation nodes (default 80).
    n_knots         : alias for N (backward compatibility).
    a_max_g         : optimizer acceleration limit (g).  Defaults to params.a_lat_max.
    w_range         : weight on RangeOnLine (positive = maximise distance).
    w_effort        : weight on normalised control effort.
    w_time          : weight on flight time (s).
    angle_tol_deg   : angular tolerance for RangeOnLine (deg).
    angle_sharpness : sigmoid sharpness parameter.
    method          : scipy optimizer ('SLSQP' required for constraints).
    max_iter        : max SLSQP iterations.
    verbose         : print progress.
    miss_threshold  : miss (m) below which 'solved' = True.
    dt              : timestep for final high-resolution forward simulation.

    Returns
    -------
    dict:
        'summary'  : scalar results including solved, miss, RangeOnLine
        'schedule' : collocation node times and commands
        'series'   : high-res time-series from final sim_open_loop call
        'nominal'  : IACPN warm-start result
    """
    from .simulate import simulate as _iacpn

    # n_knots is accepted as alias for N
    if n_knots is not None and N == 80:
        N = n_knots

    a_max_ms2 = (a_max_g if a_max_g is not None else params.a_lat_max) * G0
    Pc        = _make_precomp(params, a_max_ms2)

    x_t = params.x_target * 1_000.0
    y_t = params.y_target * 1_000.0
    z_t = params.z_target * 1_000.0

    hit_gV   = math.radians(params.hit_gamma_v) if params.hit_gamma_v is not None else None
    hit_gH   = math.radians(params.hit_gamma_h) if params.hit_gamma_h is not None else None
    tol_rad  = math.radians(angle_tol_deg)

    # ── Warm start: IACPN nominal run ────────────────────────────────────────
    nominal  = _iacpn(params, dt=dt)
    t_nom    = nominal['summary']['flight_time_s']
    miss_nom = nominal['summary']['miss_distance_m']

    if verbose:
        print(f"  Nominal IACPN : miss = {miss_nom:.1f} m,  t_f = {t_nom:.1f} s")

    T_f_init = t_nom * 1.2
    dt_coll  = T_f_init / N

    # ── Initial state (fixed, node 0) ────────────────────────────────────────
    s0 = np.array([
        0.0,                                    # v
        math.radians(params.launch_angle),      # gV
        math.radians(params.launch_azimuth),    # gH
        0.0,                                    # h
        0.0,                                    # x
        0.0,                                    # y
    ])

    # ── Build warm-start controls from IACPN series ──────────────────────────
    ser     = nominal['series']
    ws_t    = np.array(ser['t'], dtype=float)
    t_nodes = np.linspace(0.0, T_f_init, N + 1)          # N+1 node times
    t_ctrls = t_nodes[:-1]                                 # N control times (node 0..N-1)

    ws_anV  = np.interp(t_ctrls, ws_t, np.array(ser['a_nV']) * G0)   # g → m/s²
    ws_anH  = np.interp(t_ctrls, ws_t, np.array(ser['a_nH']) * G0)

    u_nV_init = np.clip(ws_anV / a_max_ms2, -1.0, 1.0)
    u_nH_init = np.clip(ws_anH / a_max_ms2, -1.0, 1.0)

    # Forward-propagate warm-start controls at dt_coll to get near-zero defects
    states_ws = np.empty((N, 6))
    s_k = s0.copy()
    for k in range(N):
        s_next = _euler_node(
            s_k,
            float(u_nV_init[k]) * a_max_ms2,
            float(u_nH_init[k]) * a_max_ms2,
            dt_coll, k * dt_coll, Pc,
        )
        states_ws[k] = s_next
        s_k = s_next

    # Pack: z = [T_f, u_nV_0..N-1, u_nH_0..N-1, s_1_flat .. s_N_flat]
    z0 = np.concatenate([[T_f_init], u_nV_init, u_nH_init, states_ws.ravel()])

    # ── Bounds ────────────────────────────────────────────────────────────────
    _gV_lim = math.pi / 2 - 1e-4
    bounds  = [(t_nom * 0.5, t_nom * 2.5)]           # T_f
    bounds += [(-1.0, 1.0)] * N                       # u_nV (normalised)
    bounds += [(-1.0, 1.0)] * N                       # u_nH (normalised)
    for _ in range(N):
        bounds += [
            (0.0,   5_000.0),           # v (m/s)
            (-_gV_lim, _gV_lim),        # gV (rad)
            (-4 * math.pi, 4 * math.pi),# gH (rad)
            (0.0,   200_000.0),         # h (m)
            (-5e6,  5e6),               # x (m)
            (-5e6,  5e6),               # y (m)
        ]

    # ── Unpack helper ─────────────────────────────────────────────────────────
    def _unpack(z):
        T_f      = z[0]
        u_nV_n   = z[1:N + 1]
        u_nH_n   = z[N + 1: 2 * N + 1]
        states   = z[2 * N + 1:].reshape(N, 6)   # states[k] = node k+1
        return T_f, u_nV_n, u_nH_n, states

    # ── Objective ─────────────────────────────────────────────────────────────
    def _objective(z):
        T_f, u_nV_n, u_nH_n, states = _unpack(z)
        dt_c = T_f / N

        # Assemble all-node arrays (node 0 fixed + decision vars 1..N)
        v_all  = np.empty(N + 1)
        gV_all = np.empty(N + 1)
        gH_all = np.empty(N + 1)
        v_all[0]  = s0[0]
        gV_all[0] = s0[1]
        gH_all[0] = s0[2]
        v_all[1:]  = states[:, 0]
        gV_all[1:] = states[:, 1]
        gH_all[1:] = states[:, 2]

        # RangeOnLine (smooth approximation)
        if hit_gV is not None and hit_gH is not None:
            rl = _smooth_range_on_line(
                gV_all, gH_all, v_all, hit_gV, hit_gH, tol_rad, dt_c, angle_sharpness
            )
        else:
            rl = 0.0

        # Normalised control effort  Σ|u_norm|² · dt_c / (N · dt_c) = Σ|u_norm|² / N
        effort = float(np.sum(u_nV_n ** 2 + u_nH_n ** 2)) / N

        return -w_range * rl + w_effort * effort + w_time * T_f

    # ── Collocation constraints ───────────────────────────────────────────────
    def _constraints(z):
        T_f, u_nV_n, u_nH_n, states = _unpack(z)
        dt_c = T_f / N

        defects = np.empty(6 * N + 3)
        s_k = s0
        for k in range(N):
            t_k          = k * dt_c
            u_nV         = float(u_nV_n[k]) * a_max_ms2
            u_nH         = float(u_nH_n[k]) * a_max_ms2
            s_next_pred  = _euler_node(s_k, u_nV, u_nH, dt_c, t_k, Pc)
            s_next_dv    = states[k]
            defects[6 * k: 6 * k + 6] = s_next_pred - s_next_dv
            s_k = s_next_dv   # next step uses the decision-variable state

        # Terminal position constraints (h, x, y)
        defects[6 * N]     = states[N - 1, 3] - z_t   # h_N = z_target
        defects[6 * N + 1] = states[N - 1, 4] - x_t   # x_N = x_target
        defects[6 * N + 2] = states[N - 1, 5] - y_t   # y_N = y_target
        return defects

    # ── Run SLSQP ─────────────────────────────────────────────────────────────
    n_vars  = len(z0)
    n_con   = 6 * N + 3
    if verbose:
        print(f"  Direct collocation: N={N},  dt_coll={dt_coll:.2f} s,  "
              f"vars={n_vars},  constraints={n_con}")
        print(f"  Method: {method},  max_iter={max_iter}")

    opt_result = minimize(
        _objective, z0,
        method=method,
        bounds=bounds,
        constraints={'type': 'eq', 'fun': _constraints},
        options={'maxiter': max_iter, 'ftol': 1e-9, 'disp': False},
    )

    z_opt = opt_result.x
    T_f_opt, u_nV_opt_n, u_nH_opt_n, _ = _unpack(z_opt)

    anV_opt = u_nV_opt_n * a_max_ms2
    anH_opt = u_nH_opt_n * a_max_ms2

    # Collocation node times (N values spanning 0 → T_f_opt)
    t_coll_opt = np.linspace(0.0, T_f_opt, N)

    # ── Final high-resolution forward simulation ──────────────────────────────
    final  = sim_open_loop(params, t_coll_opt, anV_opt, anH_opt, dt=dt)
    fs     = final['final_state']
    miss_opt = final['miss_m']

    if verbose:
        print(f"  SLSQP status: {opt_result.message}")
        print(f"  Optimised: miss = {miss_opt:.2f} m,  t_f = {final['flight_time_s']:.1f} s")

    # ── RangeOnLine on final high-res trajectory (exact, hard threshold) ──────
    ser_f     = final['series']
    gV_f      = np.radians(np.array(ser_f['gamma_v_deg']))
    gH_f      = np.radians(np.array(ser_f['gamma_h_deg']))
    v_f       = np.array(ser_f['v'])
    dt_f_eff  = dt * _LOG_EVERY            # effective sample interval in series

    rol_m = 0.0
    if hit_gV is not None and hit_gH is not None:
        on_line = True
        for i in range(len(gV_f) - 1, 0, -1):
            ev = abs(gV_f[i] - hit_gV)
            dh = ((gH_f[i] - hit_gH + math.pi) % (2 * math.pi)) - math.pi
            eh = abs(dh)
            if on_line and ev <= tol_rad and eh <= tol_rad:
                rol_m += v_f[i] * dt_f_eff
            else:
                on_line = False

    if verbose:
        print(f"  RangeOnLine = {rol_m / 1_000:.2f} km")

    # ── Angle errors at impact ────────────────────────────────────────────────
    angle_errs: dict = {}
    if hit_gV is not None:
        angle_errs['gamma_v_err_deg'] = round(math.degrees(fs.gamma_v - hit_gV), 2)
    if hit_gH is not None:
        dh = (fs.gamma_h - hit_gH + math.pi) % (2 * math.pi) - math.pi
        angle_errs['gamma_h_err_deg'] = round(math.degrees(dh), 2)

    return {
        'summary': {
            'solved':                  miss_opt < miss_threshold,
            'nominal_miss_m':          miss_nom,
            'optimised_miss_m':        round(miss_opt, 2),
            'nominal_flight_time_s':   t_nom,
            'optimised_flight_time_s': round(final['flight_time_s'], 1),
            'range_on_line_km':        round(rol_m / 1_000, 3),
            'angle_tol_deg':           angle_tol_deg,
            'slsqp_success':           bool(opt_result.success),
            'slsqp_message':           opt_result.message,
            **angle_errs,
        },
        'schedule': {
            't_knots_s':      t_coll_opt.tolist(),
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
    4-panel comparison plot: nominal IACPN vs direct-collocation optimised.

    Panels
    ------
    1. Ground track  (x = North, y = East)
    2. Altitude vs time
    3. a_nV  (pitch-normal command) vs time
    4. a_nH  (yaw-normal command)   vs time
    """
    nom   = result['nominal']['series']
    opt   = result['series']
    sched = result['schedule']
    summ  = result['summary']

    t_k   = np.array(sched['t_knots_s'])
    anV_k = np.array(sched['a_nV_knots_g'])
    anH_k = np.array(sched['a_nH_knots_g'])

    rol_str = (f"  RoL: {summ['range_on_line_km']:.2f} km"
               if 'range_on_line_km' in summ else '')
    gv_err  = summ.get('gamma_v_err_deg')
    gh_err  = summ.get('gamma_h_err_deg')
    err_str = ''
    if gv_err is not None:
        err_str += f'  ΔγV={gv_err:+.1f}°'
    if gh_err is not None:
        err_str += f'  ΔγH={gh_err:+.1f}°'

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(
        f"{title}\n"
        f"miss: {summ['nominal_miss_m']:.1f} m → {summ['optimised_miss_m']:.1f} m  |  "
        f"t_f: {summ['nominal_flight_time_s']:.1f} s → {summ['optimised_flight_time_s']:.1f} s  |  "
        f"solved: {summ['solved']}{rol_str}{err_str}",
        fontsize=11,
    )

    C_NOM = '#888888'
    C_OPT = '#D85A30'
    C_KNT = '#1D9E75'

    # Panel 1: Ground track
    ax = axes[0, 0]
    ax.plot(nom['y'], nom['x'], color=C_NOM, lw=1.5, label='Nominal IACPN', zorder=2)
    ax.plot(opt['y'], opt['x'], color=C_OPT, lw=2.0, label='Optimised',     zorder=3)
    ax.scatter([0], [0], marker='^', s=80, color='k', zorder=5, label='Launch')
    ax.set_xlabel('East (km)'); ax.set_ylabel('North (km)')
    ax.set_title('Ground track')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # Panel 2: Altitude
    ax = axes[0, 1]
    ax.plot(nom['t'], nom['h'], color=C_NOM, lw=1.5, label='Nominal IACPN')
    ax.plot(opt['t'], opt['h'], color=C_OPT, lw=2.0, label='Optimised')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Altitude (km)')
    ax.set_title('Altitude vs time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Panel 3: a_nV
    ax = axes[1, 0]
    ax.plot(nom['t'], nom['a_nV'], color=C_NOM, lw=1.5, alpha=0.8, label='Nominal IACPN')
    ax.plot(opt['t'], opt['a_nV_g'], color=C_OPT, lw=2.0, label='Optimised')
    ax.scatter(t_k, anV_k, color=C_KNT, s=30, zorder=5, label='Collocation nodes')
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('a_nV (g)')
    ax.set_title('Pitch-normal command (a_nV)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Panel 4: a_nH
    ax = axes[1, 1]
    ax.plot(nom['t'], nom['a_nH'], color=C_NOM, lw=1.5, alpha=0.8, label='Nominal IACPN')
    ax.plot(opt['t'], opt['a_nH_g'], color=C_OPT, lw=2.0, label='Optimised')
    ax.scatter(t_k, anH_k, color=C_KNT, s=30, zorder=5, label='Collocation nodes')
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('a_nH (g)')
    ax.set_title('Yaw-normal command (a_nH)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()
