"""
sim/collocate.py  —  Direct Collocation (Hermite-Simpson) Trajectory Optimizer
===============================================================================
Finds the lateral acceleration schedule  (a_nV(t), a_nH(t))  that minimises
the same merit function as opt_trajectory.py, but via a fundamentally different
mathematical approach.

Unlike opt_trajectory.py (which runs a forward simulation inside every scipy
function evaluation), this module transcribes the OCP into a single large NLP:
states and controls at every collocation node become optimisation variables and
the dynamics appear as equality constraints.  IPOPT solves the NLP in one pass
using exact gradients from CasADi's automatic differentiation.

Approach comparison
-------------------
  opt_trajectory.py   — indirect / shooting:
    - Parameterise u(t) at N knot points.
    - Call forward-sim inside every gradient evaluation (finite-diff or DE).
    - Gradient noise from discrete simulation; no guarantee of constraint
      feasibility at intermediate iterates.

  collocate.py (this file) — direct collocation:
    - Optimisation variables = states + controls at every node.
    - Dynamics become NLP equality constraints (Hermite-Simpson defects).
    - CasADi supplies exact analytic gradients (forward/reverse AD).
    - IPOPT exploits sparsity; scales to hundreds of nodes.
    - Solution is guaranteed to satisfy the discrete dynamics exactly.

Two-phase problem structure
---------------------------
  Phase 1 — Powered:  N1 Hermite-Simpson segments, t ∈ [0, t_burn].
    State  s₁ = [v, γV, γH, h, x, y, mass]          (7 states)
  Phase 2 — Coast:    N2 Hermite-Simpson segments, t ∈ [t_burn, T_f].
    State  s₂ = [v, γV, γH, h, x, y]                (6 states; mass fixed)

  t_burn is fixed (determined by propellant and burn_max).
  T_f    is a free optimisation variable (total flight time).

Hermite-Simpson defect (per segment, step Δt)
---------------------------------------------
  s_mid  = (sₖ + sₖ₊₁)/2  +  (Δt/8) · [f(sₖ,uₖ) − f(sₖ₊₁,uₖ₊₁)]
  u_mid  = (uₖ + uₖ₊₁)/2
  defect = sₖ₊₁ − sₖ − (Δt/6) · [f(sₖ,uₖ) + 4·f(s_mid,u_mid) + f(sₖ₊₁,uₖ₊₁)] = 0

Atmosphere
----------
Exponential model  ρ = 1.225·exp(−h/8 500)  — smoothly differentiable.
The piecewise ISA model is not used here because CasADi requires
smooth (at least C¹) functions for IPOPT's interior-point solver.

Merit function
--------------
  Hit-line mode  (both hit_gamma_v AND hit_gamma_h set in params):
    A line is drawn through the target point in the desired impact direction
    d̂ = (cos γV_des·cos γH_des,  cos γV_des·sin γH_des,  sin γV_des).
    The booster must land anywhere on this line (offset s is a free NLP
    variable) with angle error ≤ angle_tol_deg on both channels.

      J = w_ctrl · ctrl_effort(pct_mean)
        − w_final_line · FinalLine_km
        + w_time · T_f

    FinalLine: arc length of the terminal approach segment where BOTH γV
    and γH are within final_line_tol_deg of the desired angles (default 3°).
    Added to J as a quadratic angle penalty over the last final_line_nodes nodes:
      penalty = Σ_{k=N2−M}^{N2} exp(0.2·(k−N2)) · (err_gV² + err_gH_cos²) / tol²
    Quadratic gradients are well-conditioned: err→0 at solution → gradient→0
    naturally, giving IPOPT clean convergence (unlike sigmoid merit terms).
    The actual FinalLine arc length (exact, hard 3° threshold) is computed in
    post-processing and reported in summary['final_line_km'].

    Terminal constraints (hard):
      x_f = x_t + s·d̂_x,  y_f = y_t + s·d̂_y,  h_f = z_t + s·d̂_z
      |γV_f − γV_des| ≤ angle_tol_deg
      |γH_f − γH_des| ≤ angle_tol_deg   (handled via cos(Δγ_H) ≥ cos(tol))

    Output key: 'hit_line_offset_m' = s  (signed distance from nominal target)

  Point-target mode  (original, backward compatible, angle constraint absent):
      J = w_miss · miss_m²  +  w_angle_v · γV_err²  +  w_angle_h · γH_err²
        + w_time · T_f

Warm start
----------
An IACPN reference simulation is run first.  Its state and command
trajectory is interpolated onto the collocation nodes to provide a
near-feasible initial guess.  This is critical for IPOPT convergence.

Dependencies
------------
  pip install casadi          # CasADi NLP framework with bundled IPOPT
"""
from __future__ import annotations

import math
import numpy as np

try:
    import casadi as ca
except ImportError as exc:
    raise ImportError(
        "CasADi is required for direct collocation.\n"
        "Install it with:  pip install casadi"
    ) from exc

from .physics import Params, G0
from .simulate import simulate as _sim_iacpn

# ── Atmosphere / gravity (CasADi-compatible) ──────────────────────────────────

_RHO0    = 1.225       # sea-level density  (kg/m³)
_H_SCALE = 8_500.0     # exponential scale height  (m)
_RE      = 6_371_000.0


def _rho(h):
    """Exponential atmosphere density — CasADi MX or float."""
    return _RHO0 * ca.exp(-h / _H_SCALE)


def _grav(h):
    """Altitude-dependent gravity — CasADi MX or float."""
    return G0 * (_RE / (_RE + h)) ** 2


# ── Velocity-frame dynamics ───────────────────────────────────────────────────

def _f_powered(s, u, p: Params, thrust_n: float, mdot: float, a_max_fn=None):
    """
    Continuous-time ODE for the POWERED phase.

    s = [v, γV, γH, h, x, y, mass]   (7-vector, CasADi MX)
    u = [a_nV, a_nH]                  (2-vector, m/s²)

    a_max_fn : callable(v) → a_max_m_s2, or None (uses p.a_lat_max * G0).
               May return a CasADi MX when v is symbolic.

    Matches the velocity-frame equations in physics.integrate_step with
    grav_turn=True and a_cmd_* from params applied as bias terms.
    """
    v, gV, gH, h, x, y, mass = (s[i] for i in range(7))
    a_nV, a_nH = u[0], u[1]

    Aref = math.pi * (p.diam / 2.0) ** 2
    spd  = ca.fmax(v, 1e-4)          # regularise to avoid 1/0 at v=0
    rho  = _rho(h)
    g    = _grav(h)
    qS   = 0.5 * rho * spd**2 * Aref

    drag    = qS * p.cd              # powered-ascent drag
    lift    = qS * p.cl
    a_max   = a_max_fn(v) if a_max_fn is not None else p.a_lat_max * G0
    # Control drag (actuator penalty proportional to lateral demand²)
    a_lat2  = a_nV**2 + a_nH**2 + 1e-12
    d_ctrl  = qS * p.cd_ctrl * (a_lat2 / ca.fmax(a_max**2, 1.0)) if p.cd_ctrl > 0 else 0.0
    drag_t  = drag + d_ctrl

    # Gravity turn: thrust aligned with velocity (T_nV = T_nH = 0)
    T_t = thrust_n

    # Velocity-frame ODEs (see physics.py for derivation)
    dv   = (T_t - drag_t) / mass - g * ca.sin(gV)

    # Angle rates are singular at v=0 (g*cos(gV)/v → ∞).
    # Clamp the denominator speed to max(v, v_thresh) in the angle-rate
    # equations — this bounds g/v ≤ g/v_thresh (finite) and is differentiable
    # everywhere (no NaN in CasADi Jacobians).  The clamp effectively holds
    # angles fixed at very low speeds, matching physics.integrate_step behaviour.
    _v_thresh = max(p.grav_turn_v_min, 1.0)
    spd_ang = ca.fmax(v, _v_thresh)     # speed used only in angle-rate denominators

    cos_gV  = ca.cos(gV)
    vcos_gV = spd_ang * cos_gV
    vcos_safe = ca.fmax(ca.fabs(vcos_gV), 0.5) * ca.sign(cos_gV + 1e-9)

    dgV = ((lift / (mass * spd_ang))
           - g * cos_gV / spd_ang
           + (p.a_cmd_nv + a_nV) / spd_ang)
    dgH = (p.a_cmd_nh + a_nH) / vcos_safe

    dh   = v * ca.sin(gV)
    dx   = v * ca.cos(gV) * ca.cos(gH)
    dy   = v * ca.cos(gV) * ca.sin(gH)
    dm   = -mdot

    return ca.vertcat(dv, dgV, dgH, dh, dx, dy, dm)


def _f_coast(s, u, p: Params, mass_bo: float, a_max_fn=None):
    """
    Continuous-time ODE for the COAST phase.

    s = [v, γV, γH, h, x, y]   (6-vector; mass fixed at burnout value)
    u = [a_nV, a_nH]            (m/s²)

    a_max_fn : callable(v) → a_max_m_s2, or None (uses p.a_lat_max * G0).
    """
    v, gV, gH, h, x, y = (s[i] for i in range(6))
    a_nV, a_nH = u[0], u[1]
    mass = mass_bo

    Aref = math.pi * (p.diam / 2.0) ** 2
    spd  = ca.fmax(v, 1e-4)
    rho  = _rho(h)
    g    = _grav(h)
    qS   = 0.5 * rho * spd**2 * Aref

    drag    = qS * p.cd_fall         # tumbling drag post-burnout
    lift    = qS * p.cl
    a_max   = a_max_fn(v) if a_max_fn is not None else p.a_lat_max * G0
    a_lat2  = a_nV**2 + a_nH**2 + 1e-12
    d_ctrl  = qS * p.cd_ctrl * (a_lat2 / ca.fmax(a_max**2, 1.0)) if p.cd_ctrl > 0 else 0.0
    drag_t  = drag + d_ctrl

    dv   = (-drag_t / mass - g * ca.sin(gV))

    # Same singularity fix as powered phase
    _v_thresh = max(p.grav_turn_v_min, 1.0)
    spd_ang = ca.fmax(v, _v_thresh)

    cos_gV  = ca.cos(gV)
    vcos_gV = spd_ang * cos_gV
    vcos_safe = ca.fmax(ca.fabs(vcos_gV), 0.5) * ca.sign(cos_gV + 1e-9)

    dgV = ((lift / (mass * spd_ang))
           - g * cos_gV / spd_ang
           + (p.a_cmd_nv + a_nV) / spd_ang)
    dgH = (p.a_cmd_nh + a_nH) / vcos_safe

    dh   = v * ca.sin(gV)
    dx   = v * ca.cos(gV) * ca.cos(gH)
    dy   = v * ca.cos(gV) * ca.sin(gH)

    return ca.vertcat(dv, dgV, dgH, dh, dx, dy)


# ── Hermite-Simpson defect ────────────────────────────────────────────────────

def _hs_defect(f, sk, sk1, uk, uk1, h_step):
    """
    Hermite-Simpson collocation defect (must equal zero at the solution).

      s_mid  = (sₖ + sₖ₊₁)/2  +  (Δt/8) · [f(sₖ,uₖ) − f(sₖ₊₁,uₖ₊₁)]
      u_mid  = (uₖ + uₖ₊₁)/2
      defect = sₖ₊₁ − sₖ − (Δt/6) · [fₖ + 4·f_mid + fₖ₊₁]
    """
    fk   = f(sk,    uk)
    fk1  = f(sk1,   uk1)
    smid = (sk + sk1) / 2.0 + (h_step / 8.0) * (fk - fk1)
    umid = (uk + uk1) / 2.0
    fmid = f(smid,  umid)
    return sk1 - sk - (h_step / 6.0) * (fk + 4.0 * fmid + fk1)


# ── Warm start ────────────────────────────────────────────────────────────────

def _warm_start(opti, S1, U1, S2, U2, T_f_var,
                params, t_burn, N1, N2, iacpn_ref):
    """
    Initialise NLP variables by interpolating the IACPN reference trajectory
    onto the collocation nodes.  Falls back to linear guess on any error.
    """
    try:
        ser   = iacpn_ref['series']
        t_sim = np.array(ser['t'], dtype=float)
        v_sim = np.array(ser['v'], dtype=float)
        gV_sim = np.radians(np.array(ser['gamma_v'], dtype=float))
        gH_sim = np.radians(np.array(ser['gamma_h'], dtype=float))
        h_sim  = np.array(ser['h'],  dtype=float) * 1_000.0   # km → m
        x_sim  = np.array(ser['x'],  dtype=float) * 1_000.0
        y_sim  = np.array(ser['y'],  dtype=float) * 1_000.0
        anV_sim = np.array(ser['a_nV'], dtype=float) * G0     # g → m/s²
        anH_sim = np.array(ser['a_nH'], dtype=float) * G0

        thrust_n = params.thrust_kn * 1_000.0
        mdot     = thrust_n / (params.isp * G0)
        m_tot    = params.m_pay + params.m_prop + params.m_str
        prop_used = min(params.m_prop, mdot * params.burn_max)
        m_bo     = params.m_pay + params.m_str + (params.m_prop - prop_used)

        T_f_sim = float(iacpn_ref['summary']['flight_time_s'])
        opti.set_initial(T_f_var, T_f_sim)

        def ip(arr, t_q):
            return float(np.interp(t_q, t_sim, arr))

        t1_nodes = np.linspace(0.0,    t_burn,  N1 + 1)
        t2_nodes = np.linspace(t_burn, T_f_sim, N2 + 1)

        for k, tk in enumerate(t1_nodes):
            alpha = tk / max(t_burn, 1e-9)
            opti.set_initial(S1[0, k], ip(v_sim,  tk))
            opti.set_initial(S1[1, k], ip(gV_sim, tk))
            opti.set_initial(S1[2, k], ip(gH_sim, tk))
            opti.set_initial(S1[3, k], ip(h_sim,  tk))
            opti.set_initial(S1[4, k], ip(x_sim,  tk))
            opti.set_initial(S1[5, k], ip(y_sim,  tk))
            opti.set_initial(S1[6, k], m_tot - (m_tot - m_bo) * alpha)
            opti.set_initial(U1[0, k], ip(anV_sim, tk))
            opti.set_initial(U1[1, k], ip(anH_sim, tk))

        for k, tk in enumerate(t2_nodes):
            opti.set_initial(S2[0, k], ip(v_sim,  tk))
            opti.set_initial(S2[1, k], ip(gV_sim, tk))
            opti.set_initial(S2[2, k], ip(gH_sim, tk))
            opti.set_initial(S2[3, k], ip(h_sim,  tk))
            opti.set_initial(S2[4, k], ip(x_sim,  tk))
            opti.set_initial(S2[5, k], ip(y_sim,  tk))
            opti.set_initial(U2[0, k], ip(anV_sim, tk))
            opti.set_initial(U2[1, k], ip(anH_sim, tk))

    except Exception:
        _linear_warm_start(opti, S1, U1, S2, U2, T_f_var,
                           params, t_burn, N1, N2)


def _linear_warm_start(opti, S1, U1, S2, U2, T_f_var,
                       params, t_burn, N1, N2):
    """Fallback: straight-line state interpolation, zero control."""
    thrust_n  = params.thrust_kn * 1_000.0
    mdot      = thrust_n / (params.isp * G0)
    m_tot     = params.m_pay + params.m_prop + params.m_str
    prop_used = min(params.m_prop, mdot * params.burn_max)
    m_bo      = params.m_pay + params.m_str + (params.m_prop - prop_used)
    gV0       = math.radians(params.launch_angle)
    gH0       = math.radians(params.launch_azimuth)
    x_t       = params.x_target * 1_000.0
    y_t       = params.y_target * 1_000.0
    z_t       = params.z_target * 1_000.0

    v_bo_est  = thrust_n / m_tot * t_burn * 0.5
    h_bo_est  = max(v_bo_est * math.sin(gV0) * t_burn * 0.25, 500.0)
    tf_est    = t_burn + max(2.0 * h_bo_est / max(v_bo_est * abs(math.sin(gV0)), 1.0), 20.0)
    opti.set_initial(T_f_var, tf_est)

    for k in range(N1 + 1):
        a = k / N1
        opti.set_initial(S1[0, k], a * v_bo_est)
        opti.set_initial(S1[1, k], gV0 * (1 - 0.4 * a))
        opti.set_initial(S1[2, k], gH0)
        opti.set_initial(S1[3, k], a * h_bo_est)
        opti.set_initial(S1[4, k], a * x_t * 0.3)
        opti.set_initial(S1[5, k], a * y_t * 0.3)
        opti.set_initial(S1[6, k], m_tot - (m_tot - m_bo) * a)
        opti.set_initial(U1[0, k], 0.0)
        opti.set_initial(U1[1, k], 0.0)

    h_max_est = h_bo_est * 2.5
    for k in range(N2 + 1):
        a = k / N2
        arc = 4 * a * (1 - a)    # parabolic altitude arc guess
        opti.set_initial(S2[0, k], v_bo_est * max(1 - a, 0.05))
        opti.set_initial(S2[1, k], gV0 - (gV0 - math.radians(-75)) * a)
        opti.set_initial(S2[2, k], gH0)
        opti.set_initial(S2[3, k],
                         h_bo_est + (h_max_est - h_bo_est) * arc
                         + (z_t - h_bo_est) * a)
        opti.set_initial(S2[4, k], x_t * (0.3 + 0.7 * a))
        opti.set_initial(S2[5, k], y_t * (0.3 + 0.7 * a))
        opti.set_initial(U2[0, k], 0.0)
        opti.set_initial(U2[1, k], 0.0)


# ── Main solver ───────────────────────────────────────────────────────────────

def collocation_solve(
    params:              Params,
    N1:                  int        = 15,
    N2:                  int        = 30,
    tf_guess:            float|None = None,
    # ── Hit-line merit (active when both hit_gamma_v and hit_gamma_h are set) ──
    angle_tol_deg:          float      = 5.0,
    hit_line_range_max:     float      = 50.0,   # km — max |s| along the line
    w_ctrl:                 float      = 1.0,    # weight on ctrl_effort
    w_time:                 float      = 1e-3,
    # ── FinalLine (hit-line mode only) ───────────────────────────────────────
    w_final_line:           float      = 0.1,    # weight on quadratic approach-angle penalty
    final_line_tol_deg:     float      = 3.0,    # angle tolerance for FinalLine post-processing (deg)
    final_line_nodes:       int        = 10,     # how many terminal coast nodes are penalized
    # ── Point-target merit (fallback when angle constraints absent) ────────────
    w_miss:              float      = 1.0,
    w_angle_v:           float      = 0.01,
    w_angle_h:           float      = 0.01,
    # ── Soft miss penalty (both modes) ───────────────────────────────────────
    w_miss_soft:         float      = 0.0,    # 0 = disabled (backward compat)
    miss_deadzone:       float      = 100.0,  # m — no penalty below this miss
    # ── Solver options ────────────────────────────────────────────────────────
    warm_start:          str        = 'iacpn',
    ipopt_opts:          dict|None  = None,
) -> dict:
    """
    Solve the booster trajectory OCP via Hermite-Simpson direct collocation.

    Parameters
    ----------
    params             : Params — vehicle and target specification.
    N1                 : Hermite-Simpson segments in powered phase (default 15).
    N2                 : Hermite-Simpson segments in coast phase   (default 30).
    tf_guess           : Initial guess for total flight time (s). Estimated if None.
    angle_tol_deg      : Angle tolerance for hit-line mode (deg). Default 5.
    hit_line_range_max : Max signed offset |s| along the hit line (km). Default 50.
    w_ctrl             : Weight on ctrl_effort (softplus mean % of limiter).
    w_time             : Weight on total flight time.
    w_final_line       : Enable FinalLine (hit-line mode). Any value > 0 activates
                         hard angle constraints on the last final_line_nodes coast
                         nodes. 0 = disabled (only terminal node constrained).
    final_line_tol_deg : Angle tolerance for the final approach segment (deg).
                         Both γV and γH must stay within this. Default 3°.
    final_line_nodes   : Number of terminal coast-phase nodes that must satisfy
                         the final-approach angle. Default 5.
    w_miss             : (point-target fallback) Weight on miss².
    w_angle_v          : (point-target fallback) Weight on γV error².
    w_angle_h          : (point-target fallback) Weight on γH error².
    w_miss_soft        : Weight on softplus miss penalty (both modes). 0 = disabled.
    miss_deadzone      : Dead-zone radius (m). No penalty for miss < miss_deadzone.
    warm_start         : 'iacpn' — interpolate IACPN reference run onto nodes.
                         'linear' — simple straight-line interpolation.
    ipopt_opts         : Extra IPOPT options dict (overrides defaults).

    Returns
    -------
    dict with keys:
        'summary'   — scalar results (miss_m, flight_time_s, solved, …)
        'schedule'  — per-phase knot times and commands (g units)
        'series'    — combined state/control time-series for plotting
        'iacpn'     — IACPN reference summary (for comparison)

    Raises
    ------
    ImportError if CasADi is not installed.
    RuntimeError if IPOPT fails and even the debug iterate is unavailable.
    """
    # ── Fixed scalars ──────────────────────────────────────────────────────
    thrust_n  = params.thrust_kn * 1_000.0
    x_t       = params.x_target  * 1_000.0
    y_t       = params.y_target  * 1_000.0
    z_t       = params.z_target  * 1_000.0
    a_max     = params.a_lat_max * G0

    # ── Velocity-dependent a_max ──────────────────────────────────────────
    # Build a CasADi interpolant if a_lat_max_table is provided.
    # _a_max_fn(v) returns a_max in m/s² — works with both float and CasADi MX.
    if params.a_lat_max_table is not None:
        _v_bp = [float(r[0]) for r in params.a_lat_max_table]
        _a_bp = [float(r[1]) * G0 for r in params.a_lat_max_table]
        _interp = ca.interpolant('a_max_v', 'linear', [_v_bp], _a_bp)
        def _a_max_fn(v_sym): return _interp(v_sym)
        a_max_ref = float(max(_a_bp))   # peak value — for display / fallback only
    else:
        def _a_max_fn(v_sym): return a_max
        a_max_ref = a_max
    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)
    t_burn    = prop_used / mdot
    mass_0    = params.m_pay + params.m_prop + params.m_str
    mass_bo   = params.m_pay + params.m_str + (params.m_prop - prop_used)
    gV0       = math.radians(params.launch_angle)
    gH0       = math.radians(params.launch_azimuth)

    # Powered-phase segment duration (fixed scalar — t_burn is not a variable)
    h1 = t_burn / N1

    # ── IACPN reference run (no-angle, for warm start) ────────────────────
    # Angle-constrained IACPN fights itself and produces a chaotic trajectory
    # that makes a bad warm start.  Always use unconstrained IACPN for the
    # initial guess, then IPOPT steers from there to the angle solution.
    from dataclasses import replace as _dc_replace
    _p_ws = _dc_replace(params, hit_gamma_v=None, hit_gamma_h=None)
    iacpn_ref = _sim_iacpn(_p_ws)
    if tf_guess is None:
        tf_guess = iacpn_ref['summary']['flight_time_s'] * 1.1

    # ── CasADi Opti framework ─────────────────────────────────────────────
    opti = ca.Opti()

    # Free variable: total flight time
    T_f = opti.variable()
    opti.set_initial(T_f, float(tf_guess))
    opti.subject_to(T_f >= t_burn + 2.0)
    opti.subject_to(T_f <= 7_200.0)

    # Coast-phase segment duration (symbolic: T_f is free)
    h2 = (T_f - t_burn) / N2

    # State and control nodes
    S1 = opti.variable(7, N1 + 1)   # powered: [v, γV, γH, h, x, y, mass]
    U1 = opti.variable(2, N1 + 1)   # powered controls: [a_nV, a_nH]  (m/s²)
    S2 = opti.variable(6, N2 + 1)   # coast:   [v, γV, γH, h, x, y]
    U2 = opti.variable(2, N2 + 1)   # coast controls: [a_nV, a_nH]    (m/s²)

    # ── Initial conditions ────────────────────────────────────────────────
    opti.subject_to(S1[0, 0] == 0.0)      # v(0) = 0
    opti.subject_to(S1[1, 0] == gV0)      # γV(0)
    opti.subject_to(S1[2, 0] == gH0)      # γH(0)
    opti.subject_to(S1[3, 0] == 0.0)      # h(0) = 0
    opti.subject_to(S1[4, 0] == 0.0)      # x(0) = 0
    opti.subject_to(S1[5, 0] == 0.0)      # y(0) = 0
    opti.subject_to(S1[6, 0] == mass_0)   # mass(0)

    # ── Phase continuity at burnout ───────────────────────────────────────
    for i in range(6):                     # [v, γV, γH, h, x, y] match
        opti.subject_to(S2[i, 0] == S1[i, N1])

    # ── Path constraints ──────────────────────────────────────────────────
    # Control limits: circular bound  |u| ≤ a_max(v_k) at each node
    for k in range(N1 + 1):
        a_k = _a_max_fn(S1[0, k])
        opti.subject_to(U1[0, k]**2 + U1[1, k]**2 <= a_k**2)
    for k in range(N2 + 1):
        a_k = _a_max_fn(S2[0, k])
        opti.subject_to(U2[0, k]**2 + U2[1, k]**2 <= a_k**2)

    # Altitude ≥ 0 at nodes
    for k in range(N1 + 1):
        opti.subject_to(S1[3, k] >= 0.0)
    for k in range(N2 + 1):
        opti.subject_to(S2[3, k] >= 0.0)

    # Speed ≥ 0 at nodes
    for k in range(N1 + 1):
        opti.subject_to(S1[0, k] >= 0.0)
    for k in range(N2 + 1):
        opti.subject_to(S2[0, k] >= 0.0)

    # Flight-path angle bounded to physical range — prevents IPOPT from
    # driving γV to ±1000° (numerically valid via sin/cos but meaningless).
    _gV_lim = math.pi / 2 - 1e-3
    for k in range(N1 + 1):
        opti.subject_to(S1[1, k] >= -_gV_lim)
        opti.subject_to(S1[1, k] <=  _gV_lim)
    for k in range(N2 + 1):
        opti.subject_to(S2[1, k] >= -_gV_lim)
        opti.subject_to(S2[1, k] <=  _gV_lim)

    # Heading angle bounded to prevent indefinite wrapping
    _gH0  = math.radians(params.launch_azimuth)
    _gH_w = math.pi + 0.5    # ±(180° + 30°) around launch azimuth
    for k in range(N1 + 1):
        opti.subject_to(S1[2, k] >= _gH0 - _gH_w)
        opti.subject_to(S1[2, k] <= _gH0 + _gH_w)
    for k in range(N2 + 1):
        opti.subject_to(S2[2, k] >= _gH0 - _gH_w)
        opti.subject_to(S2[2, k] <= _gH0 + _gH_w)

    # ── Dynamics constraints: Hermite-Simpson defects ─────────────────────
    def f1(s, u):
        return _f_powered(s, u, params, thrust_n, mdot, a_max_fn=_a_max_fn)

    def f2(s, u):
        return _f_coast(s, u, params, mass_bo, a_max_fn=_a_max_fn)

    for k in range(N1):
        d = _hs_defect(f1, S1[:, k], S1[:, k + 1], U1[:, k], U1[:, k + 1], h1)
        opti.subject_to(d == 0)

    for k in range(N2):
        d = _hs_defect(f2, S2[:, k], S2[:, k + 1], U2[:, k], U2[:, k + 1], h2)
        opti.subject_to(d == 0)

    # ── Terminal state references ──────────────────────────────────────────
    x_f, y_f, h_f = S2[4, N2], S2[5, N2], S2[3, N2]
    gV_f, gH_f    = S2[1, N2], S2[2, N2]

    # ── Softplus: f(x) = log(1 + exp(x)) ────────────────────────────────
    # Standard mathematical softplus, numerically stable for large x.
    # Direct formula log(1+exp(x)) overflows for x > ~710 in float64.
    # Equivalent stable form: clamp the exp input to 50, add the linear
    # tail for x > 50.  Result is identical to log(1+exp(x)) everywhere.
    def _softplus(x):
        return ca.log(1.0 + ca.exp(ca.fmin(x, 50.0))) + ca.fmax(x - 50.0, 0.0)

    # ── Control effort: mean |u_k|/a_max(v_k) across all nodes (%) ──────
    # pct_k = |u_k| / a_max(v_k) * 100  — how far each node is from its limit
    # pct_mean = mean(pct_k) over all N1+1 + N2+1 nodes
    #
    # Penalty shape (two-term softplus):
    #   pct < 50 % : ≈ 0          (dead zone)
    #   pct = 80 % : ≈ 100        (calibrated)
    #   pct = 95 % : ≈ 900
    #   pct = 100% : ≈ 1 668      (very large)
    #
    # Term 1 — gradual softplus from 50%:
    #   a1 * (softplus(pct - 50) - log2)   calibrated so term1(80) = 100
    # Term 2 — steep softplus near 100%:
    #   a2 * softplus(β2 * (pct - 90))     essentially 0 below ~85%, steep above
    _log2 = math.log(2.0)
    _a1   = 100.0 / (math.log(1.0 + math.exp(30.0)) - _log2)   # ≈ 3.412
    _a2   = 100.0
    _beta2 = 1.5

    def _ctrl_merit(pct):
        """Softplus penalty on mean control effort percentage (0–100)."""
        term1 = _a1  * (_softplus(pct - 50.0) - _log2)
        term2 = _a2  *  _softplus(_beta2 * (pct - 90.0))
        return term1 + term2

    # Accumulate pct_k = |u_k| / a_max_k * 100 at every node
    _n_nodes = (N1 + 1) + (N2 + 1)
    _pct_sum = ca.MX(0)
    for k in range(N1 + 1):
        u_mag = ca.sqrt(U1[0, k]**2 + U1[1, k]**2 + 1e-8)
        ak    = ca.fmax(_a_max_fn(S1[0, k]), 1.0)
        _pct_sum = _pct_sum + u_mag / ak * 100.0
    for k in range(N2 + 1):
        u_mag = ca.sqrt(U2[0, k]**2 + U2[1, k]**2 + 1e-8)
        ak    = ca.fmax(_a_max_fn(S2[0, k]), 1.0)
        _pct_sum = _pct_sum + u_mag / ak * 100.0

    pct_mean    = _pct_sum / _n_nodes
    ctrl_effort = _ctrl_merit(pct_mean)

    # miss_merit uses softplus shifted so it is exactly 0 at the dead-zone
    # boundary: softplus(0) = log(2), so softplus(x) - log(2) = 0 at x = 0.
    #
    # miss_merit(miss) = scale * (softplus(miss - deadzone) - log(2))
    #   = 0      at miss = miss_deadzone            (exact)
    #   ≈ 0      for miss < miss_deadzone            (dead zone)
    #   ≈ 50     at miss = miss_deadzone + 400 m
    #   grows linearly beyond that
    _sp_scale = 50.0 / (400.0 + math.log(1.0 + math.exp(-400.0)))  # ≈ 0.125

    def _miss_merit(miss_m):
        return _sp_scale * (_softplus(miss_m - miss_deadzone) - math.log(2.0))

    # ── Hit-line mode vs point-target fallback ────────────────────────────
    hit_line_mode = (params.hit_gamma_v is not None and
                     params.hit_gamma_h is not None)

    if hit_line_mode:
        # ── HIT-LINE formulation ───────────────────────────────────────────
        # Direction unit vector along the desired approach line
        gV_des = math.radians(params.hit_gamma_v)
        gH_des = math.radians(params.hit_gamma_h)
        d_x = math.cos(gV_des) * math.cos(gH_des)
        d_y = math.cos(gV_des) * math.sin(gH_des)
        d_z = math.sin(gV_des)

        angle_tol_rad        = math.radians(angle_tol_deg)
        hit_line_range_max_m = hit_line_range_max * 1_000.0

        # s: signed offset along the hit line (m) — free NLP variable
        s = opti.variable()
        opti.set_initial(s, 0.0)
        opti.subject_to(s >= -hit_line_range_max_m)
        opti.subject_to(s <=  hit_line_range_max_m)

        # Terminal position must lie exactly on the hit line
        opti.subject_to(x_f == x_t + s * d_x)
        opti.subject_to(y_f == y_t + s * d_y)
        opti.subject_to(h_f == z_t + s * d_z)

        # Hard angle tolerance constraints
        opti.subject_to(gV_f - gV_des <=  angle_tol_rad)
        opti.subject_to(gV_des - gV_f <=  angle_tol_rad)
        # Heading: cos(Δγ_H) ≥ cos(tol) handles the ±180° wrap cleanly
        opti.subject_to(ca.cos(gH_f - gH_des) >= math.cos(angle_tol_rad))

        # ── FinalLine: quadratic angle penalty over last final_line_nodes ───────
        # Adds a terminal-weighted quadratic penalty for angle deviation over
        # the last `final_line_nodes` coast-phase nodes.  This drives the optimizer
        # to maintain the correct approach angle for a longer final segment.
        #
        # Formulation:
        #   err_gV_k  = gV_k − gV_des                          (rad)
        #   err_gH_k  = 1 − cos(gH_k − gH_des)                (cosine-space, wrap-safe)
        #   weight_k  = exp(0.2·(k − N2))                      (1 at terminal, decays back)
        #   penalty   = Σ weight_k·(err_gV_k² + err_gH_k²) / tol²
        #
        # Quadratic gradients are well-conditioned: near the solution err→0 so
        # gradient→0 naturally, giving IPOPT clean KKT conditions.
        # The tol² normalization makes the penalty ≈ 1 when err = final_line_tol.
        if w_final_line > 0 and final_line_nodes > 0:
            _tol_fl    = math.radians(final_line_tol_deg)
            _k_start   = max(0, N2 - final_line_nodes)
            _angle_pen = ca.MX(0.0)
            for _k in range(_k_start, N2 + 1):
                _err_gV = S2[1, _k] - gV_des
                _err_gH = 1.0 - ca.cos(S2[2, _k] - gH_des)   # cosine-space
                _wt     = math.exp(0.2 * (_k - N2))           # terminal-weighted
                _angle_pen = _angle_pen + _wt * (_err_gV**2 + _err_gH**2)
            _angle_pen = _angle_pen / (_tol_fl ** 2)           # normalize by tol²

        # Objective: minimise control effort + final-approach angle error + flight time
        J = w_ctrl * ctrl_effort + w_time * T_f
        if w_final_line > 0 and final_line_nodes > 0:
            J = J + w_final_line * _angle_pen
        if w_miss_soft > 0:
            # In hit-line mode, miss = |s| (signed offset along approach line)
            J = J + w_miss_soft * _miss_merit(ca.fabs(s))

    else:
        # ── POINT-TARGET formulation (backward compatible) ─────────────────
        miss_sq = (x_f - x_t)**2 + (y_f - y_t)**2 + (h_f - z_t)**2
        J = w_miss * miss_sq + w_time * T_f
        if w_miss_soft > 0:
            miss_m = ca.sqrt(miss_sq + 1e-4)   # sqrt(ε) avoids gradient singularity at 0
            J = J + w_miss_soft * _miss_merit(miss_m)

        if params.hit_gamma_v is not None:
            gV_des     = math.radians(params.hit_gamma_v)
            gV_err_deg = (gV_f - gV_des) * (180.0 / math.pi)
            J += w_angle_v * gV_err_deg**2

        if params.hit_gamma_h is not None:
            gH_des     = math.radians(params.hit_gamma_h)
            gH_err_deg = (gH_f - gH_des) * (180.0 / math.pi)
            J += w_angle_h * gH_err_deg**2

        s = None   # no hit-line offset in point-target mode

    opti.minimize(J)

    # ── Warm start ────────────────────────────────────────────────────────
    if warm_start == 'iacpn':
        _warm_start(opti, S1, U1, S2, U2, T_f,
                    params, t_burn, N1, N2, iacpn_ref)
    else:
        _linear_warm_start(opti, S1, U1, S2, U2, T_f,
                           params, t_burn, N1, N2)

    # ── IPOPT options ──────────────────────────────────────────────────────
    opts: dict = {
        'ipopt.print_level':     3,
        'ipopt.max_iter':        600,
        'ipopt.tol':             1e-6,
        'ipopt.acceptable_tol':  1e-4,
        'ipopt.acceptable_iter': 10,
        'ipopt.mu_strategy':     'adaptive',
        'ipopt.nlp_scaling_method': 'gradient-based',
        'print_time':            False,
    }
    if ipopt_opts:
        opts.update(ipopt_opts)
    opti.solver('ipopt', opts)

    # ── Solve ─────────────────────────────────────────────────────────────
    try:
        sol    = opti.solve()
        solved = True
    except RuntimeError:
        # Return best iterate found so far even if IPOPT did not converge
        sol    = opti.debug
        solved = False

    # ── Extract solution ───────────────────────────────────────────────────
    S1_s   = np.array(sol.value(S1), dtype=float)   # 7 × (N1+1)
    U1_s   = np.array(sol.value(U1), dtype=float)   # 2 × (N1+1)
    S2_s   = np.array(sol.value(S2), dtype=float)   # 6 × (N2+1)
    U2_s   = np.array(sol.value(U2), dtype=float)   # 2 × (N2+1)
    T_f_s  = float(sol.value(T_f))

    t1_arr = np.linspace(0.0,    t_burn, N1 + 1)
    t2_arr = np.linspace(t_burn, T_f_s,  N2 + 1)

    # Combined series (drop duplicate burnout point from Phase 2)
    t_all = np.concatenate([t1_arr, t2_arr[1:]])
    S_all = np.hstack([S1_s[:6, :], S2_s[:, 1:]])   # [v,γV,γH,h,x,y] only
    U_all = np.hstack([U1_s,        U2_s[:, 1:]])

    x_fv  = float(S2_s[4, -1])
    y_fv  = float(S2_s[5, -1])
    h_fv  = float(S2_s[3, -1])
    gVfv  = float(S2_s[1, -1])
    gHfv  = float(S2_s[2, -1])
    miss_v = math.sqrt((x_fv - x_t)**2 + (y_fv - y_t)**2 + (h_fv - z_t)**2)

    # Hit-line offset s and angle errors
    if hit_line_mode:
        s_val = float(sol.value(s))
        gV_des_out = math.radians(params.hit_gamma_v)
        gH_des_out = math.radians(params.hit_gamma_h)
        gV_err_out = math.degrees(gVfv - gV_des_out)
        dgh = (gHfv - gH_des_out + math.pi) % (2 * math.pi) - math.pi
        gH_err_out = math.degrees(dgh)
        angle_ok = (abs(gV_err_out) <= angle_tol_deg and
                    abs(gH_err_out) <= angle_tol_deg)
    else:
        s_val = None
        gV_err_out = (math.degrees(gVfv) - params.hit_gamma_v
                      if params.hit_gamma_v is not None else None)
        gH_err_out = (math.degrees(gHfv) - params.hit_gamma_h
                      if params.hit_gamma_h is not None else None)
        angle_ok = None

    n_vars = 7*(N1+1) + 2*(N1+1) + 6*(N2+1) + 2*(N2+1) + 1 + (1 if hit_line_mode else 0)
    n_ceq  = 7*N1 + 6*N2 + 7 + 6   # HS defects + ICs + continuity

    summary: dict = {
        'method':            'direct_collocation_HS',
        'mode':              'hit_line' if hit_line_mode else 'point_target',
        'solved':            solved,
        'miss_m':            round(miss_v, 2),
        'flight_time_s':     round(T_f_s, 2),
        'impact_x_km':       round(x_fv / 1_000, 3),
        'impact_y_km':       round(y_fv / 1_000, 3),
        'impact_h_km':       round(h_fv / 1_000, 3),
        'final_gamma_v_deg': round(math.degrees(gVfv), 2),
        'final_gamma_h_deg': round(math.degrees(gHfv), 2),
        'burnout_time_s':    round(t_burn, 2),
        'N1_segments':       N1,
        'N2_segments':       N2,
        'n_variables':       n_vars,
        'n_eq_constraints':  n_ceq,
    }
    if hit_line_mode:
        summary['hit_line_offset_m']  = round(s_val, 2)
        summary['hit_line_offset_km'] = round(s_val / 1_000, 4)
        summary['angle_tol_deg']      = angle_tol_deg
        summary['gamma_v_err_deg']    = round(gV_err_out, 3)
        summary['gamma_h_err_deg']    = round(gH_err_out, 3)
        summary['angle_satisfied']    = angle_ok

        # ── FinalLine (exact, hard threshold on solved trajectory) ──────────
        # Count contiguous nodes from terminal backward where both angles in tol.
        gV_des_fl  = math.radians(params.hit_gamma_v)
        gH_des_fl  = math.radians(params.hit_gamma_h)
        tol_fl_rad = math.radians(final_line_tol_deg)
        dt_coast   = (T_f_s - t_burn) / N2
        fl_m = 0.0
        for _k in range(N2, 0, -1):
            _gv = float(S2_s[1, _k])
            _gh = float(S2_s[2, _k])
            _dh = (_gh - gH_des_fl + math.pi) % (2 * math.pi) - math.pi
            if abs(_gv - gV_des_fl) <= tol_fl_rad and abs(_dh) <= tol_fl_rad:
                fl_m += float(S2_s[0, _k]) * dt_coast
            else:
                break   # contiguous from terminal — stop at first out-of-tol node
        summary['final_line_km']     = round(fl_m / 1_000, 3)
        summary['final_line_tol_deg'] = final_line_tol_deg

    elif params.hit_gamma_v is not None:
        summary['gamma_v_err_deg']    = round(gV_err_out, 3)
    elif params.hit_gamma_h is not None:
        summary['gamma_h_err_deg']    = round(gH_err_out, 3)

    schedule = {
        't_powered_s':   t1_arr.tolist(),
        'anV_powered_g': (U1_s[0, :] / G0).tolist(),
        'anH_powered_g': (U1_s[1, :] / G0).tolist(),
        't_coast_s':     t2_arr.tolist(),
        'anV_coast_g':   (U2_s[0, :] / G0).tolist(),
        'anH_coast_g':   (U2_s[1, :] / G0).tolist(),
    }

    series = {
        't':           t_all.tolist(),
        'x':           (S_all[4, :] / 1_000).tolist(),
        'y':           (S_all[5, :] / 1_000).tolist(),
        'h':           (S_all[3, :] / 1_000).tolist(),
        'v':           S_all[0, :].tolist(),
        'gamma_v_deg': np.degrees(S_all[1, :]).tolist(),
        'gamma_h_deg': np.degrees(S_all[2, :]).tolist(),
        'a_nV_g':      (U_all[0, :] / G0).tolist(),
        'a_nH_g':      (U_all[1, :] / G0).tolist(),
        'aLat_g':      (np.sqrt(U_all[0, :]**2 + U_all[1, :]**2) / G0).tolist(),
    }

    return {
        'summary':  summary,
        'schedule': schedule,
        'series':   series,
        'iacpn':    iacpn_ref['summary'],
    }


# ── Plot ──────────────────────────────────────────────────────────────────────

def plot_collocated(result: dict, title: str = '') -> None:
    """
    4-panel comparison: IACPN reference (dashed) vs direct collocation (solid).

    Panels
    ------
      Top-left  : Ground track (North–East plane)
      Top-right : Altitude vs time
      Bot-left  : Pitch command a_nV  vs time
      Bot-right : Yaw command  a_nH  vs time
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    ser   = result['series']
    sched = result['schedule']
    summ  = result['summary']
    ref   = result.get('iacpn', {})

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    miss_str = f"{summ['miss_m']:.1f} m"
    iacpn_miss = ref.get('miss_distance_m', '?')
    fig.suptitle(
        (f"Direct Collocation (Hermite-Simpson)"
         f"{' — ' + title if title else ''}\n"
         f"Collocation miss = {miss_str}   "
         f"IACPN miss = {iacpn_miss} m   "
         f"t_f = {summ['flight_time_s']:.1f} s   "
         f"solved = {summ['solved']}"),
        fontsize=10,
    )

    # Ground track
    ax = axes[0, 0]
    ax.plot(ser['x'], ser['y'], 'b-', lw=2, label='Collocation')
    ax.scatter([ser['x'][-1]], [ser['y'][-1]], c='blue',  s=60, zorder=5)
    ax.scatter([0],            [0],            c='green', s=80, marker='^',
               zorder=5, label='Launch')
    x_t_km = summ['impact_x_km']   # collocation impact ≈ target
    y_t_km = summ['impact_y_km']
    ax.scatter([x_t_km], [y_t_km], c='red', marker='x', s=100,
               linewidths=2, zorder=5, label='Target')
    ax.set_xlabel('North (km)')
    ax.set_ylabel('East (km)')
    ax.set_title('Ground Track')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Altitude vs time
    ax = axes[0, 1]
    ax.plot(ser['t'], ser['h'], 'b-', lw=2, label='Collocation')
    ax.axvline(summ['burnout_time_s'], color='orange', ls='--',
               alpha=0.7, label=f"Burnout t={summ['burnout_time_s']:.1f}s")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (km)')
    ax.set_title('Altitude vs Time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Pitch command
    ax = axes[1, 0]
    t1, anV1 = sched['t_powered_s'], sched['anV_powered_g']
    t2, anV2 = sched['t_coast_s'],   sched['anV_coast_g']
    ax.step(t1, anV1, 'b-',  lw=1.5, where='post', label='Powered phase')
    ax.step(t2, anV2, 'b--', lw=1.5, where='post', label='Coast phase')
    ax.scatter(t1, anV1, c='blue',  s=20, zorder=4)
    ax.scatter(t2, anV2, c='navy', s=20, zorder=4)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('a_nV (g)')
    ax.set_title('Pitch Command a_nV')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Yaw command
    ax = axes[1, 1]
    t1, anH1 = sched['t_powered_s'], sched['anH_powered_g']
    t2, anH2 = sched['t_coast_s'],   sched['anH_coast_g']
    ax.step(t1, anH1, 'r-',  lw=1.5, where='post', label='Powered phase')
    ax.step(t2, anH2, 'r--', lw=1.5, where='post', label='Coast phase')
    ax.scatter(t1, anH1, c='red',      s=20, zorder=4)
    ax.scatter(t2, anH2, c='darkred', s=20, zorder=4)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('a_nH (g)')
    ax.set_title('Yaw Command a_nH')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()
