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
  Objective:

      J = (w_range · MeritRange
         + w_acc   · MeritAcc
         + w_miss_d· MeritMiss) · FlightTime

  Each component uses the softplus-fence density function (see DensityParams):

      density(ratio) = fenceC1 · softplus(ratio − fenceX, betaX)²
                     + fenceC2 · softplus(ratio − fenceY, betaY)⁴

  Ratios:
    MeritRange  → ratio = 1 − FinalLineLength_km / 2
                  FinalLineLength: smooth proxy for the length of the approach
                  segment where both angle conditions are met (hit-line mode).
                  Backward search from terminal starts at final_line_togo_m.
                  In point-target mode: 1 − (horizontal range / target range).
    MeritAcc    → ratio = mean of density(|u_k|/a_max_k) over all nodes
    MeritMiss   → ratio = miss_at_StartFinalLine / miss_ref_m
                  Miss is the perpendicular distance from StartFinalLine to the
                  approach line (hit-line mode) or 3-D distance to target.

  Terminal constraints for hit-line mode (hard, not in objective):
      x_f = x_t + s·d̂_x,  y_f = y_t + s·d̂_y,  h_f = z_t + s·d̂_z
      |γV_f − γV_des| ≤ angle_tol_deg
      cos(γH_f − γH_des) ≥ cos(angle_tol_deg)
      Output key: 'hit_line_offset_m' = s  (signed distance from nominal target)

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

import dataclasses
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


@dataclasses.dataclass
class DensityParams:
    """Parameters for one softplus-fence density merit component.

    density(ratio) = fenceC1 · softplus(ratio − fenceX, betaX)²
                   + fenceC2 · softplus(ratio − fenceY, betaY)⁴

    softplus(x, beta) = (1/beta) · log(1 + exp(beta·x))

    Both posts should satisfy fenceX ≤ fenceY for a smooth two-stage ramp.
    Negative thresholds are valid when the ratio can be negative (e.g. range).
    """
    fenceX:  float = 0.2    # ratio threshold for the quadratic ramp
    fenceY:  float = 0.5    # ratio threshold for the quartic ramp
    betaX:   float = 5.0    # softplus sharpness at fenceX
    betaY:   float = 10.0   # softplus sharpness at fenceY
    fenceC1: float = 1.0    # quadratic coefficient
    fenceC2: float = 0.5    # quartic coefficient


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
    onto the collocation nodes.

    Falls back to _linear_warm_start when:
      - IACPN miss > max(2 km, 50% of target range)  — trajectory too far off
      - any exception during interpolation            — data error
    """
    # Quality gate: poor IACPN trajectory is a worse starting point than linear
    try:
        iacpn_miss     = float(iacpn_ref['summary']['miss_distance_m'])
        target_range_m = math.sqrt(params.x_target**2 + params.y_target**2) * 1_000.0
        miss_threshold = max(2_000.0, 0.5 * target_range_m)
        if iacpn_miss > miss_threshold:
            _linear_warm_start(opti, S1, U1, S2, U2, T_f_var,
                               params, t_burn, N1, N2)
            return
    except Exception:
        pass   # can't read miss — proceed to interpolation attempt below

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
    angle_tol_deg:          float      = 5.0,
    hit_line_range_max:     float      = 50.0,   # km — max |s| along the line
    # ── FinalLine options (used for MeritRange ratio and miss at StartFinalLine) ─
    final_line_tol_deg:  float = 3.0,   # angle tolerance for "inside final line" (deg)
    final_line_togo_m:   float = 500.0, # backward search starts at this togo distance (m)
    # ── Density merit weights ────────────────────────────────────────────────
    w_range:           float            = 1.0,   # weight for MeritRange
    w_acc:             float            = 1.0,   # weight for MeritAcc
    w_miss_d:          float            = 1.0,   # weight for MeritMiss
    miss_ref_m:        float|None       = 500.0, # miss normalisation ref (m); None → target range
    range_dp:          DensityParams|None = None,  # Range density params (see DensityParams)
    acc_dp:            DensityParams|None = None,  # Acc density params
    miss_dp:           DensityParams|None = None,  # Miss density params
    # ── Solver options ────────────────────────────────────────────────────────
    warm_start:        str             = 'iacpn',
    ipopt_opts:        dict|None       = None,
) -> dict:
    """
    Solve the booster trajectory OCP via Hermite-Simpson direct collocation.

    Parameters
    ----------
    params             : Params — vehicle and target specification.
    N1                 : Hermite-Simpson segments in powered phase (default 15).
    N2                 : Hermite-Simpson segments in coast phase   (default 30).
    tf_guess           : Initial guess for total flight time (s). Estimated if None.
    angle_tol_deg      : Hard angle tolerance at terminal node for hit-line mode (deg). Default 5.
    hit_line_range_max : Max signed offset |s| along the hit line (km). Default 50.

    FinalLine options (MeritRange ratio and miss at StartFinalLine):
    final_line_tol_deg : Angle tolerance for the "inside final line" condition (deg). Default 3°.
    final_line_togo_m  : Backward search starts at this distance-to-go from terminal (m). Default 500.

    Density merit weights:
    w_range            : Weight for MeritRange. Default 1.0.
    w_acc              : Weight for MeritAcc. Default 1.0.
    w_miss_d           : Weight for MeritMiss. Default 1.0.
    miss_ref_m         : Reference miss distance for normalisation (m).
                         None → use horizontal target range sqrt(x_t²+y_t²).
    range_dp           : DensityParams for MeritRange. Defaults if None.
    acc_dp             : DensityParams for MeritAcc. Defaults if None.
    miss_dp            : DensityParams for MeritMiss. Defaults if None.

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

    _n_nodes = (N1 + 1) + (N2 + 1)

    # ── Terminal constraints (mode-specific) ─────────────────────────────────
    hit_line_mode = (params.hit_gamma_v is not None and
                     params.hit_gamma_h is not None)

    if hit_line_mode:
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

        # Terminal position constrained to approach line
        opti.subject_to(x_f == x_t + s * d_x)
        opti.subject_to(y_f == y_t + s * d_y)
        opti.subject_to(h_f == z_t + s * d_z)

        # Hard angle tolerance at terminal node
        opti.subject_to(gV_f - gV_des <=  angle_tol_rad)
        opti.subject_to(gV_des - gV_f <=  angle_tol_rad)
        opti.subject_to(ca.cos(gH_f - gH_des) >= math.cos(angle_tol_rad))
    else:
        s = None

    # ── FinalLine length and miss at StartFinalLine ───────────────────────────
    # Algorithm (hit-line mode):
    #   1. Per node k: ratio_angle_k = |gV_k − gV_des|  and  (1−cos(gH_k−gH_des))
    #      inside_k = σ(β·(tol_gV − ratio_gV_k)) · σ(β·(tol_cos − ratio_gH_k))
    #   2. togo_k = approximate distance-to-terminal at node k  (m)
    #      in_window_k = σ(α·(togo_k − final_line_togo_m))   ≈ 1 when togo > togo_m
    #   3. Cumulative product from terminal going backward:
    #      cum[N2] = 1;  cum[k] = cum[k+1]·(1 − in_window[k]·(1−inside[k]))
    #      (nodes closer than final_line_togo_m are treated as always satisfied)
    #   4. FL_smooth_km = Σ_k cum[k]·v_k·h2 / 1000
    #   5. StartFinalLine position: soft-weighted node at the inside→outside transition
    #      miss_m_sq = perp distance² from SFL to approach line
    _tol_fl     = math.radians(final_line_tol_deg)
    _tol_fl_cos = 1.0 - math.cos(_tol_fl)
    _beta_fl    = 10.0   # sigmoid sharpness for angle condition (rad⁻¹)
    _alpha_win  = 0.02   # sigmoid sharpness for togo window (m⁻¹)

    def _sig(x):
        return 1.0 / (1.0 + ca.exp(-x))

    if hit_line_mode:
        # Step 1 — per-node angle condition (product of two sigmoids)
        _inside = []
        for _k in range(N2 + 1):
            _sc_v = _sig(_beta_fl * (_tol_fl     - ca.fabs(S2[1, _k] - gV_des)))
            _sc_h = _sig(_beta_fl * (_tol_fl_cos - (1.0 - ca.cos(S2[2, _k] - gH_des))))
            _inside.append(_sc_v * _sc_h)

        # Step 2 — togo_k (m) and window indicator
        _togo = [None] * (N2 + 1)
        _togo[N2] = ca.MX(0.0)
        for _k in range(N2 - 1, -1, -1):
            _togo[_k] = _togo[_k + 1] + S2[0, _k] * h2   # v_k · h2
        _in_win = [_sig(_alpha_win * (_togo[_k] - final_line_togo_m))
                   for _k in range(N2 + 1)]

        # Step 3 — cumulative product from terminal (backward)
        _cum = [None] * (N2 + 1)
        _cum[N2] = ca.MX(1.0)
        for _k in range(N2 - 1, -1, -1):
            _cum[_k] = _cum[_k + 1] * (1.0 - _in_win[_k] * (1.0 - _inside[_k]))

        # Step 4 — FL_smooth_km: path-length integral weighted by cumulative inside
        _fl_smooth_km = ca.MX(0.0)
        for _k in range(N2 + 1):
            _fl_smooth_km = _fl_smooth_km + _cum[_k] * S2[0, _k] * h2 / 1_000.0

        # Step 5 — Miss at StartFinalLine: weighted sum of per-node perpendicular distances²
        # transition_weight_k = cum[k+1] · in_window[k] · (1 − inside[k])
        #   peaks at the node where angles first fail going backward from terminal.
        #   When the whole trajectory is inside: all weights ≈ 0 → miss_m_sq ≈ 0 (correct).
        #   No position average / no division → no degenerate-case gradient issues.
        miss_m_sq = ca.MX(0.0)
        for _k in range(N2):   # terminal node has no "transition from outside"
            _w    = _cum[_k + 1] * _in_win[_k] * (1.0 - _inside[_k])
            _dx_k = S2[4, _k] - x_t
            _dy_k = S2[5, _k] - y_t
            _dh_k = S2[3, _k] - z_t
            _proj_k   = _dx_k * d_x + _dy_k * d_y + _dh_k * d_z
            _perp_sq_k = ((_dx_k - _proj_k*d_x)**2
                        + (_dy_k - _proj_k*d_y)**2
                        + (_dh_k - _proj_k*d_z)**2)
            miss_m_sq = miss_m_sq + _w * _perp_sq_k
    else:
        _fl_smooth_km = ca.MX(0.0)
        miss_m_sq     = (x_f - x_t)**2 + (y_f - y_t)**2 + (h_f - z_t)**2

    # ── Softplus-fence density merit ──────────────────────────────────────────
    # Parameterised softplus: (1/beta)·log(1+exp(beta·x)), numerically stable.
    # Matches MATLAB softplus(x,beta) in softplus.txt — no branching needed.
    def _sp_beta(x, beta):
        bx = beta * x
        return ca.log(1.0 + ca.exp(-ca.fabs(bx))) / beta + ca.fmax(x, 0.0)

    def _fence_density(ratio, dp: DensityParams):
        """fenceC1·sp(ratio−fenceX,betaX)² + fenceC2·sp(ratio−fenceY,betaY)⁴"""
        qx = _sp_beta(ratio - dp.fenceX, dp.betaX)
        qy = _sp_beta(ratio - dp.fenceY, dp.betaY)
        return dp.fenceC1 * qx**2 + dp.fenceC2 * qy**4

    # Resolve default density parameters per component
    _rdp = range_dp if range_dp is not None else DensityParams(
        fenceX=0.05, fenceY=0.2, betaX=5.0, betaY=3.0, fenceC1=1.0, fenceC2=0.5)
    _adp = acc_dp   if acc_dp   is not None else DensityParams(
        fenceX=0.5,  fenceY=0.8, betaX=5.0, betaY=3.0, fenceC1=1.0, fenceC2=2.0)
    _mdp = miss_dp  if miss_dp  is not None else DensityParams(
        fenceX=0.0,  fenceY=0.1, betaX=10.0, betaY=5.0, fenceC1=1.0, fenceC2=2.0)

    # Miss normalisation reference
    _range_ref_m = max(math.sqrt(x_t**2 + y_t**2), 1.0)
    _miss_ref_m  = float(miss_ref_m) if miss_ref_m is not None else _range_ref_m

    # ── Ratios ────────────────────────────────────────────────────────────────
    # MeritRange : ratio = 1 − FinalLineLength_km / 2
    #   In point-target mode: fallback to 1 − horizontal_range / target_range.
    if hit_line_mode:
        _ratio_range = 1.0 - _fl_smooth_km / 2.0
    else:
        _ratio_range = 1.0 - ca.sqrt(x_f**2 + y_f**2 + 1e-6) / _range_ref_m

    # MeritAcc : mean of density(ratio_k) over all collocation nodes
    # ratio_k = |u_k| / a_max_k  (normalised lateral acceleration, 0–1)
    _acc_dens_sum = ca.MX(0.0)
    for k in range(N1 + 1):
        _u_mag        = ca.sqrt(U1[0, k]**2 + U1[1, k]**2 + 1e-8)
        _ak           = ca.fmax(_a_max_fn(S1[0, k]), 1.0)
        _acc_dens_sum = _acc_dens_sum + _fence_density(_u_mag / _ak, _adp)
    for k in range(N2 + 1):
        _u_mag        = ca.sqrt(U2[0, k]**2 + U2[1, k]**2 + 1e-8)
        _ak           = ca.fmax(_a_max_fn(S2[0, k]), 1.0)
        _acc_dens_sum = _acc_dens_sum + _fence_density(_u_mag / _ak, _adp)
    _merit_acc_d = _acc_dens_sum / _n_nodes

    # MeritMiss  : normalised miss distance
    _ratio_miss = ca.sqrt(miss_m_sq + 1e-12) / _miss_ref_m

    # ── Objective ─────────────────────────────────────────────────────────────
    # J = (w_range·MeritRange + w_acc·MeritAcc + w_miss_d·MeritMiss) · FlightTime
    J = (w_range  * _fence_density(_ratio_range, _rdp)
       + w_acc    * _merit_acc_d
       + w_miss_d * _fence_density(_ratio_miss,  _mdp)) * T_f

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

    # Practical quality check — independent of IPOPT KKT certification.
    # The FinalLine sigmoid proxy can leave residual dual infeasibility at an
    # otherwise excellent primal solution; quality_ok reflects the engineering
    # result rather than the NLP solver's convergence certificate.
    quality_ok = (miss_v < 200.0 and
                  (angle_ok if angle_ok is not None else True))

    summary: dict = {
        'method':            'direct_collocation_HS',
        'mode':              'hit_line' if hit_line_mode else 'point_target',
        'solved':            solved,
        'quality_ok':        quality_ok,
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

    iacpn_ser = iacpn_ref['series']
    return {
        'summary':  summary,
        'schedule': schedule,
        'series':   series,
        'iacpn':    {
            **iacpn_ref['summary'],
            'series': {
                't': iacpn_ser['t'],
                'x': iacpn_ser['x'],
                'y': iacpn_ser['y'],
                'h': iacpn_ser['h'],
            },
        },
    }


# ── Plot ──────────────────────────────────────────────────────────────────────

def plot_collocated(result: dict, title: str = '') -> None:
    """
    6-panel comparison: IACPN reference (dashed) vs direct collocation (solid).

    Panels
    ------
      [0,0] Ground track (North–East plane)
      [0,1] Altitude vs time
      [1,0] Pitch command a_nV  vs time
      [1,1] Yaw command  a_nH  vs time
      [2,0] Flight-path angle γV vs time
      [2,1] Heading azimuth   γH vs time
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    ser   = result['series']
    sched = result['schedule']
    summ  = result['summary']
    ref   = result.get('iacpn', {})
    ref_ser = ref.get('series', {})

    fig, axes = plt.subplots(3, 2, figsize=(12, 11))
    miss_str = f"{summ['miss_m']:.1f} m"
    iacpn_miss = ref.get('miss_distance_m', '?')
    quality_str = f"  quality_ok={summ.get('quality_ok', '?')}"
    fl_str = (f"  FinalLine={summ.get('final_line_km', '?')} km"
              if 'final_line_km' in summ else '')
    fig.suptitle(
        (f"Direct Collocation (Hermite-Simpson)"
         f"{' — ' + title if title else ''}\n"
         f"Collocation miss = {miss_str}   "
         f"IACPN miss = {iacpn_miss} m   "
         f"t_f = {summ['flight_time_s']:.1f} s   "
         f"solved = {summ['solved']}{quality_str}{fl_str}"),
        fontsize=10,
    )

    # Ground track
    ax = axes[0, 0]
    if ref_ser:
        ax.plot(ref_ser['x'], ref_ser['y'], 'b--', lw=1.2, alpha=0.55,
                label=f'IACPN ref ({iacpn_miss} m miss)')
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
    if ref_ser:
        ax.plot(ref_ser['t'], ref_ser['h'], 'b--', lw=1.2, alpha=0.55,
                label='IACPN ref')
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

    # Desired angles (available in hit-line mode: err = final − desired)
    _des_gv = (summ['final_gamma_v_deg'] - summ['gamma_v_err_deg']
               if 'gamma_v_err_deg' in summ else None)
    _des_gh = (summ['final_gamma_h_deg'] - summ['gamma_h_err_deg']
               if 'gamma_h_err_deg' in summ else None)

    # Flight-path angle γV vs time
    ax = axes[2, 0]
    if ref_ser and 'gamma_v' in ref_ser:
        ax.plot(ref_ser['t'], ref_ser['gamma_v'], 'b--', lw=1.2, alpha=0.55,
                label='IACPN ref')
    ax.plot(ser['t'], ser['gamma_v_deg'], 'b-', lw=2, label='Collocation')
    ax.axvline(summ['burnout_time_s'], color='orange', ls='--', alpha=0.7,
               label=f"Burnout t={summ['burnout_time_s']:.1f}s")
    if _des_gv is not None:
        ax.axhline(_des_gv, color='red', ls=':', lw=1.2,
                   label=f'Desired γV={_des_gv:.1f}°')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('γV (deg)')
    ax.set_title('Flight-path angle γV vs Time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Heading azimuth γH vs time
    ax = axes[2, 1]
    if ref_ser and 'gamma_h' in ref_ser:
        ax.plot(ref_ser['t'], ref_ser['gamma_h'], 'r--', lw=1.2, alpha=0.55,
                label='IACPN ref')
    ax.plot(ser['t'], ser['gamma_h_deg'], 'r-', lw=2, label='Collocation')
    ax.axvline(summ['burnout_time_s'], color='orange', ls='--', alpha=0.7,
               label=f"Burnout t={summ['burnout_time_s']:.1f}s")
    if _des_gh is not None:
        ax.axhline(_des_gh, color='darkred', ls=':', lw=1.2,
                   label=f'Desired γH={_des_gh:.1f}°')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('γH (deg)')
    ax.set_title('Heading azimuth γH vs Time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()
