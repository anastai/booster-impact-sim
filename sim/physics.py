"""
Physical constants, parameter / state dataclasses, and one velocity-frame
integration step.

Integration scheme
------------------
Forward-Euler (dt = 0.2 s), matching the JavaScript reference implementation.

Velocity frame
--------------
State is tracked as (v, gamma_v, gamma_h, h, x, y) where:
  v         – total speed (m/s)
  gamma_v   – flight-path angle from horizontal (rad), positive = climbing
  gamma_h   – heading azimuth (rad), 0 = North, π/2 = East
  h, x, y   – altitude, north, east in the inertial frame (m)

The three orthogonal velocity-frame axes are:
  ê_t   – along the velocity vector
  ê_nV  – normal, in the pitch plane (perpendicular to v, pointing "above" v)
  ê_nH  – lateral / yaw  (ê_t × ê_nV, right-hand rule)

Equations of motion
-------------------
  m · (dv/dt)               = T_t  − D  − m·g·sin(γV)  + m·a_cmd_t
  m · v · (dγV/dt)          = T_nV + L  − m·g·cos(γV)  + m·a_cmd_nV + m·a_zem_nV
  m · v · cos(γV) · (dγH/dt)= T_nH               + m·a_cmd_nH + m·a_zem_nH
"""
import math
from dataclasses import dataclass

G0 = 9.80665        # standard gravity  (m/s²)
RE = 6_371_000.0    # Earth mean radius (m)


# ---------------------------------------------------------------------------
# Input parameters
# ---------------------------------------------------------------------------

@dataclass
class Params:
    """
    All user-facing simulation parameters.

    Angles in degrees, distances in km, forces in kN, accelerations in m/s²
    (except a_lat_max which is in g).  simulate() converts to SI internally.
    """
    # ── Booster ──────────────────────────────────────────────────────────
    m_pay:        float = 300.0     # payload mass                (kg)
    m_prop:       float = 5_000.0   # propellant mass             (kg)
    m_str:        float = 800.0     # booster structural dry mass (kg)
    # ── Motor ────────────────────────────────────────────────────────────
    isp:          float = 260.0     # specific impulse            (s)
    thrust_kn:    float = 120.0     # thrust                      (kN)
    burn_max:     float = 60.0      # max burn duration           (s)
    # ── Aerodynamics ─────────────────────────────────────────────────────
    cd:           float = 0.4       # drag coeff – powered ascent
    cd_fall:      float = 1.2       # drag coeff – tumbling post-burnout
    cl:           float = 0.0       # lift coefficient  (NEW — v2.0)
    cd_ctrl:      float = 0.3       # actuator drag coefficient (ZEM guidance)
    diam:         float = 1.2       # reference body diameter     (m)
    atm:          str   = 'isa'     # atmosphere: 'isa' | 'exp' | 'none'
    # ── Launch angles ────────────────────────────────────────────────────
    launch_angle:    float = 75.0   # initial flight-path angle γV₀   (deg)
    launch_azimuth:  float = 90.0   # initial heading azimuth   γH₀   (deg, 0=N, 90=E)  (NEW)
    # ── Velocity-frame dynamics ───────────────────────────────────────────
    grav_turn:       bool  = True    # True → thrust ∥ ê_t; False → fixed body angle  (NEW)
    grav_turn_v_min: float = 30.0   # speed threshold to activate gravity-turn rotation (m/s)
                                     # Below this, angle is held fixed even when grav_turn=True.
                                     # Matches real-rocket "fly straight until minimum speed" logic.
    a_cmd_t:      float = 0.0       # constant tangential    cmd  (m/s²)  (NEW)
    a_cmd_nv:     float = 0.0       # constant pitch-normal  cmd  (m/s²)  (NEW)
    a_cmd_nh:     float = 0.0       # constant yaw-normal    cmd  (m/s²)  (NEW)
    # ── Guidance target ──────────────────────────────────────────────────
    x_target:     float = 100.0     # target downrange   (km)
    y_target:     float = 0.0       # target crossrange  (km)
    z_target:     float = 0.0       # target altitude    (km)
    a_lat_max:    float = 1.0       # max lateral accel limit  (g)
    # ── Impact angle constraints (None = unconstrained) ──────────────────
    hit_gamma_v:  float | None = None  # desired impact flight-path angle (deg)
                                       # e.g. -70 = steep near-vertical dive
    hit_gamma_h:  float | None = None  # desired impact azimuth (deg)
                                       # e.g. 45 = hit from the NE
    hit_angle_range: float | None = None  # range from target at which angle law activates (km)
                                          # None = active throughout entire post-burnout flight
    # ── Simulation stop conditions ───────────────────────────────────────
    v_min_stop:   float | None = None  # stop when speed drops below this after burnout (m/s)
                                        # None = disabled
    # ── Velocity-dependent acceleration limit ────────────────────────────
    a_lat_max_table: list | None = None
    # If set: [(v1_m/s, a1_g), (v2_m/s, a2_g), ...] sorted by ascending v.
    # collocate.py interpolates a_max at each node's speed.
    # Overrides a_lat_max in the optimizer; a_lat_max still used by simulate().


# ---------------------------------------------------------------------------
# Simulation state  —  velocity-frame primary quantities
# ---------------------------------------------------------------------------

@dataclass
class State:
    """
    Booster state at one instant.

    Primary integration variables are in the velocity frame (v, gamma_v,
    gamma_h).  Inertial velocity components are derived via vel_inertial().
    """
    v:              float = 0.0   # total speed          (m/s)
    gamma_v:        float = 0.0   # flight-path angle    (rad)
    gamma_h:        float = 0.0   # heading azimuth      (rad)
    h:              float = 0.0   # altitude             (m)
    x:              float = 0.0   # north displacement   (m)
    y:              float = 0.0   # east displacement    (m)
    mass:           float = 0.0   # current total mass   (kg)
    t:              float = 0.0   # elapsed time         (s)
    engine_on:      bool  = True
    prop_remaining: float = 0.0   # propellant left      (kg)

    def vel_inertial(self) -> tuple:
        """Return (vx, vy, vh) — velocity components in the inertial frame."""
        vH = self.v * math.cos(self.gamma_v)
        return (
            vH * math.cos(self.gamma_h),   # vx – north
            vH * math.sin(self.gamma_h),   # vy – east
            self.v * math.sin(self.gamma_v) # vh – up
        )


# ---------------------------------------------------------------------------
# Per-step logged quantities
# ---------------------------------------------------------------------------

@dataclass
class StepLog:
    """Forces and derived scalars at one timestep."""
    thr:         float = 0.0   # engine thrust         (N)
    drag_aero:   float = 0.0   # aerodynamic drag      (N)
    lift:        float = 0.0   # aerodynamic lift      (N)   NEW
    drag_ctrl:   float = 0.0   # actuator / ZEM drag   (N)
    acc:         float = 0.0   # |total acceleration|  (m/s²)
    gamma_v_deg: float = 0.0   # flight-path angle     (deg) NEW
    gamma_h_deg: float = 0.0   # heading azimuth       (deg) NEW
    aLat_x:      float = 0.0
    aLat_y:      float = 0.0
    aLat_z:      float = 0.0
    aLat_mag:    float = 0.0


# ---------------------------------------------------------------------------
# Physics helpers
# ---------------------------------------------------------------------------

def gravity(h: float) -> float:
    """Local gravitational acceleration (m/s²) at altitude h (m)."""
    return G0 * (RE / (RE + h)) ** 2


def integrate_step(
    state:     State,
    params:    Params,
    thrust_n:  float,
    drag_aero: float,
    lift:      float,
    drag_ctrl: float,
    grav:      float,
    a_zem:     tuple,
    dt:        float,
) -> tuple:
    """
    Advance the booster state by one timestep dt using velocity-frame ODEs.

    Thrust decomposition
    --------------------
    grav_turn = True  → T_t = F, T_nV = T_nH = 0  (thrust ∥ velocity)
    grav_turn = False → body held at (launch_angle, launch_azimuth);
                        α = γV₀ − γV is the angle-of-attack in the pitch plane
                        T_t  = F·cos(α)
                        T_nV = F·sin(α)

    ZEM command
    -----------
    a_zem is a (a_nV, a_nH) tuple already expressed in velocity-frame normal
    axes (output of guidance.lateral_accel_command).  Applied directly to the
    dγV/dt and dγH/dt equations without further projection.

    Returns
    -------
    (new_state, acc_mag)
    """
    v, gV, gH = state.v, state.gamma_v, state.gamma_h
    spd = max(v, 1e-6)

    # ── Thrust in velocity frame ───────────────────────────────────────────
    if params.grav_turn:
        T_t, T_nV, T_nH = thrust_n, 0.0, 0.0
    else:
        gV0   = math.radians(params.launch_angle)
        alpha = gV0 - gV           # angle of attack (pitch plane)
        T_t   = thrust_n * math.cos(alpha)
        T_nV  = thrust_n * math.sin(alpha)
        T_nH  = 0.0

    # ── ZEM command (already in velocity-frame normals) ───────────────────
    a_zem_nV, a_zem_nH = a_zem

    drag_tot = drag_aero + drag_ctrl

    # ── Velocity-frame ODEs ────────────────────────────────────────────────
    dv_dt  = (T_t - drag_tot) / state.mass - grav * math.sin(gV) + params.a_cmd_t

    # Only apply pitch/yaw rotation when above minimum speed.
    # Below grav_turn_v_min (in grav-turn mode) the body angle is held fixed,
    # matching real-rocket practice of flying straight off the rail first.
    rotation_active = spd > (params.grav_turn_v_min if params.grav_turn else 0.5)

    if rotation_active:
        dgV_dt = ((T_nV + lift) / (state.mass * spd)
                  - grav * math.cos(gV) / spd
                  + params.a_cmd_nv / spd
                  + a_zem_nV / spd)

        vcos_gV = spd * math.cos(gV)
        if abs(vcos_gV) > 0.5:
            dgH_dt = (T_nH / (state.mass * vcos_gV)
                      + params.a_cmd_nh / vcos_gV
                      + a_zem_nH / vcos_gV)
        else:
            dgH_dt = 0.0
    else:
        dgV_dt = 0.0
        dgH_dt = 0.0

    if not rotation_active:
        dgV_dt = 0.0
        dgH_dt = 0.0

    # ── Forward-Euler integration ──────────────────────────────────────────
    v_raw = v + dv_dt * dt
    if v_raw < 0:
        # Apogee passed: vehicle reverses direction.  Flip γV and heading so
        # the velocity vector points in the correct (now-falling) direction.
        new_v  = -v_raw
        new_gV = -(gV + dgV_dt * dt)
        new_gH = gH + dgH_dt * dt + math.pi
    else:
        new_v  = v_raw
        new_gV = gV + dgV_dt * dt
        new_gH = gH + dgH_dt * dt
    new_gV = max(-math.pi / 2 + 1e-4, min(math.pi / 2 - 1e-4, new_gV))

    # ── Position update (inertial frame) ──────────────────────────────────
    new_h = state.h + new_v * math.sin(new_gV) * dt
    new_x = state.x + new_v * math.cos(new_gV) * math.cos(new_gH) * dt
    new_y = state.y + new_v * math.cos(new_gV) * math.sin(new_gH) * dt

    new_state = State(
        v=new_v, gamma_v=new_gV, gamma_h=new_gH,
        h=new_h, x=new_x, y=new_y,
        mass=state.mass, t=state.t + dt,
        engine_on=state.engine_on,
        prop_remaining=state.prop_remaining,
    )

    acc_mag = math.sqrt(
        dv_dt**2
        + (v * dgV_dt)**2
        + (v * math.cos(gV) * dgH_dt)**2
    )
    return new_state, acc_mag
