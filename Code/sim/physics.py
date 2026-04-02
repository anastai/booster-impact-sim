"""
Physical constants, parameter / state dataclasses, and one integration step.

Integration scheme
------------------
Semi-implicit (symplectic) Euler — velocities are updated first, then
positions use the *new* velocities.  This matches the JavaScript reference
implementation and conserves energy better than plain forward Euler.
"""
import math
from dataclasses import dataclass

G0 = 9.80665        # standard gravity (m/s²)
RE = 6_371_000.0    # Earth mean radius (m)


# ---------------------------------------------------------------------------
# Input parameters (user-friendly units — converted to SI inside simulate())
# ---------------------------------------------------------------------------

@dataclass
class Params:
    """
    All user-facing simulation parameters.

    Angles in degrees, distances in km, forces in kN, accelerations in g.
    simulate() converts everything to SI internally.
    """
    m_prop:       float = 5_000.0   # propellant mass          (kg)
    m_str:        float = 800.0     # booster dry mass          (kg)
    isp:          float = 260.0     # specific impulse          (s)
    thrust_kn:    float = 120.0     # thrust                    (kN)
    burn_max:     float = 60.0      # max burn duration         (s)
    cd:           float = 0.4       # drag coeff – powered
    cd_fall:      float = 1.2       # drag coeff – tumbling (post-burnout)
    cd_ctrl:      float = 0.3       # actuator drag coefficient
    diam:         float = 1.2       # reference diameter        (m)
    atm:          str   = 'isa'     # atmosphere: 'isa' | 'exp' | 'none'
    launch_angle: float = 75.0      # elevation angle           (deg)
    x_target:     float = 100.0     # target downrange          (km)
    y_target:     float = 0.0       # target crossrange         (km)
    z_target:     float = 0.0       # target altitude           (km, + above / - below launch)
    a_lat_max:    float = 1.0       # max lateral acceleration  (g)


# ---------------------------------------------------------------------------
# Simulation state
# ---------------------------------------------------------------------------

@dataclass
class State:
    """Booster state at one instant in time."""
    x:              float = 0.0     # downrange position  (m)
    y:              float = 0.0     # crossrange position (m)
    h:              float = 0.0     # altitude            (m)
    vx:             float = 0.0     # downrange velocity  (m/s)
    vy:             float = 0.0     # crossrange velocity (m/s)
    vh:             float = 0.0     # vertical velocity   (m/s)
    mass:           float = 0.0     # current total mass  (kg)
    t:              float = 0.0     # elapsed time        (s)
    engine_on:      bool  = True
    prop_remaining: float = 0.0     # propellant left to burn (kg)


# ---------------------------------------------------------------------------
# Per-step logged quantities
# ---------------------------------------------------------------------------

@dataclass
class StepLog:
    """Forces and derived scalars computed at one timestep."""
    thr:       float = 0.0   # engine thrust        (N)
    drag_aero: float = 0.0   # aerodynamic drag     (N)
    drag_ctrl: float = 0.0   # actuator/control drag(N)
    acc:       float = 0.0   # total |acceleration| (m/s²)
    aLat_x:    float = 0.0   # lateral accel x      (m/s²)
    aLat_y:    float = 0.0   # lateral accel y      (m/s²)
    aLat_mag:  float = 0.0   # lateral accel |mag|  (m/s²)


# ---------------------------------------------------------------------------
# Physics helpers
# ---------------------------------------------------------------------------

def gravity(h: float) -> float:
    """Local gravitational acceleration (m/s²) at altitude h (m)."""
    return G0 * (RE / (RE + h)) ** 2


def integrate_step(
    state: State,
    a_lat: tuple[float, float, float],
    thrust_n: float,
    drag_tot: float,
    grav: float,
    launch_angle_rad: float,
    dt: float,
) -> tuple[State, float]:
    """
    Advance the booster state by one timestep dt (s).

    Parameters
    ----------
    state            : current booster state
    a_lat            : lateral acceleration command (ax, ay, az) in m/s²
    thrust_n         : engine thrust magnitude (N); 0 after burnout
    drag_tot         : total drag force magnitude (N)
    grav             : local gravity (m/s²)
    launch_angle_rad : fixed thrust elevation angle (rad)
    dt               : timestep (s)

    Returns
    -------
    new_state        : booster state at t + dt
    acc_mag          : total acceleration magnitude (m/s²) — for logging
    """
    spd    = math.sqrt(state.vx**2 + state.vy**2 + state.vh**2)
    aLat_x, aLat_y, aLat_z = a_lat

    # Thrust components (fixed launch direction, x-z plane only)
    thr_x = thrust_n * math.cos(launch_angle_rad)
    thr_z = thrust_n * math.sin(launch_angle_rad)

    # Drag components opposing velocity
    if spd > 0.01:
        drag_x = drag_tot * state.vx / spd
        drag_y = drag_tot * state.vy / spd
        drag_z = drag_tot * state.vh / spd
    else:
        drag_x = drag_y = drag_z = 0.0

    # Net acceleration components
    ax = (thr_x - drag_x) / state.mass + aLat_x
    ay = (      - drag_y) / state.mass + aLat_y
    az = (thr_z - drag_z) / state.mass - grav + aLat_z

    # Semi-implicit Euler: update velocity first, then position with new velocity
    new_vx = state.vx + ax * dt
    new_vy = state.vy + ay * dt
    new_vh = state.vh + az * dt

    new_state = State(
        x=state.x + new_vx * dt,
        y=state.y + new_vy * dt,
        h=state.h + new_vh * dt,
        vx=new_vx, vy=new_vy, vh=new_vh,
        mass=state.mass,
        t=state.t + dt,
        engine_on=state.engine_on,
        prop_remaining=state.prop_remaining,
    )
    acc_mag = math.sqrt(ax**2 + ay**2 + az**2)
    return new_state, acc_mag
