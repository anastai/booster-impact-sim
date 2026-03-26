"""
Guidance law: zero-effort miss (ZEM).

The booster predicts where it will land (ignoring drag) and commands a lateral
acceleration to eliminate the miss.  The command is projected onto the plane
perpendicular to the current velocity vector so it is always a true lateral
(not axial) acceleration.
"""
import math

GUIDANCE_GAIN = 4.0   # navigation constant (N in PN terminology)
_TTG_REG      = 1.0   # regularisation to avoid division by zero near impact (s²)


def estimate_ttg(h: float, vh: float, grav: float) -> float:
    """
    Drag-free time-to-ground estimate (s).
    Solves  h + vh·t − ½·g·t² = 0  for the positive root.
    """
    disc = vh * vh + 2.0 * grav * h
    if disc < 0.0 or h <= 0.0:
        return 0.1
    return max((vh + math.sqrt(disc)) / grav, 0.1)


def lateral_accel_command(
    x: float, y: float, h: float,
    vx: float, vy: float, vh: float, spd: float,
    x_target: float, y_target: float,
    a_lat_max: float,
    grav: float,
    a_drag_x: float = 0.0,
    a_drag_y: float = 0.0,
) -> tuple[float, float, float]:
    """
    Compute the lateral acceleration command (m/s²) using ZEM guidance.

    The command is perpendicular to the velocity vector — it cannot add or
    subtract from the axial speed, only steer the booster sideways.

    Parameters
    ----------
    x, y, h        : current position (m)
    vx, vy, vh     : current velocity components (m/s)
    spd            : current speed |v| (m/s)
    x_target       : target downrange position (m)
    y_target       : target crossrange position (m)
    a_lat_max      : saturation limit (m/s²)
    grav           : local gravitational acceleration (m/s²)
    a_drag_x       : current horizontal drag deceleration x-component (m/s²,
                     negative when vx > 0).  Used to correct the impact
                     prediction for aerodynamic drag.
    a_drag_y       : same for y-component (m/s²)

    Returns
    -------
    (ax, ay, az)   : lateral acceleration in world frame (m/s²)
    """
    if h <= 50.0 or spd <= 1.0 or a_lat_max <= 0.0:
        return 0.0, 0.0, 0.0

    ttg = estimate_ttg(h, vh, grav)

    # Drag-corrected predicted landing point (constant-drag extrapolation).
    # Without this, the drag-free prediction is too optimistic and the guidance
    # systematically over-corrects, making accuracy worse than ballistic.
    x_pred = x + vx * ttg + 0.5 * a_drag_x * ttg ** 2
    y_pred = y + vy * ttg + 0.5 * a_drag_y * ttg ** 2
    miss_x = x_target - x_pred
    miss_y = y_target - y_pred

    # Desired correction acceleration (horizontal plane)
    denom  = ttg * ttg + _TTG_REG
    ax_des = GUIDANCE_GAIN * miss_x / denom
    ay_des = GUIDANCE_GAIN * miss_y / denom

    # Project onto plane perpendicular to velocity
    vhx = vx / spd
    vhy = vy / spd
    vhz = vh / spd
    dot = ax_des * vhx + ay_des * vhy   # az_des = 0
    ax  = ax_des - dot * vhx
    ay  = ay_des - dot * vhy
    az  =        - dot * vhz

    # Saturate at a_lat_max
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag > a_lat_max:
        s   = a_lat_max / mag
        ax *= s
        ay *= s
        az *= s

    return ax, ay, az
