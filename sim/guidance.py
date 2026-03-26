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


def estimate_ttg(h: float, vh: float, grav: float, h_target: float = 0.0) -> float:
    """
    Drag-free time-to-target-altitude estimate (s).
    Solves  (h - h_target) + vh·t − ½·g·t² = 0  for the positive root.
    """
    delta_h = h - h_target
    disc = vh * vh + 2.0 * grav * delta_h
    if disc < 0.0 or delta_h <= 0.0:
        return 0.1
    return max((vh + math.sqrt(disc)) / grav, 0.1)


def lateral_accel_command(
    x: float, y: float, h: float,
    vx: float, vy: float, vh: float, spd: float,
    x_target: float, y_target: float, z_target: float,
    a_lat_max: float,
    grav: float,
    a_drag_x: float = 0.0,
    a_drag_y: float = 0.0,
    a_drag_z: float = 0.0,
) -> tuple[float, float, float]:
    """
    Compute the lateral acceleration command (m/s²) using ZEM guidance.

    The command is perpendicular to the velocity vector — it cannot add or
    subtract from the axial speed, only steer the booster sideways.
    When z_target != 0 the guidance also corrects altitude, producing a
    non-zero Z component in the lateral acceleration.

    Parameters
    ----------
    x, y, h        : current position (m)
    vx, vy, vh     : current velocity components (m/s)
    spd            : current speed |v| (m/s)
    x_target       : target downrange  (m)
    y_target       : target crossrange (m)
    z_target       : target altitude   (m)  — may be above or below launch point
    a_lat_max      : saturation limit (m/s²)
    grav           : local gravitational acceleration (m/s²)
    a_drag_x/y/z   : current drag deceleration components (m/s², negative
                     when opposing positive velocity).  Used to correct the
                     impact prediction for aerodynamic drag.

    Returns
    -------
    (ax, ay, az)   : lateral acceleration in world frame (m/s²),
                     guaranteed perpendicular to velocity and ≤ a_lat_max
    """
    delta_h = h - z_target
    if delta_h <= 50.0 or spd <= 1.0 or a_lat_max <= 0.0:
        return 0.0, 0.0, 0.0

    ttg = estimate_ttg(h, vh, grav, h_target=z_target)

    # Drag-corrected predicted impact point (constant-drag extrapolation)
    x_pred = x + vx * ttg + 0.5 * a_drag_x * ttg ** 2
    y_pred = y + vy * ttg + 0.5 * a_drag_y * ttg ** 2
    z_pred = h + vh * ttg + 0.5 * a_drag_z * ttg ** 2
    miss_x = x_target - x_pred
    miss_y = y_target - y_pred
    miss_z = z_target - z_pred

    # Desired 3-D correction acceleration
    denom  = ttg * ttg + _TTG_REG
    ax_des = GUIDANCE_GAIN * miss_x / denom
    ay_des = GUIDANCE_GAIN * miss_y / denom
    az_des = GUIDANCE_GAIN * miss_z / denom   # non-zero when z_target != 0

    # Project onto plane perpendicular to velocity (enforce true lateral)
    vhx = vx / spd
    vhy = vy / spd
    vhz = vh / spd
    dot = ax_des * vhx + ay_des * vhy + az_des * vhz
    ax  = ax_des - dot * vhx
    ay  = ay_des - dot * vhy
    az  = az_des - dot * vhz

    # Saturate at a_lat_max
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag > a_lat_max:
        s   = a_lat_max / mag
        ax *= s
        ay *= s
        az *= s

    return ax, ay, az
