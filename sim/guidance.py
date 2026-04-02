"""
Guidance law: Zero-Effort Miss (ZEM) — velocity-frame edition (v2.0).

The guidance works entirely in velocity-frame coordinates:
  - Impact is predicted in the inertial frame (using current v, γV, γH)
  - The ZEM correction is computed in inertial frame
  - It is projected DIRECTLY onto ê_nV and ê_nH (no intermediate
    "perpendicular to velocity" step — that projection is implicit because
    ê_nV and ê_nH already span the normal plane)
  - Output: (a_nV, a_nH) in m/s²  — ready to drop into the dγV/dt and
    dγH/dt equations without further transformation

Velocity-frame axes (inertial components)
------------------------------------------
  ê_t   = [ cos(γV)·cos(γH),  cos(γV)·sin(γH),  sin(γV) ]
  ê_nV  = [-sin(γV)·cos(γH), -sin(γV)·sin(γH),  cos(γV) ]   (pitch-up normal)
  ê_nH  = [-sin(γH),           cos(γH),           0       ]   (yaw-left normal)
"""
import math

GUIDANCE_GAIN = 4.0   # navigation constant (N in PN terminology)
_TTG_REG      = 1.0   # regularisation — avoids division by zero near impact (s²)


def estimate_ttg(h: float, vh: float, grav: float, h_target: float = 0.0) -> float:
    """
    Drag-free time-to-target-altitude estimate (s).
    Solves  (h − h_target) + vh·t − ½·g·t² = 0  for the positive root.
    """
    delta_h = h - h_target
    disc    = vh * vh + 2.0 * grav * delta_h
    if disc < 0.0 or delta_h <= 0.0:
        return 0.1
    return max((vh + math.sqrt(disc)) / grav, 0.1)


def lateral_accel_command(
    x: float, y: float, h: float,
    v: float, gamma_v: float, gamma_h: float,
    x_target: float, y_target: float, z_target: float,
    a_lat_max: float,
    grav: float,
    a_drag_t: float = 0.0,
) -> tuple:
    """
    Compute lateral acceleration commands (m/s²) in the velocity frame.

    Parameters
    ----------
    x, y, h        : current position (m, inertial frame)
    v              : current speed (m/s)
    gamma_v        : flight-path angle (rad)
    gamma_h        : heading azimuth (rad)
    x_target       : target North position (m)
    y_target       : target East  position (m)
    z_target       : target altitude      (m)
    a_lat_max      : saturation limit (m/s²)
    grav           : local gravity (m/s²)
    a_drag_t       : tangential drag deceleration (m/s², negative = decelerating).
                     Used to correct the impact prediction for aerodynamic drag.

    Returns
    -------
    (a_nV, a_nH)   : acceleration commands in the velocity frame (m/s²)
                     a_nV acts along ê_nV (pitch-normal, upward from velocity)
                     a_nH acts along ê_nH (yaw-normal, lateral)
                     Both are perpendicular to the velocity vector by construction.
    """
    delta_h = h - z_target
    if delta_h <= 50.0 or v <= 1.0 or a_lat_max <= 0.0:
        return 0.0, 0.0

    # ── Velocity components in inertial frame ─────────────────────────────
    cos_gV = math.cos(gamma_v)
    sin_gV = math.sin(gamma_v)
    cos_gH = math.cos(gamma_h)
    sin_gH = math.sin(gamma_h)

    vx = v * cos_gV * cos_gH   # North
    vy = v * cos_gV * sin_gH   # East
    vh = v * sin_gV             # Up

    # ── Time-to-target estimate ───────────────────────────────────────────
    ttg = estimate_ttg(h, vh, grav, h_target=z_target)

    # ── Drag-corrected predicted impact (constant-drag extrapolation) ─────
    # Drag acts along ê_t → inertial drag components
    a_drag_x = a_drag_t * cos_gV * cos_gH
    a_drag_y = a_drag_t * cos_gV * sin_gH
    a_drag_z = a_drag_t * sin_gV

    x_pred = x + vx * ttg + 0.5 * a_drag_x * ttg ** 2
    y_pred = y + vy * ttg + 0.5 * a_drag_y * ttg ** 2
    z_pred = h + vh * ttg + 0.5 * a_drag_z * ttg ** 2

    # ── ZEM desired correction (inertial frame) ───────────────────────────
    denom  = ttg * ttg + _TTG_REG
    ax_des = GUIDANCE_GAIN * (x_target - x_pred) / denom
    ay_des = GUIDANCE_GAIN * (y_target - y_pred) / denom
    az_des = GUIDANCE_GAIN * (z_target - z_pred) / denom

    # ── Project directly onto velocity-frame normal axes ──────────────────
    # ê_nV = [-sin(γV)·cos(γH), -sin(γV)·sin(γH), cos(γV)]
    a_nV = (-ax_des * sin_gV * cos_gH
            -ay_des * sin_gV * sin_gH
            +az_des * cos_gV)

    # ê_nH = [-sin(γH), cos(γH), 0]
    a_nH = -ax_des * sin_gH + ay_des * cos_gH

    # ── Saturate resultant ────────────────────────────────────────────────
    mag = math.sqrt(a_nV * a_nV + a_nH * a_nH)
    if mag > a_lat_max:
        s    = a_lat_max / mag
        a_nV *= s
        a_nH *= s

    return a_nV, a_nH
