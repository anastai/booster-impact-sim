"""
Guidance law: Proportional Navigation (PN) — velocity-frame edition (v2.0).

Pure PN formulation
-------------------
The acceleration command is proportional to the LOS (line-of-sight) angular
rate, scaled by the closing velocity:

    a_cmd = N · Vc · dλ/dt

where:
    λ     = (target − missile) / |target − missile|   — LOS unit vector
    Vc    = v_missile · λ                              — closing velocity (m/s)
    dλ/dt = (Ṙ − (Ṙ·λ)λ) / r                         — LOS angular rate (rad/s)
    Ṙ     = −v_missile                                 — relative velocity (target fixed)
    N     = navigation constant (typically 3–5)

The resulting 3-D acceleration vector is perpendicular to the LOS direction
and is then projected onto the velocity-frame normal axes ê_nV and ê_nH to
produce the (a_nV, a_nH) commands used by the equations of motion.

Velocity-frame axes (inertial components)
------------------------------------------
  ê_t   = [ cos(γV)·cos(γH),  cos(γV)·sin(γH),  sin(γV) ]
  ê_nV  = [-sin(γV)·cos(γH), -sin(γV)·sin(γH),  cos(γV) ]   (pitch-up normal)
  ê_nH  = [-sin(γH),           cos(γH),           0       ]   (yaw-left normal)
"""
import math

GUIDANCE_GAIN = 4.0   # navigation constant N


def lateral_accel_command(
    x: float, y: float, h: float,
    v: float, gamma_v: float, gamma_h: float,
    x_target: float, y_target: float, z_target: float,
    a_lat_max: float,
    grav: float,
    a_drag_t: float = 0.0,
) -> tuple:
    """
    Compute lateral acceleration commands (m/s²) in the velocity frame
    using Proportional Navigation.

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
    grav           : local gravity (m/s²)  — unused by pure PN, kept for API compat
    a_drag_t       : tangential drag (m/s²) — unused by pure PN, kept for API compat

    Returns
    -------
    (a_nV, a_nH)   : acceleration commands in the velocity frame (m/s²)
    """
    # ── Range to target ───────────────────────────────────────────────────────
    Rx = x_target - x
    Ry = y_target - y
    Rz = z_target - h
    r  = math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz)

    if r < 50.0 or v <= 1.0 or a_lat_max <= 0.0:
        return 0.0, 0.0

    # ── LOS unit vector ────────────────────────────────────────────────────────
    lx, ly, lz = Rx / r, Ry / r, Rz / r

    # ── Missile velocity (inertial) ────────────────────────────────────────────
    cos_gV = math.cos(gamma_v)
    sin_gV = math.sin(gamma_v)
    cos_gH = math.cos(gamma_h)
    sin_gH = math.sin(gamma_h)

    vx = v * cos_gV * cos_gH   # North
    vy = v * cos_gV * sin_gH   # East
    vz = v * sin_gV             # Up

    # ── Closing velocity: Vc = v_missile · λ ──────────────────────────────────
    Vc = vx*lx + vy*ly + vz*lz
    if Vc <= 0.0:               # moving away from target — no command
        return 0.0, 0.0

    # ── Relative velocity: Ṙ = d(target − missile)/dt = −v_missile ────────────
    Rvx, Rvy, Rvz = -vx, -vy, -vz

    # ── LOS angular rate: dλ/dt = (Ṙ − (Ṙ·λ)λ) / r ──────────────────────────
    Rdl  = Rvx*lx + Rvy*ly + Rvz*lz   # = −Vc
    dlx  = (Rvx - Rdl*lx) / r
    dly  = (Rvy - Rdl*ly) / r
    dlz  = (Rvz - Rdl*lz) / r

    # ── PN command in inertial frame: a = N · Vc · dλ/dt ─────────────────────
    ax_des = GUIDANCE_GAIN * Vc * dlx
    ay_des = GUIDANCE_GAIN * Vc * dly
    az_des = GUIDANCE_GAIN * Vc * dlz

    # ── Project onto velocity-frame normal axes ────────────────────────────────
    # ê_nV = [−sin(γV)cos(γH), −sin(γV)sin(γH), cos(γV)]
    a_nV = (-ax_des * sin_gV * cos_gH
            -ay_des * sin_gV * sin_gH
            +az_des * cos_gV)

    # ê_nH = [−sin(γH), cos(γH), 0]
    a_nH = -ax_des * sin_gH + ay_des * cos_gH

    # ── Saturate resultant ────────────────────────────────────────────────────
    mag = math.sqrt(a_nV*a_nV + a_nH*a_nH)
    if mag > a_lat_max:
        s    = a_lat_max / mag
        a_nV *= s
        a_nH *= s

    return a_nV, a_nH
