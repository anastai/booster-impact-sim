"""
Guidance law: Impact Angle Constrained Proportional Navigation (IACPN).

Two additive terms, both in velocity-frame normals (ê_nV, ê_nH)
----------------------------------------------------------------
1. Standard PN  — drives position error to zero by nulling the LOS rate:

       a_PN = N · gain · dλ/dt
       gain = Vc  when Vc > 0  (closing — original behaviour, best accuracy)
            = v   when Vc ≤ 0  (receding — keeps guidance active for targets
                                 in any direction, including "behind" the launch)

   Projected onto ê_nV and ê_nH, this gives the (a_nV_PN, a_nH_PN) pair.

2. Impact angle term  — corrects the terminal approach angle relative to the
   LOS direction.  Under pure PN the missile arrives approximately along the
   current LOS, so LOS elevation / azimuth ≈ natural impact angles.
   The correction drives the difference between desired and LOS angles to zero:

       a_nV_angle = K · v · sin(hit_γV − γV_LOS) / tgo
       a_nH_angle = K · v · cos(γV) · sin(hit_γH − γH_LOS) / tgo

   Signs are consistent with the velocity-frame ODEs:
       dγV/dt = a_nV / v               →  a_nV = v · Δγ_V / tgo
       dγH/dt = a_nH / (v · cos γV)   →  a_nH = v · cos(γV) · Δγ_H / tgo

If hit_gamma_v and hit_gamma_h are both None the law reduces to pure PN.

Velocity-frame normal axes (inertial components)
-------------------------------------------------
  ê_nV  = [−sin(γV)·cos(γH), −sin(γV)·sin(γH),  cos(γV)]   (pitch-up)
  ê_nH  = [−sin(γH),           cos(γH),           0       ]   (yaw-left)
"""
import math

GUIDANCE_GAIN = 4.0   # PN navigation constant  N
ANGLE_GAIN    = 2.0   # impact-angle correction gain K
_TTG_MIN      = 1.0   # minimum tgo clamp — prevents blow-up near impact (s)


def lateral_accel_command(
    x: float, y: float, h: float,
    v: float, gamma_v: float, gamma_h: float,
    x_target: float, y_target: float, z_target: float,
    a_lat_max: float,
    grav: float,
    a_drag_t:    float       = 0.0,
    hit_gamma_v: float | None = None,
    hit_gamma_h: float | None = None,
) -> tuple:
    """
    Compute lateral acceleration commands (m/s²) in the velocity frame.

    Parameters
    ----------
    x, y, h        : current position (m, inertial frame)
    v              : current speed (m/s)
    gamma_v        : flight-path angle (rad)
    gamma_h        : heading azimuth (rad)
    x_target       : target North (m)
    y_target       : target East  (m)
    z_target       : target altitude (m)
    a_lat_max      : saturation limit (m/s²)
    grav           : local gravity (m/s²) — kept for API compatibility
    a_drag_t       : tangential drag (m/s²) — kept for API compatibility
    hit_gamma_v    : desired impact flight-path angle (rad). None = unconstrained
    hit_gamma_h    : desired impact azimuth (rad).            None = unconstrained

    Returns
    -------
    (a_nV, a_nH)   : acceleration commands in velocity-frame normals (m/s²)
    """
    # ── Range vector ──────────────────────────────────────────────────────────
    Rx = x_target - x
    Ry = y_target - y
    Rz = z_target - h
    r  = math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz)

    if r < 50.0 or v <= 1.0 or a_lat_max <= 0.0:
        return 0.0, 0.0

    # ── LOS unit vector ───────────────────────────────────────────────────────
    lx, ly, lz = Rx / r, Ry / r, Rz / r

    # ── Missile velocity (inertial) ───────────────────────────────────────────
    cos_gV = math.cos(gamma_v)
    sin_gV = math.sin(gamma_v)
    cos_gH = math.cos(gamma_h)
    sin_gH = math.sin(gamma_h)

    vx = v * cos_gV * cos_gH
    vy = v * cos_gV * sin_gH
    vz = v * sin_gV

    # ── Closing velocity ──────────────────────────────────────────────────────
    # When Vc > 0 (closing): use Vc as PN gain — original behaviour,
    # preserves 0.9 m miss on forward targets.
    # When Vc ≤ 0 (receding / flying away from target): fall back to v so
    # guidance stays active and can steer toward targets in any direction,
    # enabling engagement of targets "behind" the launch azimuth.
    Vc = vx*lx + vy*ly + vz*lz
    _pn_gain = Vc if Vc > 0.0 else v

    # ── PN term: a_PN = N · _pn_gain · dλ/dt ────────────────────────────────
    Rvx, Rvy, Rvz = -vx, -vy, -vz          # relative velocity (target fixed)
    Rdl  = Rvx*lx + Rvy*ly + Rvz*lz        # = −Vc
    dlx  = (Rvx - Rdl*lx) / r
    dly  = (Rvy - Rdl*ly) / r
    dlz  = (Rvz - Rdl*lz) / r

    ax_pn = GUIDANCE_GAIN * _pn_gain * dlx
    ay_pn = GUIDANCE_GAIN * _pn_gain * dly
    az_pn = GUIDANCE_GAIN * _pn_gain * dlz

    # ── Project PN onto velocity-frame normals ────────────────────────────────
    # ê_nV = [−sin(γV)cos(γH), −sin(γV)sin(γH), cos(γV)]
    a_nV = -ax_pn*sin_gV*cos_gH - ay_pn*sin_gV*sin_gH + az_pn*cos_gV

    # ê_nH = [−sin(γH), cos(γH), 0]
    a_nH = -ax_pn*sin_gH + ay_pn*cos_gH

    # ── Impact angle correction ───────────────────────────────────────────────
    # Under pure PN the missile arrives approximately along the LOS.
    # LOS elevation / azimuth are therefore the "natural" predicted impact angles.
    # The correction biases (a_nV, a_nH) to steer the actual terminal angles
    # toward (hit_gamma_v, hit_gamma_h) relative to those LOS angles.
    if hit_gamma_v is not None or hit_gamma_h is not None:
        r_H    = math.sqrt(Rx*Rx + Ry*Ry)                  # horizontal range

        # LOS elevation: negative when missile is above target
        gV_los = math.atan2(Rz, max(r_H, 1.0))

        # LOS azimuth: heading toward target
        gH_los = math.atan2(Ry, Rx) if r_H > 1.0 else gamma_h

        tgo = max(r / v, _TTG_MIN)

        if hit_gamma_v is not None:
            dg_V  = hit_gamma_v - gV_los
            # a_nV drives dγV/dt = a_nV/v  →  a_nV = v · δγV / tgo
            a_nV += ANGLE_GAIN * v * math.sin(dg_V) / tgo

        if hit_gamma_h is not None:
            dg_H  = hit_gamma_h - gH_los
            # wrap angle error to [−π, π]
            dg_H  = (dg_H + math.pi) % (2 * math.pi) - math.pi
            # a_nH drives dγH/dt = a_nH/(v·cos γV)  →  a_nH = v·cos(γV)·δγH/tgo
            a_nH += ANGLE_GAIN * v * cos_gV * math.sin(dg_H) / tgo

    # ── Saturate resultant ────────────────────────────────────────────────────
    mag = math.sqrt(a_nV*a_nV + a_nH*a_nH)
    if mag > a_lat_max:
        s    = a_lat_max / mag
        a_nV *= s
        a_nH *= s

    return a_nV, a_nH
