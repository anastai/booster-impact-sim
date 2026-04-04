"""
Top-level simulation runner — velocity-frame edition (v2.0).

Usage
-----
    from sim.simulate import simulate
    from sim.physics import Params

    result = simulate(Params(x_target=150.0, grav_turn=True, cl=0.0))
    print(result['summary'])
"""
import math
from .atmosphere import density
from .guidance import lateral_accel_command
from .physics import G0, Params, State, gravity, integrate_step

_LOG_EVERY = 5       # record one sample per this many timesteps
_MAX_STEPS = 60_000  # safety limit (~3.3 h at dt = 0.2 s)


def simulate(params: Params, dt: float = 0.2) -> dict:
    """
    Run the full velocity-frame booster simulation with optional ZEM guidance.

    Parameters
    ----------
    params : Params dataclass
    dt     : integration timestep (s)

    Returns
    -------
    dict with keys:
        'summary' : scalar results (dict)
        'series'  : time-series arrays (dict of lists)
    """
    # ── Convert to SI ──────────────────────────────────────────────────────
    thrust_n      = params.thrust_kn * 1_000.0
    x_target_m    = params.x_target  * 1_000.0
    y_target_m    = params.y_target  * 1_000.0
    z_target_m    = params.z_target  * 1_000.0
    a_lat_max_ms2 = params.a_lat_max * G0
    Aref          = math.pi * (params.diam / 2.0) ** 2

    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)

    # ── Initial state (velocity frame) ────────────────────────────────────
    gV0 = math.radians(params.launch_angle)
    gH0 = math.radians(params.launch_azimuth)
    state = State(
        v=0.0,
        gamma_v=gV0,
        gamma_h=gH0,
        mass=params.m_pay + params.m_prop + params.m_str,
        prop_remaining=prop_used,
    )

    # ── Time-series storage ────────────────────────────────────────────────
    series: dict = {k: [] for k in [
        't', 'x', 'y', 'h', 'v',
        'gamma_v', 'gamma_h',
        'thr', 'drag_aero', 'lift', 'drag_ctrl', 'acc',
        'aLat', 'a_nV', 'a_nH',
    ]}

    # ── Accumulators ──────────────────────────────────────────────────────
    max_h             = 0.0
    max_v             = 0.0
    max_lift          = 0.0
    burnout_h         = 0.0
    burnout_t         = 0.0
    burnout_gamma_v   = gV0
    passed_above_target = (z_target_m <= 0.0)

    # ── Main loop ──────────────────────────────────────────────────────────
    for i in range(_MAX_STEPS):
        spd  = state.v
        grav = gravity(state.h)
        rho  = density(params.atm, state.h)
        qS   = 0.5 * rho * spd * spd * Aref

        # ── Engine: thrust, mass, propellant ──────────────────────────────
        thrust_n_now      = 0.0
        new_engine_on     = state.engine_on
        new_prop          = state.prop_remaining
        new_mass          = state.mass

        if state.engine_on and state.prop_remaining > 0:
            thrust_n_now = thrust_n
            new_prop     = state.prop_remaining - mdot * dt
            new_mass     = state.mass           - mdot * dt
            if new_prop <= 0.0 or state.t >= params.burn_max:
                new_engine_on   = False
                burnout_h       = state.h
                burnout_t       = state.t
                burnout_gamma_v = state.gamma_v
                new_mass        = params.m_pay + params.m_str + (params.m_prop - prop_used)

        state = State(
            v=state.v, gamma_v=state.gamma_v, gamma_h=state.gamma_h,
            h=state.h, x=state.x, y=state.y,
            mass=new_mass, t=state.t,
            engine_on=new_engine_on,
            prop_remaining=max(new_prop, 0.0),
        )

        # ── Aerodynamic forces ────────────────────────────────────────────
        cd_now    = params.cd if state.engine_on else params.cd_fall
        drag_aero = qS * cd_now
        lift      = qS * params.cl

        if lift > max_lift:
            max_lift = lift

        # ── IACPN guidance (terminal phase: post-burnout only) ───────────
        # During powered flight the booster follows the gravity turn to build
        # up altitude and kinetic energy. Activating PN earlier would
        # continuously pitch the nose toward the ground target and destroy range.
        if not state.engine_on and spd > 0.01 and state.mass > 0:
            a_drag_t = -(0.5 * rho * spd * spd * Aref * params.cd_fall) / state.mass

            # Range gate: angle correction only activates within hit_angle_range of target
            r_to_target = math.sqrt(
                (state.x - x_target_m)**2 +
                (state.y - y_target_m)**2 +
                (state.h - z_target_m)**2
            )
            hit_angle_range_m = (params.hit_angle_range * 1_000.0
                                 if params.hit_angle_range is not None else None)
            angle_law_active = (hit_angle_range_m is None or
                                r_to_target <= hit_angle_range_m)

            hit_gv = (math.radians(params.hit_gamma_v)
                      if params.hit_gamma_v is not None and angle_law_active else None)
            hit_gh = (math.radians(params.hit_gamma_h)
                      if params.hit_gamma_h is not None and angle_law_active else None)
            a_zem = lateral_accel_command(
                state.x, state.y, state.h,
                spd, state.gamma_v, state.gamma_h,
                x_target_m, y_target_m, z_target_m, a_lat_max_ms2, grav,
                a_drag_t=a_drag_t,
                hit_gamma_v=hit_gv,
                hit_gamma_h=hit_gh,
            )
        else:
            a_zem = (0.0, 0.0)

        a_nV, a_nH = a_zem
        aLat_mag = math.sqrt(a_nV**2 + a_nH**2)

        # Actuator drag (proportional to lateral demand²)
        drag_ctrl = (qS * params.cd_ctrl * (aLat_mag / a_lat_max_ms2) ** 2
                     if a_lat_max_ms2 > 0 else 0.0)

        # ── Log (before integration, forces match current state) ──────────
        if i % _LOG_EVERY == 0:
            downrange = math.sqrt(state.x**2 + state.y**2)
            series['t']        .append(round(state.t, 1))
            series['x']        .append(round(state.x / 1_000, 2))
            series['y']        .append(round(state.y / 1_000, 2))
            series['h']        .append(round(state.h / 1_000, 2))
            series['v']        .append(round(spd, 1))
            series['gamma_v']  .append(round(math.degrees(state.gamma_v), 2))
            series['gamma_h']  .append(round(math.degrees(state.gamma_h), 2))
            series['thr']      .append(round(thrust_n_now / 1_000, 2))
            series['drag_aero'].append(round(drag_aero    / 1_000, 2))
            series['lift']     .append(round(lift         / 1_000, 2))
            series['drag_ctrl'].append(round(drag_ctrl    / 1_000, 2))
            series['acc']      .append(0.0)   # overwritten after integrate
            series['aLat']  .append(round(aLat_mag / G0, 4))
            series['a_nV']  .append(round(a_nV     / G0, 4))
            series['a_nH']  .append(round(a_nH     / G0, 4))

        # ── Integrate one step ────────────────────────────────────────────
        state, acc_mag = integrate_step(
            state, params,
            thrust_n_now, drag_aero, lift, drag_ctrl,
            grav, a_zem, dt,
        )

        if i % _LOG_EVERY == 0:
            series['acc'][-1] = round(acc_mag / G0, 3)

        # ── Extrema ───────────────────────────────────────────────────────
        if state.h > max_h: max_h = state.h
        if spd     > max_v: max_v = spd

        # ── Termination ───────────────────────────────────────────────────
        if state.h > z_target_m + 50.0:
            passed_above_target = True
        if passed_above_target and state.h <= z_target_m:
            break
        if state.h <= 0.0:
            break

    # ── Miss distance ──────────────────────────────────────────────────────
    miss = math.sqrt(
        (state.x - x_target_m)**2 +
        (state.y - y_target_m)**2 +
        (state.h - z_target_m)**2
    )

    summary = {
        'max_altitude_km':      round(max_h / 1_000, 2),
        'max_velocity_ms':      round(max_v, 1),
        'max_lift_kn':          round(max_lift / 1_000, 3),
        'burnout_altitude_km':  round(burnout_h / 1_000, 2),
        'burnout_time_s':       round(burnout_t, 1),
        'burnout_gamma_v_deg':  round(math.degrees(burnout_gamma_v), 1),
        'impact_x_km':          round(state.x / 1_000, 2),
        'impact_y_km':          round(state.y / 1_000, 2),
        'impact_z_km':          round(state.h / 1_000, 2),
        'impact_range_km':      round(math.sqrt(state.x**2 + state.y**2) / 1_000, 2),
        'miss_distance_m':      round(miss, 1),
        'flight_time_s':        round(state.t, 1),
        'final_gamma_v_deg':    round(math.degrees(state.gamma_v), 1),
        'final_gamma_h_deg':    round(math.degrees(state.gamma_h), 1),
    }

    return {'summary': summary, 'series': series}
