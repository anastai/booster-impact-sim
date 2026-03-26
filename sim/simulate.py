"""
Top-level simulation runner.

Usage
-----
    from sim.simulate import simulate
    from sim.physics import Params

    result = simulate(Params(x_target=150.0, a_lat_max=2.0))
    print(result['summary'])
    # {'max_altitude_km': ..., 'miss_distance_m': ..., ...}
"""
import math
from dataclasses import dataclass, field

from .atmosphere import density
from .guidance import lateral_accel_command
from .physics import G0, Params, State, StepLog, gravity, integrate_step

_LOG_EVERY   = 5      # record one sample per this many timesteps
_MAX_STEPS   = 60_000 # safety limit (~3.3 hours at dt=0.2 s)


def simulate(params: Params, dt: float = 0.2) -> dict:
    """
    Run the full 3-D guided booster simulation.

    Parameters
    ----------
    params : Params dataclass (user-friendly units — degrees, km, kN, g)
    dt     : integration timestep (s); default 0.2 s

    Returns
    -------
    dict with two keys:
        'summary' : scalar results (dict)
        'series'  : time-series arrays (dict of lists)
    """
    # --- Convert parameters to SI ---
    thrust_n         = params.thrust_kn * 1_000.0
    x_target_m       = params.x_target  * 1_000.0
    y_target_m       = params.y_target  * 1_000.0
    a_lat_max_ms2    = params.a_lat_max * G0
    launch_angle_rad = math.radians(params.launch_angle)
    Aref             = math.pi * (params.diam / 2.0) ** 2

    mdot      = thrust_n / (params.isp * G0)
    prop_used = min(params.m_prop, mdot * params.burn_max)

    # --- Initial state ---
    state = State(
        mass=params.m_prop + params.m_str,
        prop_remaining=prop_used,
    )

    # --- Time-series storage ---
    series: dict[str, list] = {k: [] for k in [
        't', 'x', 'y', 'h', 'v',
        'thr', 'drag_aero', 'drag_ctrl', 'acc',
        'aLat', 'aLat_x', 'aLat_y',
    ]}

    # --- Accumulators ---
    max_h      = 0.0
    max_v      = 0.0
    burnout_h  = 0.0
    burnout_t  = 0.0

    # --- Main integration loop ---
    for i in range(_MAX_STEPS):
        spd  = math.sqrt(state.vx**2 + state.vy**2 + state.vh**2)
        grav = gravity(state.h)
        rho  = density(params.atm, state.h)
        qS   = 0.5 * rho * spd * spd * Aref

        # Engine: compute thrust and update mass/propellant
        thrust_n_now = 0.0
        new_engine_on      = state.engine_on
        new_prop           = state.prop_remaining
        new_mass           = state.mass
        burnout_this_step  = False

        if state.engine_on and state.prop_remaining > 0:
            thrust_n_now = thrust_n
            new_prop     = state.prop_remaining - mdot * dt
            new_mass     = state.mass           - mdot * dt
            if new_prop <= 0.0 or state.t >= params.burn_max:
                new_engine_on     = False
                burnout_this_step = True
                burnout_h         = state.h
                burnout_t         = state.t
                new_mass          = params.m_str + (params.m_prop - prop_used)

        # Apply engine changes to state before force calculation
        state = State(
            x=state.x, y=state.y, h=state.h,
            vx=state.vx, vy=state.vy, vh=state.vh,
            mass=new_mass, t=state.t,
            engine_on=new_engine_on,
            prop_remaining=max(new_prop, 0.0),
        )

        # Horizontal drag deceleration (for guidance impact prediction)
        cd_now_pre    = params.cd if state.engine_on else params.cd_fall
        drag_aero_pre = 0.5 * density(params.atm, state.h) * spd * spd * Aref * cd_now_pre
        if spd > 0.01 and state.mass > 0:
            a_drag_x_now = -(drag_aero_pre / state.mass) * (state.vx / spd)
            a_drag_y_now = -(drag_aero_pre / state.mass) * (state.vy / spd)
        else:
            a_drag_x_now = a_drag_y_now = 0.0

        # Guidance: ZEM, active post-burnout only.
        # During powered ascent the ballistic TTG from current position is a
        # few seconds (booster just left the ground), so the predictor has no
        # knowledge of the remaining thrust and saturates guidance at max
        # lateral accel for the entire burn — dramatically increasing range.
        # A thrust-aware trajectory predictor is needed for in-burn guidance;
        # that is left as a future enhancement.
        if not state.engine_on:
            a_lat = lateral_accel_command(
                state.x, state.y, state.h,
                state.vx, state.vy, state.vh, spd,
                x_target_m, y_target_m, a_lat_max_ms2, grav,
                a_drag_x=a_drag_x_now,
                a_drag_y=a_drag_y_now,
            )
        else:
            a_lat = (0.0, 0.0, 0.0)
        aLat_x, aLat_y, aLat_z = a_lat
        aLat_mag = math.sqrt(aLat_x**2 + aLat_y**2 + aLat_z**2)

        # Drag
        cd_now    = params.cd if state.engine_on else params.cd_fall
        drag_aero = qS * cd_now
        drag_ctrl = (qS * params.cd_ctrl * (aLat_mag / a_lat_max_ms2) ** 2
                     if a_lat_max_ms2 > 0 else 0.0)
        drag_tot  = drag_aero + drag_ctrl

        # Log (before integrating — forces correspond to current state)
        if i % _LOG_EVERY == 0:
            series['t'].append(round(state.t, 1))
            series['x'].append(round(state.x / 1_000, 2))
            series['y'].append(round(state.y / 1_000, 2))
            series['h'].append(round(state.h / 1_000, 2))
            series['v'].append(round(spd, 1))
            series['thr'].append(round(thrust_n_now / 1_000, 2))
            series['drag_aero'].append(round(drag_aero / 1_000, 2))
            series['drag_ctrl'].append(round(drag_ctrl / 1_000, 2))
            series['acc'].append(round(spd / G0, 3))   # placeholder; overwritten below
            series['aLat'].append(round(aLat_mag / G0, 4))
            series['aLat_x'].append(round(aLat_x  / G0, 4))
            series['aLat_y'].append(round(aLat_y  / G0, 4))

        # Integrate one step
        state, acc_mag = integrate_step(
            state, a_lat, thrust_n_now, drag_tot, grav, launch_angle_rad, dt
        )

        # Overwrite acc with the correctly computed value
        if i % _LOG_EVERY == 0:
            series['acc'][-1] = round(acc_mag / G0, 3)

        # Extrema
        if state.h > max_h:
            max_h = state.h
        if spd > max_v:
            max_v = spd

        # Ground hit
        if state.h <= 0.0:
            state = State(
                x=state.x, y=state.y, h=0.0,
                vx=state.vx, vy=state.vy, vh=state.vh,
                mass=state.mass, t=state.t,
                engine_on=state.engine_on,
                prop_remaining=state.prop_remaining,
            )
            break

    miss = math.sqrt(
        (state.x - x_target_m) ** 2 + (state.y - y_target_m) ** 2
    )

    summary = {
        'max_altitude_km':     round(max_h / 1_000, 2),
        'max_velocity_ms':     round(max_v, 1),
        'burnout_altitude_km': round(burnout_h / 1_000, 2),
        'burnout_time_s':      round(burnout_t, 1),
        'impact_x_km':         round(state.x / 1_000, 2),
        'impact_y_km':         round(state.y / 1_000, 2),
        'miss_distance_m':     round(miss, 1),
        'flight_time_s':       round(state.t, 1),
    }

    return {'summary': summary, 'series': series}
