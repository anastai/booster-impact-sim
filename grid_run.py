"""
Grid runner — sweep hit points and hit angles.

Runs the booster simulation (or open-loop optimizer) for every combination
of hit_points × hit_angles (Cartesian product).  All other parameters are
shared from base_params.

Usage (script):
    python grid_run.py

Usage (import):
    from grid_run import run_grid, plot_grid

    # IACPN guidance (original behaviour)
    runs = run_grid(base, hit_points, hit_angles=hit_angles)

    # Open-loop optimizer for every cell
    runs = run_grid(base, hit_points, hit_angles=hit_angles,
                    optimise=True, opt_kwargs=dict(n_knots=20, a_max_g=3.0))

    plot_grid(runs, title='My grid')
"""
import math
import itertools
from dataclasses import replace

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

from sim.simulate import simulate
from sim.physics import Params, G0

_LINESTYLES = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--']


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _norm_series(raw_series: dict, optimised: bool) -> dict:
    """
    Return a series dict with unified key names regardless of source.

    simulate() uses:   gamma_v, gamma_h, a_nV, a_nH, aLat   (all in deg or g)
    sim_open_loop() uses: gamma_v_deg, gamma_h_deg, a_nV_g, a_nH_g, aLat_g
    """
    if not optimised:
        return raw_series          # already in the right format
    return {
        't':       raw_series['t'],
        'x':       raw_series['x'],
        'y':       raw_series['y'],
        'h':       raw_series['h'],
        'v':       raw_series['v'],
        'gamma_v': raw_series['gamma_v_deg'],
        'gamma_h': raw_series['gamma_h_deg'],
        'a_nV':    raw_series['a_nV_g'],
        'a_nH':    raw_series['a_nH_g'],
        'aLat':    raw_series['aLat_g'],
    }


def _norm_summary(result: dict, optimised: bool) -> dict:
    """
    Return a flat summary dict with unified keys for plotting/printing.

    Keys always present:
        miss_m, flight_time_s, final_gamma_v_deg, final_gamma_h_deg,
        burnout_time_s, impact_range_km
    Keys present only when optimised=True:
        solved, gamma_v_err_deg, gamma_h_err_deg
    """
    if not optimised:
        sm = result['summary']
        return {
            'miss_m':             sm['miss_distance_m'],
            'flight_time_s':      sm['flight_time_s'],
            'final_gamma_v_deg':  sm['final_gamma_v_deg'],
            'final_gamma_h_deg':  sm['final_gamma_h_deg'],
            'burnout_time_s':     sm['burnout_time_s'],
            'impact_range_km':    sm['impact_range_km'],
            'solved':             None,
            'gamma_v_err_deg':    None,
            'gamma_h_err_deg':    None,
        }
    else:
        sm  = result['summary']
        # Final state comes from the optimised series
        ser = result['series']
        gv  = ser['gamma_v_deg'][-1] if ser['gamma_v_deg'] else 0.0
        gh  = ser['gamma_h_deg'][-1] if ser['gamma_h_deg'] else 0.0
        # burnout_time from the nominal IACPN warm-start run
        bo  = result['nominal']['summary']['burnout_time_s']
        rng = math.sqrt(ser['x'][-1]**2 + ser['y'][-1]**2) if ser['x'] else 0.0
        return {
            'miss_m':             sm['optimised_miss_m'],
            'flight_time_s':      sm['optimised_flight_time_s'],
            'final_gamma_v_deg':  gv,
            'final_gamma_h_deg':  gh,
            'burnout_time_s':     bo,
            'impact_range_km':    rng,
            'solved':             sm['solved'],
            'gamma_v_err_deg':    sm.get('gamma_v_err_deg'),
            'gamma_h_err_deg':    sm.get('gamma_h_err_deg'),
        }


# ---------------------------------------------------------------------------
# Grid runner
# ---------------------------------------------------------------------------

def run_grid(
    base_params:      Params,
    hit_points:       list,
    hit_angles:       list | None = None,
    hit_angle_range:  float | None = None,
    optimise:         bool = False,
    opt_kwargs:       dict | None = None,
) -> list[dict]:
    """
    Run the simulation or optimizer for every combination of
    hit_points × hit_angles (Cartesian product).

    Parameters
    ----------
    base_params      : Params — shared vehicle parameters for all runs.
    hit_points       : list of (x_km, y_km, z_km) tuples.
    hit_angles       : None → no angle constraints.
                       list of (gv_deg, gh_deg) tuples — cross-product with
                       hit_points.  Either element may be None (unconstrained).
    hit_angle_range  : range gate (km) for IACPN runs.  Ignored when
                       optimise=True (optimizer handles angles internally).
    optimise         : if True, use optimize_trajectory() for each run
                       instead of the IACPN simulate().
    opt_kwargs       : dict of keyword arguments forwarded to
                       optimize_trajectory() for every run.
                       Common keys: n_knots, a_max_g, w_miss, w_angle_v,
                       w_angle_h, w_time, method, max_iter, use_de.
                       Defaults: n_knots=20, verbose=False (quiet grid run).

    Returns
    -------
    list of dicts, one per run:
        'index'       : int — run number (0-based)
        'hit_point'   : (x, y, z) km
        'hit_angles'  : (gv_deg, gh_deg) or (None, None)
        'params'      : Params used for this run
        'result'      : raw output from simulate() or optimize_trajectory()
        'optimised'   : bool
        'series'      : normalised time-series (unified key names)
        'summary'     : normalised summary dict (unified key names)
        'label'       : short human-readable string
    """
    if optimise:
        from sim.opt_trajectory import optimize_trajectory

    kw = {'n_knots': 20, 'verbose': False}
    if opt_kwargs:
        kw.update(opt_kwargs)

    angle_list = hit_angles if hit_angles is not None else [(None, None)]
    total = len(hit_points) * len(angle_list)

    runs = []
    for idx, ((x, y, z), (gv, gh)) in enumerate(
            itertools.product(hit_points, angle_list)):

        angle_str = (f'γV={gv}° γH={gh}°'
                     if (gv is not None or gh is not None) else 'no angle')
        mode = 'OPT' if optimise else 'SIM'
        print(f'  [{mode}] Run {idx+1}/{total}:  '
              f'target=({x},{y},{z}) km  {angle_str}')

        p = replace(
            base_params,
            x_target        = x,
            y_target        = y,
            z_target        = z,
            hit_gamma_v     = gv,
            hit_gamma_h     = gh,
            hit_angle_range = hit_angle_range,
        )

        if optimise:
            result = optimize_trajectory(p, **kw)
        else:
            result = simulate(p)

        label = f'#{idx+1} ({x},{y},{z})km {angle_str}'

        sm_norm  = _norm_summary(result, optimise)
        ser_norm = _norm_series(
            result['series'] if optimise else result['series'],
            optimise,
        )

        # Print one-line result
        solved_str = (f'  solved={sm_norm["solved"]}'
                      if optimise else '')
        print(f'         miss={sm_norm["miss_m"]:.1f} m{solved_str}')

        runs.append({
            'index':      idx,
            'hit_point':  (x, y, z),
            'hit_angles': (gv, gh),
            'params':     p,
            'result':     result,
            'optimised':  optimise,
            'series':     ser_norm,
            'summary':    sm_norm,
            'label':      label,
        })

    return runs


# ---------------------------------------------------------------------------
# Grid plot
# ---------------------------------------------------------------------------

def plot_grid(runs: list[dict], title: str = 'Grid run results') -> None:
    """
    Four-panel dashboard for all grid runs.

    Panels
    ------
    [0] Ground track  (North/East) — ▲ launch  ● burnout  ★ target
    [1] Altitude vs time           — ● burnout point
    [2] Flight-path angle γV vs time — dashed lines for desired hit angles
    [3] Miss distance bar chart    — green = solved (<1 m), amber = not solved,
                                     grey = IACPN run (no solved concept)

    Summary table is printed to console after the plot.
    """
    n      = len(runs)
    colors = [cm.tab10(i % 10) for i in range(n)]

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    optimised_any = any(r['optimised'] for r in runs)
    mode_str = 'Optimised' if optimised_any else 'IACPN'
    fig.suptitle(f'{title}  ({n} runs, {mode_str})',
                 fontsize=13, fontweight='bold')

    ax_track = axes[0, 0]
    ax_alt   = axes[0, 1]
    ax_gv    = axes[1, 0]
    ax_miss  = axes[1, 1]

    ax_track.plot(0, 0, 'k^', markersize=10, zorder=10, label='Launch')

    desired_gv_set = set()   # collect unique desired γV values for reference lines

    for run, col, ls in zip(runs, colors, _LINESTYLES):
        s   = run['series']
        sm  = run['summary']
        p   = run['params']
        lbl = run['label']

        # burnout index
        bo_t   = sm['burnout_time_s']
        bo_idx = next((i for i, t in enumerate(s['t']) if t >= bo_t),
                      len(s['t']) - 1)

        # ── Ground track ──────────────────────────────────────────────────
        ax_track.plot(s['x'], s['y'], color=col, lw=2.0, ls=ls, label=lbl)
        ax_track.plot(s['x'][bo_idx], s['y'][bo_idx],
                      'o', color=col, markersize=7, zorder=6)
        ax_track.plot(p.x_target, p.y_target,
                      '*', color=col, markersize=14, zorder=7)

        # ── Altitude ──────────────────────────────────────────────────────
        ax_alt.plot(s['t'], s['h'], color=col, lw=2.0, ls=ls, label=lbl)
        ax_alt.plot(s['t'][bo_idx], s['h'][bo_idx],
                    'o', color=col, markersize=7, zorder=6)

        # ── γV vs time ────────────────────────────────────────────────────
        ax_gv.plot(s['t'], s['gamma_v'], color=col, lw=2.0, ls=ls, label=lbl)
        ax_gv.plot(s['t'][bo_idx], s['gamma_v'][bo_idx],
                   'o', color=col, markersize=7, zorder=6)
        if p.hit_gamma_v is not None:
            desired_gv_set.add(p.hit_gamma_v)

    # draw desired γV reference lines
    for gv_des in sorted(desired_gv_set):
        ax_gv.axhline(gv_des, color='#555', lw=1.2, ls=':',
                      label=f'desired γV={gv_des}°')

    ax_track.plot([], [], 'ko', markersize=7, label='Burnout')

    # ── Miss distance bar chart ───────────────────────────────────────────
    x_pos  = list(range(n))
    misses = [r['summary']['miss_m'] for r in runs]

    bar_colors = []
    for r in runs:
        sv = r['summary']['solved']
        if sv is None:          # IACPN run — use run colour
            bar_colors.append(colors[r['index'] % 10])
        elif sv:                # optimised and solved
            bar_colors.append('#2ca02c')   # green
        else:                   # optimised but not solved
            bar_colors.append('#d62728')   # red

    bars = ax_miss.bar(x_pos, misses, color=bar_colors, width=0.6)
    ax_miss.axhline(1.0, color='#2ca02c', lw=1.2, ls='--',
                    label='1 m threshold' if optimised_any else None)
    ax_miss.set_xticks(x_pos)
    ax_miss.set_xticklabels([f'#{i+1}' for i in range(n)], fontsize=9)
    ax_miss.set_ylabel('Miss distance (m)')
    ax_miss.set_title('Miss distance  (■ green=solved <1 m, ■ red=not solved)')
    ax_miss.grid(alpha=0.3, axis='y')

    top = max(misses) if max(misses) > 0 else 1
    for bar, val, run in zip(bars, misses, runs):
        # main value label
        ax_miss.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + top * 0.015,
                     f'{val:.1f} m', ha='center', va='bottom', fontsize=8)
        # angle error sub-label (optimised runs only)
        ev = run['summary']['gamma_v_err_deg']
        eh = run['summary']['gamma_h_err_deg']
        parts = []
        if ev is not None:
            parts.append(f'ΔγV={ev:+.1f}°')
        if eh is not None:
            parts.append(f'ΔγH={eh:+.1f}°')
        if parts:
            ax_miss.text(bar.get_x() + bar.get_width() / 2,
                         bar.get_height() + top * 0.08,
                         '\n'.join(parts),
                         ha='center', va='bottom', fontsize=7, color='#444')

    # ── Axis formatting ───────────────────────────────────────────────────
    ax_track.set_xlabel('North X (km)'); ax_track.set_ylabel('East Y (km)')
    ax_track.set_title('Ground tracks  (▲=launch  ●=burnout  ★=target)')
    ax_track.legend(fontsize=7, loc='best'); ax_track.grid(alpha=0.3)
    ax_track.set_aspect('equal', adjustable='datalim')

    ax_alt.set_xlabel('Time (s)'); ax_alt.set_ylabel('Altitude (km)')
    ax_alt.set_title('Altitude profiles  (●=burnout)')
    ax_alt.legend(fontsize=7, loc='best'); ax_alt.grid(alpha=0.3)

    ax_gv.axhline(0, color='#aaa', lw=0.8, ls=':')
    ax_gv.set_xlabel('Time (s)'); ax_gv.set_ylabel('γV (deg)')
    ax_gv.set_title('Flight-path angle γV  (●=burnout  ··· desired)')
    ax_gv.legend(fontsize=7, loc='best'); ax_gv.grid(alpha=0.3)

    if optimised_any:
        ax_miss.legend(fontsize=8)

    plt.tight_layout()
    plt.show()

    # ── Summary table ─────────────────────────────────────────────────────
    has_angle = any(r['hit_angles'] != (None, None) for r in runs)
    has_opt   = any(r['optimised'] for r in runs)

    header = (f'{"#":>3}  {"Target (km)":>18}  {"Hit angles":>18}  '
              f'{"miss_m":>8}  {"γV_imp":>8}  {"γH_imp":>8}  {"range_km":>9}')
    if has_opt:
        header += f'  {"solved":>6}  {"ΔγV":>8}  {"ΔγH":>8}'
    print(f'\n{header}')
    print('-' * (len(header) + 2))

    for run in runs:
        sm     = run['summary']
        hp     = run['hit_point']
        gv, gh = run['hit_angles']
        gv_s   = f'{gv}°' if gv is not None else '—'
        gh_s   = f'{gh}°' if gh is not None else '—'
        row = (f"{run['index']+1:>3}  "
               f"({hp[0]:5.1f},{hp[1]:5.1f},{hp[2]:4.1f})  "
               f"{'γV='+gv_s+' γH='+gh_s:>18}  "
               f"{sm['miss_m']:>8.1f}  "
               f"{sm['final_gamma_v_deg']:>7.1f}°  "
               f"{sm['final_gamma_h_deg']:>7.1f}°  "
               f"{sm['impact_range_km']:>9.2f}")
        if has_opt:
            sv  = sm['solved']
            ev  = sm['gamma_v_err_deg']
            eh  = sm['gamma_h_err_deg']
            sv_s = ('Yes' if sv else 'No') if sv is not None else '—'
            ev_s = f'{ev:+.1f}°' if ev is not None else '—'
            eh_s = f'{eh:+.1f}°' if eh is not None else '—'
            row += f'  {sv_s:>6}  {ev_s:>8}  {eh_s:>8}'
        print(row)


# ---------------------------------------------------------------------------
# Main — demonstrates both IACPN and optimizer grid runs
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    base = Params(
        m_pay=50, m_prop=495, m_str=50, isp=260, thrust_kn=31, burn_max=20,
        cd=0.4, cd_fall=1.2, cl=0.0, cd_ctrl=0.3, diam=0.5,
        launch_angle=45, launch_azimuth=45, a_lat_max=3.0,
    )

    hit_points = [
        (10, 10, 0),   # NE   ~14 km
        ( 0, 12, 0),   # N    ~12 km
        ( 8,  3, 0),   # ESE  ~9 km
    ]
    hit_angles = [
        (-45, None),
        (-45, 45.0),
    ]

    # ── IACPN grid ─────────────────────────────────────────────────────────
    print(f'=== IACPN grid: {len(hit_points)} points × {len(hit_angles)} angles '
          f'= {len(hit_points)*len(hit_angles)} runs ===\n')
    runs_sim = run_grid(base, hit_points, hit_angles=hit_angles,
                        hit_angle_range=10)
    plot_grid(runs_sim, title='IACPN — 3 points × 2 angle constraints')

    # ── Optimizer grid ─────────────────────────────────────────────────────
    print(f'\n=== Optimizer grid: {len(hit_points)} points × {len(hit_angles)} angles '
          f'= {len(hit_points)*len(hit_angles)} runs ===\n')
    runs_opt = run_grid(base, hit_points, hit_angles=hit_angles,
                        optimise=True,
                        opt_kwargs=dict(n_knots=20, a_max_g=3.0,
                                        w_angle_v=0.01, max_iter=400))
    plot_grid(runs_opt, title='Optimizer — 3 points × 2 angle constraints')
