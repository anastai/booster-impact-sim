"""
Grid runner — sweep hit points and hit angles.

Runs the booster simulation for every combination of hit_points × hit_angles
(Cartesian product). All other parameters are shared from base_params.

Usage (script):
    python grid_run.py

Usage (import):
    from grid_run import run_grid, plot_grid
"""
import math
import itertools
from dataclasses import replace

import matplotlib.pyplot as plt
import matplotlib.cm as cm

from sim.simulate import simulate
from sim.physics import Params

_LINESTYLES = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--']


# ---------------------------------------------------------------------------
# Grid runner
# ---------------------------------------------------------------------------

def run_grid(
    base_params:      Params,
    hit_points:       list,
    hit_angles:       list | None = None,
    hit_angle_range:  float | None = None,
) -> list[dict]:
    """
    Simulate for every combination of hit_points × hit_angles (Cartesian product).

    Parameters
    ----------
    base_params     : Params — shared parameters for all runs
    hit_points      : list of (x_km, y_km, z_km) tuples
    hit_angles      : None → no angle constraints on any run
                      list of (gv_deg, gh_deg) — cross-product with hit_points.
                      Either element may be None to leave that axis unconstrained.
    hit_angle_range : range gate (km) applied to all angle-constrained runs

    Returns
    -------
    list of dicts, one per run:
        'index'      : int
        'hit_point'  : (x, y, z) km
        'hit_angles' : (gv_deg, gh_deg) or (None, None)
        'params'     : Params used
        'result'     : output dict from simulate()
        'label'      : short human-readable string
    """
    angle_list = hit_angles if hit_angles is not None else [(None, None)]
    total = len(hit_points) * len(angle_list)

    runs = []
    for idx, ((x, y, z), (gv, gh)) in enumerate(
            itertools.product(hit_points, angle_list)):

        print(f'  Run {idx+1}/{total}:  target=({x},{y},{z}) km  '
              f'angles=(γV={gv}, γH={gh})')

        p = replace(
            base_params,
            x_target        = x,
            y_target        = y,
            z_target        = z,
            hit_gamma_v     = gv,
            hit_gamma_h     = gh,
            hit_angle_range = hit_angle_range,
        )

        angle_str = (f'γV={gv}° γH={gh}°'
                     if (gv is not None or gh is not None) else 'no angle')
        label = f'#{idx+1} ({x},{y},{z})km {angle_str}'

        runs.append({
            'index':      idx,
            'hit_point':  (x, y, z),
            'hit_angles': (gv, gh),
            'params':     p,
            'result':     simulate(p),
            'label':      label,
        })

    return runs


# ---------------------------------------------------------------------------
# Grid plot
# ---------------------------------------------------------------------------

def plot_grid(runs: list[dict], title: str = 'Grid run results') -> None:
    """
    Four-panel dashboard for all grid runs:
      [0] Ground track  (North/East) — ★ target, ● burnout point
      [1] Altitude vs time           — ● burnout point
      [2] Flight-path angle γV vs time
      [3] Miss distance bar chart

    Prints a summary table to console after the plot.
    """
    n      = len(runs)
    colors = [cm.tab10(i % 10) for i in range(n)]

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'{title}  ({n} runs)', fontsize=13, fontweight='bold')

    ax_track = axes[0, 0]
    ax_alt   = axes[0, 1]
    ax_gv    = axes[1, 0]
    ax_miss  = axes[1, 1]

    # launch marker — same for all runs
    ax_track.plot(0, 0, 'k^', markersize=10, zorder=10, label='Launch')

    for run, col, ls in zip(runs, colors, _LINESTYLES):
        s   = run['result']['series']
        sm  = run['result']['summary']
        p   = run['params']
        hp  = run['hit_point']
        lbl = run['label']

        # find burnout index in series
        bo_t  = sm['burnout_time_s']
        bo_idx = next((i for i, t in enumerate(s['t']) if t >= bo_t), len(s['t']) - 1)

        # ── Ground track ──────────────────────────────────────────────────
        ax_track.plot(s['x'], s['y'], color=col, linewidth=2.0,
                      linestyle=ls, label=lbl)
        # burnout marker
        ax_track.plot(s['x'][bo_idx], s['y'][bo_idx],
                      'o', color=col, markersize=7, zorder=6)
        # target marker
        ax_track.plot(p.x_target, p.y_target,
                      '*', color=col, markersize=14, zorder=7)

        # ── Altitude vs time ──────────────────────────────────────────────
        ax_alt.plot(s['t'], s['h'], color=col, linewidth=2.0,
                    linestyle=ls, label=lbl)
        ax_alt.plot(s['t'][bo_idx], s['h'][bo_idx],
                    'o', color=col, markersize=7, zorder=6)

        # ── γV vs time ────────────────────────────────────────────────────
        ax_gv.plot(s['t'], s['gamma_v'], color=col, linewidth=2.0,
                   linestyle=ls, label=lbl)
        ax_gv.plot(s['t'][bo_idx], s['gamma_v'][bo_idx],
                   'o', color=col, markersize=7, zorder=6)

    # burnout legend proxy
    ax_track.plot([], [], 'ko', markersize=7, label='Burnout')

    # ── Miss distance bar chart ───────────────────────────────────────────
    x_pos  = list(range(n))
    misses = [r['result']['summary']['miss_distance_m'] for r in runs]
    bars   = ax_miss.bar(x_pos, misses, color=colors[:n], width=0.6)
    ax_miss.set_xticks(x_pos)
    ax_miss.set_xticklabels([f'#{i+1}' for i in range(n)], fontsize=9)
    ax_miss.set_ylabel('Miss distance (m)')
    ax_miss.set_title('Miss distance per run')
    ax_miss.grid(alpha=0.3, axis='y')
    top = max(misses) if max(misses) > 0 else 1
    for bar, val in zip(bars, misses):
        ax_miss.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + top * 0.02,
                     f'{val:.0f} m', ha='center', va='bottom', fontsize=8)

    # ── Axis formatting ───────────────────────────────────────────────────
    ax_track.set_xlabel('North X (km)'); ax_track.set_ylabel('East Y (km)')
    ax_track.set_title('Ground tracks  (▲=launch  ●=burnout  ★=target)')
    ax_track.legend(fontsize=7, loc='best'); ax_track.grid(alpha=0.3)
    ax_track.set_aspect('equal', adjustable='datalim')

    ax_alt.set_xlabel('Time (s)'); ax_alt.set_ylabel('Altitude (km)')
    ax_alt.set_title('Altitude profiles  (●=burnout)')
    ax_alt.legend(fontsize=7, loc='best'); ax_alt.grid(alpha=0.3)

    ax_gv.axhline(0, color='#aaa', linewidth=0.8, linestyle=':')
    ax_gv.set_xlabel('Time (s)'); ax_gv.set_ylabel('γV (deg)')
    ax_gv.set_title('Flight-path angle γV  (●=burnout)')
    ax_gv.legend(fontsize=7, loc='best'); ax_gv.grid(alpha=0.3)

    plt.tight_layout()
    plt.show()

    # ── Summary table ─────────────────────────────────────────────────────
    print(f'\n{"#":>3}  {"Target (km)":>18}  {"Hit angles":>18}  '
          f'{"miss_m":>8}  {"γV_impact":>10}  {"γH_impact":>10}  {"range_km":>9}')
    print('-' * 85)
    for run in runs:
        sm     = run['result']['summary']
        hp     = run['hit_point']
        gv, gh = run['hit_angles']
        gv_s   = f'{gv}°' if gv is not None else '—'
        gh_s   = f'{gh}°' if gh is not None else '—'
        print(f"{run['index']+1:>3}  "
              f"({hp[0]:5.1f},{hp[1]:5.1f},{hp[2]:4.1f})  "
              f"{'γV='+gv_s+' γH='+gh_s:>18}  "
              f"{sm['miss_distance_m']:>8.1f}  "
              f"{sm['final_gamma_v_deg']:>9.1f}°  "
              f"{sm['final_gamma_h_deg']:>9.1f}°  "
              f"{sm['impact_range_km']:>9.2f}")


# ---------------------------------------------------------------------------
# Main — single example with well-spread hit points
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    base = Params(
        m_pay=50, m_prop=495, m_str=50, isp=260, thrust_kn=31, burn_max=20,
        cd=0.4, cd_fall=1.2, cl=0.0, cd_ctrl=0.3, diam=0.5,
        launch_angle=45, launch_azimuth=45, a_lat_max=3.0,
    )


    # Hit points at very different ranges AND azimuths — maximally distinct paths
    hit_points = [
        ( 5,  0, 0),   # close East   ~5 km
        (10, 10, 0),   # medium NE    ~14 km
        ( 0, 12, 0),   # medium North ~12 km
        ( 8,  3, 0),   # ESE          ~9 km
    ]
    hit_angles = [
        (-70, None),   # steep dive
        (-45, None),   # shallow dive
    ]

    print(f'Running grid: {len(hit_points)} hit points × {len(hit_angles)} angle pairs'
          f' = {len(hit_points)*len(hit_angles)} runs\n')

    runs = run_grid(base, hit_points, hit_angles=hit_angles, hit_angle_range=10)
    plot_grid(runs, title='Grid — 4 hit points × 2 angle constraints')
