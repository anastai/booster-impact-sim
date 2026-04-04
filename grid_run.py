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


# ---------------------------------------------------------------------------
# Grid runner
# ---------------------------------------------------------------------------

def run_grid(
    base_params:      Params,
    hit_points:       list,              # [(x_km, y_km, z_km), ...]
    hit_angles:       list | None = None,  # [(gv_deg, gh_deg), ...] or None
    hit_angle_range:  float | None = None, # km — range gate for angle law
) -> list[dict]:
    """
    Simulate for every combination of hit_points × hit_angles.

    Parameters
    ----------
    base_params     : Params — shared parameters for all runs
    hit_points      : list of (x_km, y_km, z_km) tuples
    hit_angles      : None → no angle constraints on any run
                      list of (gv_deg, gh_deg) → cross-product with hit_points.
                      Either angle may be None to leave that axis unconstrained.
    hit_angle_range : range gate (km) passed to all constrained runs

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

    runs = []
    for idx, ((x, y, z), (gv, gh)) in enumerate(
            itertools.product(hit_points, angle_list)):

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
        label = f'({x},{y},{z})km  {angle_str}'

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
      [0] Ground track  (North/East, ★ = target)
      [1] Altitude vs time
      [2] Flight-path angle γV vs time
      [3] Miss distance bar chart + summary table
    """
    n      = len(runs)
    colors = [cm.tab10(i % 10) for i in range(n)]

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(title, fontsize=13, fontweight='bold')

    ax_track  = axes[0, 0]
    ax_alt    = axes[0, 1]
    ax_gv     = axes[1, 0]
    ax_miss   = axes[1, 1]

    short_labels = []

    for run, col in zip(runs, colors):
        s    = run['result']['series']
        sm   = run['result']['summary']
        p    = run['params']
        gv, gh = run['hit_angles']
        hp   = run['hit_point']

        short = f"#{run['index']+1} ({hp[0]},{hp[1]},{hp[2]})"
        if gv is not None or gh is not None:
            short += f' gV={gv} gH={gh}'
        short_labels.append(short)

        # ── Ground track ──────────────────────────────────────────────────
        ax_track.plot(s['x'], s['y'], color=col, linewidth=1.8, label=short)
        ax_track.plot(p.x_target, p.y_target,
                      marker='*', color=col, markersize=12, zorder=5)

        # ── Altitude vs time ──────────────────────────────────────────────
        ax_alt.plot(s['t'], s['h'], color=col, linewidth=1.8, label=short)

        # ── γV vs time ────────────────────────────────────────────────────
        ax_gv.plot(s['t'], s['gamma_v'], color=col, linewidth=1.8, label=short)

    # ── Miss distance bar chart ───────────────────────────────────────────
    x_pos  = range(n)
    misses = [r['result']['summary']['miss_distance_m'] for r in runs]
    bars   = ax_miss.bar(x_pos, misses, color=colors[:n], width=0.6)
    ax_miss.set_xticks(x_pos)
    ax_miss.set_xticklabels([f'#{i+1}' for i in range(n)], fontsize=9)
    ax_miss.set_ylabel('Miss distance (m)')
    ax_miss.set_title('Miss distance per run')
    ax_miss.grid(alpha=0.3, axis='y')
    for bar, val in zip(bars, misses):
        ax_miss.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(misses) * 0.01,
            f'{val:.0f} m', ha='center', va='bottom', fontsize=8,
        )

    # ── Axis formatting ───────────────────────────────────────────────────
    ax_track.set_xlabel('North X (km)'); ax_track.set_ylabel('East Y (km)')
    ax_track.set_title('Ground tracks  (★ = target)')
    ax_track.legend(fontsize=7, loc='best'); ax_track.grid(alpha=0.3)

    ax_alt.set_xlabel('Time (s)'); ax_alt.set_ylabel('Altitude (km)')
    ax_alt.set_title('Altitude profiles')
    ax_alt.legend(fontsize=7, loc='best'); ax_alt.grid(alpha=0.3)

    ax_gv.axhline(0, color='#aaa', linewidth=0.8, linestyle=':')
    ax_gv.set_xlabel('Time (s)'); ax_gv.set_ylabel('γV (deg)')
    ax_gv.set_title('Flight-path angle γV')
    ax_gv.legend(fontsize=7, loc='best'); ax_gv.grid(alpha=0.3)

    plt.tight_layout()
    plt.show()

    # ── Summary table ─────────────────────────────────────────────────────
    hdr = (f"{'#':>3}  {'Target (km)':>18}  {'Hit angles':>18}  "
           f"{'miss_m':>8}  {'γV_impact':>10}  {'γH_impact':>10}  {'range_km':>9}")
    print(f'\n{hdr}')
    print('-' * len(hdr))
    for run in runs:
        sm      = run['result']['summary']
        hp      = run['hit_point']
        gv, gh  = run['hit_angles']
        gv_str  = f'{gv}°' if gv is not None else '—'
        gh_str  = f'{gh}°' if gh is not None else '—'
        ang_str = f'γV={gv_str} γH={gh_str}'
        print(f"{run['index']+1:>3}  "
              f"({hp[0]:5.1f},{hp[1]:5.1f},{hp[2]:4.1f})  "
              f"{ang_str:>18}  "
              f"{sm['miss_distance_m']:>8.1f}  "
              f"{sm['final_gamma_v_deg']:>9.1f}°  "
              f"{sm['final_gamma_h_deg']:>9.1f}°  "
              f"{sm['impact_range_km']:>9.2f}")


# ---------------------------------------------------------------------------
# Main — example usage
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    base = Params(
        m_pay=50, m_prop=495, m_str=50, isp=260, thrust_kn=31, burn_max=20,
        cd=0.4, cd_fall=1.2, cl=0.0, cd_ctrl=0.3, diam=0.5,
        launch_angle=45, launch_azimuth=45, a_lat_max=3.0,
    )

    # ── Example 1: grid of hit points, no angle constraints ───────────────
    print('=== Example 1: hit points only ===')
    hit_points = [
        (10, 10, 0),
        (8,  12, 0),
        (12,  8, 0),
        (10, 10, 1),   # elevated target
    ]
    runs1 = run_grid(base, hit_points)
    plot_grid(runs1, title='Grid — hit points only (no angle constraints)')

    # ── Example 2: hit points × angle constraints ─────────────────────────
    print('\n=== Example 2: hit points × angle constraints ===')
    hit_points2 = [(10, 10, 0), (8, 12, 0)]
    hit_angles2 = [
        (-60, None),   # steep vertical, any azimuth
        (-45, 45),     # 45° dive, from NE
    ]
    runs2 = run_grid(base, hit_points2, hit_angles=hit_angles2, hit_angle_range=15)
    plot_grid(runs2, title='Grid — hit points × angle constraints (range gate 15 km)')
