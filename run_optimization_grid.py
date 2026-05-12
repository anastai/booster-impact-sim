"""
run_optimization_grid.py
------------------------
Grid run of the collocation optimizer over multiple target points.

For each point:
  - Runs collocation_solve with the softplus-fence density merit
  - Prints a summary line
  - Saves one sheet per trajectory in grid_optimization_states.xlsx
    columns: time_s, x_km, y_km, z_km, vel_m_s,
             gamma_h_deg, gamma_v_deg, accV_g, accH_g, mass_kg
  - Saves a 6-panel figure: ground track, altitude, γV, γH, a_nV, a_nH

Usage:
    python3 run_optimization_grid.py
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import replace
from openpyxl import Workbook

from sim.collocate import collocation_solve, DensityParams
from sim.physics import Params, G0

# ── Base vehicle (everything except target / angles) ────────────────────────
BASE_PARAMS = Params(
    m_pay=50, m_prop=495, m_str=50, isp=260, thrust_kn=31, burn_max=20,
    cd=0.4, cd_fall=1.2, cl=0.0, cd_ctrl=0.3, diam=0.5,
    launch_angle=45, launch_azimuth=45,          # azimuth overridden per point
    x_target=10, y_target=10, z_target=0,        # overridden per point
    a_lat_max=3.0,
    hit_gamma_v=-45.0, hit_gamma_h=45.0,         # overridden per point
)

# ── Density merit parameters (shared) ───────────────────────────────────────
RANGE_DP = DensityParams(fenceX=0.05, fenceY=0.3,  betaX=5.0,  betaY=3.0, fenceC1=1.0, fenceC2=0.5)
ACC_DP   = DensityParams(fenceX=0.3,  fenceY=0.7,  betaX=5.0,  betaY=3.0, fenceC1=1.0, fenceC2=2.0)
MISS_DP  = DensityParams(fenceX=0.0,  fenceY=0.1,  betaX=10.0, betaY=5.0, fenceC1=1.0, fenceC2=2.0)

# ── Grid: (x_km, y_km, z_km, hit_gamma_v_deg, hit_gamma_h_deg) ─────────────
GRID = [
    (10, 10,  0, -45,  45),
    (15,  5,  0, -60,  18),
    ( 8, 12,  0, -45,  56),
]


# ── Helpers ──────────────────────────────────────────────────────────────────

def mass_profile(t_arr: np.ndarray, params: Params) -> np.ndarray:
    """Reconstruct mass at each time node from vehicle parameters."""
    thrust_n  = params.thrust_kn * 1_000.0
    m_dot     = thrust_n / (params.isp * G0)
    m_tot     = params.m_pay + params.m_prop + params.m_str
    prop_used = min(params.m_prop, m_dot * params.burn_max)
    m_bo      = params.m_pay + params.m_str + (params.m_prop - prop_used)
    t_burn    = prop_used / m_dot
    return np.where(t_arr <= t_burn, m_tot - m_dot * t_arr, m_bo)


# ── Main grid runner ─────────────────────────────────────────────────────────

def run_grid():
    runs = []   # list of (params, result, sheet_label)

    for idx, (x, y, z, gv, gh) in enumerate(GRID):
        label = f"run{idx+1:02d}_x{x}_y{y}_gv{gv}"
        print(f"\n{'='*60}")
        print(f"[{idx+1}/{len(GRID)}]  target=({x}, {y}, {z}) km   "
              f"γV={gv}°  γH={gh}°")
        print('='*60)

        # Point the launch azimuth roughly toward the target
        az = math.degrees(math.atan2(y, x))
        p  = replace(BASE_PARAMS,
                     x_target=x, y_target=y, z_target=z,
                     hit_gamma_v=float(gv), hit_gamma_h=float(gh),
                     launch_azimuth=az)

        result = collocation_solve(
            p,
            N1=15, N2=30,
            angle_tol_deg=5.0,
            final_line_tol_deg=5.0,
            final_line_togo_m=500.0,
            w_range=1.0, w_acc=0.5, w_miss_d=2.0,
            range_dp=RANGE_DP, acc_dp=ACC_DP, miss_dp=MISS_DP,
            ipopt_opts={'ipopt.print_level': 0, 'print_time': False},
        )

        s = result['summary']
        print(f"  quality_ok={s['quality_ok']}  "
              f"miss={s['miss_m']} m  "
              f"final_line={s.get('final_line_km', '?')} km  "
              f"t_f={s['flight_time_s']} s")

        runs.append((p, result, label))

    _plot(runs)
    _save_excel(runs)


# ── Plotting ─────────────────────────────────────────────────────────────────

def _plot(runs):
    colors = plt.cm.tab10.colors
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('Collocation grid optimisation', fontsize=13, fontweight='bold')

    ax_gt, ax_alt = axes[0]
    ax_gv, ax_gh  = axes[1]
    ax_av, ax_ah  = axes[2]

    ax_gt.scatter([0], [0], c='green', marker='^', s=120, zorder=9, label='Launch')

    for i, (p, result, label) in enumerate(runs):
        c    = colors[i % len(colors)]
        ser  = result['series']
        summ = result['summary']
        t    = ser['t']

        legend_lbl = (f"{label}  miss={summ['miss_m']} m  "
                      f"FL={summ.get('final_line_km','?')} km")

        # Ground track
        ax_gt.plot(ser['x'], ser['y'], color=c, lw=2)
        ax_gt.scatter([ser['x'][-1]], [ser['y'][-1]], color=c, s=60, zorder=5)
        ax_gt.scatter([p.x_target], [p.y_target],
                      color=c, marker='x', s=140, linewidths=2.5, zorder=6,
                      label=legend_lbl)

        # Other panels
        ax_alt.plot(t, ser['h'],           color=c, lw=2, label=label)
        ax_gv.plot( t, ser['gamma_v_deg'], color=c, lw=2, label=label)
        ax_gh.plot( t, ser['gamma_h_deg'], color=c, lw=2, label=label)
        ax_av.plot( t, ser['a_nV_g'],      color=c, lw=2, label=label)
        ax_ah.plot( t, ser['a_nH_g'],      color=c, lw=2, label=label)

        # Burnout marker on altitude plot
        ax_alt.axvline(summ['burnout_time_s'], color=c, ls=':', alpha=0.5)

    panels = [
        (ax_gt,  'Ground Track',            'East (km)',    'North (km)'),
        (ax_alt, 'Altitude vs Time',        'Altitude (km)', 'Time (s)'),
        (ax_gv,  'Flight-path angle γV',   'γV (deg)',      'Time (s)'),
        (ax_gh,  'Heading γH',             'γH (deg)',      'Time (s)'),
        (ax_av,  'Pitch acceleration a_nV', 'a_nV (g)',     'Time (s)'),
        (ax_ah,  'Yaw acceleration a_nH',   'a_nH (g)',     'Time (s)'),
    ]
    for ax, title, ylabel, xlabel in panels:
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

    ax_gt.set_aspect('equal')

    plt.tight_layout()
    fname = 'grid_optimization_plot.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"\nPlot saved  →  {fname}")


# ── Excel export ─────────────────────────────────────────────────────────────

def _save_excel(runs):
    wb = Workbook()
    wb.remove(wb.active)   # drop the default blank sheet

    headers = ['time_s', 'x_km', 'y_km', 'z_km',
               'vel_m_s', 'gamma_h_deg', 'gamma_v_deg',
               'accV_g', 'accH_g', 'mass_kg']

    for p, result, label in runs:
        ser  = result['series']
        t    = np.array(ser['t'])
        mass = mass_profile(t, p)

        ws = wb.create_sheet(title=label[:31])  # sheet names capped at 31 chars
        ws.append(headers)

        for k in range(len(t)):
            ws.append([
                round(float(t[k]),              4),
                round(float(ser['x'][k]),       4),
                round(float(ser['y'][k]),       4),
                round(float(ser['h'][k]),       4),
                round(float(ser['v'][k]),       3),
                round(float(ser['gamma_h_deg'][k]), 4),
                round(float(ser['gamma_v_deg'][k]), 4),
                round(float(ser['a_nV_g'][k]),  6),
                round(float(ser['a_nH_g'][k]),  6),
                round(float(mass[k]),           3),
            ])

    fname = 'grid_optimization_states.xlsx'
    wb.save(fname)
    print(f"Excel saved →  {fname}")


if __name__ == '__main__':
    run_grid()
