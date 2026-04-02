"""
Plot simulation results using matplotlib.

Usage:
    from sim.simulate import simulate
    from sim.physics import Params
    from plot import plot_results

    result = simulate(Params(x_target=120.0, y_target=20.0, a_lat_max=1.5))
    plot_results(result, Params(x_target=120.0, y_target=20.0))
"""
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from sim.physics import Params


def plot_results(result: dict, params: Params = None) -> None:
    """
    Display a 6-panel figure matching the HTML dashboard:
      - Vertical trajectory  (x vs altitude)
      - Top-down ground track (x vs y)
      - Altitude vs time
      - Velocity vs time
      - Drag vs thrust vs time
      - Control effort vs time
    """
    s   = result['series']
    sm  = result['summary']
    t   = s['t']

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"Booster Impact Tracker — miss {sm['miss_distance_m']:.0f} m  |  "
        f"max alt {sm['max_altitude_km']:.1f} km  |  "
        f"impact ({sm['impact_x_km']:.1f}, {sm['impact_y_km']:.1f}) km",
        fontsize=11,
    )
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.38, wspace=0.32)

    # --- Panel 0: Vertical trajectory ---
    ax0 = fig.add_subplot(gs[0, 0])
    ax0.plot(s['x'], s['h'], color='#378ADD', linewidth=1.5, label='Trajectory')
    if params is not None:
        ax0.scatter([params.x_target], [0], color='#D85A30', marker='x',
                    s=120, zorder=5, label='Target')
    ax0.set_xlabel('Downrange X (km)')
    ax0.set_ylabel('Altitude (km)')
    ax0.set_title('Vertical trajectory')
    ax0.legend(fontsize=8)
    ax0.grid(alpha=0.3)

    # --- Panel 1: Top-down ground track ---
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(s['x'], s['y'], color='#1D9E75', linewidth=1.5, label='Ground track')
    if params is not None:
        ax1.scatter([params.x_target], [params.y_target], color='#D85A30',
                    marker='x', s=120, zorder=5, label='Target')
    ax1.set_xlabel('Downrange X (km)')
    ax1.set_ylabel('Crossrange Y (km)')
    ax1.set_title('Top-down view')
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # --- Panel 2: Altitude vs time ---
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.fill_between(t, s['h'], alpha=0.15, color='#378ADD')
    ax2.plot(t, s['h'], color='#378ADD', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (km)')
    ax2.set_title('Altitude')
    ax2.grid(alpha=0.3)

    # --- Panel 3: Velocity vs time ---
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.fill_between(t, s['v'], alpha=0.15, color='#D85A30')
    ax3.plot(t, s['v'], color='#D85A30', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Velocity')
    ax3.grid(alpha=0.3)

    # --- Panel 4: Drag vs thrust ---
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, s['thr'],      color='#D85A30', linewidth=1.5, label='Thrust')
    ax4.plot(t, s['drag_aero'],color='#D4537E', linewidth=1.5, linestyle='--', label='Aero drag')
    ax4.plot(t, s['drag_ctrl'],color='#E8A020', linewidth=1.5, linestyle=':',  label='Actuator drag')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('kN')
    ax4.set_title('Drag vs thrust')
    ax4.legend(fontsize=8)
    ax4.grid(alpha=0.3)

    # --- Panel 5: Control effort ---
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.fill_between(t, s['aLat'], alpha=0.18, color='#1D9E75')
    ax5.plot(t, s['aLat'],  color='#1D9E75', linewidth=1.5, label='|aLat|')
    ax5.plot(t, s['aLat_x'],color='#378ADD', linewidth=1.0, linestyle='--', label='X component')
    ax5.plot(t, s['aLat_y'],color='#7F77DD', linewidth=1.0, linestyle='--', label='Y component')
    if params is not None:
        lim = params.a_lat_max
        ax5.axhline( lim, color='#D85A30', linewidth=1.0, linestyle=':', label=f'±{lim:.1f} g limit')
        ax5.axhline(-lim, color='#D85A30', linewidth=1.0, linestyle=':')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('g')
    ax5.set_title('Control effort')
    ax5.legend(fontsize=8)
    ax5.grid(alpha=0.3)

    plt.show()


if __name__ == '__main__':
    from sim.simulate import simulate
    p = Params(x_target=120.0, y_target=20.0, a_lat_max=1.5)
    r = simulate(p)
    print('Summary:', r['summary'])
    plot_results(r, p)
