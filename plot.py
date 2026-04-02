"""
Plot simulation results using matplotlib — velocity-frame edition (v2.0).

Usage:
    from sim.simulate import simulate
    from sim.physics import Params
    from plot import plot_results

    result = simulate(Params(x_target=120.0, y_target=20.0, grav_turn=True))
    plot_results(result, Params(x_target=120.0, y_target=20.0))
"""
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from sim.physics import Params


def plot_results(result: dict, params: Params = None) -> None:
    """
    Display a 7-panel figure matching the HTML dashboard (v2.0):
      [0] Vertical trajectory  (downrange vs altitude)
      [1] Top-down ground track (x vs y)
      [2] Altitude vs time
      [3] Velocity vs time
      [4] Forces vs time  (thrust + aero-drag + lift + actuator-drag)
      [5] Acceleration vs time
      [6] Flight angles  (γV and γH vs time)   ← NEW in v2.0
    """
    s  = result['series']
    sm = result['summary']
    t  = s['t']

    # Downrange (horizontal distance) for vertical trajectory panel
    import math
    downrange_km = [math.sqrt(xi**2 + yi**2) for xi, yi in zip(s['x'], s['y'])]

    fig = plt.figure(figsize=(18, 11))
    fig.suptitle(
        f"Booster Impact Tracker v2.0  —  "
        f"miss {sm['miss_distance_m']:.0f} m  |  "
        f"max alt {sm['max_altitude_km']:.1f} km  |  "
        f"impact range {sm['impact_range_km']:.1f} km  |  "
        f"γV burnout {sm['burnout_gamma_v_deg']:.1f}°",
        fontsize=11,
    )

    # 3 rows × 3 cols; last cell left empty
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.42, wspace=0.33)

    # ── [0] Vertical trajectory ──────────────────────────────────────────
    ax0 = fig.add_subplot(gs[0, 0])
    ax0.plot(downrange_km, s['h'], color='#378ADD', linewidth=1.5, label='Trajectory')
    if params is not None:
        target_downrange = math.sqrt(params.x_target**2 + params.y_target**2)
        ax0.scatter([target_downrange], [0], color='#D85A30',
                    marker='x', s=120, zorder=5, label='Target')
    ax0.set_xlabel('Downrange √(x²+y²) (km)')
    ax0.set_ylabel('Altitude (km)')
    ax0.set_title('Vertical trajectory')
    ax0.legend(fontsize=8)
    ax0.grid(alpha=0.3)

    # ── [1] Top-down ground track ────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(s['x'], s['y'], color='#1D9E75', linewidth=1.5, label='Ground track')
    if params is not None:
        ax1.scatter([params.x_target], [params.y_target], color='#D85A30',
                    marker='x', s=120, zorder=5, label='Target')
    ax1.set_xlabel('North X (km)')
    ax1.set_ylabel('East Y (km)')
    ax1.set_title('Top-down view')
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # ── [2] Altitude vs time ─────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.fill_between(t, s['h'], alpha=0.15, color='#378ADD')
    ax2.plot(t, s['h'], color='#378ADD', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (km)')
    ax2.set_title('Altitude')
    ax2.grid(alpha=0.3)

    # ── [3] Velocity vs time ─────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.fill_between(t, s['v'], alpha=0.15, color='#D85A30')
    ax3.plot(t, s['v'], color='#D85A30', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Velocity')
    ax3.grid(alpha=0.3)

    # ── [4] Forces vs time ───────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, s['thr'],       color='#D85A30', linewidth=1.8, label='Thrust')
    ax4.plot(t, s['drag_aero'], color='#D4537E', linewidth=1.5,
             linestyle='--', label='Aero drag')
    ax4.plot(t, s['lift'],      color='#1D9E75', linewidth=1.5,
             linestyle='-.',  label='Lift')          # NEW
    ax4.plot(t, s['drag_ctrl'], color='#E8A020', linewidth=1.2,
             linestyle=':',   label='Actuator drag')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('kN')
    ax4.set_title('Forces (velocity frame)')
    ax4.legend(fontsize=8)
    ax4.grid(alpha=0.3)

    # ── [5] Acceleration vs time ─────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.fill_between(t, s['acc'], alpha=0.15, color='#7F77DD')
    ax5.plot(t, s['acc'], color='#7F77DD', linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('g')
    ax5.set_title('Total acceleration')
    ax5.grid(alpha=0.3)

    # ── [6] Flight angles γV and γH vs time  (NEW) ───────────────────────
    ax6 = fig.add_subplot(gs[2, 0])
    ax6.plot(t, s['gamma_v'], color='#378ADD', linewidth=1.8,
             label='γV — flight-path angle')
    ax6.plot(t, s['gamma_h'], color='#7F77DD', linewidth=1.5,
             linestyle='--', label='γH — heading azimuth')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('degrees')
    ax6.set_title('Flight angles (velocity frame)')
    ax6.legend(fontsize=8)
    ax6.grid(alpha=0.3)

    # ── [7] Control effort (ZEM lateral) ────────────────────────────────
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.fill_between(t, s['aLat'], alpha=0.18, color='#1D9E75')
    ax7.plot(t, s['aLat'], color='#1D9E75', linewidth=1.5, label='|aLat|')
    ax7.plot(t, s['a_nV'], color='#378ADD', linewidth=1.0,
             linestyle='--', label='ê_nV (pitch)')
    ax7.plot(t, s['a_nH'], color='#D4537E', linewidth=1.0,
             linestyle='--', label='ê_nH (yaw)')
    if params is not None:
        lim = params.a_lat_max
        ax7.axhline( lim, color='#D85A30', linewidth=1.0,
                     linestyle=':', label=f'±{lim:.1f} g limit')
        ax7.axhline(-lim, color='#D85A30', linewidth=1.0, linestyle=':')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('g')
    ax7.set_title('ZEM control effort')
    ax7.legend(fontsize=8)
    ax7.grid(alpha=0.3)

    # ── [8] Summary text box ─────────────────────────────────────────────
    ax8 = fig.add_subplot(gs[2, 2])
    ax8.axis('off')
    lines = [
        f"Max altitude:        {sm['max_altitude_km']:.1f} km",
        f"Max velocity:        {sm['max_velocity_ms']:.0f} m/s",
        f"Max lift:            {sm['max_lift_kn']:.3f} kN",
        f"Burnout altitude:    {sm['burnout_altitude_km']:.1f} km",
        f"Burnout time:        {sm['burnout_time_s']:.1f} s",
        f"γV at burnout:       {sm['burnout_gamma_v_deg']:.1f}°",
        f"Impact range:        {sm['impact_range_km']:.1f} km",
        f"Impact (x,y):        ({sm['impact_x_km']:.1f}, {sm['impact_y_km']:.1f}) km",
        f"Miss distance:       {sm['miss_distance_m']:.1f} m",
        f"Flight time:         {sm['flight_time_s']:.1f} s",
        f"Final γV:            {sm['final_gamma_v_deg']:.1f}°",
        f"Final γH:            {sm['final_gamma_h_deg']:.1f}°",
    ]
    ax8.text(0.05, 0.95, '\n'.join(lines),
             transform=ax8.transAxes,
             fontsize=9, verticalalignment='top',
             fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='#f0efe8', alpha=0.8))
    ax8.set_title('Summary', fontsize=10)

    plt.show()


if __name__ == '__main__':
    from sim.simulate import simulate
    p = simulate(Params(x_target=120.0, y_target=20.0, grav_turn=True, a_lat_max=1.5))
    print('Summary:', p['summary'])
    plot_results(p, Params(x_target=120.0, y_target=20.0, a_lat_max=1.5))
