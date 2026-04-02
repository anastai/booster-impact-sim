"""
Quick-start entry point — velocity-frame edition (v2.0).

Run:  python run.py
"""
from sim.simulate import simulate
from sim.physics import Params
from plot import plot_results

params = Params(
    # ── Booster ──────────────────────────────────────
    m_pay        = 50,      # kg
    m_prop       = 200,     # kg
    m_str        = 50,      # kg
    # ── Motor ────────────────────────────────────────
    isp          = 260,     # s
    thrust_kn    = 15,      # kN
    burn_max     = 20,      # s
    # ── Aerodynamics ─────────────────────────────────
    cd           = 0.4,
    cd_fall      = 1.2,
    cl           = 0.0,     # lift coeff (0 = no lift)
    cd_ctrl      = 0.3,
    diam         = 0.5,     # m
    atm          = 'isa',
    # ── Launch ───────────────────────────────────────
    launch_angle   = 45,    # deg — initial γV
    launch_azimuth = 90,    # deg — initial γH (90 = East)
    # ── Velocity-frame dynamics ───────────────────────
    grav_turn       = True,  # gravity turn ON
    grav_turn_v_min = 30.0,  # hold fixed angle until v ≥ 30 m/s
    a_cmd_t         = 0.0,   # tangential accel command  (m/s²)
    a_cmd_nv        = 0.0,   # pitch-normal accel command (m/s²)
    a_cmd_nh        = 0.0,   # yaw-normal accel command   (m/s²)
    # ── ZEM guidance target ───────────────────────────
    x_target     = 5,        # km North
    y_target     = 8,        # km East  (ballistic ~10.5 km, target offset tests ZEM)
    z_target     = 0,        # km altitude
    a_lat_max    = 3.0,      # g  (ZEM saturation limit)
)

result = simulate(params)

print('=== Summary ===')
for k, v in result['summary'].items():
    print(f'  {k:<30} {v}')

plot_results(result, params)
