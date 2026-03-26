"""
Quick-start entry point.

Run:  python run.py
"""
from sim.simulate import simulate
from sim.physics import Params
from plot import plot_results

# Define parameters (user-friendly units)
params = Params(
    m_prop       = 5_000,   # kg
    m_str        = 800,     # kg
    isp          = 260,     # s
    thrust_kn    = 120,     # kN
    burn_max     = 60,      # s
    cd           = 0.4,
    cd_fall      = 1.2,
    cd_ctrl      = 0.3,
    diam         = 1.2,     # m
    atm          = 'isa',
    launch_angle = 75,      # deg
    x_target     = 100,     # km downrange
    y_target     = 0,       # km crossrange
    a_lat_max    = 1.0,     # g
)

result = simulate(params)

print('=== Summary ===')
for k, v in result['summary'].items():
    print(f'  {k:<25} {v}')

plot_results(result, params)
