"""
Quick-start entry point.

Run:  python run.py
"""
from sim.simulate import simulate
from sim.physics import Params
from plot import plot_results

# Define parameters (user-friendly units)
params = Params(
    m_prop       = 50,   # kg
    m_str        = 100,     # kg
    isp          = 230,     # s
    thrust_kn    = 20,     # kN
    burn_max     = 10,      # s
    cd           = 0.4,
    cd_fall      = 0.5,
    cd_ctrl      = 0.3,
    diam         = 1.2,     # m
    atm          = 'isa',
    launch_angle = 50,      # deg
    x_target     = 10,     # km downrange
    y_target     = 10,       # km crossrange
    a_lat_max    = 5.0,     # g
)

result = simulate(params)

print('=== Summary ===')
for k, v in result['summary'].items():
    print(f'  {k:<25} {v}')

plot_results(result, params)
