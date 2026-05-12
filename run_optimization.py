from sim.collocate import collocation_solve, DensityParams
from sim.physics import Params

p = Params(
    m_pay=50, m_prop=495, m_str=50, isp=260, thrust_kn=31, burn_max=20,
    cd=0.4, cd_fall=1.2, cl=0.0, cd_ctrl=0.3, diam=0.5,
    launch_angle=45, launch_azimuth=45,
    x_target=10, y_target=10, z_target=0, a_lat_max=3.0,
    hit_gamma_v=-45.0,
    hit_gamma_h=45.0,
)

result = collocation_solve(
    p,
    N1=15, N2=30,
    angle_tol_deg=5.0,
    final_line_tol_deg=5.0,
    final_line_togo_m=500.0,
    w_range=1.0,
    w_acc=0.5,
    w_miss_d=2.0,
    range_dp=DensityParams(fenceX=0.05, fenceY=0.3, betaX=5.0, betaY=3.0, fenceC1=1.0, fenceC2=0.5),
    acc_dp  =DensityParams(fenceX=0.3,  fenceY=0.7, betaX=5.0, betaY=3.0, fenceC1=1.0, fenceC2=2.0),
    miss_dp =DensityParams(fenceX=0.0,  fenceY=0.1, betaX=10.0, betaY=5.0, fenceC1=1.0, fenceC2=2.0),
)

print()
print("=== Summary ===")
for k, v in result['summary'].items():
    print(f"  {k:<35} {v}")
