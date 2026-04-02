"""
Check vectors for the booster simulation — v2.0 (velocity frame).

Run with:  pytest tests/check_vectors.py -v

Test baseline assumptions
--------------------------
make() defaults to launch_azimuth=0 (North = +x direction) and
grav_turn=False (fixed body angle, engine always completes full burn_max).
This matches the old Cartesian-state convention so that:
  - x = downrange (North)
  - y = crossrange (East)
  - engine burnout always occurs → ZEM guidance always activates post-burnout
"""
import math
import pytest
from sim.simulate import simulate
from sim.physics import Params


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make(base: Params = None, **overrides) -> Params:
    """
    Return a Params instance with selected fields overridden.

    Defaults used here:
      launch_azimuth = 0   → booster flies North (+x), matching test expectations
                             that treat x as the downrange axis.
      grav_turn      = False → fixed body angle; engine always runs to burn_max,
                               guaranteeing burnout occurs and ZEM guidance activates.
    """
    from dataclasses import replace
    p = base if base is not None else Params(
        launch_azimuth=0.0,   # fly North so impact_x_km is the natural downrange metric
        grav_turn=False,      # fixed angle → predictable burnout → guidance always active
    )
    return replace(p, **overrides)


# ---------------------------------------------------------------------------
# CV-1: Determinism
# ---------------------------------------------------------------------------

class TestDeterminism:
    def test_same_params_same_result(self):
        """Identical inputs must produce identical outputs."""
        p  = make()
        r1 = simulate(p)
        r2 = simulate(p)
        assert r1['summary'] == r2['summary']


# ---------------------------------------------------------------------------
# CV-2: Physics invariants
# ---------------------------------------------------------------------------

class TestPhysicsInvariants:
    def test_vertical_launch_zero_crossrange(self):
        """
        90° launch with azimuth=0, grav_turn=False, no crossrange target and
        zero lateral limit → booster must stay on the x-z plane (y ≈ 0).

        With grav_turn=False and launch_angle=90°:
          alpha = launch_angle - gamma_v = 90° - 90° = 0° → T_nV = 0
          dgamma_v/dt = 0 → gamma_v stays at 90° throughout
          gamma_h = 0 → all horizontal motion in x, none in y.
        """
        p = make(launch_angle=90.0, y_target=0.0, a_lat_max=0.0)
        r = simulate(p)
        assert abs(r['summary']['impact_y_km']) < 0.01   # < 10 m drift

    def test_zero_propellant_stays_on_ground(self):
        """No propellant → booster cannot rise significantly."""
        p = make(m_prop=0.0, burn_max=0.0, a_lat_max=0.0)
        r = simulate(p)
        assert r['summary']['max_altitude_km'] < 0.01

    def test_vacuum_zero_aero_drag(self):
        """In vacuum the aerodynamic drag series must be all zeros."""
        p = make(atm='none', a_lat_max=0.0)
        r = simulate(p)
        assert all(d == 0.0 for d in r['series']['drag_aero'])

    def test_altitude_non_negative(self):
        """Altitude must never go below ground."""
        p = make()
        r = simulate(p)
        assert all(h >= 0.0 for h in r['series']['h'])

    def test_burnout_altitude_below_max(self):
        """Burnout happens during ascent, so burnout altitude ≤ max altitude."""
        p = make()
        r = simulate(p)
        s = r['summary']
        assert s['burnout_altitude_km'] <= s['max_altitude_km']


# ---------------------------------------------------------------------------
# CV-3: Guidance behaviour
# ---------------------------------------------------------------------------

class TestGuidance:
    def test_guidance_reduces_miss(self):
        """
        Guidance should reduce miss when the target is close to the ballistic
        landing point (reachable with finite lateral acceleration).

        Strategy: find ballistic impact x, set target 10 km further, then
        compare guided vs unguided miss to that target.
        """
        # Step 1 — find ballistic landing point (no guidance)
        p_bal = make(a_lat_max=0.0)
        bx = simulate(p_bal)['summary']['impact_x_km']

        # Step 2 — target 10 km beyond ballistic point (clearly reachable)
        target_x = bx + 10.0
        p_guided  = make(a_lat_max=2.0, x_target=target_x)
        p_no_guid = make(a_lat_max=0.0, x_target=target_x)

        miss_guided = simulate(p_guided)['summary']['miss_distance_m']
        miss_no     = simulate(p_no_guid)['summary']['miss_distance_m']
        assert miss_guided < miss_no

    def test_lateral_accel_never_exceeds_limit(self):
        """Lateral acceleration magnitude must always respect the saturation limit."""
        a_lim = 1.5   # g
        p = make(a_lat_max=a_lim)
        r = simulate(p)
        for a in r['series']['aLat']:
            assert a <= a_lim + 1e-6, f"aLat {a:.4f} g exceeded limit {a_lim} g"

    def test_crossrange_guidance(self):
        """Guidance should drive the booster toward a non-zero crossrange target."""
        p_no_target   = make(y_target=0.0,  a_lat_max=2.0)
        p_with_target = make(y_target=30.0, a_lat_max=2.0)
        y_no  = simulate(p_no_target  )['summary']['impact_y_km']
        y_hit = simulate(p_with_target)['summary']['impact_y_km']
        # The guided run should land closer to y=30 km than the unguided run
        assert abs(y_hit - 30.0) < abs(y_no - 30.0)

    def test_zero_lateral_limit_is_ballistic(self):
        """a_lat_max=0 → no guidance → aLat series must be all zeros."""
        p = make(a_lat_max=0.0)
        r = simulate(p)
        assert all(a == 0.0 for a in r['series']['aLat'])


# ---------------------------------------------------------------------------
# CV-4: Actuator drag
# ---------------------------------------------------------------------------

class TestActuatorDrag:
    def test_nonzero_cdctrl_produces_control_drag(self):
        """cd_ctrl > 0 with active guidance must produce non-zero actuator drag."""
        p = make(cd_ctrl=0.5, a_lat_max=2.0)
        r = simulate(p)
        assert sum(r['series']['drag_ctrl']) > 0.0

    def test_zero_cdctrl_produces_no_control_drag(self):
        """cd_ctrl = 0 must produce zero actuator drag regardless of guidance."""
        p = make(cd_ctrl=0.0, a_lat_max=2.0)
        r = simulate(p)
        assert all(d == 0.0 for d in r['series']['drag_ctrl'])

    def test_actuator_drag_increases_with_cdctrl(self):
        """Higher cd_ctrl → more total actuator drag."""
        p_low  = make(cd_ctrl=0.1, a_lat_max=2.0)
        p_high = make(cd_ctrl=0.8, a_lat_max=2.0)
        drag_low  = sum(simulate(p_low )['series']['drag_ctrl'])
        drag_high = sum(simulate(p_high)['series']['drag_ctrl'])
        assert drag_high > drag_low
