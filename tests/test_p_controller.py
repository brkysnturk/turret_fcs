"""Uretim P kontrolcusu testleri."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.p_controller import PController


def test_default_controller_is_wired_into_manager():
    manager = FCSManager()
    assert isinstance(manager.p_controller, PController)
    assert math.isclose(manager.p_controller.kp_frame_pan, CONFIG.control.kp_pan, abs_tol=1e-6)
    assert math.isclose(manager.p_controller.kp_frame_tilt, CONFIG.control.kp_tilt, abs_tol=1e-6)


def test_soft_deadband_gain_schedule_and_step_limit():
    p = PController()
    assert p.compute_p(0.5, 0.5, dt=0.03333) == (0.0, 0.0)

    limit = p.max_step_for(0.03333)
    u_pan, u_tilt = p.compute_p(30.0, 20.0, dt=0.03333)
    assert 0.0 < u_pan <= limit
    assert 0.0 < u_tilt <= limit

    u_small, _ = PController().compute_p(3.0, 0.0, dt=0.03333)
    u_large, _ = PController().compute_p(8.0, 0.0, dt=0.03333)
    assert u_large > u_small > 0.0


def test_command_acceleration_and_dt_scaling():
    dt = 0.03333
    p = PController(cmd_accel_pan_deg_s2=10.0)
    u1, _ = p.compute_p(30.0, 0.0, dt=dt)
    u2, _ = p.compute_p(30.0, 0.0, dt=dt)
    assert u1 == pytest.approx(10.0 * dt, abs=1e-6)
    assert u2 - u1 == pytest.approx(10.0 * dt, abs=1e-6)

    slow, _ = PController().compute_p(3.0, 0.0, dt=0.03333)
    fast, _ = PController().compute_p(3.0, 0.0, dt=0.01667)
    assert fast == pytest.approx(slow / 2.0, rel=0.02)


def test_no_velocity_feedforward():
    p = PController()
    assert p.compute_velocity_ff(10.0, 10.0, 0.03333) == (0.0, 0.0)
    u_plain, _ = PController().compute_p(5.0, 3.0, dt=0.03333)
    u_vel, _ = PController().compute_p(
        5.0, 3.0, dt=0.03333, omega_pan_deg_s=10.0, omega_tilt_deg_s=10.0
    )
    assert u_vel == pytest.approx(u_plain)


def test_step_limit_tracks_frame_rate():
    for dt, expected in ((0.03333, 1.50), (0.0622, 1.50), (0.01667, 0.75)):
        assert PController().max_step_for(dt) == pytest.approx(expected, abs=0.01)


def test_soft_deadband_and_output_clipping():
    p = PController()
    assert p.compute_p(0.50, 0.50, dt=0.033) == (0.0, 0.0)
    u_pan, u_tilt = p.compute_p(30.0, 20.0, dt=0.033)
    assert 0.0 < u_pan <= p.max_output
    assert 0.0 < u_tilt <= p.max_output


def test_angles_and_reset():
    p = PController()
    pan, tilt = p.compute_angles(740.0, 300.0, 640.0, 360.0, 1411.0, 1411.0)
    assert pan > 0.0
    assert tilt > 0.0
    p.compute_p(pan, tilt, dt=0.03333)
    p.reset()
    assert (p.actual_pan, p.actual_tilt, p.smooth_cmd_pan, p.smooth_cmd_tilt) == (0.0, 0.0, 0.0, 0.0)
