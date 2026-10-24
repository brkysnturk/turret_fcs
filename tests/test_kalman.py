"""Kalman filtresi: yakınsama, ego telafisi, sıçrama reddi ve çıkışlar."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.kalman_filter import KalmanFilter


def _constant(kf, speed_px_s, frames, dt=0.033, x0=640.0):
    step = speed_px_s * dt
    for i in range(frames):
        kf.update((x0 + step * i, 360.0), dt=dt)


def test_initialization_constant_velocity_and_lead():
    kf = KalmanFilter()
    assert not kf.initialized

    px, py, vx, vy = kf.update((640.0, 360.0), dt=0.033)
    assert kf.initialized
    assert (px, py, vx, vy) == pytest.approx((640.0, 360.0, 0.0, 0.0))

    _constant(kf, 100.0, 61)
    assert kf.raw_vx == pytest.approx(100.0, rel=0.03)
    assert kf.smooth_vx == pytest.approx(100.0, rel=0.03)

    lead_x, _, lead_vx, _ = kf.predict_next_position(lead_time=0.1)
    assert lead_vx == pytest.approx(kf.smooth_vx)
    assert lead_x > float(kf.x[0, 0])

    omega_pan, omega_tilt = kf.get_angular_velocity_deg_s(fx=1000.0, fy=1000.0)
    assert omega_pan == pytest.approx(math.degrees(0.1), rel=0.03)
    assert omega_tilt == 0.0


def test_uneven_dt_is_time_normalized():
    kf = KalmanFilter()
    elapsed = 0.0
    for dt in [0.02, 0.05, 0.03, 0.06, 0.012, 0.04] * 10:
        elapsed += dt
        kf.update((640.0 + 200.0 * elapsed, 360.0), dt=dt)
    assert kf.raw_vx == pytest.approx(200.0, rel=0.03)


def test_ego_shift_creates_no_target_velocity():
    kf = KalmanFilter()
    for i in range(61):
        kf.update((640.0 - 5.0 * i, 360.0), dt=0.033, ego_dx=-5.0)
    assert abs(kf.raw_vx) < 5.0
    assert kf.smooth_vx == 0.0


def test_single_outlier_rejected_persistent_jump_reinitializes():
    kf = KalmanFilter()
    _constant(kf, 100.0, 41)
    velocity_before = kf.smooth_vx

    kf.update((640.0 + 100.0 * 0.033 * 40 + 300.0, 360.0), dt=0.033)
    assert kf.initialized is True
    assert abs(kf.smooth_vx - velocity_before) < 20.0

    for i in range(5):
        kf.update((640.0 + 100.0 * 0.033 * (41 + i), 360.0), dt=0.033)
    assert kf.smooth_vx == pytest.approx(100.0, abs=20.0)

    for _ in range(CONFIG.kalman.reacquire_frames):
        kf.update((1100.0, 360.0), dt=0.033)
    assert kf.initialized is True
    assert float(kf.x[0, 0]) == pytest.approx(1100.0)
    assert kf.raw_vx == 0.0
    assert kf._outlier_frames == 0


def test_reversal_and_soft_deadband():
    kf = KalmanFilter()
    _constant(kf, 150.0, 41)
    assert kf.smooth_vx > 100.0
    for i in range(21):
        kf.update((640.0 - 150.0 * 0.033 * (i + 1), 360.0), dt=0.033)
    assert kf.raw_vx < -100.0
    assert kf.smooth_vx < -100.0

    slow = KalmanFilter()
    for i in range(61):
        slow.update((640.0 + 0.5 * i, 360.0), dt=0.033)
    assert slow.smooth_vx == 0.0

    fast = KalmanFilter()
    for i in range(61):
        fast.update((640.0 + 2.0 * i, 360.0), dt=0.033)
    assert fast.smooth_vx > CONFIG.kalman.velocity_deadband_high_px_s
