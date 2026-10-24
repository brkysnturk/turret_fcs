"""Kalman filtresi: yakinsama, ego telafisi, aykiri deger reddi ve olu bolge konsolide testleri."""
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


def test_kalman_convergence_lead_and_dt_normalization():
    """Kalman baslatma, sabit hiz yakinsamasi, onleme kestirimi ve degisken dt normalizasyonunu dogrular."""
    kf = KalmanFilter()
    assert not kf.initialized

    # 1. Ilk olcumle baslatma
    px, py, vx, vy = kf.update((640.0, 360.0), dt=0.033)
    assert kf.initialized
    assert (px, py, vx, vy) == pytest.approx((640.0, 360.0, 0.0, 0.0))

    # 2. Sabit hiz yakinsamasi (100 px/s)
    _constant(kf, 100.0, 61)
    assert kf.raw_vx == pytest.approx(100.0, rel=0.03)
    assert kf.smooth_vx == pytest.approx(100.0, rel=0.03)

    # 3. Kinematik onleme (lead) hesabi
    lead_x, _, lead_vx, _ = kf.predict_next_position(lead_time=0.1)
    assert lead_vx == pytest.approx(kf.smooth_vx)
    assert lead_x > float(kf.x[0, 0])

    omega_pan, omega_tilt = kf.get_angular_velocity_deg_s(fx=1000.0, fy=1000.0)
    assert omega_pan == pytest.approx(math.degrees(0.1), rel=0.03)
    assert omega_tilt == 0.0

    # 4. Duzensiz dt orneklemesinde zaman normalizasyonu (200 px/s)
    kf_uneven = KalmanFilter()
    elapsed = 0.0
    for dt in [0.02, 0.05, 0.03, 0.06, 0.012, 0.04] * 10:
        elapsed += dt
        kf_uneven.update((640.0 + 200.0 * elapsed, 360.0), dt=dt)
    assert kf_uneven.raw_vx == pytest.approx(200.0, rel=0.03)


def test_kalman_ego_shift_observer():
    """Tareti dondururken hedefin sahte hiz uretmesini engelleyen 1. derece eyleyici gozlemcisini dogrular."""
    kf = KalmanFilter()
    for i in range(61):
        kf.update((640.0 - 5.0 * i, 360.0), dt=0.033, ego_dx=-5.0)
    assert abs(kf.raw_vx) < 5.0
    assert kf.smooth_vx == 0.0


def test_kalman_reversal_and_deadband():
    """Ani yon degisimi ve hiz olu bolgesini dogrular."""
    # 1. Ani ters yone hareket (150 px/s -> -150 px/s)
    kf_rev = KalmanFilter()
    _constant(kf_rev, 150.0, 41)
    assert kf_rev.smooth_vx > 100.0
    for i in range(21):
        kf_rev.update((640.0 - 150.0 * 0.033 * (i + 1), 360.0), dt=0.033)
    assert kf_rev.raw_vx < -100.0
    assert kf_rev.smooth_vx < -100.0

    # 4. Hiz olu bolgesi (titresim yok etme)
    slow = KalmanFilter()
    for i in range(61):
        slow.update((640.0 + 0.5 * i, 360.0), dt=0.033)
    assert slow.smooth_vx == 0.0

    fast = KalmanFilter()
    for i in range(61):
        fast.update((640.0 + 2.0 * i, 360.0), dt=0.033)
    assert fast.smooth_vx > CONFIG.kalman.velocity_deadband_high_px_s


def test_kalman_nis_outlier_gating_and_reacquire():
    """Tekil sahte YOLO sicramalarini Ki-Kare NIS kapisiyla sonumlemeyi ve 3 karede yeniden yakalamayi dogrular."""
    kf = KalmanFilter()
    dt = 0.033
    # 1. 100 px/s sabit hizla oturma
    _constant(kf, 100.0, 41, dt=dt)
    assert kf.smooth_vx == pytest.approx(100.0, rel=0.05)
    last_x = float(kf.x[0, 0])

    # 2. Tekil 300 px sahte kutu sicramasi
    jump_x = last_x + 300.0
    kf.update((jump_x, 360.0), dt=dt)
    assert kf.last_nis > kf.nis_threshold
    assert kf._outlier_frames == 1
    # Tekil sicrama R * 100 ile sonumlendigi icin hiz 400+ px/s'ye firlamaz
    assert abs(kf.smooth_vx - 100.0) < 35.0

    # 3. Sonraki kare gercek hedefe donus
    normal_x = last_x + 100.0 * dt * 2
    kf.update((normal_x, 360.0), dt=dt)
    assert kf.last_nis <= kf.nis_threshold
    assert kf._outlier_frames == 0

    # 4. Ust uste 3 kare sapma: yeni hedef kabul edilip sifirdan baslatilmali
    jump_pos = 950.0
    for frame in range(1, 4):
        px, py, vx, vy = kf.update((jump_pos, 360.0), dt=dt)
        if frame < 3:
            assert kf._outlier_frames == frame
        else:
            # 3. karede reacquire gerceklesir: konum olcumden, hiz sifirdan
            assert kf._outlier_frames == 0
            assert px == pytest.approx(jump_pos)
            assert vx == 0.0
            assert vy == 0.0

