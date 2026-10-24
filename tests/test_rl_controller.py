"""Uretim RL kontrolcusu testleri."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.rl_controller import RLController, rate_scaled_step_limits


class _FakeModel:
    def __init__(self, action):
        self.action = np.asarray(action, dtype=np.float32)
        self.last_obs = None

    def predict(self, obs, deterministic=True):
        self.last_obs = np.asarray(obs, dtype=np.float32)
        return self.action, None


def test_rl_mode_wires_production_controller():
    manager = FCSManager("RL")
    assert isinstance(manager.rl_controller, RLController)


def test_default_numpy_policy_loads():
    rl = RLController()
    assert rl.is_loaded, rl.load_error
    assert rl.policy_format == "numpy"
    assert (rl.model.obs_dim, rl.model.act_dim) == (11, 2)
    action, _ = rl.model.predict(np.zeros(11, dtype=np.float32))
    assert action.shape == (2,)
    assert np.all(np.isfinite(action))
    assert np.all(np.abs(action) <= 1.0)


def test_observation_and_action_history():
    fake = _FakeModel([0.5, -0.25])
    rl = RLController(auto_load=False, deadband_px=0.0)
    rl.model = fake
    rl.is_loaded = True

    d_pan, d_tilt = rl.predict_angles(
        700.0, 360.0, 640.0, 360.0, 1280.0, 720.0,
        0.0, 0.0, 2.0, 0.0, 0.0, 0.03333,
    )
    assert fake.last_obs.shape == (11,)
    assert fake.last_obs[0] == pytest.approx(60.0 / 640.0)
    assert d_pan == pytest.approx(0.5 * CONFIG.actuator.max_step_pan_deg)
    assert d_tilt == pytest.approx(-0.25 * CONFIG.actuator.max_step_tilt_deg)
    assert rl.act_hist_1 == pytest.approx([0.5, -0.25], abs=1e-6)


def test_deadband_and_reversal_damping():
    fake = _FakeModel([1.0, 1.0])
    rl = RLController(auto_load=False, deadband_px=5.0, reversal_damping=0.5)
    rl.model = fake
    rl.is_loaded = True

    # Merkezde ve yavas: politika bias'i bastirilir.
    assert rl.predict_angles(642.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 0, 0, 0.033) == (0.0, 0.0)

    # Buyuk hata, ayni yon: tam aksiyon.
    d_pan, d_tilt = rl.predict_angles(800.0, 200.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 0, 0, 0.033)
    assert d_pan == pytest.approx(CONFIG.actuator.max_step_pan_deg, abs=0.02)
    assert d_tilt == pytest.approx(CONFIG.actuator.max_step_tilt_deg, abs=0.02)

    # Yon degisimi: aksiyon yariya indirilir, salinim kisilir.
    fake.action = np.array([-1.0, -1.0], dtype=np.float32)
    d_pan, d_tilt = rl.predict_angles(800.0, 200.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 0, 0, 0.033)
    assert d_pan == pytest.approx(-0.5 * CONFIG.actuator.max_step_pan_deg, abs=0.02)
    assert d_tilt == pytest.approx(-0.5 * CONFIG.actuator.max_step_tilt_deg, abs=0.02)


def test_reset_and_target_lost_clear_history():
    fake = _FakeModel([0.4, 0.2])
    rl = RLController(auto_load=False)
    rl.model = fake
    rl.is_loaded = True
    rl.predict_angles(700.0, 300.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.03333)
    assert np.any(rl.act_hist_1 != 0.0)

    rl.on_target_lost()
    assert np.all(rl.act_hist_1 == 0.0)
    assert rl.has_prev_target is False

    rl.predict_angles(700.0, 300.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.03333)
    rl.reset()
    assert np.all(rl.act_hist_1 == 0.0)
    assert rl.last_action.tolist() == [0.0, 0.0]


def test_rate_scaling_matches_training_semantics():
    assert rate_scaled_step_limits(0.80, 0.50, 0.03333)[:2] == pytest.approx((0.80, 0.50), abs=1e-3)
    fast = rate_scaled_step_limits(0.80, 0.50, 0.01667)
    assert fast[:2] == pytest.approx((0.40, 0.25), abs=1e-3)
    slow = rate_scaled_step_limits(0.80, 0.50, 0.06666)
    assert slow[0] <= CONFIG.control.max_output_deg
    assert slow[1] <= CONFIG.control.max_output_deg


def test_no_p_blend_path():
    rl = RLController(auto_load=False)
    assert not hasattr(rl, "blend_with_p")
    assert not hasattr(rl, "blend_commands")
    assert rl.uses_velocity_obs is False


def test_unloaded_controller_returns_zero():
    rl = RLController(auto_load=False)
    assert rl.is_loaded is False
    assert rl.predict_angles(700, 300, 640, 360, 1280, 720, 0, 0, 2.0, 0, 0, 0.033) == (0.0, 0.0)


def test_soft_deadband_taper_and_accel_limit():
    fake = _FakeModel([1.0, 1.0])
    rl = RLController(
        auto_load=False,
        deadband_pan_deg=0.85,
        deadband_tilt_deg=0.95,
        taper_pan_deg=1.00,
        cmd_accel_pan_deg_s2=120.0,
    )
    rl.model = fake
    rl.is_loaded = True

    # 1. Olu bolge icinde (0.50 deg): sifir komut (tam kilit)
    d_pan, d_tilt = rl.predict_angles(650.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 0.50, 0.50, 0.033)
    assert d_pan == 0.0
    assert d_tilt == 0.0
    assert rl._in_deadband is True

    # 2. Taper bolgesinde (1.35 deg = db + 0.5): komut yumusatilir
    d_pan_taper, _ = rl.predict_angles(700.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 1.35, 1.35, 0.033)
    assert 0.0 < d_pan_taper < CONFIG.actuator.max_step_pan_deg

    # 3. Ivme sinirlama: kare basina degisim accel * dt ile sinirlanir
    rl_accel = RLController(auto_load=False, cmd_accel_pan_deg_s2=10.0)
    rl_accel.model = fake
    rl_accel.is_loaded = True
    dt = 0.03333
    u1, _ = rl_accel.predict_angles(800.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 10.0, 10.0, dt)
    assert u1 == pytest.approx(10.0 * dt, abs=1e-3)

