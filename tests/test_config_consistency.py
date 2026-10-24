"""SSOT configuration consistency: parametreler ve dosya yollari."""
import math

from turret_fcs.config import CONFIG
from turret_fcs.p_controller import PController
from turret_fcs.lock_manager import LockManager
from turret_fcs.kalman_filter import KalmanFilter
from turret_fcs.rl_controller import RLController
from turret_fcs.rl_sim.turret_env import TurretEnv


def test_config_ssot():
    p = PController()
    lm = LockManager()
    kf = KalmanFilter()
    rl = RLController(auto_load=False)
    env = TurretEnv(enable_randomization=False)

    assert math.isclose(p.kp_frame_pan, CONFIG.control.kp_pan, abs_tol=1e-5)
    assert math.isclose(p.kp_frame_tilt, CONFIG.control.kp_tilt, abs_tol=1e-5)
    assert math.isclose(p.deadband_pan, CONFIG.control.deadband_pan_deg, abs_tol=1e-5)
    assert math.isclose(p.deadband_tilt, CONFIG.control.deadband_tilt_deg, abs_tol=1e-5)
    assert math.isclose(p.max_output, CONFIG.control.max_output_deg, abs_tol=1e-5)
    assert math.isclose(p.alpha, CONFIG.control.alpha_filter, abs_tol=1e-5)

    assert lm.lock_threshold_frames == CONFIG.lock.lock_threshold_frames
    assert math.isclose(lm.lock_exit_margin_deg, CONFIG.lock.lock_exit_margin_deg, abs_tol=1e-5)

    assert math.isclose(kf.sigma_a, CONFIG.kalman.sigma_a, abs_tol=1e-5)
    assert math.isclose(kf.outlier_residual, CONFIG.kalman.outlier_residual_px, abs_tol=1e-5)
    assert math.isclose(kf.r_var, CONFIG.kalman.r_var, abs_tol=1e-5)

    for obj in (rl, env):
        assert math.isclose(obj.max_step_pan_deg, CONFIG.actuator.max_step_pan_deg, abs_tol=1e-5)
        assert math.isclose(obj.max_step_tilt_deg, CONFIG.actuator.max_step_tilt_deg, abs_tol=1e-5)
    assert math.isclose(env.servo_deadband, CONFIG.actuator.servo_deadband_deg, abs_tol=1e-5)

    assert CONFIG.optics.compute_fx(1280) > 1000.0
    assert CONFIG.optics.compute_fy(720) > 1000.0
    assert CONFIG.optics.h_fov_deg > CONFIG.optics.v_fov_deg

    assert CONFIG.paths.yolo_model.endswith(".pt")
    assert CONFIG.paths.fallback_yolo_model.endswith(".pt")
