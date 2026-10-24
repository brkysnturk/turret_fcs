"""FCSManager duman ve entegrasyon testleri."""
import pytest

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager


def test_init_pose_update_and_lost_recovery():
    fcs = FCSManager()
    assert fcs.control_mode in ("P", "RL")
    assert fcs.PAN_LIMIT_MIN < fcs.PAN_LIMIT_MAX
    assert fcs.target_angular_velocity_deg_s == (0.0, 0.0)

    fcs.sync_turret_pose(15.5, -5.2)
    assert (fcs.turret_pan_deg, fcs.turret_tilt_deg) == pytest.approx((15.5, -5.2))

    pan_cmd, tilt_cmd = fcs.update(
        [620, 340, 660, 380], 640, 360, 1100.0, 1100.0, 0.033,
        is_firing=False, dist_cm=200,
    )
    assert abs(pan_cmd) <= CONFIG.control.max_output_deg
    assert abs(tilt_cmd) <= CONFIG.control.max_output_deg

    fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.frames_since_lost > 0


def test_ballistic_slew_and_lead_origin():
    fcs = FCSManager(control_mode="P")
    fx = fy = 1100.0
    box = [620, 340, 660, 380]

    fcs.update(box, 640, 360, fx, fy, 0.033, dist_cm=300, slew_factor=0.0)
    assert fcs.prev_applied_lead_tilt == 0.0
    _, tilt_cmd = fcs.update(box, 640, 360, fx, fy, 0.033, is_firing=True, dist_cm=300, slew_factor=0.3)
    assert fcs.prev_applied_lead_tilt > 0.0
    assert tilt_cmd > 0.0

    fcs = FCSManager(control_mode="P")
    for i in range(30):
        bx = 640.0 + 100.0 * 0.033 * i
        box = [bx - 20, 340, bx + 20, 380]
        fcs.update(box, 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)

    lead_time = min(0.25, max(0.03, fcs.last_valid_target_dist_m / CONFIG.ballistics.projectile_speed_mps + 0.015))
    measured_x = (box[0] + box[2]) // 2
    measured_y = (box[1] + box[3]) // 2
    assert fcs.pred_center == pytest.approx(
        [measured_x + fcs.target_velocity[0] * lead_time,
         measured_y + fcs.target_velocity[1] * lead_time],
        abs=1e-6,
    )


def test_lock_evaluation_after_physical_firing():
    fcs = FCSManager(control_mode="P")
    box = [630, 350, 650, 370]
    for _ in range(15):
        fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=False, dist_cm=250)
    assert fcs.is_locked is True

    fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=True, dist_cm=250)
    assert fcs.is_locked is True

    for _ in range(fcs.max_lost_frames + 2):
        fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.is_locked is False
