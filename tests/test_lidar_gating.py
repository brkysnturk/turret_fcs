"""LiDAR menzil kapılama: sıçrama reddi, ateş dondurma, geometri ve yeniden yakalama."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from turret_fcs.fcs_manager import FCSManager


def _stable():
    fcs = FCSManager(control_mode="P")
    for _ in range(3):
        fcs._update_lidar_range(200, False, None, 1000.0, 1000.0)
    assert fcs.range_valid is True
    assert fcs.range_stable is True
    return fcs


def test_lidar_rejects_wall_jump_and_geometry_mismatch():
    fcs = _stable()
    fcs._update_lidar_range(408, False, None, 1000.0, 1000.0)
    assert fcs.range_jump is True
    assert fcs.range_valid is False
    assert fcs.last_valid_target_dist_m == 2.0

    fcs = FCSManager(control_mode="P")
    for _ in range(3):
        fcs._update_lidar_range(200, False, (211, 211), 1410.0, 1412.0)
    fcs._update_lidar_range(408, False, (211, 211), 1410.0, 1412.0)
    assert fcs.range_geom_mismatch is True
    assert fcs.range_valid is False
    assert fcs.last_valid_target_dist_m == 2.0


def test_lidar_frozen_during_firing_and_reacquires_new_distance():
    fcs = _stable()
    fcs._update_lidar_range(408, True, None, 1000.0, 1000.0)
    assert fcs.last_valid_target_dist_m == 2.0
    assert fcs.range_valid is True

    for _ in range(5):
        fcs._update_lidar_range(300, False, None, 1000.0, 1000.0)
    assert fcs.last_valid_target_dist_m == 3.0
    assert fcs.range_valid is True
