"""Angajman yoneticisi, tus haritasi ve presenter birim testleri."""
import pytest
import numpy as np

from turret_fcs.engagement import EngagementManager, EngagementState
from turret_fcs.keymap import process_key_event
from turret_fcs.presenter import HUDPresenter
from turret_fcs.turret_hud import TurretHUD
from turret_fcs.config import FCSState
from turret_fcs.fcs_manager import FCSManager


def test_engagement_manager_transitions():
    """SPINUP -> SETTLE -> FIRING -> RECOVER -> EVAL asamalarini dogrular."""
    em = EngagementManager(
        idle_spinup_time_s=0.20,
        cold_spinup_time_s=0.50,
        settle_time_s=0.10,
        firing_pulse_s=0.08,
        recovery_time_s=0.20,
        eval_display_time_s=0.50,
    )

    # 1. Kilitli degilken atis reddedilmeli
    ok, reason = em.trigger(now=10.0, is_locked=False, is_range_valid=True)
    assert not ok
    assert "TARGET NOT LOCKED" in reason

    # 2. Menzil gecersizken atis reddedilmeli
    ok, reason = em.trigger(now=10.0, is_locked=True, is_range_valid=False)
    assert not ok
    assert "LIDAR" in reason

    # 3. Gecerli sartlarda atis baslamali (SPINUP)
    ok, reason = em.trigger(now=10.0, is_locked=True, is_range_valid=True, is_flywheel_idle=True)
    assert ok
    state = em.update(now=10.05, is_locked=True)
    assert state.phase == "SPINUP"
    assert state.is_physical_firing is True
    assert 0.0 < state.slew_factor < 1.0

    # 4. SETTLE asamasi (10.0 + 0.20 -> 10.25)
    state = em.update(now=10.25, is_locked=True)
    assert state.phase == "SETTLE"
    assert state.slew_factor == 1.0

    # 5. FIRING asamasi (10.0 + 0.20 + 0.10 -> 10.32)
    state = em.update(now=10.32, is_locked=True)
    assert state.phase == "FIRING"
    assert state.fcs_fire_cmd_state == FCSState.FIRING

    # 6. RECOVER asamasi (10.0 + 0.20 + 0.10 + 0.08 -> 10.45)
    state = em.update(now=10.45, is_locked=True)
    assert state.phase == "RECOVER"
    assert state.is_physical_firing is True

    # 7. EVAL asamasi (10.0 + 0.20 + 0.10 + 0.08 + 0.20 -> 10.65)
    state = em.update(now=10.65, is_locked=True)
    assert state.phase == "EVAL"
    assert state.is_physical_firing is False

    # 8. Oturum tamamlaninca IDLE'a donus (10.0 + 1.08 + 0.50 -> 12.0)
    state = em.update(now=12.0, is_locked=True)
    assert state.phase is None
    assert state.is_active is False


def test_keymap_events():
    """Operator klavye girdilerinin FCS ve angajmana etkisini dogrular."""
    fcs = FCSManager(control_mode="P")
    em = EngagementManager()
    events = []

    def log_cb(msg, color):
        events.append(msg)

    # Mod degisimi (M)
    should_exit, estop = process_key_event(ord("m"), fcs, em, False, 10.0, True, True, log_cb)
    assert not should_exit
    assert fcs.control_mode == "RL"
    assert any("RL CONTROLLER" in e for e in events)

    # Devriye ac/kapa (P)
    process_key_event(ord("p"), fcs, em, False, 10.0, True, True, log_cb)
    assert fcs.patrol_enabled is True

    # Acil fren (Space)
    _, estop = process_key_event(32, fcs, em, False, 10.0, True, True, log_cb)
    assert estop is True

    # Cikis (Q)
    should_exit, _ = process_key_event(ord("q"), fcs, em, estop, 10.0, True, True, log_cb)
    assert should_exit is True


def test_hud_presenter_render():
    """HUDPresenter render isleminin hatasiz calistigini dogrular."""
    hud = TurretHUD()
    presenter = HUDPresenter(hud)
    fcs = FCSManager(control_mode="P")
    em = EngagementManager()

    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    state = em.update(now=0.0, is_locked=False)

    # Hedef yokken render
    presenter.render(
        frame=frame,
        fcs=fcs,
        engagement_state=state,
        target_box=None,
        center_x=640,
        center_y=360,
        fps=30.0,
        latency_ms=12.5,
        dist_cm=250,
        strength=1200,
        temp_c=24.0,
        is_estop=False,
    )
    assert frame.shape == (720, 1280, 3)

    # Hedef varken render
    presenter.render(
        frame=frame,
        fcs=fcs,
        engagement_state=state,
        target_box=(600, 340, 680, 420),
        center_x=640,
        center_y=360,
        fps=30.0,
        latency_ms=12.5,
        dist_cm=250,
        strength=1200,
        temp_c=24.0,
        is_estop=False,
    )
    assert frame.shape == (720, 1280, 3)
