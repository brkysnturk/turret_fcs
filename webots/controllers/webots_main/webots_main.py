"""
================================================================================
MODUL: WEBOTS OTONOM ATIS KONTROL SISTEMI ANA DONGUSU (WEBOTS MAIN)
================================================================================
Bu modul, Turret FCS uretim mimarisini (FCSManager, P, RL, CV Kalman,
histerezisli LockManager ve TurretHUD) Webots simulasyon ortaminda kosturan
ince donanim adaptoru ve ana calistiricisidir.

CALISMA ZAMANI SOZLESMESI:
- Tesis Dogrulugu: Inkremental aci integrasyonu (theta += delta_theta), 46.9 deg/s limit (1.50 deg / 0.032 s).
- Kontrolcu: Uretim FCSManager; varsayilan P (Kp=0.22/0.11).
- RL: saf PPO, 11-64-64-2 NumPy politika; P harmani yok.
- Mod Yapisi: Ayrik 'P' veya 'RL'; WEBOTS_CONTROL_MODE ile baslangic modu secilir.
- Zaman Tabani: Saf simulasyon zamani (sim_t = hw.get_time()).
- Atis Emniyeti: 'Y' yalnizca kilitli, menzili dogrulanmis ve sekans bostayken atesler.
- Atis Protokolu: SPINUP/SLEW + SETTLE 0.15 s + 80 ms solenoid + RECOVER 0.35 s + EVAL 1.5 s.
- E-Stop: Bosluk tusu mandalli acil durdurma; motorlar merkezlenir, launcher kesilir.
================================================================================
"""

import sys
import time
import math
import os
from pathlib import Path
import cv2
import numpy as np

# Proje kokunu ve venv site-packages'i sys.path'e ekle (Webots dahili Python korumasi)
project_root = Path(__file__).resolve().parents[3]
turret_fcs_dir = project_root / "turret_fcs"
venv_site = project_root / "venv" / "Lib" / "site-packages"

for p in [str(project_root), str(turret_fcs_dir), str(venv_site)]:
    if p not in sys.path and Path(p).exists():
        sys.path.insert(0, p)

from ultralytics import YOLO

from turret_fcs.config import CONFIG, FCSState
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.turret_hud import TurretHUD

from webots_hardware_interface import WebotsHardwareInterface
from webots_flywheel_launcher import WebotsFlywheelLauncher
from webots_virtual_ballistics import VirtualBallisticsEvaluator
from webots_telemetry_logger import WebotsTelemetryLogger
from webots_tuning import apply_tuning, load_tuning


def main():
    ENABLE_TELEMETRY = True

    # 1. Webots Donanim ve Atis Firlaticisini Baslat
    hw = WebotsHardwareInterface()
    launcher = WebotsFlywheelLauncher(hw.robot)
    hud = TurretHUD()
    start_mode = os.environ.get("WEBOTS_CONTROL_MODE", "P").strip().upper()
    fcs = FCSManager(control_mode=start_mode if start_mode in ("P", "RL") else "P", patrol_enabled=False)
    tuning = load_tuning()
    apply_tuning(fcs, tuning)
    ballistics = VirtualBallisticsEvaluator(bullet_speed=CONFIG.ballistics.projectile_speed_mps, supervisor=hw.robot)
    telemetry = WebotsTelemetryLogger(hw.robot) if ENABLE_TELEMETRY else None

    # 2. YOLO Hedef Tespit Modelini Yukle (CONFIG SSOT yollariyla)
    model_path = project_root / CONFIG.paths.yolo_model
    if not model_path.exists():
        model_path = project_root / CONFIG.paths.fallback_yolo_model
    model = YOLO(str(model_path))

    WINDOW_NAME = "Turret FCS - Webots Digital Twin"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # 3. Optik Kamera Parametreleri (Logitech C270 SSOT - CONFIG)
    fx = CONFIG.optics.compute_fx(hw.w_cam)
    fy = CONFIG.optics.compute_fy(hw.h_cam)
    center_x, center_y = hw.w_cam // 2, hw.h_cam // 2

    # 4. Atis ve Zamanlama Yonetimi (Saf sim_t uzerinden)
    fire_start_sim_t = 0.0
    COLD_SPINUP_DURATION = CONFIG.flywheel.cold_spinup_time_s   # Bosta degilse 1.00 s
    spinup_duration = COLD_SPINUP_DURATION
    SETTLE_DURATION = CONFIG.ballistics.settle_time_s          # 0.15 s sabit oturma
    GUN_FIRE_DURATION = CONFIG.ballistics.firing_pulse_s        # 0.08 s
    RECOVER_DURATION = CONFIG.ballistics.recovery_time_s        # 0.35 s
    BALLISTIC_DISPLAY_DURATION = CONFIG.ballistics.eval_display_time_s  # EVAL afis suresi

    is_estop = False
    fire_stage = None
    spinup_progress = None
    slew_factor = 0.0
    frame_id = 0

    hud.log_event("[SYS] WEBOTS TURRET FCS DIGITAL TWIN ONLINE", color=(0, 255, 128))
    hud.log_event(f"[CAM] C270 VIRTUAL OPTICS (1280x720, FOV: 48.8 deg)", color=(0, 255, 128))
    hud.log_event("[CTRL] P + RL READY (Ki=0, Kd=0)", color=(0, 215, 255))
    hud.log_event(
        f"[TUNE] {tuning['label']} | Kp={tuning['kp_pan']:.3f}/{tuning['kp_tilt']:.3f} "
        f"MAX={tuning['max_output_deg']:.2f}",
        color=(0, 215, 255),
    )

    try:
        while hw.step() != -1:
            frame_id += 1

            sim_t = hw.get_time()
            # 80 ms gecikmeli gozlem: goruntu + servo pozu + LiDAR ayni karede
            frame, curr_pan_deg, curr_tilt_deg, dist_cm, strength = hw.get_observation()
            if frame is None:
                continue

            fps = int(round(1.0 / max(0.001, hw.dt)))
            fcs.sync_turret_pose(curr_pan_deg, curr_tilt_deg)

            # YOLO Takip (ByteTrack ile)
            conf_val = fcs.selector.min_conf
            results = model.track(frame, imgsz=640, conf=conf_val, persist=True, tracker="bytetrack.yaml", verbose=False)
            boxes = results[0].boxes if len(results) > 0 else []

            # Kademeli balistik firlatma durum makinesi: SPINUP/SLEW -> SETTLE -> FIRING
            t_fire = (sim_t - fire_start_sim_t) if fire_start_sim_t > 0 else 999.0
            is_hud_firing = False
            total_fire_cycle = spinup_duration + SETTLE_DURATION + GUN_FIRE_DURATION + RECOVER_DURATION + BALLISTIC_DISPLAY_DURATION

            if fire_start_sim_t > 0 and t_fire < total_fire_cycle:
                is_hud_firing = True
                if t_fire < spinup_duration:
                    fire_stage = "SPINUP"
                    launcher.start_spinup()
                    spinup_progress = float(t_fire / spinup_duration)
                    # Kosinus yumusatma ile aim-point super-elevasyonuna suzulme
                    slew_factor = float(0.5 * (1.0 - np.cos(np.pi * spinup_progress)))
                elif t_fire < (spinup_duration + SETTLE_DURATION):
                    # Slew bitti; komut sabit, servo oturuyor. Lead/gravity canli kalir.
                    fire_stage = "SETTLE"
                    spinup_progress = 1.0
                    slew_factor = 1.0
                elif t_fire < (spinup_duration + SETTLE_DURATION + GUN_FIRE_DURATION):
                    if fire_stage != "FIRING":
                        # Solenoid tahrigi ve mermi firlatma (Tek kenar tetik)
                        launcher.trigger()
                        # LiDAR'in ham (spin-up'ta duvara sicrayan) degerini degil,
                        # FCS'nin ates icin dondurdugu son gecerli hedef mesafesini kullan.
                        ballistics.fire_shot(dist_m=fcs.last_valid_target_dist_m, sim_time=sim_t)
                    fire_stage = "FIRING"
                    spinup_progress = 1.0
                    slew_factor = 1.0
                elif t_fire < (spinup_duration + SETTLE_DURATION + GUN_FIRE_DURATION + RECOVER_DURATION):
                    fire_stage = "RECOVER"
                    spinup_progress = 1.0
                    rec_p = float((t_fire - spinup_duration - SETTLE_DURATION - GUN_FIRE_DURATION) / RECOVER_DURATION)
                    slew_factor = float(0.5 * (1.0 + np.cos(np.pi * rec_p)))
                else:
                    fire_stage = "EVAL"
                    launcher.stop_flywheels()
                    spinup_progress = 1.0
                    slew_factor = 0.0
            else:
                fire_stage = None
                spinup_progress = None
                slew_factor = 0.0
                is_hud_firing = False
                if fire_start_sim_t > 0:
                    launcher.stop_flywheels()
                    fire_start_sim_t = 0.0
                    spinup_duration = COLD_SPINUP_DURATION

            # Fiziksel atis yalnizca SPINUP, FIRING ve RECOVER sirasinda etkindir
            is_physical_firing = fire_stage in ("SPINUP", "SETTLE", "FIRING", "RECOVER")

            # Hakem ve Firlatici Simulasyon Guncellemeleri
            launcher.update(sim_time=sim_t)
            ballistics.update(sim_time=sim_t)

            # Uretim FCSManager Karar ve Kontrol Dongusu
            best_box = fcs.select_best_target(boxes, center_x, center_y)
            # Gercek YOLO kutu titremesine yakin deterministik algilama gurultusu.
            if best_box is not None:
                jitter_px = float(os.environ.get("WEBOTS_BBOX_JITTER", "3.0"))
                if jitter_px > 0.0:
                    jx = jitter_px * (0.65 * math.sin(7.3 * sim_t) + 0.35 * math.sin(17.1 * sim_t))
                    jy = jitter_px * (0.60 * math.sin(9.7 * sim_t + 0.7) + 0.40 * math.sin(21.3 * sim_t))
                    b = list(best_box)
                    b[0] = max(0.0, b[0] + jx)
                    b[1] = max(0.0, b[1] + jy)
                    b[2] = max(b[0] + 2.0, b[2] + jx)
                    b[3] = max(b[1] + 2.0, b[3] + jy)
                    best_box = b
            pan_cmd, tilt_cmd = fcs.update(
                best_box, center_x, center_y, fx, fy, hw.dt,
                is_firing=is_physical_firing,
                dist_cm=dist_cm,
                slew_factor=slew_factor
            )

            target_detected = best_box is not None
            target_centered = False

            # FF devre disi: P ve RL hiz feedforward kullanmaz.
            # Flywheel idle: hedef kilitliyken volanlari bosta dondur, atista boost et.
            if not is_hud_firing and not is_estop:
                if fcs.is_locked:
                    launcher.start_idle()
                else:
                    launcher.stop_flywheels()

            # FCS Durum Bayragi (uretim main.py ile ayni: RECOVER/EVAL kilit durumunu yansitir)
            if is_estop:
                fcs_state = FCSState.ESTOP
            elif fire_stage == "FIRING":
                fcs_state = FCSState.FIRING
            elif fcs.is_locked:
                fcs_state = FCSState.LOCKED
            else:
                fcs_state = FCSState.TRACKING

            # E-STOP veya Normal Inkremental Motor Surusu
            if is_estop:
                hw.set_neutral()
                status_text = "STATUS: EMERGENCY STOP (LATCHED)"
                status_color = (0, 0, 255)
                angle_err_text = "ANGULAR ERR:   [HALTED] deg"
                if target_detected:
                    x1, y1, x2, y2 = best_box
                    hud.draw_corner_box(frame, x1, y1, x2, y2, color=(100, 100, 100), length=25, thickness=1, label=hud.get_box_label())
            else:
                hw.apply_delta_commands(pan_cmd, tilt_cmd)

                if target_detected:
                    x1, y1, x2, y2 = best_box
                    bx, by = (x1 + x2) // 2, (y1 + y2) // 2
                    target_centered = hud.is_target_centered(center_x, center_y, x1, y1, x2, y2)

                    if fire_stage == "SPINUP":
                        ms_left = max(0, int((1.0 - spinup_progress) * spinup_duration * 1000))
                        status_text = f"STATUS: SPIN-UP & SLEW ({ms_left:3d}ms)"
                        status_color = (0, 215, 255)
                    elif fire_stage == "SETTLE":
                        status_text = "STATUS: SETTLING AIM"
                        status_color = (0, 215, 255)
                    elif fire_stage == "FIRING":
                        status_text = "STATUS: ENGAGING (SOLENOID RELEASE)"
                        status_color = (0, 0, 255)
                    elif fire_stage == "RECOVER":
                        status_text = "STATUS: RECOVERING BORESIGHT"
                        status_color = (0, 255, 128)
                    elif fire_stage == "EVAL":
                        status_text = "STATUS: ENGAGEMENT EVALUATION"
                        status_color = (0, 215, 255)
                    elif is_hud_firing:
                        status_text = "STATUS: ENGAGING TARGET"
                        status_color = (0, 0, 255)
                    elif fcs.is_locked:
                        status_text = "STATUS: TARGET LOCKED"
                        status_color = (0, 0, 255)
                    else:
                        status_text = "STATUS: TRACKING"
                        status_color = (0, 191, 255)

                    box_color = (0, 0, 255) if (fcs.is_locked or is_hud_firing) else status_color
                    hud.draw_corner_box(frame, x1, y1, x2, y2, color=box_color, length=30, thickness=2, label=hud.get_box_label())

                    if not is_hud_firing:
                        if not fcs.is_locked:
                            hud.draw_tracking_line(frame, center_x, center_y, bx, by, status_color)
                        hud.draw_lead_indicator(
                            frame, bx, by, int(fcs.pred_center[0]), int(fcs.pred_center[1]),
                        )

                    angle_err_text = f"ANGULAR ERR:   [{fcs.actual_pan:+.1f} , {fcs.actual_tilt:+.1f}] deg"
                else:
                    if fcs.patrol_enabled:
                        status_text = f"PATROL: {fcs.patrol_status}"
                        status_color = (0, 255, 0)
                        angle_err_text = f"TURRET POSE:   [{fcs.turret_pan_deg:+.1f} , {fcs.turret_tilt_deg:+.1f}] deg"
                    else:
                        status_text = "STATUS: STANDBY"
                        status_color = (180, 180, 180)
                        angle_err_text = "ANGULAR ERR:   [--, --] deg"

            # Merkez nisangah her karede tek kez (uretim main.py ile ayni)
            reticle_color = (0, 0, 255) if (fcs.is_locked or is_hud_firing) else status_color
            hud.draw_grid_and_reticle(frame, center_x, center_y, reticle_color=reticle_color)

            # Telemetri Kaydi (Schema v2) - Gercek fiziksel atis bayragi iletilir
            if telemetry:
                telemetry.log(frame_id, sim_t, hw.dt, fps, fcs, hw, best_box, is_firing=is_physical_firing, fire_stage=fire_stage, fcs_state=fcs_state, ballistics=ballistics, tuning=tuning)

            # Atis degerlendirme afisi: uretimdeki sabit "SUCCESSFUL" yerine
            # Webots analitik hakeminin GERCEK HIT/MISS sonucunu goster.
            ballistic_hud_data = None
            if is_hud_firing:
                res_txt, res_col = ballistics.get_hud_result(sim_t)
                if res_txt:
                    ballistic_hud_data = (res_txt, res_col)
                else:
                    elapsed_fire = sim_t - fire_start_sim_t
                    if elapsed_fire >= (spinup_duration + SETTLE_DURATION + GUN_FIRE_DURATION):
                        ballistic_hud_data = ("AWAITING IMPACT", (0, 215, 255))

            # Askeri Standartta Taktiksel HUD
            hud.draw_telemetry(
                frame,
                fps,
                status_text,
                status_color,
                angle_err_text,
                status_color,
                dist_cm,
                strength,
                target_detected=target_detected,
                target_centered=target_centered,
                is_locked=fcs.is_locked,
                firing=is_hud_firing,
                center_x=center_x,
                center_y=center_y,
                ballistic_result=ballistic_hud_data,
                fire_stage=fire_stage,
                spinup_progress=spinup_progress,
                spinup_duration_s=spinup_duration,
            )

            if target_detected and not is_hud_firing:
                hud.draw_lock_bar(frame, center_x, center_y, fcs.lock_counter, fcs.LOCK_THRESHOLD_FRAMES, fcs.is_locked)
                hud.draw_lock_quality(frame, center_x, center_y, fcs.lock_quality)

            hud.draw_mode_banner(frame, fcs.control_mode, fcs.patrol_enabled, is_locked=fcs.is_locked, firing=is_hud_firing)
            hud.draw_footer(frame, is_locked=fcs.is_locked, firing=is_hud_firing)
            hud.draw_event_log(frame, is_locked=fcs.is_locked, firing=is_hud_firing)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            # Operator Etkilesim ve Klavye Matrisi
            if key in [27, ord('q'), ord('Q')]:
                hud.log_event("[SYS] WEBOTS SESSION TERMINATED", color=(180, 180, 180))
                break
            elif key in [ord('m'), ord('M')]:
                # Yalnizca P ve RL arasinda ayrik gecis (HYBRID yok)
                mode_cycle = {"P": "RL", "RL": "P"}
                fcs.control_mode = mode_cycle.get(fcs.control_mode, "P")
                mode_names = {"P": "P (PRECISION LOCK)", "RL": "RL (SAF PPO)"}
                hud.log_event(f"[MODE] {mode_names.get(fcs.control_mode, fcs.control_mode)}", color=(0, 215, 255))
            elif key in [ord('p'), ord('P')]:
                fcs.patrol_enabled = not fcs.patrol_enabled
                p_text = "FIGURE-8 SCAN ENGAGED" if fcs.patrol_enabled else "STANDBY (DISABLED)"
                p_color = (0, 255, 0) if fcs.patrol_enabled else (180, 180, 180)
                hud.log_event(f"[PATROL] {p_text}", color=p_color)
            elif key == 32:  # SPACE - Mandalli E-STOP
                is_estop = not is_estop
                fcs.reset()
                fire_start_sim_t = 0.0
                if is_estop:
                    fcs.patrol_enabled = False
                    launcher.stop_flywheels()
                    hw.set_neutral()
                    hud.log_event("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", color=(0, 0, 255))
                else:
                    hud.log_event("[E-STOP] DISENGAGED - RESUMING FCS", color=(0, 255, 128))
            elif key in [ord('y'), ord('Y')]:
                # Guvenlik Kurali: Yalnizca KILITLI ve atis sekansi bosta iken tetiklenir!
                if fcs.is_locked and fire_stage is None and not is_estop and fcs.range_valid and fcs.range_stable:
                    spinup_duration = (CONFIG.flywheel.idle_spinup_time_s
                                       if launcher.is_idle_ready()
                                       else COLD_SPINUP_DURATION)
                    fire_start_sim_t = sim_t
                    hud.log_event("[FIRE] 2-STAGE ENGAGEMENT: SPIN-UP & SLEW ENGAGED", color=(0, 215, 255))
                else:
                    if not fcs.is_locked:
                        hud.log_event("[FIRE INHIBIT] TARGET NOT LOCKED!", color=(0, 0, 255))
                    elif not (fcs.range_valid and fcs.range_stable):
                        hud.log_event("[FIRE INHIBIT] LIDAR RANGE NOT STABLE/VALID", color=(0, 0, 255))
            elif key in [ord('n'), ord('N')]:
                fcs.reset()
                fire_start_sim_t = 0.0
                launcher.stop_flywheels()
                hud.log_event("[LOCK] TARGET DISENGAGED BY OPERATOR", color=(180, 180, 180))

    finally:
        if telemetry:
            telemetry.save()
        launcher.stop_flywheels()
        hw.set_neutral()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

