"""Telemetry logu uzerinde RL post-processing A/B taramasi (offline replay).

Politika ve gozlem ayni kalir; yalnizca cikis katmani degisir:
- taper  : hedefe yakinken komutu yumusakca ezen olcek (0 = kapali, 1.0 = 1 derece rampa)
- accel  : komut ivme siniri (derece/s^2, None = kapali)
- vmax   : politika hiz gozlemi olcegi. Runtime'da hiz gozlemi 0.0 gonderildigi
           icin egitimdeki 300 px/s olcegi yanlis anlam tasir: 0.0 -> en yavas
           hedef kabul edilir. Daha kucuk olcek bu yanlis anlami giderir.

Bu bir kapali cevrim degil, sabit log uzerinde acik cevrim bir taramadir:
kayitli kamera hatasi korunur, yalnizca controller cikisi yeniden uretilir.
Kullanim:
    python analysis/rl_postproc_sweep.py logs/telemetry_test_20261024_143300.csv
"""

from __future__ import annotations

import csv
import itertools
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.rl_controller import RLController

FX = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
FY = CONFIG.optics.compute_fy(CONFIG.optics.default_height)
W = float(CONFIG.optics.default_width)
H = float(CONFIG.optics.default_height)
CX, CY = W / 2.0, H / 2.0


def load_rl_rows(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    return [r for r in rows if r["control_mode"] == "RL"]


def replay(rows, controller):
    """Kayitli hatayi korur, sadece controller cikisini yeniden uretir."""
    controller.reset()
    errs = []
    cmds = []
    near_cmds = []
    sat = 0

    prev_t = float(rows[0]["time_sec"])
    for i, r in enumerate(rows):
        t = float(r["time_sec"])
        dt = max(0.005, min(0.1, t - prev_t)) if i else 1.0 / 30.0
        prev_t = t
        if r["target_detected"] != "1":
            controller.on_target_lost()
            continue
        err_pan = float(r["angular_err_pan"])
        err_tilt = float(r["angular_err_tilt"])
        bx = CX - np.tan(np.radians(err_pan)) * FX
        by = CY + np.tan(np.radians(err_tilt)) * FY

        d_pan, _ = controller.predict_angles(
            bx, by, CX, CY, W, H, 0.0, 0.0, 2.0, err_pan, err_tilt, dt
        )
        mag = float(np.hypot(err_pan, err_tilt))
        errs.append(mag)
        cmds.append(d_pan)
        if abs(err_pan) < 1.5:
            near_cmds.append(abs(d_pan))
        if abs(d_pan) >= 0.999 * CONFIG.actuator.max_step_pan_deg:
            sat += 1

    errs = np.array(errs)
    cmds = np.array(cmds)
    moves = cmds[np.abs(cmds) > 1e-4]
    reversals = int(np.sum(np.sign(moves[1:]) != np.sign(moves[:-1]))) if moves.size > 1 else 0
    return {
        "rms": float(np.sqrt(np.mean(errs**2))),
        "p90": float(np.percentile(errs, 90)),
        "locked": float(np.mean(errs < 2.40)),
        "rev": 100.0 * reversals / max(1, moves.size - 1),
        "near": float(np.mean(near_cmds)) if near_cmds else 0.0,
        "sat": 100.0 * sat / len(errs),
    }


def recorded(rows):
    det = [r for r in rows if r["target_detected"] == "1"]
    errs = np.array([np.hypot(float(r["angular_err_pan"]), float(r["angular_err_tilt"])) for r in det])
    cmds = np.array([float(r["cmd_pan"]) for r in det])
    moves = cmds[np.abs(cmds) > 1e-4]
    rev = int(np.sum(np.sign(moves[1:]) != np.sign(moves[:-1]))) if moves.size > 1 else 0
    near = [abs(float(r["cmd_pan"])) for r in det if abs(float(r["angular_err_pan"])) < 1.5]
    return {
        "rms": float(np.sqrt(np.mean(errs**2))),
        "p90": float(np.percentile(errs, 90)),
        "locked": float(np.mean(errs < 2.40)),
        "rev": 100.0 * rev / max(1, moves.size - 1),
        "near": float(np.mean(near)) if near else 0.0,
    }


def make_controller(taper, accel, vmax):
    ctrl = RLController(
        deadband_px=0.0,
        taper_pan_deg=1.0,
        taper_tilt_deg=1.0,
        cmd_accel_pan_deg_s2=accel,
        cmd_accel_tilt_deg_s2=(accel * 0.66 if accel else None),
        v_max_px_s=vmax,
    )
    ctrl.deadband_pan_deg = taper
    ctrl.deadband_tilt_deg = taper
    return ctrl


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("logs/telemetry_test_20261024_143300.csv")
    rows = load_rl_rows(path)
    det = sum(1 for r in rows if r["target_detected"] == "1")
    print(f"{len(rows)} RL frames ({det} detected) from {path.name}")
    print("recorded:", {k: round(v, 3) for k, v in recorded(rows).items()})

    results = []
    for taper, accel, vmax in itertools.product((0.0, 0.5, 1.0), (None, 120.0, 60.0), (300.0, 100.0, 20.0)):
        metrics = replay(rows, make_controller(taper, accel, vmax))
        metrics.update({"taper": taper, "accel": accel, "vmax": vmax})
        results.append(metrics)

    results.sort(key=lambda m: (-m["locked"], m["rev"]))
    print("\ntaper accel vmax | rms  p90 locked rev%  near  sat%")
    for m in results[:15]:
        print(
            f"{m['taper']:>5} {str(m['accel']):>5} {m['vmax']:>4} | "
            f"{m['rms']:5.2f} {m['p90']:5.2f} {m['locked']*100:5.1f}% "
            f"{m['rev']:5.1f} {m['near']:5.3f} {m['sat']:5.1f}"
        )
