"""Kalman filtre dogrulama araci (gercek telemetri ve Webots zemin gercegi).

Kullanim:
    python analysis/kalman_validation.py logs/telemetry_test_....csv
    python analysis/kalman_validation.py webots/logs/webots_telemetry_....csv

Gercek loglarda referans, servo geri beslemesi + kutu acisindan turetilen dunya
LOS hizidir. Webots loglarinda referans, balonun gt_x/gt_y konumundan turetilen
gercek dunya yatay acisi hizidir. Her iki durumda da uretim KalmanFilter'i
komut tabanli ego modeliyle yeniden oynatilir ve kayitli target_vel_x ile
karsilastirilir.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.kalman_filter import KalmanFilter

FX = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
PAN_INVERT = CONFIG.mount.pan_invert
EGO_TAU = CONFIG.control.ego_tau_s
MAX_GAP_S = 0.20


def load(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def value(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return float(default)


def robust_slope(t, y, t0, half_window):
    """Tricube agirlikli yerel dogru fit, Huber yeniden agirliklandirma."""
    sel = np.abs(t - t0) <= half_window
    if int(sel.sum()) < 4:
        return np.nan
    tt = t[sel] - t0
    yy = y[sel]
    base = np.clip((1.0 - np.abs(tt / half_window) ** 3) ** 3, 1e-3, None)
    design = np.column_stack([np.ones_like(tt), tt])
    w = base.copy()
    beta = np.zeros(2)
    for _ in range(3):
        root = np.sqrt(w)
        beta = np.linalg.lstsq(design * root[:, None], yy * root, rcond=None)[0]
        resid = yy - design @ beta
        scale = max(1e-6, 1.4826 * np.median(np.abs(resid - np.median(resid))))
        w = base * np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(resid), 1e-9))
    return float(beta[1])


def is_webots(rows):
    return bool(rows) and "sim_time_sec" in rows[0]


def detected_indices(rows):
    return [
        i for i, row in enumerate(rows)
        if int(value(row, "target_detected")) == 1 and value(row, "box_cx") > 0
    ]


def time_array(rows, webots):
    key = "sim_time_sec" if webots else "time_sec"
    return np.array([value(row, key) for row in rows])


def world_bearing_deg(rows, webots):
    if webots:
        return np.degrees(np.arctan2(
            np.array([value(row, "gt_x") for row in rows]),
            np.array([value(row, "gt_y") for row in rows]),
        ))
    out = []
    for row in rows:
        feedback = value(row, "servo_pan_fb", -999.0)
        camera = (-feedback if PAN_INVERT else feedback) if feedback > -900.0 else 0.0
        box_angle = math.degrees(math.atan((value(row, "box_cx") - 640.0) / FX))
        out.append(camera + box_angle)
    return np.array(out)


def replay(rows, webots):
    """Uretim filtresini komut tabanli ego ile yeniden oynatir."""
    indices = detected_indices(rows)
    times = time_array(rows, webots)
    out = []
    kf = KalmanFilter()
    prev_t = None
    prev_step = 0.0
    motor_step = 0.0
    prev_cmd_angle = None

    for i in indices:
        row = rows[i]
        t = times[i]
        if prev_t is not None and (t - prev_t) > MAX_GAP_S:
            kf.reset()
            motor_step = 0.0
            prev_step = 0.0
            prev_cmd_angle = None
        dt = 0.033 if prev_t is None else max(0.01, min(t - prev_t, 0.1))

        if webots:
            cmd_angle = value(row, "cmd_pan_deg")
            step = 0.0 if prev_cmd_angle is None else cmd_angle - prev_cmd_angle
            prev_cmd_angle = cmd_angle
        else:
            cmd = value(row, "cmd_pan")
            step = -cmd if PAN_INVERT else cmd

        alpha = 1.0 - math.exp(-dt / max(1e-3, EGO_TAU))
        motor_step = (1.0 - alpha) * motor_step + alpha * prev_step
        ego_dx = -FX * math.tan(math.radians(motor_step))

        _, _, vx, _ = kf.update(
            (value(row, "box_cx"), value(row, "box_cy")),
            dt=dt,
            ego_dx=ego_dx,
            ego_dy=0.0,
        )
        out.append((i, vx))
        prev_t = t
        prev_step = step

    return out


def metrics(estimate, reference, label):
    mask = np.isfinite(estimate) & np.isfinite(reference)
    est, ref = estimate[mask], reference[mask]
    if est.size < 20:
        print(f"{label}: yetersiz veri")
        return
    corr = float(np.corrcoef(est, ref)[0, 1])
    sign = float(np.mean(np.sign(est) == np.sign(ref)))
    high = np.abs(ref) > 120.0
    sign_high = float(np.mean(np.sign(est[high]) == np.sign(ref[high]))) if high.any() else float("nan")
    gain = float(np.dot(est, ref) / max(1e-9, np.dot(ref, ref)))
    rmse = float(np.sqrt(np.mean((est - ref) ** 2)))
    jitter = float(np.median(np.abs(np.diff(est))))
    print(
        f"{label:>28}: corr={corr:+.3f} sign={sign*100:5.1f}% "
        f"sign>120={sign_high*100:5.1f}% gain={gain:+.3f} "
        f"rmse={rmse:6.1f} px/s jit={jitter:5.1f}"
    )


def evaluate(path):
    rows = load(path)
    webots = is_webots(rows)
    times = time_array(rows, webots)

    if webots:
        bearing = world_bearing_deg(rows, True)
        reference = np.array([robust_slope(times, bearing, t0, 0.15) for t0 in times])
        reference = reference * math.pi / 180.0 * FX
        reference_for = lambda idx: reference[idx]
    else:
        # box_cx sadece tespit varken anlamlidir; referansi tespit satirlarinda kur.
        det = detected_indices(rows)
        det_times = times[det]
        det_rows = [rows[i] for i in det]
        det_bearing = world_bearing_deg(det_rows, False)
        det_reference = np.array([
            robust_slope(det_times, det_bearing, t0, 0.15) for t0 in det_times
        ]) * math.pi / 180.0 * FX
        lookup = {row_index: det_reference[j] for j, row_index in enumerate(det)}
        reference_for = lambda idx: lookup[idx]

    replay_result = replay(rows, webots)
    indices = np.array([i for i, _ in replay_result])
    estimate = np.array([v for _, v in replay_result])
    reference_values = np.array([reference_for(i) for i in indices])

    print(f"\n{path.name} | {'Webots GT' if webots else 'gercek log'} | n={indices.size}")
    metrics(estimate, reference_values, "yeni CV Kalman")
    metrics(
        np.array([value(rows[i], "target_vel_x") for i in indices]),
        reference_values,
        "kayitli (eski) filtre",
    )


if __name__ == "__main__":
    if len(sys.argv) > 1:
        paths = [Path(arg) for arg in sys.argv[1:]]
    else:
        production = sorted((ROOT / "logs").glob("telemetry_test_*.csv"))
        webots = sorted((ROOT / "webots" / "logs").glob("webots_telemetry_*.csv"))
        paths = ([production[-1]] if production else []) + ([webots[-1]] if webots else [])
    for path in paths:
        evaluate(path)
