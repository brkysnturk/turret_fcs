"""Webots RL oturumlarini karsilastirir: gercek pan hatasi, komut titremesi, kilit."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

FX = 1411.0


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return [r for r in csv.DictReader(fh) if int(float(r.get("target_detected") or 0)) == 1]


def analyze(path, limit=600):
    rows = load(path)[:limit]
    pan = []
    cmd = []
    cmd_t = []
    for row in rows:
        gx = float(row.get("gt_x") or 0.0)
        gy = float(row.get("gt_y") or 0.0)
        bearing = math.degrees(math.atan2(gx, gy))
        pan.append(bearing - float(row.get("turret_pan_deg") or 0.0))
        cmd.append(float(row.get("cmd_pan_deg") or 0.0))
        cmd_t.append(float(row.get("cmd_tilt_deg") or 0.0))
    pan = np.array(pan)
    cmd = np.array(cmd)
    cmd_t = np.array(cmd_t)
    d = np.diff(cmd)
    dt = np.diff(cmd_t)
    moves = d[np.abs(d) > 1e-6]
    rev = np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0 if moves.size > 2 else 0.0
    pixel = np.abs(pan) * math.pi / 180.0 * FX
    tilt_err = np.array([abs(float(r.get("angular_err_tilt") or 0.0)) for r in rows])
    return {
        "pan_rms": float(np.sqrt(np.mean(pan**2))),
        "pan_mean": float(np.mean(np.abs(pan))),
        "pixel_mean": float(pixel.mean()),
        "lock30": float(np.mean(pixel < 30.0) * 100.0),
        "rev": float(rev),
        "step_pan": float(np.mean(np.abs(d))),
        "step_tilt": float(np.mean(np.abs(dt))),
        "tilt_err": float(np.mean(tilt_err)),
        "locked": float(np.mean([int(float(r.get("is_locked") or 0)) for r in rows]) * 100.0),
        "frames": len(rows),
    }


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        path = Path(arg)
        s = analyze(path)
        print(
            f"{path.name}: pan_rms={s['pan_rms']:.3f} pixel={s['pixel_mean']:.1f} "
            f"lock30={s['lock30']:.1f}% locked={s['locked']:.1f}% rev={s['rev']:.1f}% "
            f"step={s['step_pan']:.3f}/{s['step_tilt']:.3f} tilt_err={s['tilt_err']:.2f} frames={s['frames']}"
        )
