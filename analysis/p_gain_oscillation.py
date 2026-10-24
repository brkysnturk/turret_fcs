"""P kazanc taramasi için taret titresim (limit cycle) analizi.

Her kazanç için gerçek pan hatası, komut yön değiştirme oranı ve komut adımı
birlikte raporlanır. Limit cycle başlangıcı, komut yön değiştirme oranının
sıçradığı noktadır.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

# (etiket, log dosyası) - Webots profil 2, ~90 saniye, aynı ayarlar
RUNS = [
    ("0.20", "webots_telemetry_20261024_051314.csv"),
    ("0.28", "webots_telemetry_20261024_051448.csv"),
    ("0.30", "webots_telemetry_20261024_052303.csv"),
    ("0.35", "webots_telemetry_20261024_051622.csv"),
    ("0.39", "webots_telemetry_20261024_052437.csv"),
    ("0.42", "webots_telemetry_20261024_051756.csv"),
    ("0.50", "webots_telemetry_20261024_051930.csv"),
    ("0.35_t0.18", "webots_telemetry_20261024_052104.csv"),
]


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return [r for r in csv.DictReader(fh) if int(float(r.get("target_detected") or 0)) == 1]


def analyze(path):
    rows = load(path)
    pan = []
    for row in rows:
        gx = float(row.get("gt_x") or 0.0)
        gy = float(row.get("gt_y") or 0.0)
        bearing = np.degrees(np.arctan2(gx, gy))
        pan.append(bearing - float(row.get("turret_pan_deg") or 0.0))
    pan = np.array(pan)
    cmd = np.array([float(row.get("cmd_pan_deg") or 0.0) for row in rows])
    cmd_t = np.array([float(row.get("cmd_tilt_deg") or 0.0) for row in rows])

    late = pan[-300:]
    diff = np.diff(cmd[-300:])
    moves = diff[np.abs(diff) > 1e-6]
    reversals = np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0 if moves.size > 2 else float("nan")

    # Komutun butcesini tamamen doyurması: doyum klasik limit cycle isaretidir.
    saturated = float(np.mean(np.abs(diff) > 1.35) * 100.0) if diff.size else 0.0
    return {
        "true_rms": float(np.sqrt(np.mean(pan**2))),
        "late_rms": float(np.sqrt(np.mean(late**2))),
        "late_p95": float(np.percentile(np.abs(late), 95)),
        "reversal_pct": float(reversals),
        "move_count": int(moves.size),
        "step95": float(np.percentile(np.abs(np.diff(cmd)), 95)),
        "saturated_pct": saturated,
        "tilt_step": float(np.mean(np.abs(np.diff(cmd_t)))),
        "locked": float(np.mean([int(float(r.get("is_locked") or 0)) for r in rows]) * 100.0),
    }


if __name__ == "__main__":
    log_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("webots/logs")
    print(f"{'kp':>10} {'true_rms':>9} {'late_rms':>9} {'late_p95':>9} {'rev%':>7} {'moves':>6} {'step95':>7} {'locked':>7}")
    for label, name in RUNS:
        path = log_dir / name
        if not path.exists():
            print(f"{label:>10} dosya yok: {name}")
            continue
        s = analyze(path)
        print(
            f"{label:>10} {s['true_rms']:9.3f} {s['late_rms']:9.3f} {s['late_p95']:9.3f} "
            f"{s['reversal_pct']:7.1f} {s['move_count']:6d} {s['step95']:7.3f} {s['locked']:6.1f}%"
        )
