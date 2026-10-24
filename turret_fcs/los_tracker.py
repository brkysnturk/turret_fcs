"""Dunya gorus hatti (LOS) acisi icin alpha-beta prototipi.

Kamera tarete sabit oldugu icin hedefin dunya acisi:
    psi = taret_acisi + kamera_hata_acisi
Bu modul ego kompanzasyonundan bagimsiz bir hiz tahmini uretir. Kontrol
hattina bagli degildir; telemetri ve gercek log/Webots A-B testi icindir.
"""

from .config import CONFIG


class LosTracker:
    def __init__(self, alpha=None, beta=None):
        self.alpha = float(CONFIG.control.los_alpha if alpha is None else alpha)
        self.beta = float(CONFIG.control.los_beta if beta is None else beta)
        self.pan = None
        self.tilt = None
        self.pan_rate = 0.0
        self.tilt_rate = 0.0

    def update(self, pan_deg, tilt_deg, dt):
        dt = max(1e-3, float(dt))
        if self.pan is None:
            self.pan = float(pan_deg)
            self.tilt = float(tilt_deg)
            return self.pan, self.tilt, self.pan_rate, self.tilt_rate

        pred_pan = self.pan + self.pan_rate * dt
        pred_tilt = self.tilt + self.tilt_rate * dt
        res_pan = float(pan_deg) - pred_pan
        res_tilt = float(tilt_deg) - pred_tilt

        self.pan = pred_pan + self.alpha * res_pan
        self.tilt = pred_tilt + self.alpha * res_tilt
        self.pan_rate += self.beta * res_pan / dt
        self.tilt_rate += self.beta * res_tilt / dt
        return self.pan, self.tilt, self.pan_rate, self.tilt_rate

    def reset(self):
        self.pan = None
        self.tilt = None
        self.pan_rate = 0.0
        self.tilt_rate = 0.0
