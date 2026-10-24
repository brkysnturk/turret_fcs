"""
================================================================================
MODUL: URETIM P KONTROLCUSU (P CONTROLLER)
================================================================================
Tip-1 P omurgasini koruyan, ancak eski kontrolcunun bilinen iki zayifligini
gideren deneysel kontrolcu:

1. ESKI SORUN - CAYI GECIKMESI:
   Eski kontrolcu kameradan gelen aciyi compute_angles icinde EMA ile
   filtreliyor, compute_p icinde tekrar EMA'liyor ve ustune 0.20 derece/kare
   pan slew koyuyordu. 30 FPS'de bu 6 derece/s tavan demektir; hedef 10-12
   derece/s hareket ettiginde P matematiksel olarak yetisemez.

2. ESKI SORUN - DOGRUSAL OLMAYAN KAZANC:
   Sabit Kp buyuk hatalarda yavas, olu bolge yakininda ise gereksiz serttir.
   Yeni kontrolcu hata buyudukce kazanci yumusak sekilde artirir (gain
   scheduling), boylece cevik hedefte hizli yakalar, merkezde sakin kalir.

TASARIM:
- Tek EMA (tek filtre katmani), varsayilan alpha = 0.85.
- Yumusak olu bolge korunur (pan 0.85, tilt 0.95 derece).
- Hata buyuklugune gore yumusak gecisli Kp carpani (1.0 -> 1.6).
- Pan/tilt icin ayri ivme (jerk) siniri; hiz tavani yok, sadece komutun
  kare basina degisimi sinirlanir. Boylece steady-state takip hizi
  kisitlanmaz, kalkis ve donuslerde mekanik darbe onlenir.
- Ki ve Kd yoktur. Hiz feedforward'i yoktur; Kalman hizi P motoruna girmez.

Uretim kontrolcusu budur. Eski yumusak olu bolgeli kontrolcu silindi;
v1.3 davranisi icin git gecmisindeki v1.3 tag'i kullanilir.

DOGRULAMA (Webots dijital ikiz, 6 profil x 8 kazanc, analysis/p_profile_sweep.py):
Hicbir tek kazanc tum profillerde kararli degildir. Komut yon degistirme orani
(rev%) ve doyum (sat%) limit cycle gostergesi olarak kullanildi.
- Standart sinus (profil 2): 0.20-0.35 kararli (rev %0.8-2.7), 0.39+ limit cycle.
- Figure-8 (profil 7): 0.20-0.32 kararli (rev %0.8-1.9), 0.32 en iyi kilit.
- Hizli sinus (profil 3): 0.20-0.32 arasinda fark yok; 0.25 guvenli tavan.
- Zigzag (profil 5): 0.28'e kadar kararli (kilit %89), 0.30+ limit cycle.
- Step/hold (profil 4): 0.20 bile doyuyor (%25); 0.30'da kilit %0. En zor profil.

SAHA GERI BILDIRIMI (Webots'tan ONEMLI FARK):
Gercek tarete 0.30 pan kazanci rezonansa girdi. Webots'un 1.2 derece backlash
modeli sahanin gercek disli boslugu + potansiyometre gurultusunu tam temsil
etmiyor. Bu yuzden taban kazanc 0.22/0.11'e cekildi: Webots'ta test edilen
tum profillerde kararli bolgenin icinde, sahaya daha fazla marj birakir.
Sahada hala titrerse 0.18/0.09'a kadar inilebilir; 0.30 ve uzeri sahada
kullanilmamalidir.
================================================================================
"""

import numpy as np

from .config import CONFIG


class PController:
    """Tek filtreli, kazanc planlamali, ivme sinirli Tip-1 P kontrolcusu."""

    DEFAULT_DISTANCE_M = CONFIG.ballistics.default_distance_m
    PROJECTILE_SPEED_MPS = CONFIG.ballistics.projectile_speed_mps
    GRAVITY_ACCEL = CONFIG.ballistics.gravity_accel

    MAX_OUTPUT_DEG = CONFIG.control.max_output_deg
    DEADBAND_PAN_DEG = CONFIG.control.deadband_pan_deg
    DEADBAND_TILT_DEG = CONFIG.control.deadband_tilt_deg
    ALPHA_FILTER = CONFIG.control.alpha_filter
    KP_PAN_FRAME = CONFIG.control.kp_pan
    KP_TILT_FRAME = CONFIG.control.kp_tilt

    def __init__(
        self,
        kp_pan=KP_PAN_FRAME,
        kp_tilt=KP_TILT_FRAME,
        alpha=ALPHA_FILTER,
        max_output=MAX_OUTPUT_DEG,
        deadband_pan=DEADBAND_PAN_DEG,
        deadband_tilt=DEADBAND_TILT_DEG,
        far_gain=1.6,
        far_error_deg=5.0,
        cmd_accel_pan_deg_s2=120.0,
        cmd_accel_tilt_deg_s2=80.0,
    ):
        self.kp_frame_pan = float(kp_pan)
        self.kp_frame_tilt = float(kp_tilt)
        self.kp_pan = self.kp_frame_pan / 0.03333
        self.kp_tilt = self.kp_frame_tilt / 0.03333

        self.alpha = float(alpha)
        self.max_output = float(max_output)
        self.deadband_pan = float(deadband_pan)
        self.deadband_tilt = float(deadband_tilt)

        self.far_gain = max(1.0, float(far_gain))
        self.far_error_deg = max(0.5, float(far_error_deg))
        self.cmd_accel_pan_deg_s2 = max(1.0, float(cmd_accel_pan_deg_s2))
        self.cmd_accel_tilt_deg_s2 = max(1.0, float(cmd_accel_tilt_deg_s2))

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0

    def compute_angles(self, bx, by, center_x, center_y, fx, fy):
        """Kamera merkezinden hedefin acisal sapmasini hesaplar; tek EMA uygular."""
        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_pan = np.degrees(np.arctan(target_err_x / max(fx, 1e-6)))
        raw_tilt = np.degrees(np.arctan(-target_err_y / max(fy, 1e-6)))

        self.actual_pan = self.alpha * raw_pan + (1.0 - self.alpha) * self.actual_pan
        self.actual_tilt = self.alpha * raw_tilt + (1.0 - self.alpha) * self.actual_tilt
        return self.actual_pan, self.actual_tilt

    def compute_gravity_comp(self, dist_cm):
        """Analitik yercekimi dususu kompanzasyonu: theta = arctan(g * D / (2 * v^2))."""
        target_d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm > 20) else self.DEFAULT_DISTANCE_M
        v_sq_2 = 2.0 * (self.PROJECTILE_SPEED_MPS ** 2)
        return float(np.degrees(np.arctan((self.GRAVITY_ACCEL * target_d_m) / v_sq_2)))

    def compute_lead_angles(self, d_lead_x, d_lead_y, fx, fy):
        """Hedefin gelecekteki konumuna dogru atis onleme acilarini hesaplar."""
        lead_pan = np.degrees(np.arctan(d_lead_x / max(fx, 1e-6)))
        lead_tilt = np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6)))
        return float(lead_pan), float(lead_tilt)

    def max_step_for(self, dt):
        """FPS'ten bagimsiz kare basi adim limiti (30 FPS'te max_output)."""
        dt_safe = max(1e-3, float(dt))
        rate_scale = float(np.clip(dt_safe / 0.03333, 0.25, 3.0))
        return float(min(self.max_output * rate_scale, CONFIG.control.max_command_step_deg))

    def compute_velocity_ff(self, omega_pan_deg_s, omega_tilt_deg_s, dt, max_step_deg=None):
        """API uyumlulugu; P kontrolcu bilincli olarak hiz FF kullanmaz."""
        return 0.0, 0.0

    def _scheduled_gain_for(self, error_deg, deadband, kp_frame):
        """Hata buyudukce Kp'yi yumusak sekilde artirir (1.0 -> far_gain)."""
        excess = max(0.0, abs(float(error_deg)) - float(deadband))
        x = float(np.clip(excess / self.far_error_deg, 0.0, 1.0))
        smooth = x * x * (3.0 - 2.0 * x)
        return float(kp_frame * (1.0 + (self.far_gain - 1.0) * smooth))

    def _axis_command(self, error_deg, kp_frame, deadband, accel_deg_s2, previous, dt):
        abs_error = abs(float(error_deg))
        if abs_error <= deadband:
            desired = 0.0
        else:
            effective = float(np.sign(error_deg)) * (abs_error - deadband)
            # kp_frame 30 FPS nominal kazancidir; dt ile olceklenir.
            desired = kp_frame * effective * (dt / 0.03333)

        max_delta = accel_deg_s2 * dt
        return float(np.clip(desired, previous - max_delta, previous + max_delta))

    def compute_p(self, active_pan, active_tilt, dt=0.033,
                  omega_pan_deg_s=0.0, omega_tilt_deg_s=0.0, **kwargs):
        """Tek filtreli, kazanc planlamali, ivme sinirli Tip-1 P komutu."""
        effective_dt = max(0.010, min(float(dt), 0.100))
        self.filtered_pan = float(active_pan)
        self.filtered_tilt = float(active_tilt)

        pan_gain = self._scheduled_gain_for(active_pan, self.deadband_pan, self.kp_frame_pan)
        tilt_gain = self._scheduled_gain_for(active_tilt, self.deadband_tilt, self.kp_frame_tilt)

        u_pan = self._axis_command(
            active_pan, pan_gain, self.deadband_pan,
            self.cmd_accel_pan_deg_s2, self.smooth_cmd_pan, effective_dt,
        )
        u_tilt = self._axis_command(
            active_tilt, tilt_gain, self.deadband_tilt,
            self.cmd_accel_tilt_deg_s2, self.smooth_cmd_tilt, effective_dt,
        )

        max_step = self.max_step_for(dt)
        u_pan = float(np.clip(u_pan, -max_step, max_step))
        u_tilt = float(np.clip(u_tilt, -max_step, max_step))
        self.smooth_cmd_pan = u_pan
        self.smooth_cmd_tilt = u_tilt
        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic durumlarini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0
