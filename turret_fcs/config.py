"""
================================================================================
MODUL: TARET ATIS KONTROL SISTEMI KONFIGURASYONU (SSOT - CONFIG)
================================================================================
Tum alt sistemlerin (Optik, Kontrol, Balistik, Eyleyici, Kilit, Yollar ve Durum)
ortak fiziksel ve algoritmik parametrelerini tek bir kaynakta (Single Source of Truth)
toplayan, harici kutuphane bagimliligi olmayan saf yaprak moduldur.
================================================================================
"""

from dataclasses import dataclass
from enum import IntEnum
import math


class FCSState(IntEnum):
    """
    FCS durum makinesi ve STM32 donanim protokol durumlari.
    STM32 firmware main.c protokolu ile birebir eslesir:
      0: TRACKING  (Hedef arama / dinamik takip)
      1: LOCKED    (Hedef kilit alaninda)
      2: FIRING    (Atis / Solenoid tahrigi - firmware revizyonunda rezerve)
      3: ESTOP     (Acil durum durdurma / tum motorlar enerjisiz)
    """
    TRACKING = 0
    LOCKED = 1
    FIRING = 2
    ESTOP = 3


@dataclass(frozen=True)
class OpticsConfig:
    """Logitech C270 kamera optik parametreleri."""
    h_fov_deg: float = 48.8
    v_fov_deg: float = 28.6
    default_width: int = 1280
    default_height: int = 720

    def compute_fx(self, width: int = 1280) -> float:
        return float((width / 2.0) / math.tan(math.radians(self.h_fov_deg / 2.0)))

    def compute_fy(self, height: int = 720) -> float:
        return float((height / 2.0) / math.tan(math.radians(self.v_fov_deg / 2.0)))


@dataclass(frozen=True)
class ControlConfig:
    """P kontrolcu ve filtre parametreleri."""
    kp_pan: float = 0.22        # P: 30 FPS bazinda kare basi kazanc
    kp_tilt: float = 0.11       # P: 30 FPS bazinda kare basi kazanc
    deadband_pan_deg: float = 0.85
    deadband_tilt_deg: float = 0.95
    max_output_deg: float = 1.50
    max_command_step_deg: float = 1.50  # Firmware Servo_ApplyCorrection ust siniri
    far_gain: float = 1.6                # 5 derece ustu hatada Kp carpani
    far_error_deg: float = 5.0           # Kazanc planlamasinin basladigi hata
    cmd_accel_pan_deg_s2: float = 120.0  # P/RL pan komut ivme siniri
    cmd_accel_tilt_deg_s2: float = 80.0  # P/RL tilt komut ivme siniri
    lead_time_offset_s: float = 0.015    # Balistik lead'e eklenen sabit pay
    lead_time_min_s: float = 0.03        # Lead sure alt siniri
    lead_time_max_s: float = 0.25        # Lead sure ust siniri
    alpha_filter: float = 0.85           # P giris EMA'sinda yeni deger agirligi
    los_alpha: float = 0.70              # Dunya LOS konum duzeltmesi (log kalibrasyonu)
    los_beta: float = 0.20               # Dunya LOS hiz duzeltmesi (log kalibrasyonu)
    ego_tau_s: float = 0.030             # Ego icin komut adimi servo zaman sabiti


@dataclass(frozen=True)
class ActuatorConfig:
    """Eyleyici ve servo mekanik adim / kuantalama limitleri."""
    max_step_pan_deg: float = 1.50     # RL kare basi maksimum Pan adimi (30 FPS'te 45 deg/s)
    max_step_tilt_deg: float = 0.75    # RL kare basi maksimum Tilt adimi (30 FPS'te 22.5 deg/s - donanim eylemsizlik uyumu)
    servo_deadband_deg: float = 0.10   # STM32 0.1 deg UART kuantalama ve servo mekanik olu bolgesi


@dataclass(frozen=True)
class KalmanConfig:
    """Sabit hizli (CV) Kalman filtresi: tek Q/R cifti ve robust tekil-sicrama kapisi."""
    r_var: float = 64.0                  # Olcum gurultusu varyansi (px^2), ~8 px standart sapma
    sigma_a: float = 300.0               # Surec ivme gurultusu (px/s^2), ~12 deg/s^2
    initial_velocity_var: float = 90000.0  # Ilk hiz kovaryansi (px/s)^2, hizli yakinsama icin
    outlier_residual_px: float = 150.0   # Bu inovasyon ustunde olcum gurultusu buyutulur
    outlier_r_scale: float = 100.0       # Tekil sicramada R carpani (kazanci bastirir)
    reacquire_frames: int = 3            # Ust uste bu kadar sicramada takip yeniden baslar
    velocity_deadband_low_px_s: float = 30.0   # Bu hizin altinda cikis hizi sifir
    velocity_deadband_high_px_s: float = 60.0  # Bu hizdan itibaren cikis hizi tam gecerli


@dataclass(frozen=True)
class BallisticsConfig:
    """Dis balistik, yercekimi kompanzasyonu ve atesleme zamanlamalari."""
    projectile_speed_mps: float = 10.0
    gravity_accel: float = 9.80665
    default_distance_m: float = 2.0
    # Atis sekansi zamanlamalari (saniye)
    spinup_time_s: float = 1.00     # Volan motorlarinin maksimum devre ve balistik slew acisina ulasma suresi
    settle_time_s: float = 0.15     # Slew sonrasi servo oturma (sabit hold; lead/gravity canli)
    firing_pulse_s: float = 0.08    # Solenoid itici darbe suresi (80 ms - STM32 firmware ile uyumlu)
    recovery_time_s: float = 0.35   # Mekanik geri toplama suresi
    eval_display_time_s: float = 1.50  # Atis sonrasi EVAL gosterim suresi
    system_latency_s: float = 0.080 # 80 ms kamera-yolo-uart-solenoid toplam tasiyici gecikmesi


@dataclass(frozen=True)
class RLConfig:
    """RL cikarim ve egitim ortaminin ortak sabitleri (tek kaynak)."""
    deadband_px: float = 5.0             # Merkezde politika bias'ini bastiran olu bolge
    deadband_exit_px: float = 8.0        # Olu bolgeden cikis histerezisi
    velocity_gate_px_s: float = 30.0     # Bu hizin ustunde olu bolge uygulanmaz
    reversal_damping: float = 0.5        # Yon degistiren aksiyonun sonum orani
    taper_pan_deg: float = 1.0           # Soft deadband gecis genisligi (pan)
    taper_tilt_deg: float = 2.5          # Soft deadband gecis genisligi (tilt)
    v_max_px_s: float = 300.0            # Hiz gozlemi Bryson olcegi
    dist_min_m: float = 0.5              # Mesafe gozlemi alt siniri
    dist_max_m: float = 5.0              # Mesafe gozlemi ust siniri
    yolo_noise_std: float = 3.5          # Ortam varsayilan YOLO kutu gurultusu (px)
    yolo_noise_std_min: float = 2.0      # Domain randomization alt siniri
    yolo_noise_std_max: float = 5.0      # Domain randomization ust siniri
    servo_tau_min_s: float = 0.035       # Domain randomization servo zaman sabiti
    servo_tau_max_s: float = 0.065
    episode_steps: int = 300             # Egitim bolumu uzunlugu


@dataclass(frozen=True)
class LidarConfig:
    """TF-Luna menzil kapilama, sicrama reddi ve kararlilik filtreleri."""
    min_distance_m: float = 0.20
    max_distance_m: float = 8.00
    jump_threshold_m: float = 0.50       # Son gecerli mesafeden 50 cm'den fazla sapma = supheli
    stability_window: int = 3            # Ates icin gereken kararli okuma sayisi
    stability_tolerance_m: float = 0.10  # Bu pencerede izin verilen mesafe dalgalanmasi
    reacquire_frames: int = 3            # Gercek menzil degisimini kabul icin ust uste kare
    geometry_tolerance_ratio: float = 0.75  # Kutu boyutundan mesafe ile LiDAR uyumsuzlugu


@dataclass(frozen=True)
class FlywheelConfig:
    """Cift volan motoru: kilitliyken bosta donme ve ates boost parametreleri."""
    target_speed_rad_s: float = 2460.9   # 23.500 RPM nominal atis devri
    idle_speed_rad_s: float = 1600.0     # Kilitliyken bosta donme devri (~%65)
    idle_spinup_time_s: float = 0.25     # Bostan tam devre + kisa slew suresi
    cold_spinup_time_s: float = 1.00     # Bosta degilse emniyetli tam spin-up


@dataclass(frozen=True)
class MountConfig:
    """Gimbal mekanik limitleri ve servo dinamikleri."""
    pan_limit_min: float = -75.0
    pan_limit_max: float = 75.0
    tilt_limit_min: float = -25.0
    tilt_limit_max: float = 25.0
    pan_invert: bool = True
    tilt_invert: bool = False
    servo_tau: float = 0.045        # 1. derece motor zaman sabiti (45 ms)


@dataclass(frozen=True)
class LockConfig:
    """Kilit kararlilik sayaci ve histerezis parametreleri."""
    lock_threshold_frames: int = 10
    lock_exit_margin_deg: float = 0.60
    target_radius_m: float = 0.15
    sigma_weapon_deg: float = 1.25   # DIY cift volan ve sunger dart gercekci ic balistik sacilmasi (derece)
    post_fire_grace_duration_frames: int = 22  # 0.75 saniye @ 30 FPS kilit muafiyeti (grace period)


@dataclass(frozen=True)
class PathConfig:
    """Agirlik ve yapay zeka model dosya yollari."""
    yolo_model: str = "runs/detect/balloon_detector/weights/best.pt"
    fallback_yolo_model: str = "weights/yolov8n.pt"


@dataclass(frozen=True)
class TurretConfig:
    optics: OpticsConfig = OpticsConfig()
    control: ControlConfig = ControlConfig()
    actuator: ActuatorConfig = ActuatorConfig()
    kalman: KalmanConfig = KalmanConfig()
    ballistics: BallisticsConfig = BallisticsConfig()
    rl: RLConfig = RLConfig()
    flywheel: FlywheelConfig = FlywheelConfig()
    lidar: LidarConfig = LidarConfig()
    mount: MountConfig = MountConfig()
    lock: LockConfig = LockConfig()
    paths: PathConfig = PathConfig()


CONFIG = TurretConfig()
