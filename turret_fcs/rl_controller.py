"""
================================================================================
MODUL: RL KONTROLCU (SAF PPO CIKARIM MOTORU)
================================================================================
Sinif adi RLController'dir. v1.4 politikasi `turret_ppo_v14.zip` /
`turret_ppo_v14_policy.npz` olarak egitilir; cikarim SB3/torch surumune
bagimli degildir.

TASARIM GEREKCESI:
Onceki RL sarmalayicisi politikayi egittigi ortamdan farkli bir kapali cevrime
sokuyordu:
1. Pan cikisina EMA + 0.10 derece/kare slew, tilt cikisina ayri EMA ve
   yakin kilit bolgesinde asimetrik sonum uyguluyordu. Politika bu filtrelerle
   egitilmedi; filtreler faz gecikmesi ekleyip mikro duzeltmeleri buyuttu.
2. Eylem gecmisi (Smith Predictor) filtrelenmis motor komutunu sakliyordu.
   Politika ise egitimde kendi ham aksiyonunu goruyordu.
3. Yazilim olu bolgesi (0.85 / 0.95 derece) egitimde yoktu.
4. Hedef kaybinda eylem gecmisi temizlenmiyordu.

TASARIM (v1.4, SAF RL):
- Gozlem vektoru egitimle ayni: 11 boyut, ayni Bryson olcekleri.
- Gozlem, P kontrolcunun EMA'li hatasindan degil, olculen bbox'tan uretilir.
- Hiz gozlemi (obs[4:6]) runtime'da sifirlanir: Webots/gercek Kalman hizi
  gurultulu ve politika onunla doyuma giriyor. Politika egitimde gercek hizla
  egitildi; statik hedefte sifir hiz zaten dagilimda oldugu icin sifir guvenli.
- Cikarim, SB3/torch gerektirmeyen 11-64-64-2 NumPy MLP ile yapilir.
- Eylem gecmisi uygulanan normalize komutu saklar.
- Cikista EMA/slew yok; yalnizca:
  * 5 px yumusak olu bolge (politikanin sifir-hata bias'ini bastirir),
  * yon degistiren aksiyonda 0.5 sonum (ileri-geri mikro salinimi keser).
- P harmani YOKTUR: son komut dogrudan RL aksiyonudur.
- Egitim plant modeli: 1.2 derece backlash, 1.5 derece/kare adim siniri,
  0.10 derece servo deadband, progress odulu (k_p=2.0, k_e=0.05,
  lambda_u=0.02, mu_du=0.08).
================================================================================
"""

from pathlib import Path
import numpy as np

from .config import CONFIG


def rate_scaled_step_limits(max_step_pan_deg, max_step_tilt_deg, dt,
                            nominal_dt=0.03333, max_output_deg=None):
    """Kare basi adim limitini dt'ye gore olcekler (egitimle ayni semantik)."""
    dt_safe = max(1e-3, float(dt))
    scale = float(np.clip(dt_safe / max(1e-6, float(nominal_dt)), 0.25, 3.0))
    cap = float(CONFIG.control.max_output_deg if max_output_deg is None else max_output_deg)
    pan = min(float(max_step_pan_deg) * scale, cap)
    tilt = min(float(max_step_tilt_deg) * scale, cap)
    return float(pan), float(tilt), scale


class _NumpyPolicy:
    """SB3/torch bagimliligi olmayan 11-64-64-2 MLP cikarimi."""

    def __init__(self, path):
        data = np.load(path, allow_pickle=False)
        self.w1 = data["w1"].astype(np.float32)
        self.b1 = data["b1"].astype(np.float32)
        self.w2 = data["w2"].astype(np.float32)
        self.b2 = data["b2"].astype(np.float32)
        self.w3 = data["w3"].astype(np.float32)
        self.b3 = data["b3"].astype(np.float32)
        self.obs_dim = int(data["obs_dim"][0])
        self.act_dim = int(data["act_dim"][0])
        if self.w1.shape != (64, self.obs_dim) or self.w3.shape != (self.act_dim, 64):
            raise ValueError("npz katman boyutlari uyumsuz")

    def predict(self, obs, deterministic=True):
        x = np.asarray(obs, dtype=np.float32).reshape(self.obs_dim)
        x = np.tanh(x @ self.w1.T + self.b1)
        x = np.tanh(x @ self.w2.T + self.b2)
        action = np.clip(x @ self.w3.T + self.b3, -1.0, 1.0)
        return action.astype(np.float32), None


class RLController:
    """PPO politikasini egitim semantigine sadik calistiran temiz cikarim motoru."""

    OBS_DIM = 11
    ACT_DIM = 2

    # FCS bu bayraklarla gözlem kaynagini secer:
    # - Ölçülen bbox (P EMA'si degil), egitimdeki piksel hatasiyla ayni.
    # - Yumusatilmis Kalman hizi, egitimdeki gercek hedef hizina daha yakin.
    uses_measured_bbox = True
    prefers_smooth_velocity = False
    uses_velocity_obs = False

    def __init__(self, model_path=None, auto_load=True,
                 deadband_px=5.0, deadband_exit_px=8.0, velocity_gate_px_s=30.0,
                 reversal_damping=0.5):
        if model_path is None:
            model_path = Path(__file__).resolve().parent / "rl_sim" / "models" / "turret_ppo_v14.zip"
        self.model_path = Path(model_path)
        self.model = None
        self.is_loaded = False
        self.policy_format = None
        self.load_error = None
        if auto_load:
            self.load_model()

        self.max_step_pan_deg = CONFIG.actuator.max_step_pan_deg
        self.max_step_tilt_deg = CONFIG.actuator.max_step_tilt_deg
        self.max_step_deg = self.max_step_pan_deg
        self.v_max_px_s = 300.0
        self.dist_min_m = 0.5
        self.dist_max_m = 5.0

        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.has_prev_target = False
        self.last_action = np.zeros(2, dtype=np.float32)
        self.prev_policy_action = np.zeros(2, dtype=np.float32)
        self._in_deadband = False
        self.deadband_px = max(0.0, float(deadband_px))
        self.deadband_exit_px = max(self.deadband_px, float(deadband_exit_px))
        self.velocity_gate_px_s = max(0.0, float(velocity_gate_px_s))
        self.reversal_damping = float(np.clip(float(reversal_damping), 0.0, 1.0))

    def load_model(self):
        """NumPy politikasini (varsa) veya SB3 zip modelini yukler."""
        npz_path = (
            self.model_path
            if self.model_path.suffix == ".npz"
            else self.model_path.with_name(self.model_path.stem + "_policy.npz")
        )
        if npz_path.exists():
            try:
                model = _NumpyPolicy(npz_path)
                if model.obs_dim != self.OBS_DIM or model.act_dim != self.ACT_DIM:
                    raise ValueError(f"npz boyutu uyumsuz: obs={model.obs_dim}, action={model.act_dim}")
                self.model = model
                self.is_loaded = True
                self.policy_format = "numpy"
                self.load_error = None
                return True
            except Exception as exc:
                self.load_error = f"npz yuklenemedi: {exc}"

        try:
            from stable_baselines3 import PPO

            if not self.model_path.exists():
                self.load_error = self.load_error or f"model bulunamadi: {self.model_path}"
                return False
            model = PPO.load(str(self.model_path), device="cpu")
            obs_shape = tuple(getattr(model.observation_space, "shape", ()))
            act_shape = tuple(getattr(model.action_space, "shape", ()))
            if obs_shape != (self.OBS_DIM,) or act_shape != (self.ACT_DIM,):
                self.load_error = f"model boyutu uyumsuz: obs={obs_shape}, action={act_shape}"
                return False
            self.model = model
            self.is_loaded = True
            self.policy_format = "sb3"
            self.load_error = None
            return True
        except Exception as exc:  # model bozuksa FCS P moduna duser
            self.load_error = str(exc)
            self.model = None
            self.is_loaded = False
            return False

    def reset(self):
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.has_prev_target = False
        self.last_action = np.zeros(2, dtype=np.float32)
        self.prev_policy_action = np.zeros(2, dtype=np.float32)
        self._in_deadband = False

    def on_target_lost(self):
        """Hedef kaybinda gecmisi temizle; yeniden yakalamada hayalet aksiyon kalmasin."""
        self.reset()

    def _build_observation(self, bx, by, center_x, center_y, screen_w, screen_h,
                           vx, vy, dist_m, dt):
        err_x_norm = float(np.clip((bx - center_x) / (screen_w / 2.0), -1.5, 1.5))
        err_y_norm = float(np.clip((by - center_y) / (screen_h / 2.0), -1.5, 1.5))

        if not self.has_prev_target:
            self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)
            self.d_err_filt = np.zeros(2, dtype=np.float32)
            d_err_x = 0.0
            d_err_y = 0.0
            self.has_prev_target = True
        else:
            raw_d_x = (err_x_norm - self.last_err[0]) / max(0.005, dt) * 0.05
            raw_d_y = (err_y_norm - self.last_err[1]) / max(0.005, dt) * 0.05
            self.d_err_filt[0] = 0.40 * raw_d_x + 0.60 * self.d_err_filt[0]
            self.d_err_filt[1] = 0.40 * raw_d_y + 0.60 * self.d_err_filt[1]
            d_err_x = float(np.clip(self.d_err_filt[0], -1.0, 1.0))
            d_err_y = float(np.clip(self.d_err_filt[1], -1.0, 1.0))
            self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        dist_val = float(dist_m) if (dist_m is not None and dist_m > 0.2) else 2.0
        dist_norm = float(np.clip(
            (dist_val - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0
        )) * 2.0 - 1.0

        return np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                float(np.clip(vx / self.v_max_px_s, -1.0, 1.0)),
                float(np.clip(vy / self.v_max_px_s, -1.0, 1.0)),
                dist_norm,
                self.act_hist_1[0],
                self.act_hist_1[1],
                self.act_hist_2[0],
                self.act_hist_2[1],
            ],
            dtype=np.float32,
        )

    def predict_angles(self, bx, by, center_x, center_y, screen_w, screen_h,
                       vx, vy, dist_m, curr_pan_deg, curr_tilt_deg, dt):
        """Politikadan egitimdeki gibi ham aksiyon alir ve fiziksel adima cevirir."""
        if not self.is_loaded or self.model is None:
            return 0.0, 0.0

        dt_safe = max(0.005, float(dt))
        obs = self._build_observation(
            bx, by, center_x, center_y, screen_w, screen_h, vx, vy, dist_m, dt_safe
        )
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=np.float32).reshape(self.ACT_DIM)
        if not np.all(np.isfinite(action)):
            action = np.zeros(self.ACT_DIM, dtype=np.float32)

        max_pan_step, max_tilt_step, _ = rate_scaled_step_limits(
            self.max_step_pan_deg, self.max_step_tilt_deg, dt_safe
        )

        # Yon degistiren aksiyonu yumusat: ayni yondeki takibi geciktirmeden
        # mikro salinimi (ileri-geri titreme) azaltir.
        damped = action.copy()
        for axis in range(self.ACT_DIM):
            if action[axis] * self.prev_policy_action[axis] < 0.0:
                damped[axis] *= self.reversal_damping

        d_pan_deg = float(damped[0] * max_pan_step)
        d_tilt_deg = float(damped[1] * max_tilt_step)

        # Kucuk hata + yavas hedef: politikanin sifir-hata bias'ini bastir.
        err_px = float(np.hypot(bx - center_x, by - center_y))
        speed_px = float(np.hypot(vx, vy))
        if self._in_deadband:
            if err_px > self.deadband_exit_px or speed_px > self.velocity_gate_px_s:
                self._in_deadband = False
        elif err_px <= self.deadband_px and speed_px <= self.velocity_gate_px_s:
            self._in_deadband = True
        if self._in_deadband:
            d_pan_deg = 0.0
            d_tilt_deg = 0.0

        d_pan_deg = float(np.clip(d_pan_deg, -max_pan_step, max_pan_step))
        d_tilt_deg = float(np.clip(d_tilt_deg, -max_tilt_step, max_tilt_step))

        # Egitimdeki gibi normalize komut gecmisi (deadband/reversal sonrasi).
        applied = np.array(
            [d_pan_deg / max(1e-6, max_pan_step), d_tilt_deg / max(1e-6, max_tilt_step)],
            dtype=np.float32,
        )
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = applied
        self.last_action = applied
        self.prev_policy_action = action
        return d_pan_deg, d_tilt_deg
