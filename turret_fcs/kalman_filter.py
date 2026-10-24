"""
================================================================================
MODUL: HEDEF DURUM KESTIRIMI - SABIT HIZLI KALMAN FILTRESI (CV KALMAN)
================================================================================
Kamera piksel duzleminde hedefin konumunu (x, y) ve dunya referansli hizini
(vx, vy) kestiren sade, tek katmanli bir dogrusal Kalman filtresi.

TASARIM KARARLARI (neden bu kadar sade?):
1. Durum vektoru X = [x, y, vx, vy]^T, olcum Z = [x, y]^T.
   Sabit hizli (Constant Velocity, CV) model yeterlidir; hedef balonun
   manevralari surec gurultusu (sigma_a) ile temsil edilir. IMM/CA gibi
   karmasik modellere gerek yoktur.
2. Ego-hareket telafisi (ego_dx, ego_dy) dogrudan kestirim adiminda uygulanir.
   Boylece hiz durumu dunya referanslidir; taret donerken hedefin gercek
   hizi ile kamera kaynakli gorunur kayma birbirine karismaz.
3. Tek bir surec gurultusu (sigma_a) ve tek bir olcum gurultusu (r_var)
   kullanilir. Adaptif Q, hiz EMA'si, ivme sinirlamasi, donus dedektoru ve
   olcum egimi gibi katmanlar kaldirilmistir; bunlar gecikme ve yanlis yon
   uretiyordu.
4. Tek karelik buyuk bbox sicramalari R buyutulerek bastirilir. Ayni buyuk
   sapma ust uste `reacquire_frames` kez gelirse hedef gercekten sicramis
   (yeniden yakalanmis) demektir; konum olcume sifirlanir, hiz sifirlanir.
   Boylece tek bir sahte tespit filtrenin hizini bozamaz.
5. Cikista yalnizca yumusak bir hiz olu bolgesi vardir. Bu bir filtre katmani
   degil, duran hedefin sifir civarindaki gurultusunu lead noktasindan uzak
   tutan statik bir esiklemedir.
================================================================================
"""

import numpy as np

from .config import CONFIG


class KalmanFilter:
    """2B konum ve hiz icin sabit hizli (CV) Kalman filtresi.

    Durum: X = [x, y, vx, vy]^T  (piksel, piksel/s, dunya referansli)
    Olcum: Z = [x, y]^T          (piksel)
    """

    def __init__(
        self,
        dt=0.033,
        r_var=CONFIG.kalman.r_var,
        sigma_a=CONFIG.kalman.sigma_a,
        outlier_residual=CONFIG.kalman.outlier_residual_px,
        outlier_r_scale=CONFIG.kalman.outlier_r_scale,
        reacquire_frames=CONFIG.kalman.reacquire_frames,
        initial_velocity_var=CONFIG.kalman.initial_velocity_var,
        deadband_low=CONFIG.kalman.velocity_deadband_low_px_s,
        deadband_high=CONFIG.kalman.velocity_deadband_high_px_s,
    ):
        self.dt = float(dt)
        self.r_var = float(r_var)
        self.sigma_a = float(sigma_a)
        self.outlier_residual = float(outlier_residual)
        self.outlier_r_scale = float(outlier_r_scale)
        self.reacquire_frames = max(1, int(reacquire_frames))
        self.initial_velocity_var = float(initial_velocity_var)
        self.velocity_deadband_low_px_s = float(deadband_low)
        self.velocity_deadband_high_px_s = float(deadband_high)

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)
        self.R = np.eye(2, dtype=float) * self.r_var

        self.reset()

    def reset(self):
        """Filtreyi ilk kare durumuna dondurur."""
        self.x = np.zeros((4, 1), dtype=float)
        self.P = self._initial_covariance()
        self.initialized = False
        self.last_residual = 0.0
        self.current_sigma_a = self.sigma_a
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self._outlier_frames = 0

    def _initial_covariance(self):
        return np.diag([
            self.r_var,
            self.r_var,
            self.initial_velocity_var,
            self.initial_velocity_var,
        ])

    def _start_track(self, mx, my):
        """Yeni takip baslatir: konum olcumden, hiz sifirdan."""
        self.x = np.array([[float(mx)], [float(my)], [0.0], [0.0]], dtype=float)
        self.P = self._initial_covariance()
        self.initialized = True
        self.last_residual = 0.0
        self.raw_vx = 0.0
        self.raw_vy = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self._outlier_frames = 0
        return float(mx), float(my), 0.0, 0.0

    def _process_noise(self, dt):
        """CWNA surec gurultusu: Q = sigma_a^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]."""
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        q = self.sigma_a * self.sigma_a
        return np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ], dtype=float) * q

    def update(self, measurement, dt=0.033, ego_dx=0.0, ego_dy=0.0):
        """Bir olcumu filtreye isler.

        measurement: (mx, my) piksel merkezi
        dt: onceki olcumden bu yana gecen sure (s)
        ego_dx, ego_dy: taretin kendi donusunden kaynaklanan piksel kaymasi

        Returns: (px, py, smooth_vx, smooth_vy)
        """
        dt = max(0.010, min(float(dt), 0.100))
        mx, my = float(measurement[0]), float(measurement[1])

        if not self.initialized:
            return self._start_track(mx, my)

        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        # 1. Kestirim: hedef hizi + bilinen ego kaymasi
        x_pred = F @ self.x
        x_pred[0, 0] += float(ego_dx)
        x_pred[1, 0] += float(ego_dy)
        P_pred = F @ self.P @ F.T + self._process_noise(dt)

        # 2. Inovasyon
        z = np.array([[mx], [my]], dtype=float)
        nu = z - self.H @ x_pred
        residual = float(np.hypot(nu[0, 0], nu[1, 0]))
        self.last_residual = residual

        # 3. Tek karelik sicramayi bastir; kalici sicramada takibi yeniden baslat.
        r_eff = self.R
        if residual > self.outlier_residual:
            self._outlier_frames += 1
            if self._outlier_frames >= self.reacquire_frames:
                return self._start_track(mx, my)
            r_eff = self.R * self.outlier_r_scale
        else:
            self._outlier_frames = 0

        # 4. Kalman kazanci ve durum guncellemesi
        S = self.H @ P_pred @ self.H.T + r_eff
        K = np.linalg.solve(S, self.H @ P_pred).T
        self.x = x_pred + K @ nu

        # 5. Joseph formu ile sayisal kararli kovaryans guncellemesi
        i_kh = np.eye(4, dtype=float) - K @ self.H
        self.P = i_kh @ P_pred @ i_kh.T + K @ r_eff @ K.T

        # 6. Cikti: ham hiz + yumusak olu bolge (ek filtre katmani yok)
        self.raw_vx = float(self.x[2, 0])
        self.raw_vy = float(self.x[3, 0])
        speed = float(np.hypot(self.raw_vx, self.raw_vy))
        band = max(1e-3, self.velocity_deadband_high_px_s - self.velocity_deadband_low_px_s)
        scale = float(np.clip(
            (speed - self.velocity_deadband_low_px_s) / band, 0.0, 1.0
        ))
        self.smooth_vx = self.raw_vx * scale
        self.smooth_vy = self.raw_vy * scale

        return float(self.x[0, 0]), float(self.x[1, 0]), self.smooth_vx, self.smooth_vy

    def predict_next_position(self, lead_time):
        """lead_time saniye sonraki hedef konumunu ve hizini dondurur.

        Hiz dunya referansli oldugu icin lead, kameranin gelecekteki donusunu
        degil hedefin kendi hareketini temsil eder.
        """
        px, py = float(self.x[0, 0]), float(self.x[1, 0])
        vx, vy = float(self.smooth_vx), float(self.smooth_vy)
        return (
            px + vx * float(lead_time),
            py + vy * float(lead_time),
            vx,
            vy,
        )

    def get_velocity(self):
        """Yumusak olu bolgeden gecmis hedef hizini (vx, vy) px/s olarak doner."""
        return float(self.smooth_vx), float(self.smooth_vy)

    def get_angular_velocity_deg_s(self, fx=None, fy=None):
        """Piksel hizini kamera odak uzakligina gore acisal hiza cevirir."""
        fx_val = max(float(fx), 1.0) if fx is not None else CONFIG.optics.compute_fx(CONFIG.optics.default_width)
        fy_val = max(float(fy), 1.0) if fy is not None else CONFIG.optics.compute_fy(CONFIG.optics.default_height)
        omega_pan = float(np.degrees(self.smooth_vx / fx_val))
        omega_tilt = -float(np.degrees(self.smooth_vy / fy_val))
        return omega_pan, omega_tilt
