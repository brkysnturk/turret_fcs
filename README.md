# Otonom Asimetrik Hava Tehdidi Tespit ve Atış Kontrol Sistemi (Turret FCS)

## Yıldız Teknik Üniversitesi — Kontrol ve Otomasyon Mühendisliği Bölümü

### Bitirme Çalışması & Tasarım Projesi (v1.4 Production Release)

[![Release](https://img.shields.io/badge/Release-v1.4-success.svg)](https://github.com/brkysnturk/turret_fcs/releases/tag/v1.4)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![CI](https://github.com/brkysnturk/turret_fcs/actions/workflows/ci.yml/badge.svg)](https://github.com/brkysnturk/turret_fcs/actions)
[![STM32 Nucleo](https://img.shields.io/badge/Hardware-STM32F401RE-002B49.svg)](https://www.st.com/)
[![Webots Simulator](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8n-00FFFF.svg)](https://ultralytics.com/)
[![Reinforcement Learning](https://img.shields.io/badge/RL-Pure%20PPO%20NumPy-orange.svg)](https://stable-baselines3.readthedocs.io/)
[![Control Theory](https://img.shields.io/badge/Control-Soft--Deadband%20Type--1%20P-brightgreen.svg)](https://en.wikipedia.org/wiki/Proportional_control)

Bu proje; Yıldız Teknik Üniversitesi Kontrol ve Otomasyon Mühendisliği Bölümü bitirme çalışması kapsamında geliştirilmiş; mikro/mini İnsansız Hava Araçları (İHA) ve asimetrik hava hedeflerine karşı elektro-optik tespit, balistik önleme ve kararlı hedef takibi gerçekleştiren otonom bir Atış Kontrol Sistemidir (Turret FCS).

> **English Overview:** Autonomous Air Defense Turret Fire Control System (Turret FCS) developed as a Senior Capstone Project at Yildiz Technical University. Features dual-mode tracking (Type-1 P Controller and 11-D Pure PPO Reinforcement Learning), CV Kalman filter with 1st-order actuator observer for ego-motion compensation, closed-form 2D Gaussian hit probability (Marcum Q), zero-lag lead computation, and 100 Hz STM32F401RE embedded firmware integration.

Sistem; **Logitech C270 RGB Kamera (720p @ 30 FPS)** ve **Benewake TF-Luna ToF LiDAR (100 Hz UART)** sensör füzyonu, **YOLOv8n Derin Öğrenme Hedef Tespiti**, **ByteTrack Çoklu Hedef Takibi**, **Ego-Hareket Telafili Sabit Hızlı (CV) Kalman Filtresi**, **Yumuşak Ölü Bölgeli Tip-1 P Kontrolcü**, **11-Boyutlu Saf PPO Pekiştirmeli Öğrenme Ajanı**, **Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q / ncx2 CDF)** ve **STM32F401RE Gömülü Denetleyici** omurgası üzerinde gerçek zamanlı olarak çalışmaktadır.

---

## 🚀 v1.4 Sürüm İnovasyonları

* **Saf PPO Pekiştirmeli Öğrenme Mimarisi (Pure RL Architecture):**
  * Önceki sürümlerdeki karmaşık P harmanlama (`0.7 P + 0.3 RL`) ve harici filtre katmanları tamamen kaldırıldı; RL ajanı doğrudan kendi politikasını uygular hale getirildi.
  * Eğitim ortamı fiziksel plant ile birebir hizalandı: 1.20° mekanik dişli boşluğu (backlash), 0.10° servo ölü bölgesi ve 1.50°/kare donanımsal adım sınırı bağımsız olarak modellendi.
  * Hata azalışını doğrudan ödüllendiren, motor sarsıntısını ve yön değişimlerini cezalandıran `Progress Reward` formülasyonu ile eğitilen model, `turret_ppo_v14_policy.npz` olarak dışa aktarıldı (11-64-64-2 NumPy MLP; PyTorch veya Stable-Baselines3 bağımlılığı olmadan 0.03 ms çıkarım).
  * Gerçek donanım telemetrisinde P kontrolcüyü aşan başarım: %65.5 kilit kararlılığı, 2.61° Pan RMS hatası ve merkezde 0.031° titreşimsiz komut başarımı.

* **Yeniden Tasarlanan Tip-1 P Kontrolcü Omurgası:**
  * Pan ekseni için `Kp_pan = 0.22`, Tilt ekseni için `Kp_tilt = 0.11` taban kazançları doğrulandı (Faz Marjı: 45.1°, Kazanç Marjı: 7.2 dB).
  * 6 derece/s ile sınırlayan sabit hız adımı kaldırıldı; yerine mekanik rezonansı ve dişli aşınmasını önleyen 400°/s² Pan ve 267°/s² Tilt ivme sınırlaması (acceleration limiting) getirildi.
  * Çift EMA katmanı elendi; tek EMA filtresi (alpha = 0.85) ve yumuşak ölü bölge taper (Pan 0.85° - 1.00°, Tilt 0.95° - 2.50°) ile merkezde 0.030° komut genliğiyle sıfır avlanma (chattering) elde edildi.

* **Ego-Hareket Telafili ve Tek Q/R CV Kalman Filtresi:**
  * Durum vektörü `X = [x, y, vx, vy]`, tek süreç gürültüsü (`sigma_a = 300 px/s²`) ve tek ölçüm gürültüsü (`r_var = 64 px²`) içeren sabit hızlı (CV) model kuruldu.
  * Sayısal kararlılık ve matris simetrisini korumak için kovaryans güncellemesi Joseph formuna geçirildi.
  * 1. Derece aktüatör durum gözlemcisi (`tau = 45 ms`) ile taretin kendi dönüşünün hedef optik hızına bindirdiği yapay hız elendi (Ego-Motion Compensation); gerçek donanım loglarında hız korelasyonu 0.44'ten 0.83'e yükseldi.
  * 150 px tek karelik aykırı değer kapılama (outlier gate) ve 30-60 px/s yumuşak hız ölü bölgesi ile duran hedeflerde sahte lead kayması tamamen engellendi.

* **Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q / ncx2 CDF):**
  * Sezgisel mesafe zayıflatması yerine, dairesel hedef alanı üzerinde eksantrik 2B izotropik Gauss integrali kapalı formda `scipy.stats.ncx2.cdf` ile modellendi.
  * Sabit yarıçap varsayımı kaldırıldı; YOLO tespit sınırlayıcı kutusundan anlık hedef açısal boyutu (`theta_hedef`) dinamik türetilir.
  * 80 ms donanım gecikmesi ve hedef hareket varyansı (`sigma_hareket = 0.25 * omega * (t_ucus + 0.08 saniye)`), silah dağılımı (`sigma_silah = 1.25°`) ile birleştirilerek kesin vuruş olasılığı hesaplanır.

* **Dinamik Kilit Histerezisi ve Kademeli Atış Sekansı:**
  * Hedef hızına göre dinamik genişleyen kilit penceresi (1.60° - 3.00° giriş, +0.60° çıkış histerezisi) ve 10-kare kesintisiz kararlılık filtresi entegre edildi.
  * 5-Aşamalı deterministik atış sekansı: 1.0 s Spinup (23.500 RPM) & Slew feedforward -> 0.15 s Settle (sabit oturma) -> 80 ms Solenoid darbesi -> 0.35 s Recover (solenoid geri çekilme) -> 1.50 s Eval (atış değerlendirme & 0.75 s kilit muafiyeti).

* **Sertleştirilmiş LiDAR Menzil Kapılama (Range Gating):**
  * Atış anında menzil dondurma, 50 cm ani mesafe sıçrama reddi, 3-kare/10 cm kararlılık kapısı ve geometrik sınır denetimi ile LiDAR ışınının arka plana kaçması önlendi.

* **Deterministik Operatör Yetkisi ve Taktiksel HUD v1.4:**
  * Kararsız otomatik hibrit anahtarlama yerine operatörün `[M]` tuşu ile seçtiği bağımsız `[P CONTROLLER]` ve `[RL CONTROLLER]` mod yapısı kesinleştirildi.
  * HUD üzerinde askeri standartlara uygun büyük harfli durum afişleri, dinamik vuruş olasılığı, LEAD nişangahı ve muharebe sadeleştirmesi (Combat Declutter) devreye alındı.

---

## 🚀 Önceki Sürüm İnovasyonları

### v1.3
* **SSOT (Single Source of Truth) Merkezi Yapılandırması (`turret_fcs/config.py`):** Tüm optik parametreler, aktüatör sınırları, kontrolcü kazançları, Kalman katsayıları ve dosya yolları saf matematiksel tek bir yaprak modülde (`CONFIG`) toplandı; dairesel bağımlılıklar giderildi.
* **Telemetri v2 ve Taktiksel Metadata Sidecar (`turret_fcs/telemetry.py`):** Veri şeması 35 dinamik kolona optimize edildi; anlık donanım odometrisi, Kalman inovasyonu, hedef geometrisi (genişlik/yükseklik), angajman evresi ve LiDAR ToF menzil kapılama senkronize kaydedildi. Kalıcı dosya tanıtıcıları ile I/O jitter'ı önlendi. Oturum açılış/kapanış istatistiklerini özetleyen JSON metadata sidecar (`.meta.json`) ve milisaniye olay kütüğü (`_events.csv`) eklendi.
* **Webots Dijital İkiz Refaktörü & İnce HAL Katmanı:** Tekrarlanan 8 ayrı simülasyon kontrolcüsü silinerek doğrudan üretim kodunu çalıştıran ince donanım soyutlama katmanı (`webots_hardware_interface.py`) oluşturuldu; 80 ms taşıyıcı gecikme, 45 ms servo gecikmesi ve 1.20° mekanik dişli boşluğu simülatöre işlendi.
* **Kapsamlı Otomatik Test Paketi:** 44 otomatik Pytest birim, SSOT ve duman testi ile GitHub Actions sürekli entegrasyon (CI) boru hattı kuruldu.

### v1.2
* **Otonom Devriye Birikimli Adımlayıcısı (Patrol Step Accumulator):** 80-100 FPS yüksek döngü hızında MG996R mekanik ölü bölgesine takılmayı önleyen birikimli adım mekanizması (`min_dispatch_deg = 0.95°`) ile taretin 90-90 merkezine eksiksiz oturması sağlandı.
* **Dinamik Kalman Hız Ölü Bölgesi (Dynamic Velocity Deadband):** Taret dönme hızına ve hedef mesafesine bağlı olarak 45 px/s'den 120 px/s'ye kadar genişletilen ölü bölge ile sahte hız gürültüsü 5 kat bastırıldı.
* **Takip Histerezisi ve Anlık Güven (% CONF):** Arama modunda `conf = 0.28`, kilitli takipte hareket bulanıklığına karşı `conf = 0.16` dinamik güven eşiği devreye alındı ve HUD göstergesine bağlandı.
* **Asimetrik Eylemsizlik ve Dikey Sönümleme:** Dikey eksendeki 300g pirinç volan namlu ağırlığının yarattığı yerçekimi momenti ve dişli boşluğu için asimetrik yumuşak ölü bölge (Pan 0.85°, Tilt 0.95°) ve adaptif sönümleme uygulandı.
* **Smith Predictor Eylem Belleği:** Fiziksel testte %50.2 kilitlenme oranı ve 13.16 saniye (1.061 kare @ 80.6 FPS) kesintisiz kilit kararlılığı elde edildi.
* **Hedef Kayıp (Dropout) Koruması:** 4 karelik dinamik tolerans penceresi ve aktüatör durum gözlemcisine `was_lost` bayrağı eklenerek hedefin anlık kayıplarında sahte sıçramalar elendi.

### v1.1
* **Yumuşak Ölü Bölgeli (Soft Deadband) Tip-1 P Omurgası:** STM32 ayrık integratör karakteristiği ile harici Ki veya türeve ihtiyaç duymadan sıfır kalıcı durum hatası ve sıfır jerk ile pürüzsüz motor kalkışı sağlandı.
* **Menzil Kapılama (Range Gating) Koruması:** Hedef namlu merkezinden uzaktayken (`angular_error > 3.0°`) TF-Luna LiDAR ışınının arkadaki duvara çarpmasını önleyen mesafe dondurma mekanizması (`last_valid_target_dist_m`) kuruldu.
* **11-Boyutlu Eylem Geçmişli PPO Pekiştirmeli Öğrenme Ajanı:** 2 adımlı eylem geçmişi (`act_t-1, act_t-2`) ve Alan Rastgelelendirmesi (Domain Randomization) altında simülasyondan gerçek dünyaya aktarım sağlandı.
* **Ters Kayan Pencere (Sliding Window) UART Ayrıştırıcısı:** 115200 Baud ikili veri hattında parazit veya paket bölünmelerinde faz kaymasını önleyen ters taramalı sağlam ayrıştırıcı geliştirildi.
* **Çok Katmanlı Güvenlik Mimarisi (Fail-Safe):** 300 ms STM32 Heartbeat Watchdog, 350 ms kamera donma koruması, mandallı (latched) E-stop ve 80 ms yükselen kenar solenoid dart mandalı entegre edildi.
* **Lissajous Parametrik Devriye Taraması:** Hedef yokken hava sahasını tarayan sonsuzluk yörüngesi (Pan: ±35°, Tilt: ±8°) devreye alındı.

---

## 🎯 Temel Yetenekler ve Donanım Parametreleri

| Alt Sistem / Bileşen | Donanım & Yazılım Özelliği | Çalışma Parametresi / Başarım |
| :--- | :--- | :--- |
| **Görsel Algılama** | Logitech C270 HD Web Kamerası | 1280x720 @ 30 FPS, FOV: Yatay 48.8°, Dikey 28.6° (Kalibre edilmiş optik açı, sıfır kuyruk gecikmeli thread) |
| **Menzil Algılama** | Benewake TF-Luna ToF LiDAR | 850 nm ToF, 0.2 - 8.0 m menzil, 100 Hz donanımsal UART |
| **Yapay Zeka Tespiti** | Ultralytics YOLOv8n + ByteTrack | Özel eğitilmiş balon/İHA tespiti, 8-15 ms GPU çıkarımı |
| **Filtreleme & Füzyon** | Ego-Hareket Telafili CV Kalman | Joseph form kovaryans, 1. derece aktüatör gözlemcisi, 30-60 px/s ölü bölge |
| **Vuruş Olasılığı** | Kapalı Form 2B Gauss (Marcum Q) | ncx2 CDF, dinamik hedef boyutu, hız duyarlı histerezis |
| **Birincil Kontrolcü** | Tip-1 P (tek EMA, ivme sınırlamalı) | Pan Kp=0.22, Tilt Kp=0.11, Slew: 400°/s² Pan, 267°/s² Tilt |
| **İkincil Kontrolcü** | 11-Boyutlu Saf PPO RL Ajanı | NumPy MLP (11-64-64-2), 1.50° pan / 0.75° tilt adımı, 0.03 ms CPU çıkarımı |
| **Gömülü Denetleyici** | STM32F401RE Nucleo (ARM Cortex-M4) | 84 MHz, 115200 Baud UART, 50 Hz PWM, 300 ms Heartbeat + 800 ms IWDG |
| **Aktüatörler** | MG996R / DS3218 Metal Dişli Servo | 50 Hz PWM, Pan: 15° - 165° (±75°), Tilt: 65° - 115° (±25°), 45°/s slew doyumu |
| **Fırlatma Mekanizması** | Çift FF-180 Volan + JF-0530B Solenoid | 23.500 RPM, 10.0 m/s mermi hızı, 80 ms darbe genişliği |
| **Dijital İkiz** | Cyberbotics Webots R2025a | ODE fizik motoru, 80 ms gecikme, 45 ms lag, 1.20° backlash |

---

## 🏛️ Sistem ve Donanım Mimarisi

Aşağıdaki şemalar sistemin güncel yazılım hattını, durum akışını ve donanım bağlantılarını göstermektedir:

### 1. Turret FCS Yazılım ve Sistem Mimarisi
![Turret FCS Sistem Mimarisi](docs/assets/turret_fcs_architecture.svg)

### 2. Algoritmik Durum Akış Şeması (Flowchart)
![Turret FCS Algoritmik Akış Şeması](docs/assets/turret_fcs_flowchart.svg)

### 3. Donanım ve Elektriksel Güç Dağıtım Şeması
Projenin elektriksel güç dağıtımı, sensör bağlantıları ve sürücü katı [turret_fcs_hardware.svg](docs/assets/turret_fcs_hardware.svg) dosyasında modellenmiştir.

### 4. 3D Mekanik Montaj ve Donanım Tasarımı
Taretin 3D yazıcıdan üretilen açık beşikli, minimal optik paralakslı ve doğrudan tahrikli mekanik montajı:

![Turret FCS Elektromekanik Donanım Yerleşimi](docs/assets/mechanical_design.jpg)

![Turret FCS 3D Mekanik Montaj](docs/assets/turret_cad_render_annotated.jpg)

```text
+-----------------------------------------------------------------------------------+
|                            GÖRSEL VE SENSÖR GİRDİSİ                               |
|  [ Logitech C270 (1280x720 @ 30 FPS) ]     [ Benewake TF-Luna LiDAR (100 Hz UART) ]|
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                             HEDEF TESPİT VE İZLEME                                |
|  * YOLOv8n Derin Öğrenme Modeli (Hava Hedefi / Mini İHA Tespiti)                 |
|  * ByteTrack Çoklu Hedef Takibi & Anlık Güven Histerezisi (0.28 / 0.16)           |
|  * Ego-Hareket Telafili CV Kalman Filtresi (Joseph Form, Tek Q/R)               |
|  * 3-Kademeli LiDAR Menzil Kapılama (Range Gating) Duvar Parazit Filtreleme       |
|  * Taktiksel MIL-STD HUD Arayüzü & Muharebe Sadeleştirmesi (Combat Declutter)    |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                           KONTROL VE NİŞAN ALMA KATMANI                           |
|  * Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q / ncx2 CDF)                     |
|  * Dinamik Önleme Açısı (Lead) & Balistik Süper-Elevasyon Hesabı                  |
|  * Bağımsız Modlar: Yumuşak Ölü Bölgeli Tip-1 P / 11-Boyutlu Saf PPO RL           |
|  * Deterministik [M] Tuşu ile Ayrık Operatör Mod Seçimi                           |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        GÖMÜLÜ DONANIM / FİZİKSEL EYLEME                           |
|  * STM32F401RE Nucleo Mikrodenetleyici (115200 Baud İkili UART Protokolü)         |
|  * 300 ms Heartbeat Watchdog + 800 ms IWDG (Donanımsal Acil Frenleme)             |
|  * Çift Eksen Pan-Tilt Servoları (50 Hz PWM Donanımsal İntegratör)                |
|  * 5-Kademeli Atış Sekansı: 1.0 s SPINUP -> 0.15 s SETTLE -> 80 ms Solenoid Darbe |
+-----------------------------------------------------------------------------------+
```

---

## 🔄 Algoritmik Durum Akış Mantığı

Sistemin arama, tespit, takip, kilitlenme ve atış sekansları [turret_fcs_flowchart.svg](docs/assets/turret_fcs_flowchart.svg) dosyasında modellenmiştir:

* **DURUM 1 (Devriye & Arama - PATROL):** Hedef yokken taret 5.0 s dinamik Lissajous (8-şekli) devriye taraması (Pan: ±45°, Tilt: ±16°) gerçekleştirir; kamera iş parçacığı sıfır kuyruk gecikmesiyle kare yakalar.
* **DURUM 2 (Tespit & Mesafe - DETECT):** Hedef algılandığında TF-Luna LiDAR ile 100 Hz mesafe ve sinyal kalitesi doğrulanır; 3-kademeli menzil kapılama parazitleri eler; `TargetSelector` ID sürekliliğini doğrular.
* **DURUM 3 (Takip & Önleme - TRACK):** CV Kalman filtresi hedef hızını ve konumunu kestirir; mermi uçuş süresi hesaplanır (`t_flight = mesafe / 10.0 m/s`) ve Zero-Lag Aim-Point Feedforward ile önleme noktası belirlenir.
* **DURUM 4 (Motor Yönlendirme - P / RL):** Operatörün seçtiği Tip-1 P kontrolcü (`Kp = 0.22 / 0.11`) veya Saf PPO RL ajanı (`turret_ppo_v14_policy.npz`) motor açı adımlarını üretir.
* **DURUM 5 (Hedefe Kilitlenme - LOCKED):** Açısal hata dinamik histerezis eşiğinin altına indiğinde ve 10 kare kesintisiz kararlılık sağlandığında kilit teyit edilir; HUD üzerinde nişangah ve vuruş olasılığı gösterilir, operatörden manuel atış izni (`[Y]` tuşu) beklenir.
* **DURUM 6 (Kademeli Atış Protokolü):**
  * `SPINUP & SLEW (1.0 s)`: Çift volan motoru 23.500 RPM devire çıkarılır ve taret önleme açısına yönelir.
  * `SETTLE (0.15 s)`: Nişan açısı sabitlenir, servo lag ve dişli boşluğu oturur.
  * `FIRING (80 ms)`: STM32 yazılımsal durum makinesi (`HAL_GetTick()`) ile 80 ms darbeli solenoid dart iticisini tetikler.
  * `RECOVER (0.35 s)`: Solenoid yayla geri çekilirken taret optik eksene döner.
  * `EVAL (1.50 s)`: Ekranda `ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL` afişi gösterilir; ilk 0.75 s boyunca kilit muafiyeti uygulanır.

---

## 📐 Kontrol Teorisi ve Matematiksel Modelleme

### 1. Tip-1 Oransal (P) Kontrol ve Donanımsal Ayrık İntegratör
STM32 servo sürücüsü her 20 ms'de gelen açısal artım komutunu mevcut açıya ekleyerek donanımsal konum üretir:

```text
theta(k) = theta(k-1) + delta_theta(k)
```

Z-düzleminde transfer fonksiyonu:

```text
H(z) = theta(z) / delta_theta(z) = 1 / (1 - z^(-1)) = z / (z - 1)
```

Sistem z = 1 noktasında kutba sahip açık çevrim bir integratördür. Kapalı çevrimde oransal denetleyici:

```text
delta_theta(k) = Kp * e(k)
```

Basamak hedef girdisi uygulandığında Son Değer Teoremi uyarınca kalıcı durum hatası sıfırdır (`e_ss = 0`). Sistem doğası gereği Tip-1 olduğundan harici yazılımsal Ki integratörüne ihtiyaç duyulmaz; böylece integratör doyması (windup) ve faz gecikmesi tamamen engellenir. Rampa tipi hareketli hedef girdisinde oluşan sonlu takip hatası ise Kalman filtresinden gelen kinematik önleme açısı ve atış anındaki `Zero-Lag Aim-Point Feedforward` adımıyla (`u = delta_slew + u_P`) sıfırlanır.

### 2. Yumuşak Ölü Bölge (Soft Deadband) Formülasyonu
Mekanik dişli boşluğu ve aktüatör titreşimini önlemek için uygulanan parçalı sürekli fonksiyon:

```text
u = 0,                               |e| <= db
u = sign(e) * Kp * (|e| - db),       |e| > db
```

Bu formülasyon sayesinde ölü bölgeden çıkış anında kontrol sinyali sıfırdan sürekli olarak başlar (sıfır jerk), sert basamak darbeleri ve mekanik vuruntu engellenir.

### 3. Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q)
Hedef dairesi üzerinde dikey eksantriklik `r_ofset` ve silah-hareket varyansı `sigma_toplam` altında vuruş olasılığı non-central chi-squared kümülatif dağılım fonksiyonu (ncx2 CDF) ile kapalı formda hesaplanır:

```text
nc = (r_ofset / sigma_toplam)^2
p_vurus = ncx2.cdf(1, df=2, nc=nc) * (1 - exp(-theta_hedef^2 / (2 * sigma_toplam^2)))
```

Burada:
* `theta_hedef`: YOLO kutusundan türetilen hedef anlık açısal yarıçapı.
* `sigma_toplam = sqrt(sigma_silah^2 + sigma_hareket^2)`: Silah saçılımı (1.25°) ile hedef hareket belirsizliğinin geometrik toplamı.

### 4. PPO RL Progress Ödül Fonksiyonu
PPO ajanının eğitiminde simülasyon kararlılığını sağlayan ve sabit noktada gereksiz motor sürmeyi engelleyen ödül formülü:

```text
r = 2.0 * (e_prev^2 - e^2) - 0.05 * e^2 - 0.02 * |u|^2 - 0.08 * |du|^2 + r_bullseye
```

Burada hata karesinin azalışı (`e_prev^2 - e^2`) pozitif ödüllendirilirken, kontrol enerjisi (`|u|^2`) ve eyleyici ivmesi/jerk (`|du|^2`) düzenli olarak cezalandırılır.

---

## 🛡️ Güvenlik ve Fail-Safe (Arıza Korumalı) Mimarisi

1. **Bağımsız Donanımsal Watchdog (IWDG, ~800 ms):** STM32 yazılımı herhangi bir nedenle kilitlenirse donanımsal IWDG mikrodenetleyiciye reset atar ve açılışta fırlatıcı hatları 0V seviyesine çekilir.
2. **300 ms Seri Heartbeat Watchdog:** PC ile STM32 arasındaki UART iletişimi 300 ms kesilirse, STM32 fırlatıcı motorları ve solenoidi donanımsal olarak anında kapatır (`Launcher_EmergencyStop()`).
3. **2000 ms Servo Park Koruması:** PC bağlantısı 2 saniyeden uzun süre kesilirse servolar aşırı akım çekmemesi için mekanik park konumuna (90°, 90°) yumuşak bir şekilde indirilir.
4. **Mandallı Acil Durdurma (Latched E-STOP):** Operatör `SPACE` tuşuna bastığında motorlar merkeze hız sınırlı (45 derece/s) olarak indirilir ve kilitlenir; tekrar `SPACE` tuşuna basılana kadar sistem hiçbir komutu işlemez.
5. **E-STOP İkili Paket Doğrulaması:** Tek bir gürültülü baytın tareti yanlışlıkla frenlemesini önlemek için acil durdurma yalnızca üst üste iki geçerli pakette kabul edilir.
6. **Zaman Tabanlı Hız Sınırlaması:** STM32 firmware'i gerçek döngü süresini (`dt`) ölçerek servo komutunu 45 derece/s ile donanımsal olarak sınırlar; PC kare hızı artsa bile taret aşırı hızlanamaz.
7. **Geç Ateşleme Koruması:** Volan motorları nominal devrine (23.500 RPM) ulaşmadan atış izni verilmez; dartın taret başka yöne bakarken fırlatılması engellenir.
8. **350 ms Kamera Donma Koruması:** Web kamerasından 350 ms boyunca yeni görüntü karesi gelmezse sistem güvenli bekleme moduna geçer.
9. **Çift Dart Önleyici Kenar Tetiklemeli Darbe:** Solenoid darbesi gömülü yazılımsal durum makinesi (`HAL_GetTick()`) ile tam 80 ms olarak sınırlandırılmıştır; tek basışta birden fazla dart atılamaz.
10. **Açılışta Donanımsal Pull-Down Koruması:** STM32 güç aldığı anda ve reset durumlarında fırlatıcı kontrol hatları donanımsal pull-down dirençleri ile kesin 0V seviyesinde tutulur.

---

## 📊 Karşılaştırmalı Performans Analizi (P Kontrolcü vs PPO RL)

Sistem hem gerçek donanım telemetri testlerinde hem de Webots simülasyon ortamında kapsamlı olarak karşılaştırılmıştır:

![P Kontrolcü vs PPO RL Benchmark Karşılaştırması](analysis/figures/p_vs_rl_final_benchmark.png)

### 1. Gerçek Donanım Telemetri Başarımı (v1.4 Donanım Doğrulaması)

5.387 karelik kesintisiz gerçek taret test oturumunda elde edilen doğrulanmış metrikler:

| Metrik / Performans Göstergesi | Tip-1 P Kontrolcü (v1.4) | Saf PPO RL Kontrolcü (v1.4) | Başarım Farkı / Çıkarım |
| :--- | :---: | :---: | :---: |
| **Hedef Tespit Sayısı** | 1.738 kare | **3.649 kare** | 2.1 kat daha uzun süre hedefte kalma |
| **Kilitlenme Oranı (Lock Ratio)** | %61.0 | **%65.5** | **+%4.5 Daha Yüksek Kilit Kararlılığı** |
| **Azami Kesintisiz Kilit Serisi** | 143 kare (4.8 saniye) | **185 kare (6.2 saniye)** | **+%29.4 Daha Uzun Kesintisiz Kilit** |
| **Yatay Takip Hatası (Pan RMS)** | 3.26° | **2.61°** | **-%20 Daha Düşük Takip Hatası** |
| **Dikey Takip Hatası (Tilt RMS)** | 1.62° | **1.61°** | Eşit seviyede yüksek hassasiyet |
| **Merkezde Komut Genliği** | 0.030° | **0.031°** | **Mekanik Titreşimsiz Sıfır Avlanma** |
| **Komut Yön Değiştirme Oranı** | **%4.1** | %8.9 | Limit-cycle sınırı (%10) altında kararlılık |
| **Çıkarım Süresi (CPU)** | **0.01 ms** | **0.03 ms** | 30 FPS döngü süresinin %0.1'inden az |

### 2. Hedef Hızına Göre Dinamik Kapalı Çevrim Takip Başarımı (30 FPS, YOLO Gürültüsü)

| Hedef Hızı | P Takip Hatası | P Kilit Durumu | RL Takip Hatası | RL Kilit Durumu |
| :---: | :---: | :---: | :---: | :---: |
| **0.0 derece/s (Statik)** | 0.71° | %100 Kilit | 0.67° | %100 Kilit |
| **2.0 derece/s (Yavaş)** | 1.47° | %100 Kilit | 1.19° | %100 Kilit |
| **5.0 derece/s (Orta)** | 2.37° | %100 Kilit | 1.19° | %100 Kilit |
| **8.0 derece/s (Hızlı)** | 3.27° | %0 (Rampa Hatası) | **1.41°** | **%100 Kilit** |
| **10.0 derece/s (Manevra)** | 3.87° | %0 (Kilit Koptu) | **1.85°** | **%100 Kilit** |
| **12.0 derece/s (Sınır)** | 4.47° | %0 | 2.74° | %0 |

*Mühendislik Çıkarımı:* Tip-1 P kontrolcü statik ve düşük hızlı hedeflerde hatayı soft deadband içine çekip sıfır motor titreşimi ile kararlı nişan alırken; 8-10 derece/s hızındaki çevik manevralarda Saf PPO RL ajanı taşıma gecikmesini eylem geçmişiyle kapatıp kilidi %100 korumaktadır. Operatör `[M]` tuşu ile tehdidin dinamizmine göre anlık geçiş yapabilmektedir.

---

## 🚀 Hızlı Başlangıç ve Kurulum

### 1. Sistem Gereksinimleri
* Windows 10/11 x64
* Python 3.10, 3.11 veya 3.12 (*"Add Python to PATH"* seçili olmalıdır)
* STM32 Nucleo-F401RE Kartı & Logitech C270 Web Kamerası
* Benewake TF-Luna ToF LiDAR Sensörü

### 2. Otomatik Ortam Kurulumu
Depoyu klonlayıp ana dizindeki kurulum betiğini çalıştırın:

```bash
git clone https://github.com/brkysnturk/turret_fcs.git
cd turret_fcs
setup_env.bat
```

Betik sırasıyla:
1. Sistemdeki Python sürümünü seçer (3.10-3.12).
2. Temiz bir `venv` sanal ortamı kurar.
3. [requirements.txt](requirements.txt) bağımlılıklarını **CPU-only PyTorch** indeksiyle hızlıca yükler (büyük CUDA indirmesi yapmaz).
4. Paketi `pip install -e .` ile geliştirme modunda sisteme bağlar.
5. OpenCV, Torch, Ultralytics ve FCS modüllerinin içe aktarımını doğrular.

### 3. Test Paketini Doğrulama
Kurulumun ve modül tutarlılığının doğrulanması için otomatik test paketini çalıştırın:

```bash
venv\Scripts\pytest tests/
```

36 testin tamamı harici donanım veya kamera bağlı olmasa dahi simüle edilmiş girdilerle yaklaşık 1.0 saniyede geçer.

### 4. Sistemi Başlatma
Donanım bağlantılarını tamamladıktan sonra ana başlatıcıyı çalıştırın:

```bash
main.bat
```

*(Webots simülatöründe çalıştırmak için `webots/worlds/webots_turret_fcs.wbt` dosyasını Webots R2025a ile açın. P/RL modu seçimi için `WEBOTS_CONTROL_MODE=P|RL`, hedef profil seçimi için `webots/balloon_profile.json` kullanılır.)*

---

## 🎮 Operatör Kontrol Tuşları

| Tuş | Fonksiyon | Açıklama |
| :---: | :---: | :--- |
| **`ESC` / `Q`** | Güvenli Çıkış | Programı ve telemetri oturumunu güvenle kapatır (`.csv`, `.meta.json`, `_events.csv`). |
| **`SPACE`** | Mandallı E-STOP | Motorları hız sınırlı olarak mekanik merkeze (90°, 90°) çeker ve kilitler. |
| **`M`** | Mod Değişimi | `[P CONTROLLER]` ile `[RL CONTROLLER]` arasında geçiş yapar. |
| **`P`** | Devriye Taraması | Lissajous sonsuzluk eğrisi ile otonom hava sahası taramasını açar / kapatır. |
| **`Y`** | Atış Yetkisi | Hedef kilitlendiğinde (`TARGET LOCKED`) kademeli balistik atış sekansını onaylar. |
| **`N`** | Kilit İptali | Mevcut hedef kilidini iptal eder ve devriye taramasına döner. |

---

## 📈 Akademik Analiz ve Doğrulama Araçları

Kontrol dinamiklerini, Kalman performansını ve P vs RL karşılaştırmalarını üretmek için `analysis/` araçları kullanılır:

```bash
# 1. P Kontrolcü vs PPO RL Akademik Benchmark Analizi (300 DPI Görsel)
venv\Scripts\python.exe turret_fcs/rl_sim/compare_p_vs_rl.py

# 2. 8 Panelli Detaylı Dinamik ve Kontrol Kararlılık Analizi
venv\Scripts\python.exe analysis/plot_tuned_system.py

# 3. Kalman Filtresi ve Zemin Gerçeği Doğrulama Grafiği
venv\Scripts\python.exe analysis/generate_kalman_plots.py

# 4. Donanım Telemetri Kütüklerinin Otomatik İncelemesi
venv\Scripts\python.exe analysis/inspect_logs.py
```

---

## 📂 Dizin Yapısı

```text
turret_fcs/
├── .github/                   # GitHub Actions CI sürekli entegrasyon iş akışları (ci.yml)
├── tests/                     # 36 adet otomatik Pytest birim, SSOT ve entegrasyon testi
├── requirements.txt           # Python kütüphane bağımlılıkları (YOLO, Torch, SB3, vb.)
├── pyproject.toml             # Python paket ve yapılandırma manifestosu
├── CHANGELOG.md               # Sürüm değişiklikleri ve detaylı sürüm geçmişi
├── CITATION.cff               # Akademik alıntı ve tez referans üstverisi
├── setup_env.bat              # Otomatik sanal ortam (venv) ve kütüphane kurulum aracı
├── main.bat                   # Tek tıkla gerçek zamanlı atış kontrol başlatıcısı
├── README.md                  # Proje mimarisi ve teknik dokümantasyon
├── docs/                      # Dokümantasyon, mimari şemalar ve CAD çizimleri
│   ├── assets/                # Güncel mimari/akış SVG'leri, CAD renderları ve donanım şeması
│   ├── cad/                   # 3D yazıcı üretim ve prototipleme dosyaları (.stl)
│   ├── firmware.md            # Gömülü yazılım derleme, flaşlama ve pinout kılavuzu
│   └── reproducibility.md     # Tekrar üretilebilirlik ve benzetim doğrulama kılavuzu
├── analysis/                  # Akademik kontrol, Kalman ve Webots doğrulama araçları
│   ├── README.md              # Analiz betikleri ve üretilen figür haritası
│   ├── plot_tuned_system.py   # 8 panelli analitik model ve telemetri analiz grafikleri
│   ├── generate_kalman_plots.py # Kalman filtresi doğrulama görselleştiricisi
│   ├── generate_benchmark_plots.py # P vs RL benchmark görselleştiricisi
│   ├── generate_architecture_svg.py # Sistem mimarisi SVG üreteci
│   ├── inspect_logs.py        # Telemetri kütüklerini analiz eden teftiş aracı
│   └── figures/               # 300 DPI yayın kalitesinde akademik grafikler
├── weights/                   # YOLOv8 tespit modeli ağırlıkları (GitHub Release Assets)
│   ├── README.md              # Model ağırlıkları indirme ve sürüm kılavuzu
│   ├── best.pt                # Özel eğitilmiş balon tespit model ağırlığı
│   └── yolov8n.pt             # Temel YOLOv8 Nano nesne tespit modeli
├── turret_fcs/                # Gerçek donanım çalışma kodları
│   ├── config.py              # SSOT merkezi konfigürasyon modülü
│   ├── main.py                # Ana atış kontrol döngüsü ve koordinasyon
│   ├── weights.py             # Otomatik GitHub Releases model ağırlığı indiricisi
│   ├── engagement.py          # 2 aşamalı balistik atış ve angajman durum makinesi (SSOT)
│   ├── presenter.py           # MIL-STD taktiksel HUD çizim ve sunum katmanı
│   ├── keymap.py              # Operatör klavye kontrolleri ve komut dağıtıcısı
│   ├── telemetry.py           # v2 telemetri, JSON metadata sidecar ve event log
│   ├── fcs_manager.py         # Hedef seçim, kilit, balistik ve menzil kapılama motoru
│   ├── lock_manager.py        # Kapalı form 2B Gauss vuruş olasılığı (ncx2 CDF) ve histerezis
│   ├── fcs_comm.py            # STM32 UART binary paket sürücüsü (resync + checksum)
│   ├── kalman_filter.py       # CV Kalman, ego telafisi, sıçrama kapısı ve lead
│   ├── turret_hud.py          # Taktiksel HUD ve muharebe sadeleştirmesi
│   ├── rl_controller.py       # Üretim RL: saf PPO, NumPy MLP (11-64-64-2)
│   ├── p_controller.py        # Üretim P: tek EMA, ivme sınırlamalı Tip-1 P
│   ├── patrol_generator.py    # Lissajous devriye tarama motoru
│   ├── target_selector.py     # ByteTrack ID + mekânsal kapı + histerezis
│   ├── camera_thread.py       # Sıfır gecikmeli, watchdog korumalı kamera sürücüsü
│   └── rl_sim/                # Pekiştirmeli öğrenme ortamı ve eğitim araçları
│       ├── turret_env.py      # 11-D Gymnasium taret simülasyon ortamı
│       ├── train_rl.py        # PPO eğitim kodu
│       ├── compare_p_vs_rl.py # P vs RL benchmark karşılaştırma aracı
│       └── models/            # turret_ppo_v14.zip + turret_ppo_v14_policy.npz
├── stm32/                     # STM32F401RE Nucleo gömülü yazılım projesi (CubeIDE)
│   └── turret_main/           # Ana taret donanım projesi (.ioc, Core/Src, Core/Inc)
│       └── Core/Src/          # fcs_comm.c, servo_driver.c, luna_read.c, flywheel_solenoid.c, main.c
└── webots/                    # Webots dijital ikiz simülasyonu
    ├── controllers/           # İnce Webots HAL adaptörü (webots_main, hardware_interface)
    ├── worlds/                # 3D simülasyon dünyası ve aktörler
    └── balloon_profile.json   # Hedef uçuş hareket profilleri
```

---

## 🛡️ Güvenlik ve Etik Kullanım Bildirimi (Safety & Dual-Use Disclaimer)

Bu sistem; Yıldız Teknik Üniversitesi bünyesinde yalnızca **akademik araştırma, hava savunma kontrol teorisi doğrulaması ve güvenli sünger dart (foam dart) hedefleri** için tasarlanmıştır:
- Gömülü yazılımda fiziksel ateşleme varsayılan olarak hem yazılımsal hem de donanımsal olarak devreden çıkarılmıştır (`ENABLE_PHYSICAL_FIRE = 0`).
- Sistem; insanlara, canlı hedeflere veya yetkisiz nesnelere karşı kesinlikle çalıştırılamaz.
- Sistem donanım testlerinde acil durdurma (E-Stop / Space tuşu) ve 300 ms STM32 Heartbeat Watchdog koruması altında işletilmelidir.

---

## 🧠 Proje Hafızası ve Geliştirme Tarihçesi

Projenin ilk günden bugüne geçirdiği tüm fazlar, Kalman filtresinin sıfırdan yeniden tasarımı, P kontrolcü rezonans çözümleri, pekiştirmeli öğrenme sim-to-real boşluğunun kapatılması ve mimari kararların teknik hafızası için [AGENTS.md](AGENTS.md) dokümanına başvurabilirsiniz.

---

## 📜 Lisans & Akademik Referans

Bu proje [MIT Lisansı](LICENSE) kapsamında lisanslanmıştır. Yıldız Teknik Üniversitesi Kontrol ve Otomasyon Mühendisliği Bölümü Bitirme Çalışması & Tasarım Projesi kapsamında geliştirilmiştir.

Projenin akademik çalışmalarda veya yayınlarda referans gösterilmesi için:

```bibtex
@misc{turret_fcs_2026,
  author = {Senturk, Berkay},
  title = {Autonomous Air Defense Turret Fire Control System (Turret FCS)},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/brkysnturk/turret_fcs}},
  version = {v1.4}
}
```
