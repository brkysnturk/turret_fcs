# Taret Atış Kontrol Sistemi — Algoritma Analizi

Kapsam: `camera_thread`, `fcs_comm`, `fcs_manager`, `los_tracker`, `lock_manager`,
`patrol_generator`, `target_selector`, `turret_hud`, `telemetry`.
Entegrasyon için `main.py`, `p_controller.py`, `kalman_filter.py`, `rl_controller.py`,
`webots/` çağrı yolları ve STM32 firmware protokolü de okundu.

Yöntem: statik kod okuma, matematiksel türetme, sayısal doğrulama (odak uzaklığı,
kilit olasılığı, kapalı çevrim kalıcı hata, ivme sınırı) ve çalıştırma testi
(`python -m pytest tests`).

> ÖNEMLİ — ANALİZ ANLIK GÖRÜNTÜSÜ: Çalışma ağacı analiz sırasında aktif olarak
> düzenlendi. 18:11'de paket import edilemiyordu (`KalmanConfig.outlier_residual_px`
> yok). 18:12-18:13'te Kalman ve RL dosyaları güncellendi; import düzeldi.
> Bu belge 18:17 itibarıyla alınan anlık görüntüyü anlatır. Bkz. Bölüm 6.1.

---

## 1. Yönetici Özeti

1. **Mimari olgun ve sorumluluklar temiz ayrılmış.** Her modülün tek bir görevi var;
   `fcs_manager` dışında hiçbir modül başka bir modülün iç durumuna yazmıyor.
   Zamanlama zinciri (kamera → YOLO → seçici → FCS → UART → firmware) tek yönlü ve
   izlenebilir; telemetri bu zincirin her aşamasını kaydediyor.

2. **En büyük sistematik hata kaynağı, ölü bölge + oransal gecikmeden doğan
   kalıcı nişan geriliği.** P kontrolcü sabit hızlı bir hedefi takip edebilmek için
   sürekli bir açı hatası üzerinde çalışmak zorundadır; bu hata `db + omega/(30*Kp)`
   kadardır. Balistik lead yalnızca `omega*T_lead` kadar ileri taşır. Sonuç:
   hedef hızı arttıkça nişan, olması gereken noktanın gerisinde kalır (Bölüm 4).

3. **Ateş kapısı açı tabanlıdır, hedefin açısal boyutundan bağımsızdır.**
   Kilit penceresi 1.60-3.60 derece; hedef yarıçapı 2 m'de 4.29 derecedir ama
   300 px'lik bir kutunun açısal yarıçapı 6 derece, 60 px'lik kutunun 1.22 derecedir.
   Aynı 2.5 derece hata, büyük hedefte vuruş, küçük hedefte ıskadır; kilit mantığı
   ikisini ayırmaz. `hit_probability` bunu hesaplıyor ama ateş kararında
   kullanılmıyor (Bölüm 3.6).

4. **Çalışma ağacında commit edilmemiş bir refactor var** (Kalman tekil-sıçrama
   kapısının kaldırılması, RL hız gözleminin geri açılması, ivme sınırının
   `a*dt` yerine `a*dt^2` olması). Testler yeni sözleşmeye göre güncellendi
   (21 geçti, 3 ortam hatası); ancak bu değişiklikler v1.4'te Webots ve saha
   testiyle doğrulanmış kazançları geçersiz kılıyor: yeni ivme sınırı kapalı
   çevrimin baskın dinamik elemanı haline geldi (Bölüm 6).

5. **Modüller arası sözleşmeler sağlam:** UART çerçevesi (7 bayt TX / 10 bayt RX),
   checksum, 0.1 derece niceleme artığı, eksen bazlı sıfırlama, E-stop çift paket
   onayı ve 300 ms heartbeat firmware ile birebir uyumlu. Checksum 8 bit toplamdır
   (CRC değil); CRC-8'e geçiş tek satırlık bir kazançtır.

---

## 2. Sistem Veri Akışı ve Zaman Bütçesi

Tek karelik döngü (`main.py`):

    kamera karesi (donanım, ~33 ms periyot)
      -> dt ve fps hesabı (wall clock)
      -> LiDAR verisi + servo geri beslemesi (UART RX, 100 Hz)
      -> sync_turret_pose (donanım pozu -> FCS odometrisi)
      -> YOLOv8 + ByteTrack (conf = secici.min_conf)
      -> atış sekansı (SPINUP / SETTLE / FIRING / RECOVER / EVAL) -> slew_factor
      -> select_best_target -> best_box
      -> fcs.update(...) -> (pan_cmd, tilt_cmd)
      -> UART TX (7 bayt) -> firmware hız sınırı -> servo PWM
      -> telemetri (44 kolon) -> HUD çizimi -> klavye

| Aşama | Tipik gecikme | Kaynak |
|---|---:|---|
| Kamera pozlama + aktarım | 33 ms (1 kare) | 30 FPS |
| YOLO çıkarımı | ölçülmedi (log `latency_ms` içinde) | `telemetry.log_frame` |
| Seçici + FCS + UART yazımı | 1-3 ms | `latency_ms` |
| UART iletim (7 bayt @ 115200) | 0.6 ms | `fcs_comm.send` |
| Firmware döngüsü | 10 ms (`HAL_Delay(10)`) | `main.c` |
| Servo 1. derece yanıtı | tau = 45 ms | `CONFIG.mount.servo_tau` |
| **Toplam (ölçülen)** | **~80 ms** | `CONFIG.ballistics.system_latency_s` |

Not: `telemetry` sütunundaki `latency_ms`, kamera beklemesini ve HUD çizimini
kapsamaz; YOLO + FCS + UART toplamıdır. Zincirin gerçek gecikmesini ölçmek için
doğru sütundur.

---

## 3. Modül Analizleri

### 3.1 `camera_thread.py` — sıfır gecikmeli yakalama

**Problem:** `cv2.VideoCapture` arka planda 3-5 karelik FIFO tutar; bu, 100-150 ms
bayat kare demektir. Kontrol döngüsü bayat kareyle çalışırsa kazanç ne olursa olsun
sistem kararsızlaşır.

**Algoritma:** üretici-tüketici. Üretici daemon thread sürekli `cap.read()` yapar ve
`(grabbed, frame, frame_id, last_frame_time)` dörtlüsünü kilit altında **üzerine
yazar** (kuyruk yok). Tüketici `read()` en son kareyi kopyalar. `CAP_PROP_BUFFERSIZE=1`
sürücü tamponunu da küçültür. `new_frame_event` ile isteğe bağlı 30 FPS senkronu
sağlanır; aynı karenin iki kez işlenmesi engellenir.

**Kritik ayrıntılar:**
- `is_stale = (now - last_frame_time) > timeout` — kamera donduğunda eski kare
  "geçerli" sayılmaz, döngü kırılır. `isOpened()` 0.60 s, `read()` 0.35 s eşiği kullanır.
- Önce MSMF, açılmazsa DSHOW. C270'in MJPG 720p30 akışı MSMF'te daha kararlı.
- `detect_camera` PyGrabber aygıt adında "C270" arar (eski hata "Logi C270" idi).

**Zayıflık:** PyGrabber numaralandırma sırası ile OpenCV/MSMF indeks sırası aynı
olmak zorunda değil; C270 dışında bir aygıt takılırsa yanlış kameraya bağlanabilir.
Ayrıca yakalama zaman damgası dışarı verilmiyor; sensör gecikmesi ölçülemiyor.

### 3.2 `fcs_comm.py` — ikili protokol ve niceleme biriktiricisi

**Çerçeveler:** TX 7 bayt `[0xAA][pan int16][tilt int16][state uint8][checksum]`,
RX 10 bayt `[0x55][mesafe uint16][sinyal uint16][pan int16][tilt int16][checksum]`.
Her iki checksum da 8 bit toplamdır. Firmware tarafı birebir aynı kuralı uygular
(`fcs_comm.c`), yani sözleşme tutarlı.

**Niceleme artığı (noise shaping):** komut 0.1 derece çözünürlükte gönderilir.
Artık biriktirici, kesme hatasını bir sonraki kareye taşır:

    toplam_k = 10 * aci_k + artik_k-1
    adim_k   = round(topl am_k)
    artik_k  = toplam_k - adim_k        (daima [-0.5, +0.5])

Böylece 0.04 derecelik komutlar tek başına yutulmaz; 2-3 karede bir 0.1 derecelik
adıma dönüşür. Eksen başına bağımsız artık vardır; sıfır komut o eksenin artığını
sıfırlar (kalıcı kayma birikmesini engeller).

**RX ayrıştırma:** kalıcı `bytearray` tampon, başlık arama, 10 bayta ulaşınca
checksum, hatalıysa **tek bayt** silip yeniden senkronize olma. Tampon 4096 baytı
aşarsa son 10 bayt tutulur. Bu, bölünmüş/gecikmiş paketlere karşı dayanıklıdır.

**Zayıflıklar:**
- 8 bit toplam checksum, bayt sırası değişimini yakalayamaz; CRC-8 (0x07) aynı
  maliyetle çok daha güçlü olur.
- `send()` ana döngüdedir. `write_timeout=0.04` saniye; sürücü tamponu dolarsa bir
  kare 40 ms bloke olabilir (30 FPS -> ~14 FPS). Pratikte 210 B/s yük ile risk düşük,
  ama garantisi yok. Yazma işi ayrı bir kuyruk thread'ine alınırsa bu tavan kalkar.
- Bağlantı yokken `send()` her 2 saniyede bir port taraması yapar; tarama süresi
  ana döngüye eklenir.

### 3.3 `target_selector.py` — elemeli ve histerezisli hedef seçimi

**Dört eleme:** güven eşiği (arama 0.28 / takip 0.16), alan (400-400000 px^2),
minimum kenar (18 px), en-boy oranı (0.35-2.50).

**Üç kademeli seçim:**
1. Kilitli ByteTrack kimliği **ve** mekânsal kapı (son merkeze <= 250 px) -> kabul.
2. Kimlik yoksa/atladıysa: son merkeze en yakın aday (kapı içinde).
3. Yeni hedef: ekran merkezine en yakın aday.

**Histerezis:** `lost_counter <= max_lost_frames` iken takip modu korunur; bu sürede
güven eşiği 0.16'ya düşer. Bu, taret hızla dönerken oluşan hareket bulanıklığında
kimliğin kopmasını engeller. Sayaç aşılırsa `reset()`.

**Değerlendirme:**
- Mekânsal kapı, ByteTrack kimlik atlamasını reddeden doğru bir koruma; kimlik
  değişse bile seçim son konuma yakın kalır.
- 250 px kapı, 1280 px genişlikte yaklaşık 10 dereceye karşılık gelir; hedefin
  gerçekten 250 px'ten fazla sıçraması (yeniden yakalama) bir kare gecikir.
- **Seçim ölçütü tehdit değil, merkez yakınlığıdır.** Ekran merkezindeki küçük/uzak
  bir balon, kenardaki büyük/yakın olandan önce seçilir. Açısal boyut (yakınlık) veya
  güven ağırlıklı bir puanlama, tehdit önceliklendirmesi için daha doğru olur.
- `min_conf` main'de YOLO'ya parametre olarak geçiliyor; ancak ByteTrack'in kendi
  `new_track_thresh` değeri (~0.6) yeni kimlik üretimini sınırlar. 0.16 eşiği
  yalnızca **zaten kimliği olan** hedefi yaşatır; kimliksiz tespitler 3. kademeye düşer.

### 3.4 `fcs_manager.py` — üst yönetici (sistemin omurgası)

Sorumluluklar: hedef seçimi, Kalman, menzil kapılama, balistik lead, kilit, mod
yönetimi (P/RL), devriye, taret pozu.

**(a) Açı hesabı.** `pan = atan((bx-cx)/fx)`, `tilt = atan(-(by-cy)/fy)`. Telemetri
için ham hata `last_pan_err_deg`, kontrol için EMA'lı `actual_pan` kullanılır.

**(b) Ego-hareket telafisi.** Taret dönerken hedefin piksel konumu kayar. Bu kayma
filtreye "hedef hızı" olarak girerse hız kestirimi bozulur. Çözüm: son motor
komutunun 1. derece servo yanıtı modellenir:

    alpha = 1 - exp(-dt / tau),  tau = 45 ms
    step_pan(k) = (1-alpha)*step_pan(k-1) + alpha*cmd_pan(k-1)
    ego_dx = -fx * tan(step_pan)        (pan komutu kadar kamera döner)
    ego_dy = +fy * tan(step_tilt)

Ego kayması Kalman **kestirim** adımına eklenir; böylece hız durumu dünya
referanslı kalır. Potansiyometre farkı bilinçli olarak kullanılmaz: 0.1 derece
niceleme ve sıçramalar kare farkında büyük sahte hızlar üretir. Hedef yeni
yakalandığında (`was_lost`) gözlemci sıfırdan başlar; önceki artık komutlar
sıçrama yaratmaz.

**(c) Menzil kapılama (TF-Luna).** Dört katman:
1. Atış sırasında ölçüm dondurulur (duvar/flaş okuması engellenir).
2. Son geçerli mesafeden 50 cm'den fazla sapma reddedilir.
3. Aynı sapma 3 kare tutarlıysa gerçek mesafe değişimi kabul edilir.
4. Kutu geometrisinden (0.15 m yarıçap varsayımı) türetilen mesafeyle %75'ten fazla
   çelişen okuma duvar sayılır ve aday havuzuna hiç girmez.

Kararlılık: 3 karelik pencerede 10 cm'den az dalgalanma -> `range_stable`, ateş izni
için zorunlu. Bu, monoküler derinliğin zayıflığını tek sensörle kapatan pragmatik
ve doğru bir çözüm.

**(d) Balistik lead ve yercekimi.** Uçuş süresi `t = d/v`, lead süresi
`T = clip(d/10 + 0.015, 0.03, 0.40)`. Lead noktası **ölçülen** merkezden taşınır
(`bx + vx*T`); filtrenin konum gecikmesi ikinci kez eklenmez (doğru karar; aksi
halde lead iki kez gecikir). Yerçekimi: `theta = atan(g*D/(2*v^2))`, 2 m ve 10 m/s
için 5.6 derece.

**(e) Slew ve zero-lag feedforward.** `slew_factor` ateş sekansında 0 -> 1 arası
kosinüs yumuşatmayla açılır; `applied_lead = slew * lead`. Slew rampasının kare
başına türevi doğrudan motor komutuna eklenir; P'nin kendi gecikmesine takılmadan
nişan kayar. SEKANS: SPINUP (0.25 s) -> SETTLE (0.15 s) -> FIRING (0.08 s) ->
RECOVER (0.35 s) -> EVAL (1.5 s).

**(f) Mod yönetimi.** P ve RL aynı `active_pan/active_tilt` hedefini kullanır; RL
gözlemi ölçülen bbox'tan üretilir (`uses_measured_bbox`), P harmanı yoktur.
RL yüklenemezse sessizce P'ye düşer (`current_rl_weight = 0`).

**(g) Hedef yokken.** `max_lost_frames` (5) sonrası filtre/kilit/LOS sıfırlanır;
devriye açıksa `patrol_generator` devreye girer, değilse sıfır komut döner.

**Zayıflıklar:**
- `get_angular_velocity_deg_s` her karede çağrılıyor ama P kontrolcü bu omegayı
  kullanmıyor (`compute_p` içinde yok sayılıyor). Gereksiz hesap.
- `prev_applied_lead_*` yalnızca hedef varken güncelleniyor. Ateş sırasında hedef
  1-2 kare kaybolursa lead sıfırlanmıyor; yeniden yakalamada `delta_slew` bir karede
  büyük olabilir (adım sınırı 1.5 dereceye kırpıldığı için sınırlı).
- `reset()` `patrol_status` alanını sıfırlamıyor (kozmik).

### 3.5 `los_tracker.py` — dünya görüş hattı (alpha-beta)

`psi = taret_acisi + kamera_hata_acisi` (kamera tarete sabit). Ölçüm gürültüsünü
bastırmak için klasik alpha-beta:

    tahmin  = konum + hiz*dt
    artik   = olcum - tahmin
    konum  += alpha*artik
    hiz    += beta*artik/dt        (alpha=0.70, beta=0.20)

Karakteristik denklem `z^2 - (2-alpha-beta)z + (1-alpha) = z^2 - 1.1z + 0.3`;
kökler 0.600 ve 0.500 — gerçek, pozitif, kararlı. Yani salınım yok, sönüm
fazlasıyla yüksek; konum ~6-8 karede oturur, sabit hızlı girdide kalıcı hata sıfırdır.

**Değerlendirme:** doğru ve ucuz bir tahminci. **Kontrol hattına bağlı değildir**;
yalnızca telemetri ve log/Webots karşılaştırması içindir. Bu bilinçli bir karar
(FAZ 9'da Kalman ile karıştırılmasın diye ayrıldı) ama bakım maliyeti yaratıyor:
iki ayrı hız tahmincisi aynı logda iki farklı "hedef hızı" üretir.

### 3.6 `lock_manager.py` — kilit ve isabet olasılığı

**(a) Dinamik kilit penceresi.** Hedef hızına göre giriş eşiği:

| Hedef hızı (px/s) | Giriş | Çıkış (+0.60) |
|---|---:|---:|
| < 15 | 1.60 | 2.20 |
| 15-45 | 2.20 | 2.80 |
| >= 45 | 3.00 | 3.60 |

**(b) Kararlılık sayacı.** Pencere içinde +1, dışında -1, üst sınır 10. Kilit için
10 kare (~0.33 s) gerekir; kilitten çıkmak için sayaç 5'in altına inmeli veya hata
çıkış eşiğini aşmalı. Bu asimetri, eşik üzerinde titremeyi (chatter) engeller.

**(c) Ateş sonrası muafiyet.** `is_firing` boyunca kilit dondurulur; sonrasında 22
kare (0.73 s) sayaç alt sınırı 5'e çekilir; taret merkeze dönerken kilit erken düşmez.

**(d) İsabet olasılığı.** İki boyutlu Gauss saçılmasının dairesel integrali,
merkezî olmayan ki-kare CDF'i ile tam analitik hesaplanır:

    sigma^2   = sigma_weapon^2 + (0.25 * omega_deg_s * t_toplam)^2
    t_toplam  = d/v + 0.080
    theta_tgt = hedefin acisal yaricapi (bbox'tan veya 0.15 m / d)
    nc        = (aci_hatasi / sigma)^2
    x         = (theta_tgt / sigma)^2
    P(vurus)  = ncx2.cdf(x, df=2, nc) * 100

Sayısal doğrulama (sigma_weapon = 1.25 derece, 2 m, 60 px kutu = 1.22 derece):

| Hata (derece) | 60 px kutu | 120 px kutu |
|---|---:|---:|
| 0.00 | %37.8 | %85.0 |
| 0.85 | %31.5 | %78.5 |
| 1.50 | %21.5 | %65.3 |
| 2.00 | %13.8 | %51.9 |
| 3.00 | %3.8 | %24.4 |

**Yorum:** sonuç büyük ölçüde **hedefin görünen boyutuna** bağlıdır; silah saçılması
(1.25 derece) küçük hedeflerde tavanı %38'e indirir. Model doğru kurulmuş, ancak
ateş kararında kullanılmıyor: `main.py` yalnızca `is_locked && range_valid &&
range_stable` arıyor. 3 derece hata ile 60 px'lik hedefte "LOCKED" olunup %3.8
olasılıkla ateş edilebilir.

**Zayıflıklar:**
- Eşikler (1.60/2.20/3.00) ve `0.25` katsayısı kod içinde sabit; SSOT dışında.
- Saçılma izotropik varsayılıyor (`df=2`); gerçek pan/tilt saçılması farklıdır.
- `sigma_motion` için `t_toplam` 0.28 s kullanılırken kontrolcü lead'i 0.215 s
  hesaplıyor; iki model aynı gecikmeyi farklı varsayıyor (Bölüm 5).

### 3.7 `patrol_generator.py` — devriye

Üç durumlu makine:

1. **WAITING (1.5 s):** hedef kaybolunca son konumda bekler; hedef dönerse motor
   hiç hareket etmeden takibe döner.
2. **RETURNING:** `20 derece/s` hızla merkeze döner. **Birikimli adımlayıcı:**
   her karede `min(20*dt, 1.5)` kadar birikir; yalnızca birikim 0.95 dereceyi
   aştığında tek pakette gönderilir. Bu, MG996R'nin ~0.9 derecelik mekanik ölü
   bölgesinde 0.67 derecelik adımların yutulmasını engeller (FAZ 8'de eklenen çözüm).
3. **SCANNING:** 1:2 Lissajous sekiz motifi:
   `pan = 45*sin(w*t)`, `tilt = 16*sin(2*w*t)`, `w = 2*pi/8`, 2 s yumuşak açılma.

Hız analizi: pan tepe hızı `45*w = 35.3 derece/s` (1.18 derece/kare), tilt
`16*2w = 25.1 derece/s` (0.84 derece/kare). Her ikisi de 1.5 derece/kare adım
sınırının altında; yani desen fiziksel olarak izlenebilir ve komut hiç doyuma
girmez. Desen merkezden iki kez geçer ve köşe içermez — servo için en nazik tarama.

**Zayıflık:** genlikler (45/16), periyot (8 s), dönüş hızı (20 derece/s) ve 0.95
eşiği kod içinde sabit; `CONFIG` dışında. Devriye sırasında tilt 16 derece,
firmware sınırı 25 derece olduğu için güvenli.

### 3.8 `turret_hud.py` — taktik arayüz

Saf çizim katmanı; karar üretmez. Algoritmik değeri **bilgi mimarisi** ve
**declutter** kurallarında:

- Renk disiplini: arama amber, kilit kırmızı, devriye yeşil, E-stop kırmızı.
- Kademeli görünürlük: kilit/ateş anında mod banner'ı, footer, olay günlüğü ve FPS
  gizlenir; yalnızca nişangah, kilit barı ve ateş paneli kalır.
- Ateş paneli evreye göre metin üretir (SPINUP kalan ms, SETTLE, FIRING, RECOVER,
  EVAL); SPINUP'ta ilerleme çubuğu.
- Kilit barı `lock_counter / 10`, isabet olasılığı %70/%40 renk eşikleri.
- Olay günlüğü 6 satırlık halka tampon.

**Zayıflık:** her karede onlarca `cv2` çağrısı ve `getTextSize` yapılır; 720p'de
maliyet birkaç ms olabilir. Metin ölçümleri kareler arası önbelleklenebilir.
Ayrıca panel ROI'si çerçeve dışına taşarsa boş dilim `addWeighted`'a girer; küçük
çözünürlüklerde istisna riski var.

### 3.9 `telemetry.py` — kara kutu

**Şema v2, 44 kolon:** kimlik (frame_id, time, dt, latency, fps), hedef geometrisi,
ham ve filtrelenmiş hız, inovasyon, kontrolcü dinamikleri (kp_eff, db_eff), LiDAR,
FCS durumu, kilit/olasılık, taret pozu, servo geri beslemesi, devriye ve dünya LOS.

**Mimari:** 30 karelik tampon -> toplu CSV yazımı (I/O maliyeti 1/30'a iner),
oturum başına `.meta.json` (git commit, donanım, tüm kritik CONFIG değerleri, kolon
listesi) ve `_events.csv` (operatör/sistem olayları). Kapanışta özet: süre, ortalama
FPS, tespit ve kilit oranları.

**Değerlendirme:**
- `getattr(fcs, ...)` ile savunmacı okuma: FCS arayüzü değişse bile kayıt kırılmaz.
- `latency_ms` gerçek işlem süresidir (dt değil) — doğru.
- `angular_err_*` kontrolcünün filtrelenmiş değeri değil, ham kamera hatası — doğru
  (FAZ 12'de düzeltilmiş).
- **Ölü sütunlar:** `sensor_temp_c` hiç doldurulmuyor (TF-Luna çerçevesinde sıcaklık
  yok), `sigma_a` artık sabit 300 (adaptif Q kaldırıldı). İkisi de analizde yanıltıcı.
- Tampon 30 kare: ani çökmede en fazla 1 saniyelik veri kaybı. `fsync` yok (doğru
  tercih; CSV için gereksiz maliyet).

---

## 4. Kesit Analizi: Kontrol Gecikmesi ve Lead Bütçesi

Sabit açısal hızlı (`omega`, derece/s) bir hedef için P kontrolcünün kapalı çevrim
kalıcı durumu (yumuşak ölü bölge `db`, kare başına kazanç `Kp`, 30 FPS):

    u = Kp * (e - db)  ve  u = omega/30   =>   e = db + omega/(30*Kp)

Yani taret, **nişan noktasının** `db + omega/30Kp` kadar gerisinde kalır; bu,
hız üretmek için gereken zorunlu konum hatasıdır. Nişan noktası lead ile
`omega*T_lead` kadar ileri taşınmıştır. Dolayısıyla hedefe göre gerçek nişan hatası:

    gercek_hata = omega*(t_ucus + t_gecikme) + e - omega*T_lead

`t_ucus = d/10`, `t_gecikme = 0.08`, `T_lead = d/10 + 0.015`, `db = 0.85`,
`Kp = 0.22` ile:

| omega (derece/s) | Nişan hatası e | Lead (derece) | Olması gereken | Artık ıska (derece) |
|---:|---:|---:|---:|---:|
| 2 | 1.15 | 0.43 | 0.56 | 1.28 |
| 5 | 1.61 | 1.08 | 1.40 | 1.93 |
| 10 | 2.37 | 2.15 | 2.80 | 3.02 |
| 15 | 3.12 | 3.23 | 4.20 | 4.10 |

2 m'de hedefin açısal yarıçapı 4.29 derece (0.15 m). Buna göre 10 derece/s üzerinde
artık ıska, hedef yarıçapına yaklaşır; 15 derece/s'de aşar.

**Kök neden:** `T_lead` yalnızca uçuş süresini ve 15 ms'lik sabit payı kapsıyor;
kontrolcünün kendi birinci derece gecikmesini (`1/(30*Kp) = 151 ms`) ve taşıma
gecikmesini (80 ms) kapsamıyor. Ölü bölge ise geri kalan payı yiyor. Doğru lead:

    T_lead = (t_ucus + t_gecikme) + 1/(30*Kp) + db/omega

**Not:** Bu, doğrusal sistem teorisinin sonucudur; gerçek servo, niceleme ve
firmware hız sınırı ek gecikme ekler, yani tablo alt sınırdır. Ayrıca `lead_time_max_s`
0.40'a çıkarılmış (çalışma ağacında); bu, sorunun fark edildiğini gösteriyor ama
2 m'de `T_lead = 0.215` olduğu için tavan devreye girmiyor.

**Uygulama önerisi (en küçük müdahale):** lead süresine ölçülmüş döngü gecikmesini
ekle (`lead_time_offset_s` -> `0.08 + 1/(30*Kp)`, yani yaklaşık 0.23 s); ölü bölge
payı (`db/omega`) ayrıca ele alınmalı, çünkü sabit bir zaman payıyla
kapatılamaz — hıza bağlıdır. Alternatif: hız feedforward'u açıp ölü bölgeyi
küçültmek. Hız FF'si bilinçli olarak kapalı (`compute_velocity_ff` sıfır döner);
bu analiz, kapalı kalmasının bedelini sayısallaştırıyor.

---

## 5. Kesit Analizi: Niceleme ve Ölü Bölge Zinciri

| Katman | Çözünürlük / eşik | Etki |
|---|---|---|
| YOLO kutu merkezi | 1 px = 0.04 derece | filtre gürültüsü (r_var = 8 px) |
| Kalman çıkışı | hız ölü bölgesi 30-60 px/s | duran hedefte sıfır hız |
| P kontrolcü | 0.85 / 0.95 derece yumuşak ölü bölge | kalıcı durum hatası |
| UART | 0.1 derece + artık biriktirici | ortalama hatayı 0.05 derecenin altına indirir |
| Servo mekanik | ~0.1 derece ölü bölge, ~1.2 derece backlash | yön değişiminde kayıp |
| Firmware hız sınırı | 45 derece/s, 1.5 derece/paket | PC FPS'inden bağımsız tavan |

Zincir tutarlı: her katmanın çözünürlüğü bir üsttekinden en az 3 kat ince. Tek
zayıf halka, geri besleme nicelemesinin (0.1 derece) ego hesabında kullanılmaması;
bu bilinçli ve doğru bir tercih (potansiyometre sıçraması sahte hız üretir).

---

## 6. Bulgular

| # | Önem | Bulgu | Kanıt | Öneri |
|---|---|---|---|---|
| 1 | Yüksek | Lead süresi, kapalı çevrim gecikmesini ve ölü bölgeyi kapsamıyor; hızlı hedefte sistematik ıska | Bölüm 4 tablosu | `T_lead`'e `1/(30*Kp)` + taşıma gecikmesi ekle veya hız FF aç |
| 2 | Yüksek | Ateş kapısı hedef boyutundan bağımsız; 3 derece hatada %3.8 isabetle ateş mümkün | `lock_manager.update`, `main.py` ateş koşulu | Ateş için `hit_probability >= esik` veya `hata < k * theta_hedef` şartı |
| 3 | Orta | Çalışma ağacındaki refactor (Kalman kapısı, RL hız gözlemi, `a*dt^2`) commit edilmedi ve saha/Webots doğrulaması yok | `git status`, Bölüm 6.1 | Commit öncesi Webots profil 2/5/7 ve gerçek taret testini yinele |
| 4 | Orta | Kalman tekil-sıçrama kapısı kaldırıldı ama docstring hâlâ var; tek sahte kutu hız durumunu çekebilir | `kalman_filter.py:20-23` vs güncel kod | Kapıyı geri koy veya docstring'i güncelle + saha loguyla doğrula |
| 5 | Orta | İvme sınırı `a*dt^2`'ye çevrildi: 120 derece/s^2 ile 1.5 derece/kare'e 11 karede (0.37 s) ulaşılır; 45 derece/s'den durma mesafesi 8.4 derece | `p_controller.py:163-164`, `rl_controller.py:325-330` | Sınırı servo kapasitesine göre seç (45 derece/s'ye 50 ms'de çıkmak 900, 100 ms'de 450 derece/s^2 ister) ve Webots taramasını yinele; mevcut Kp doğrulaması artık geçersiz |
| 6 | Orta | RL'de hız gözlemi geri açıldı (`uses_velocity_obs = True`) — v1.4'te gürültülü Kalman hızı politikayı doyuma soktuğu için kapatılmıştı | `rl_controller.py:88-90` | Yeniden eğitim/doğrulama olmadan üretime alma |
| 7 | Orta | UART yazımı ana döngüde; `write_timeout = 40 ms` worst case kare düşürür | `fcs_comm.py:131-136,199-203` | Yazmayı kuyruk thread'ine al veya tamponu büyüt |
| 8 | Orta | 8 bit toplam checksum (CRC değil) | `fcs_comm.py:200`, `fcs_comm.c:56-62` | CRC-8'e geç (iki taraf birlikte) |
| 9 | Düşük | Ateş sonrası EVAL'de `range_valid` durumu eski değerde kalıyor; gating "dondu" mantığı docstring ile tam örtüşmüyor | `fcs_manager.py:142-145` | Davranış bilinçliyse docstring'i netleştir |
| 10 | Düşük | `compute_hit_probability` fy yedeği `fx * 9/16`; kare pikselde fy = fx olduğu için yanlış (latent) | `lock_manager.py:81` | Yedeği `CONFIG.optics.compute_fy(...)` ile hesapla |
| 11 | Düşük | Devriye ve kilit eşikleri kod içinde sabit (SSOT ihlali) | `patrol_generator.py:38-42`, `lock_manager.py:56-61` | `CONFIG.patrol` / `CONFIG.lock` altına taşı |
| 12 | Düşük | Atıl alanlar: `PController.kp_pan/kp_tilt` hiç okunmuyor; P modunda `get_angular_velocity_deg_s` her karede hesaplanıp kullanılmıyor | `p_controller.py:90-91`, `fcs_manager.py:350-358` | Kaldır veya FF yoluna bağla |
| 13 | Düşük | Telemetride `sensor_temp_c` hep 0, `sigma_a` hep 300 | `telemetry.py:269, 344` | Kaldır veya gerçek kaynakla besle |
| 14 | Düşük | `max_lost_frames` varsayılanı 5; dokümanda 4 | `fcs_manager.py:40`, AGENTS.md FAZ 8 | Dokümanı koda uydur |

### 6.1 Analiz penceresinde gözlenen durum değişimi

Ağaç analiz sırasında aktif olarak düzenlendi; kronoloji:

- 18:11: `import turret_fcs` -> `AttributeError: 'KalmanConfig' object has no attribute
  'outlier_residual_px'`; 9/9 test modülü toplanamadı.
- 18:12-18:13: `kalman_filter.py` ve `rl_controller.py` güncellendi (tekil-sıçrama
  kapısı kaldırıldı, RL hız gözlemi açıldı, ivme `dt^2`).
- 18:14-18:16: testler yeni sözleşmeye göre güncellendi.
- 18:17 (son durum): `pytest tests` -> **21 geçti, 3 hata**; hataların üçü de
  sandbox'ın geçici dizin izni (`tmp_path`), kod kaynaklı değil.
  `import turret_fcs` çalışıyor.

Değişikliklerin niteliği (commit edilmemiş):

| Dosya | Değişiklik | Analitik etki |
|---|---|---|
| `kalman_filter.py` | 150 px tekil-sıçrama kapısı ve `reacquire_frames` kaldırıldı | Tek sahte kutu artık hız durumunu doğrudan çekebilir; docstring hâlâ kapıyı anlatıyor |
| `rl_controller.py` | `uses_velocity_obs = True`, `prefers_smooth_velocity = True` | Politikaya gürültülü Kalman hızı giriyor (v1.4'te bu yüzden kapatılmıştı) |
| `p_controller.py`, `rl_controller.py` | İvme sınırı `a*dt` -> `a*dt^2` | Boyut olarak doğru, ama 120 derece/s^2 ile 0.37 s rampa; kapalı çevrim artık rampa tarafından yönetiliyor |
| `p_controller.py`, `fcs_manager.py` | Tilt adım sınırı ayrıldı (0.75 derece) | Doğru düzeltme; RL tilt çıkışı da artık 0.75 ile sınırlı |

Testlerin güncellenmesi davranış değişimini **gizlemez**: `test_p_controller.py`
ivme testinde rampayı atlamak için 2000 derece/s^2 kullanıyor, kazanç tarifesi
testinde 15 kare bekliyor. Yani "tek karede kazanç tarifesi" ve "dt ile doğrusal
ölçekleme" özellikleri artık varsayılan ayarda geçerli değil — bu, ivme sınırının
baskın dinamik olduğunun test tarafından da kabul edilmesidir (Bulgu 5).

---

## 7. Önerilen Sıra

1. Refactor'ü commit etmeden önce Webots profil 2/5/7 ve gerçek taret testini
   yinele; özellikle ivme sınırı değişikliği Kp doğrulamasını geçersiz kılıyor.
2. İvme sınırını fiziksel servo kapasitesine göre yeniden seç; Webots kazanç
   taramasını tekrarla (mevcut 0.22/0.11 doğrulaması eski sınırla yapıldı).
3. Lead bütçesini düzelt (Bölüm 4) ve saha logunda `angular_err_pan` ile
   `hit_prob` ilişkisini doğrula.
4. Ateş kapısına isabet olasılığı veya hedefe göre normalize edilmiş hata şartı ekle.
5. UART yazımını thread'e al, CRC-8'e geç.
6. Sabitleri `CONFIG`'e taşı, docstring'leri koda eşitle, ölü kodu sil.

---

## 8. Yeniden Üretim

    python -m pytest tests -q
    git status --short
    python -c "import turret_fcs; print(turret_fcs.CONFIG.control.kp_pan)"

Sayısal tablolar `analysis/` altındaki betiklerle değil, bu belgedeki kapalı form
ifadelerle üretildi; `fx = 640/tan(24.4) = 1410.9`, `fy = 360/tan(14.3) = 1412.3`.
