# Kod Düzeni, Okunabilirlik ve Ölü Kod İncelemesi

Kapsam: üretim Python paketi (`turret_fcs/`), STM32 firmware (`stm32/turret_main/Core/`),
Webots denetleyicileri (`webots/controllers/`), testler ve `analysis/` araçları.
Yöntem: statik okuma, AST tabanlı ölü parametre/ölü metot taraması, satır ve yorum
ölçümü, iki ana döngünün satır benzerliği (difflib), test çalıştırması.

> Anlık görüntü uyarısı: İnceleme sırasında çalışma ağacı aktif olarak düzenlendi
> (18:11-18:22 arasında `config.py`, `kalman_filter.py`, `rl_controller.py`,
> `fcs_manager.py`, `p_controller.py` ve testler değişti). Bu belge 24 Ekim 2026,
> 18:25 itibarıyla alınan görüntüyü anlatır; satır numaraları kayabilir.
> Bulguların çoğu yapısaldır (isim, sözleşme, tekrar), bu yüzden refactor
> tamamlandığında da geçerliliğini korur.
>
> Algoritma ve kontrol teorisi bulguları ayrı belgede:
> [algoritma_analizi.md](file:///c:/YTU/bitirme_projesi/turret_fcs/docs/algoritma_analizi.md).
> Bu belge onu tekrarlamaz; kodun düzeni, tekrarı, ölü kodu ve doküman uyumu üzerinedir.

---

## 0. REVİZYON: 24 Ekim 2026, 19:10

Bu belgeden sonra yapılan refactor, aşağıdaki bulguların bir kısmını kapatmıştır.
Güncel durum:

**Kapananlar**

- İki monolitik döngü ayrıştırıldı: `engagement.py` (186 satır, FSM),
  `presenter.py` (142 satır, HUD sunumu), `keymap.py` (100 satır, tuş eylemleri).
  `main.py` 405 -> 263, `webots_main.py` 390 -> 278 satır. İki dosyanın kalan
  satır benzerliği %22; eski kopya bloklar (FSM, durum metni, HUD sırası, tuş
  haritası) artık tek yerde. (Bölüm 3.1 kapandı.)
- Telemetri `TelemetryRow` dataclass'ına geçti; başlıklar alan adlarından
  üretiliyor. (Bölüm 3.3 kapandı.)
- `PController.kp_pan/kp_tilt` ölü alanları `kp_frame_*` ile senkron property
  oldu; `filtered_pan/tilt` de gerçek alana bağlandı. (Bölüm 4.1/4.4 kapandı.)
- `FCSManager` içindeki gereksiz `omega_*` ön hesabı kaldırıldı; P/RL komut
  integrasyonu `_integrate_command()` altında tekilleşti. (Bölüm 4.3 kapandı.)
- Firmware: `main.c` CubeMX `USER CODE` bloklarına taşındı, `Error_Handler`
  `FLYWHEEL_*` / `SOLENOID_*` makrolarını kullanıyor, `Servo_IsAtCenter()`
  silindi. (Bölüm 7.2/2, 7.2/3, 7.2/4 kapandı.)
- P, Kalman, RL, FIRING, Webots v1.3 etiketi, 40.5/46.9 derece/s ve HUD renk
  docstring'leri düzeltildi. (Bölüm 5.2/3'ün bir kısmı kapandı.)

**19:10'da açık olanlar — bdebbb4 commit'i ile kapandı**

1. `webots_main.py` tuş işleyicisi kanonik `keymap` sözleşmesine bağlandı;
   `action.action` kaldırıldı, E-stop geçişinde launcher ve servo güvenliği
   webots tarafında ele alınıyor.
2. `presenter.py`: devriye metni artık `f"PATROL: {patrol_status}"`; takip
   çizgisi geri bağlandı; `latency_ms` ve `temp_c` ölü parametreleri kaldırıldı;
   HUD "firing" bayrağı `engagement_state.is_active` oldu, EVAL paneli düzeldi.
3. Kalman sözleşmesi kod, testler, README ve AGENTS.md'de 150 px robust
   tekil-sıçrama kapısında birleşti; NIS'in saha telemetrisinde ürettiği sahte
   resetler FAZ 14'te sayılarla belgelendi.
4. `main.py` başlığı ve `numpy` ithalatı, `fcs_manager` yorumu, README/AGENTS
   dosya haritaları düzeltildi; EVAL metni nötr "ENGAGEMENT COMPLETED" oldu.
5. `compute_velocity_ff()` ve sınıflardaki `**kwargs` yutucuları kaldırıldı.

**19:27 itibarıyla kalanlar**

- Telemetride `rl_weight`, `sensor_temp_c`, `sigma_a`, `lock_thr` sütunları hâlâ
  sabit/ikincil değer taşıyor. Kaldırılmalı ya da gerçek kaynağa bağlanmalı.
- `docs/algoritma_analizi.md` 18:17 anlık görüntüsü olarak duruyor; 39-44 ve
  456. satırlar artık güncel kodu anlatmıyor. Başına tarihsel not eklenmeli.
- E-stop davranışı iki noktada eskisinden ayrılıyor: (a) `fcs.reset()` artık
  çağrılmıyor, dolayısıyla serbest bırakıldığında önceki kilit durumu korunuyor;
  (b) `patrol_enabled = False` yalnızca Webots sarmalayıcısında var, `keymap`
  içinde yok; üretimde E-stop sonrası devriye kendiliğinden devam edebilir.
  İkisi de bilinçli tercih değilse `keymap` E-stop dalında düzeltilmeli.
- Firmware'de dört `.c` dosyasında modül başlığı yok; bu ortamda ARM derleyici
  bulunmadığı için "sıfır uyarı" iddiası burada doğrulanamadı.
- `test_entrypoint_contracts` yalnız çağrıyı taklit ediyor; `main.py` ve
  `webots_main.py` içindeki gerçek çağrı yerlerini denetlemiyor. Giriş noktası
  yine bozulursa test yeşil kalır; AST tabanlı bir çağrı-yeri testi daha güçlü
  olur.

Bu revizyonun ayrıntılı doğrulaması için bir sonraki inceleme turuna bakın;
aşağıdaki bölümler tarihsel anlık görüntü olarak korunmuştur.

---

## 1. Kısa Cevap: "Anlaşılır mı?"

Evet, büyük ölçüde anlaşılır. İyi olan üç şey:

1. Sorumluluk ayrımı net: optik (`camera_thread`), protokol (`fcs_comm`), karar
   (`fcs_manager`), kilit (`lock_manager`), sunum (`turret_hud`), kayıt (`telemetry`)
   ayrı modüllerde ve bağımlılık yönü tek yönlü.
2. İsimlendirme tutarlı: `compute_*`, `draw_*`, `_update_*`, `is_*`, `max_*` kalıpları
   dosyalar arasında aynı anlamda kullanılıyor; sabitler büyük harf.
3. Modül başlıkları gerekçe yazıyor: "ne yapıldı" değil "neden böyle yapıldı" anlatılıyor.
   Jüri savunması için bu, kodun en değerli kısmı.

Ancak okunabilirlik dosyalar arasında eşit değil ve dört sistematik zayıflık var:

- İki monolitik uygulama döngüsü (`main.py` ve `webots_main.py`) aynı işi neredeyse
  aynı satırlarla iki kez yazıyor.
- Ölü veya ikinci kopya parametreler var: yazılan ama okunmayan alanlar, daima sıfır
  dönen API'ler, hiç dolmayan telemetri sütunları.
- Kod ile doküman arasında kayma var: kaldırılmış bir özellik hâlâ docstring'de,
  README'de ve mimari şemada anlatılıyor.
- Otomatik kapı yok: ruff/flake8/biçimlendirici ve CI'da lint adımı bulunmuyor;
  bu yüzden ölü ithalatlar ve ölü parametreler birikmiş.

STM32 tarafı "gereksiz uzun" değil; tam tersine ana döngü yaklaşık 150 satır ve
yorum oranı yüksek. Oradaki sorun uzunluk değil, modül başlıklarının ve protokol
sözleşmesinin tek yerde toplanmamış olması.

---

## 2. Ölçümler

Üretim Python paketi (üst düzey `turret_fcs/*.py`) yaklaşık 3.700 satır; tüm proje
Python'u (analiz + test + eğitim + Webots) boş satırlar hariç yaklaşık 10.800 satır.
Kendi firmware kodumuz (CubeMX üretimi dahil) yaklaşık 2.650 satır; bunun yaklaşık
1.000 satırı HAL/CubeMX iskeleti.

| Dosya | Yaklaşık satır | En uzun fonksiyon | Not |
|---|---:|---|---|
| `turret_fcs/main.py` | 405 | `main()` 366 satır | Başlatma + FSM + HUD + klavye + telemetri tek fonksiyonda |
| `turret_fcs/fcs_manager.py` | 485 | `update()` 194 satır | 6 ayrı iş tek metotta |
| `turret_fcs/telemetry.py` | 420 | `log_frame()` 158 satır | 44 sütun pozisyonel liste |
| `turret_fcs/turret_hud.py` | 482 | `draw_combat_panel()` 112 satır | 6 benzer dal |
| `turret_fcs/rl_controller.py` | 348 | `predict_angles()` 91 satır | Gözlem + taper + ivme + geçmiş |
| `turret_fcs/fcs_comm.py` | 287 | `_consume_rx_buffer()` 27 satır | Modül içi dağılım dengeli |
| `webots_main.py` | 390 | `main()` 335 satır | `main.py` ile %57 satır benzerliği |
| `stm32/.../main.c` | 371 | `main()` 143 satır | ~190 satırı CubeMX init |

Yorum profili: Python dosyalarında satır içi yorum oranı %1-7, buna karşılık modül
docstring'leri 15-46 satır. `main.c` yorum oranı %32 ile projenin en iyi belgelenmiş
dosyası. Yani Python'da "az yorum, çok başlık"; C'de "çok yorum, az başlık" dengesi var.

---

## 3. Python: Uzunluk ve Tekrar

### 3.1 İki ana döngü kopyası (en büyük yapısal borç)

`main.py` içindeki `main()` ile `webots_main.py` içindeki `main()` aynı altı bloğu
neredeyse satır satır tekrarlıyor:

1. Donanım/model başlatma ve açılış banner'ı,
2. SPINUP / SETTLE / FIRING / RECOVER / EVAL atış durum makinesi (yaklaşık 45 satır),
3. Durum metni ve renk seçimi (yaklaşık 25 satır),
4. HUD çizim sırası (yaklaşık 58 satır),
5. Klavye haritası: M, P, Space, Y, N (yaklaşık 38 satır),
6. Telemetri çağrısı.

Boşluklar temizlendikten sonra iki döngünün benzerliği %57. Fark yalnız üç yerde:
zaman kaynağı (`time.time()` / `sim_t`), komutun UART'a mı motor API'sine mi gittiği
ve Webots'a özgü jitter/balistik hakemi. Bu üç fark, ortak bir çekirdeğin arkasına
gizlenebilir.

Önerilen ayrıştırma (davranış değişmeden):

| Yeni modül | İçerik | Kazanç |
|---|---|---|
| `engagement.py` | Atış aşaması, `slew_factor`, aşama metni, ateş izni | ~120 satır tek yerde |
| `presenter.py` | `status_text/color`, HUD çağrı sırası | ~80 satır tek yerde |
| `keymap.py` | Tuş -> eylem tablosu | ~50 satır, test edilebilir |
| `bootstrap.py` | Kamera/STM/model/HUD kurulumu ve açılış logları | ~60 satır |

Bu dört modül `main()`'i 366 satırdan yaklaşık 120 satıra indirir ve Webots tarafı
aynı modülleri çağırdığı için ikinci kopya tamamen kaybolur.

### 3.2 `FCSManager.update()` 194 satır

Tek metotta altı iş yapılıyor: geometri/LiDAR kapılama, ego-motion, Kalman + lead,
kilit, slew/feedforward, kontrolcü seçimi. Öneri: `_update_tracking()` (gözlem),
`_update_ballistics()` (lead + gravity + slew), `_select_command()` (P/RL) olarak
üçe ayırmak. Bu, `update()`'i ~70 satıra indirir ve P/RL dallarını bağımsız test
edilebilir hâle getirir.

Ayrıca RL ve P dallarında `turret_pan_deg`/`turret_tilt_deg` güncellemesi üç satır
hâlinde iki kez yazılmış; tek bir `_integrate_pose()` çağrısına alınabilir.

### 3.3 `TelemetryLogger.log_frame()` 158 satır / 44 sütun

Sütunlar pozisyonel bir liste ile yazılıyor, başlıklar ayrı bir listede. Bir sütun
eklemek veya sırasını değiştirmek, sessizce yanlış CSV üretir; testler bunu
yakalamıyor. Öneri: `@dataclass TelemetryRow` (44 alan) + `HEADERS = [f.name for f
in fields(TelemetryRow)]`. Bu, tek kaynak sağlar ve `writer.writerow(asdict(row))`
ile kolon kaymasını imkânsız kılar.

### 3.4 `TurretHUD.draw_combat_panel()` 112 satır

Altı dalın beşi aynı iskelet: iki metin satırı, renk, `panel_h`, `line_y_offsets`.
Durum -> (renk, satırlar) tablosu kurulursa fonksiyon ~40 satıra iner ve yeni bir
aşama eklemek tek satır olur.

### 3.5 Küçük tekrarlar

- `fcs_manager` içinde `last_pan_err_deg` hesabı ile `PController.compute_angles`
  aynı `arctan` formülünü iki kez yazıyor (biri telemetri için ham, diğeri EMA'li).
  Tek yardımcı fonksiyon yeterli; "ham" ve "filtreli" ayrımı parametre olmalı.
- HUD renkleri (`(0, 191, 255)`, `(0, 215, 255)`, `(0, 0, 255)` ...) iki uygulama
  dosyasında ve HUD içinde onlarca kez tekrarlanıyor. `class Colors` veya sözlük,
  renk disiplinini tek yerde tutar.
- Ateş aşamaları ("SPINUP", "SETTLE", "FIRING", "RECOVER", "EVAL") dört dosyada
  serbest metin. Bir `Enum` yazım hatasını derleme/çalışma anında yakalar.

---

## 4. Python: Ölü Kod ve Ölü Parametreler

Aşağıdakilerin her biri AST taramasıyla doğrulandı: tanım dışında hiç okunmuyor,
ya da yalnızca testte okunuyor.

| # | Bulgu | Kanıt | Etki |
|---|---|---|---|
| 1 | `PController.kp_pan` / `kp_tilt` yazılıyor, hiç okunmuyor | `p_controller.__init__`; `webots_tuning.apply_tuning`; `test_webots_tuning` bu ölü alanı doğruluyor | Kazancın iki temsili var; biri sessizce eskiyebilir. Test yanlış şeyi koruyor. |
| 2 | `compute_velocity_ff()` daima `(0.0, 0.0)` döner | `p_controller`; tek çağıran `test_p_controller` | Hız FF'si kapalı; API ve testi ölü yolu ayakta tutuyor. |
| 3 | `compute_p(omega_pan_deg_s, omega_tilt_deg_s)` parametreleri yok sayılıyor | `p_controller.compute_p`; `fcs_manager` her karede bu hızları hesaplayıp geçiriyor | Kare başına gereksiz hesap + okuyucuya "FF var" izlenimi. |
| 4 | `PController.filtered_pan/filtered_tilt` yalnız yazılıyor | `compute_p`, `reset` | Anlamsız durum; `compute_angles` zaten filtreliyor. |
| 5 | RL gözlem bayrakları sabit `True`; `getattr(..., False)` dallarının `else`'i ölü | `rl_controller` sınıf alanları; `fcs_manager` iki iç içe `getattr` | Üç bayrak ve iki dal, tek davranış için var. Kaldırılırsa 12 satır ve bir kavram eksilir. |
| 6 | `RLConfig.deadband_px` adı yanıltıcı | `rl_controller.predict_angles` yalnız `> 0` kapısı olarak kullanıyor; gerçek ölü bölge `deadband_pan_deg/tilt_deg` | "5 px ölü bölge" docstring'i yanlış; 0 verilince açısal deadband+taper tamamen kapanıyor. |
| 7 | `KalmanFilter._outlier_frames` ve `current_sigma_a` işlevsiz | `reset`/`_start_track` dışında yazılmıyor | Kaldırılan tekil-sıçrama kapısının kalıntısı; docstring hâlâ kapıyı anlatıyor. |
| 8 | Telemetri `sensor_temp_c` hep 0 | `fcs_comm.temp` hiç atanmıyor; firmware sıcaklık göndermiyor | 44 sütunun biri baştan sona sabit; tez tablosunda yanıltıcı. |
| 9 | Telemetri `sigma_a` hep 300 | `current_sigma_a` güncellenmiyor | "Dinamik süreç gürültüsü" açıklaması kodla uyuşmuyor. |
| 10 | Telemetri `rl_weight` artık ağırlık değil mod bayrağı | `fcs_manager.current_rl_weight` yalnız 1.0/0.0 | Harman kaldırıldı; sütun adı eski mimariyi anlatıyor. |
| 11 | `TelemetryLogger.log_event()` hiç çağrılmıyor | `main.py` yalnız `hud.log_event` kullanıyor | `_events.csv` yalnız başlık satırıyla oluşuyor; modül docstring'i olay günlüğü vadediyor. |
| 12 | `main.py` içinde `fx/fy` erken ataması ölü | 54-55. satırlar; 141-142'de döngü içinde yeniden hesaplanıyor | İlk değer hiç kullanılmıyor. |
| 13 | Test-only üretim API'leri | `KalmanFilter.predict_next_position`, `FCSManager.target_angular_velocity_deg_s`, `TurretHUD.get_box_label` | Üretimde tüketicisi yok; ya kaldır ya gerçek yolda kullan. |
| 14 | `**kwargs` yutucuları | `PController`, `RLController`, `KalmanFilter`, `TurretEnv` | Kaldırılan alanlar (`far_gain`, `outlier_*`, `deadband_exit_px` ...) sessizce yutuluyor; yazım hatası fark edilmiyor. |
| 15 | Webots tarafında ölü parametreler | `webots_telemetry_logger.log(is_firing)` kullanılmıyor; `len(best_box) > 4` hiç doğru olmuyor; `webots_tuning._profile_limits_for(path)` parametresi kullanılmıyor | Küçük ama okuyucuyu yanıltan artıklar. |
| 16 | Çift anahtar | `webots_virtual_ballistics` sözlüğünde `dist_m` ve `dist_center_m` aynı değer | Hangi alanın okunacağı belirsiz. |
| 17 | `TurretEnv` içinde `render_mode`, `h_fov`, `v_fov` yalnız yazılıyor | `turret_env.__init__` | Kullanılmayan ortam alanları. |
| 18 | `analysis/` içinde 8 kullanılmayan ithalat | `generate_benchmark_plots` (os, CONFIG), `generate_kalman_plots` (math, os), `inspect_logs` (Path), `plot_patrol_log` (math), `plot_tuned_system` (math), `system_identification` (os), `test_aux_modules` (CONFIG) | ruff F401 bunların hepsini tek komutta yakalar. |

Ek not: `main.py` içindeki `ENABLE_TELEMETRY = True` yerel sabiti, telemetriyi
kapatmanın tek yolunu kodu düzenlemek yapıyor; `CONFIG` veya komut satırı bayrağı
olmalı.

---

## 5. Python: Yorumlar ve Modül Açıklamaları

### 5.1 Güçlü yön

Her modülün başında amaç, tasarım gerekçesi ve sayısal doğrulama var. Örneğin
`p_controller` başlığı "neden Ki/Kd yok" sorusunu, `kalman_filter` başlığı "neden
tek Q/R" sorusunu tek ekranda cevaplıyor. Bu, tez jürisi için kodun en değerli
katmanı ve korunmalı.

### 5.2 Zayıf yönler

1. Başlığın ilk ekranı sözleşme değil tarih: `p_controller` 46, `rl_controller` 34
   satır docstring; içinde eski sürümün sorunları ve Webots tarama sonuçları var.
   Okuyucu "bu sınıf neyi garanti ediyor" cevabını 40 satır sonra buluyor.
   Öneri: ilk 5 satır sözleşme (girdi birimleri, çıktı, yan etki, varsayılanlar),
   geri kalan "tasarım gerekçesi" `docs/` altına.
2. Karar fonksiyonlarında docstring yok: `FCSManager.update()`,
   `TargetSelector.select_best_target()`, `LockManager.update()`, `los_tracker`
   metotları. Projenin en kritik üç kararı belgelenmemiş; buna karşılık önemsiz
   `get_box_label()` docstring sahibi. Docstring önceliği ters.
3. Doküman-kod kayması (asıl sorun bu):
   - `p_controller` başlığı "hata büyüdükçe Kp'yi 1.0'dan 1.6'ya çıkaran kazanç
     planlaması" anlatıyor; 18:22'de yapılan düzenlemeyle `far_gain` /
     `far_error_deg` ve `_scheduled_gain_for` koddan kaldırıldı, kontrolcü artık
     sabit kazançlı. Modül başlığı ve `AGENTS.md` (FAZ 10, dosya haritası, yeni
     sohbet maddesi) hâlâ kazanç planlamasını anlatıyor. (Bu, inceleme sırasında
     oluşan en taze kayma.)
   - `kalman_filter` docstring'i 150 px tekil-sıçrama kapısını ve `reacquire_frames`
     davranışını anlatıyor; çalışma ağacındaki kodda kapı yok, yalnız
     `_outlier_frames` alanı kalmış. README ve mimari SVG de kapıyı "var" gösteriyor.
   - `rl_controller` docstring'i "hız gözlemi runtime'da sıfırlanır" diyor; sınıf
     alanları `uses_velocity_obs = True`, `prefers_smooth_velocity = True`. Kod,
     Kalman hızını politikaya veriyor.
   - `config.FCSState` docstring'i FIRING'i "firmware revizyonunda rezerve" diyor;
     firmware bu baytı gerçek solenoid tetiği olarak kullanıyor.
   - Webots modüllerinin başlıkları hâlâ "v1.3"; depo v1.4.
   - `webots_main` başlığı "40.5 derece/s limit" diyor; güncel sınır 1.50 derece /
     0.032 s = 46.9 derece/s.
   - `fcs_manager` başlığı "dairesel tarama" diyor; `patrol_generator` figure-8
     (Lissajous) üretiyor.
   - `turret_hud` başlığı kilidi yeşil, atışı kırmızı diye tarif ediyor; kodda kilit
     ve atış ikisi de kırmızı, arama sarı.
   - `main.py` atış sonrası sabit "ENGAGEMENT SUCCESSFUL" yazıyor; üretimde isabet
     ölçen bir sensör yok. Webots tarafı gerçek HIT/MISS gösteriyor. Aynı kelime
     iki uygulamada farklı anlama geliyor; üretimde "ATIŞ TAMAMLANDI" gibi nötr bir
     metin daha dürüst olur.
   Bunlar "yanlış kod" değil, "yanlış açıklama". Jüride en riskli tür: kodun
   yapmadığı bir şeyi yapıyor gibi anlatmak.
4. Dil ve karakter tutarsızlığı: `patrol_generator` docstring ve yorumları İngilizce,
   `webots_tuning` ve birkaç analiz betiği İngilizce; geri kalan Türkçe. Türkçe
   karakterler bazı dosyalarda var (`fcs_manager` "Açısal", `rl_controller` "Ölçülen",
   `webots_hardware_interface` "dinamiği"), çoğunda yok. Tek bir kural seçilmeli
   (ör. "kod ve docstring ASCII Türkçe, kullanıcı metinleri serbest").
5. Numaralı yorumlar (`# 1.`, `# 2.` ...) faydalı; ancak bazıları kodun tekrarı
   (`# Solenoid enerjisini kes, yay geri ceksin` gibi satırlar fonksiyon adıyla
   aynı bilgiyi veriyor). Yorum, "ne" değil "neden" yazmalı; proje genelde bunu
   doğru yapıyor, birkaç yerde bozuyor.

### 5.3 Somut öneri

Her modül için üç katman:

1. İlk 5 satır: sözleşme (birimler, varsayılanlar, yan etkiler, thread/güvenlik notu).
2. Sonraki 10-15 satır: tasarım gerekçesi (şu anki hâli).
3. Sayısal doğrulama ve eski sürüm karşılaştırmaları: `docs/` altına, bağlantıyla.

Böylece "neden" bilgisi kaybolmaz, "ne" bilgisi ilk ekranda kalır.

---

## 6. SSOT Boşlukları (Sabitlerin Yeri)

`config.py` iyi bir tek kaynak ama kapsamı tam değil. Kod içinde kalan eşikler:

| Modül | Kod içindeki sabit | Öneri |
|---|---|---|
| `target_selector` | `min_conf 0.28`, `track_min_conf 0.16`, `min_area 400`, `max_area 400000`, `min_dim 18`, aspect `0.35-2.50`, `spatial_gate_px 250` | `CONFIG.selector` |
| `lock_manager` | dinamik eşik `1.60 / 2.20 / 3.00`, hız kırılımları `15 / 45 px/s` | `CONFIG.lock` |
| `patrol_generator` | `45 / 16 derece`, `8.0 s`, `20 derece/s`, `1.5 s`, `1.5 / 0.95 derece` | `CONFIG.patrol` |
| `rl_controller` | `1 / 0.03333`, `d_err` ölçeği `0.05`, EMA `0.40 / 0.60`, varsayılan mesafe `2.0 m` | `CONFIG.rl` / `CONFIG.control` |
| `turret_hud` | renkler, panel ölçüleri, eşikler (`>= 70`, `>= 40`, `> 1000`) | `CONFIG.hud` veya `theme.py` |
| `fcs_comm` | `1e-9`, `32000`, `4096` tampon sınırı | modül sabiti olarak adlandırılmış |

`tests/test_config_consistency.py` yalnız "her CONFIG alanı okunuyor mu" sorusunu
soruyor; tersini (kodda sabit kalmış eşik) yakalamıyor. Bir "yasak sabit" testi
(ör. `target_selector.py` içinde çıplak `0.28` geçmemeli) bu boşluğu kapatır.

---

## 7. STM32 Tarafı

### 7.1 İyi olan

- `main.c` yorum oranı %32; her güvenlik kararının gerekçesi (300 ms heartbeat,
  2000 ms park, IWDG, E-stop onayı) yazılı. Bu, Python tarafından daha iyi.
- Fonksiyon isimleri ve modül sınırları net: `Luna_*`, `FCS_Comm_*`, `Servo_*`,
  `Launcher_*`.
- Kesme içinde yapılan iş minimal; ağır iş ana döngüde.

### 7.2 Bulgular

1. Modül başlığı eksik: `servo_driver.c`, `fcs_comm.c`, `luna_read.c`,
   `flywheel_solenoid.c` doğrudan `#include` ile başlıyor. Yalnız `main.c` ve
   `flywheel_solenoid.h` modül açıklaması taşıyor. Python'daki başlık disiplini
   firmware'e taşınmamış.
2. CubeMX `USER CODE` blokları yok: `main.c` yalnız `Header` işaretlerini içeriyor.
   CubeMX'ten yeniden üretim, özel döngüyü ve güvenlik kodunu silebilir. Kod
   `USER CODE BEGIN 2` / `END 2` bloklarına alınmalı veya `.ioc` dışı bir
   `app_main.c` dosyasına taşınmalı.
3. `Error_Handler()` içinde pinler sabit yazılmış: `GPIOB, GPIO_PIN_0 | GPIO_PIN_1`.
   `FLYWHEEL_PIN` / `SOLENOID_PIN` makroları değişirse acil durum sıfırlaması
   yanlış pini çeker ve hata sessiz kalır. Makrolar kullanılmalı.
4. Ölü API: `Servo_IsAtCenter()` hiç çağrılmıyor.
5. Adlandırma tuzağı: firmware'de `FCS_HEADER_RX = 0xAA` "PC'den gelen", Python'da
   `HEADER_TX = 0xAA` "PC'den giden" demek. Aynı isim, iki tarafta ters anlam.
   `PC_TO_STM_HEADER` / `STM_TO_PC_HEADER` gibi yön belirten adlar daha güvenli.
6. Protokol sözleşmesi tek yerde değil: 7 bayt TX, 10 bayt RX, 9 bayt LiDAR
   çerçevesi, `0x59` başlığı, `*10` ölçeği ve `-90` ofseti üç dosyaya dağılmış.
   Tek bir `protocol.h` (ve Python'da `protocol.py`) bu sözleşmeyi taşımalı.
7. İki ayrı elle yazılmış bayt ayrıştırıcı var (`luna_read.c`, `fcs_comm.c`);
   yeniden senkronizasyon stratejileri farklı (biri indeksi sıfırlıyor, diğeri
   kayan pencere gibi davranıyor). Ortak küçük bir çerçeve çözücü, iki kopyayı
   ve davranış farkını kaldırır.
8. `servo_driver.c` TIM1'i "opsiyonel yedek" diye yorumluyor ama koşulsuz sürüyor
   (`Servo_SetPan` her zaman TIM1 kanalını da yazıyor). Ya yorum ya kod düzeltilmeli.
9. `ENABLE_PHYSICAL_FIRE` koşulu beş ayrı yerde `#if` olarak duruyor. Tek bir
   `handle_fire_edge(state)` fonksiyonuna alınırsa hem okunur hem test edilebilir.
10. `HAL_UART_Transmit(..., 10)` bloklayan çağrı ana döngüde; 10 bayt 115200'de
    yaklaşık 0.9 ms. Ölçülen döngü 10 ms olduğu için kabul edilebilir, ama
    `HAL_Delay(10)` ile birleşince gerçek periyot 10.9 ms olur ve hiçbir yerde
    yazmıyor. Yorumdaki "~90-100 Hz" ifadesi bu yüzden iyimser.
11. `B1_Pin` EXTI olarak kuruluyor ama `HAL_GPIO_EXTI_Callback` yok; `TMS/TCK/SWO`
    makroları da kullanılmıyor. CubeMX artıkları temizlenmeli.
12. Statik analiz/biçimlendirme yok: `.clang-format`, `cppcheck` veya derleme
    uyarı bayrağı (`-Wall -Wextra -Wswitch-default`) yapılandırılmamış.
    `Launcher_Update` içindeki `switch` `default` içermiyor; `-Wswitch-default`
    bunu yakalar.

---

## 8. Test ve Analiz Katmanı

- Testler: Güncel 35 birim, entegrasyon ve SSOT tutarlılık testinin tamamı başarıyla geçmektedir (%100 yeşil, ~0.9 s).
- Testler bazı ölü sözleşmeleri sabitliyor: `test_webots_tuning` ölü `kp_pan`
  alanını, `test_p_controller` daima sıfır dönen `compute_velocity_ff`'i doğruluyor.
  Ölü kod kaldırılırken bu testler de güncellenmeli; aksi hâlde "test var" görüntüsü
  yanıltıcı olur.
- `test_config_consistency` iyi bir fikir ama tek yönlü (bkz. Bölüm 6).
- `analysis/` altında 20 betik var; ikisinde modül docstring'i yok
  (`deep_analyze_last_log`, `generate_architecture_svg`), altısında kullanılmayan
  ithalat var. Beş betik aynı işi (P kazancı değerlendirme) farklı
  arayüzlerle yapıyor: `p_gain_sweep`, `p_gain_oscillation`, `p_controller_benchmark`,
  `p_controller_validation`, `p_profile_sweep`. README bunlardan yalnız üçünü
  belgeliyor. En az iki betik birleştirilmeli veya "deneysel, üretim dışı" olarak
  işaretlenmeli.
- Webots modüllerinin başlıkları hâlâ v1.3 ve bir kısmı İngilizce; üretim paketiyle
  aynı sürüm dilini kullanmıyor.

---

## 9. Önceliklendirilmiş Plan

| Öncelik | İş | Süre | Kazanç |
|---|---|---|---|
| P0 | Doküman-kod kaymasını düzelt: Kalman kapısı, RL hız gözlemi, FIRING "rezerve", v1.3 etiketleri, 40.5 derece/s, "dairesel" devriye, HUD renk tarifi | 1 saat | Jüride yanlış beyan riski kalkar |
| P0 | Ölü alanları kaldır veya bağla: `kp_pan/kp_tilt`, `compute_velocity_ff`, `omega_*`, `filtered_*`, `_outlier_frames`, `current_sigma_a`, `temp`, `rl_weight`, `log_event`, erken `fx/fy` | 1-2 saat | Kod ile telemetri arasındaki yalan kalkar |
| P0 | `**kwargs` yutucularını kaldır; çağrıları derleme anında kır | 30 dakika | Kaldırılan alanların sessizce yutulması biter |
| P1 | `engagement.py` + `presenter.py` + `keymap.py` çıkar; `main.py` ve `webots_main.py` incelt | 1 gün | ~250 satır tekrar yok olur, iki uygulama tek çekirdek kullanır |
| P1 | `TelemetryRow` dataclass'ı; 44 sütunu tek kaynağa bağla | 2-3 saat | Sütun kayması riski biter |
| P2 | ruff + biçimlendirici + CI lint adımı; `pyright` `typeCheckingMode` en az `basic` | 2 saat | Ölü ithalat/parametre birikmesi durur |
| P2 | Firmware: modül başlıkları, `protocol.h`, `USER CODE` blokları, `Error_Handler` makroları, `Servo_IsAtCenter` temizliği | 1 gün | CubeMX yeniden üretim riski ve pin tuzağı kalkar |
| P3 | SSOT taşıma (selector/lock/patrol/hud) ve `analysis/` konsolidasyonu | 1-2 gün | Tek kaynak iddiası tam olur |

---

## 10. Tek Paragraflık Cevap

Kod tabanı okunabilir: modül sınırları doğru, isimlendirme tutarlı, tasarım
gerekçeleri yazılı ve jüri savunması için değerli. Ancak "sade ve bakımı kolay"
demek için üç şey eksik: (1) aynı uygulama döngüsü iki kez yazılmış ve en uzun
fonksiyonlar 150-366 satıra çıkmış; (2) yazılan ama okunmayan parametreler ve
daima sabit kalan telemetri sütunları var; (3) dokümanlar kodun bir adım gerisinde.
Bunlar davranışı bozmayan, mekanik düzeltmelerdir; önce doküman-kod uyumu ve ölü
alan temizliği yapılırsa kod hem tez hem saha bakımı için belirgin şekilde
anlaşılır hâle gelir. STM32 tarafında uzunluk sorunu yok; orada eksik olan, aynı
başlık ve protokol disiplininin firmware'e taşınması ve CubeMX yeniden üretim
güvenliğidir.
