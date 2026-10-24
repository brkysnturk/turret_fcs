# Analysis Tools & Verification Suite (Analiz ve Doğrulama Araçları)

Bu dizin; Turret FCS projesinin kontrol dinamiklerini, sistem tanıma verilerini, Kalman filtresi kestirim doğruluğunu, Webots simülasyon ve gerçek donanım telemetri loglarını incelemek için geliştirilen akademik analiz betiklerini içerir.

Tüm grafikler `analysis/figures/` dizinine 300 DPI çözünürlükte kaydedilir.

---

## 📊 Betik ve Çıktı Haritası

| Kategori | Betik Dosyası | Açıklama & Ürettiği Çıktı | Çalıştırma Komutu |
| :--- | :--- | :--- | :--- |
| **Sistem Tanıma** | `plot_tuned_system.py` | 8 panelli analitik model, basamak yanıtı, faz portresi ve Bode kararlılık analizi (`fig1`–`fig8`). | `python analysis/plot_tuned_system.py` |
| **Sistem Tanıma** | `system_identification.py` | Taret motor gecikmesi, zaman sabiti (tau) ve transfer fonksiyonu kestirimi. | `python analysis/system_identification.py` |
| **P Kontrolcü** | `p_gain_sweep.py` | Webots ortamında farklı Kp kazançlarının taranması ve yön değiştirme doyum analizi. | `python analysis/p_gain_sweep.py` |
| **P Kontrolcü** | `p_profile_sweep.py` | 6 farklı hedef uçuş profiline karşı P kazanç matrisi doğrulaması. | `python analysis/p_profile_sweep.py` |
| **P Kontrolcü** | `p_controller_benchmark.py` | Gerçek donanım ve simülasyon adım yanıtı karşılaştırması. | `python analysis/p_controller_benchmark.py` |
| **Kalman** | `generate_kalman_plots.py` | CV Kalman filtresi hız kestirimi ile yer gerçeği (ground truth) karşılaştırma grafiği. | `python analysis/generate_kalman_plots.py` |
| **Kalman** | `kalman_validation.py` | Kovaryans yakınsaması ve inovasyon sıçrama kapısı doğrulaması. | `python analysis/kalman_validation.py` |
| **Optik** | `calibrate_camera_fov.py` | Logitech C270 kamera optik FOV açılarının piksel ölçümleriyle kalibrasyonu. | `python analysis/calibrate_camera_fov.py` |
| **RL Analizi** | `generate_benchmark_plots.py`| Saf PPO RL ile Tip-1 P kontrolcünün Webots profil karşılaştırma görseli (`p_vs_rl_final_benchmark.png`). | `python analysis/generate_benchmark_plots.py` |
| **RL Analizi** | `export_rl_policy.py` | PyTorch / SB3 politikasını hafif 11-64-64-2 NumPy MLP (`.npz`) formatına aktarır. | `python analysis/export_rl_policy.py` |
| **RL Analizi** | `rl_diagnose.py` | RL politika durum dağılımlarını ve eylem çıkış sınırlarını inceler. | `python analysis/rl_diagnose.py` |
| **RL Analizi** | `rl_webots_eval.py` | Webots simülatöründe RL kapalı çevrim başarımını doğrular. | `python analysis/rl_webots_eval.py` |
| **Telemetri** | `inspect_logs.py` | En son telemetri CSV kütüğünü otomatik okur, kilit ve açı hatalarını özetler. | `python analysis/inspect_logs.py` |
| **Telemetri** | `deep_analyze_last_log.py` | Son donanım telemetri kaydının FFT spektral analizi ve titreşim tespiti. | `python analysis/deep_analyze_last_log.py` |
| **Telemetri** | `consolidate_telemetry_logs.py`| Parçalı telemetri dosyalarını tek bir özet veri setinde birleştirir. | `python analysis/consolidate_telemetry_logs.py` |
| **Telemetri** | `plot_patrol_log.py` | Lissajous devriye tarama yörüngesini ve servo açı adımlarını çizer. | `python analysis/plot_patrol_log.py` |
| **Mimari** | `generate_architecture_svg.py`| Sistem mimarisini temsil eden güncel SVG şemasını (`docs/assets/`) üretir. | `python analysis/generate_architecture_svg.py` |

---

## 🚀 Hızlı Başlangıç

Tüm temel akademik analiz grafiklerini tek seferde üretmek için:

```bash
# 1. 8 Panelli Dinamik Kararlılık Figürleri (fig1 - fig8)
python analysis/plot_tuned_system.py

# 2. Kalman Filtresi Hız ve Doğrulama Figürü
python analysis/generate_kalman_plots.py

# 3. P vs RL Benchmark Karşılaştırma Figürü
python analysis/generate_benchmark_plots.py

# 4. En Son Gerçek Donanım Kütüğünün İncelenmesi
python analysis/inspect_logs.py
```
