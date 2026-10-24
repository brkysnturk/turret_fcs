# Tekrar Üretilebilirlik Kılavuzu (Reproducibility Guide)

Bu doküman; Turret FCS projesinin testlerinin, pekiştirmeli öğrenme model eğitiminin (RL training), simülasyon benchmark'larının ve akademik analiz figürlerinin bağımsız araştırmacılar tarafından sıfırdan nasıl tekrar üretilebileceğini adım adım açıklamaktadır.

---

## 1. Ortam Kurulumu ve Bağımlılıklar

Proje Python 3.10, 3.11 ve 3.12 sürümleri ile test edilmiştir.

```bash
# Depoyu klonlayın
git clone https://github.com/brkysnturk/turret_fcs.git
cd turret_fcs

# Sanal ortam oluşturun ve aktif edin
python -m venv venv
venv\Scripts\activate   # Windows
# source venv/bin/activate # Linux / macOS

# Projeyi ve test bağımlılıklarını kurun
pip install -e .[test]
```

---

## 2. Otomatik Test Paketinin Koşulması

Donanıma (kamera veya STM32) ihtiyaç duymadan çalışan 29 birim, entegrasyon ve SSOT tutarlılık testi:

```bash
pytest -v
```

Bu komut; Tip-1 P kontrolcünün adım sınırlarını, CV Kalman filtresinin kovaryans kararlılığını, 2B Gauss kilit olasılığını, menzil kapılama histerezisini ve `CONFIG` parametrelerinin tüm kod tabanında tutarlı şekilde okunduğunu doğrular.

---

## 3. Model Ağırlıklarının İndirilmesi

Projede kullanılan derin öğrenme modelleri GitHub Releases üzerinden dağıtılır:

```bash
# Model ağırlıklarını (best.pt ve yolov8n.pt) otomatik indirmek için:
python -m turret_fcs.weights
```

Dosyalar `weights/` dizini altına kaydedilir.

---

## 4. PPO RL Modelinin Yeniden Eğitilmesi

11-boyutlu Gymnasium taret simülasyon ortamında (`TurretEnv`) sıfırdan model eğitmek için:

```bash
# 250.000 adımlık PPO eğitimini başlatın (Sabit tohum / seed: 42)
python turret_fcs/rl_sim/train_rl.py
```

Eğitim parametreleri:
- **Algoritma:** PPO (MlpPolicy: [64, 64])
- **Öğrenme Oranı (Learning Rate):** 3e-4
- **Gamma:** 0.99
- **GAE Lambda:** 0.95
- **Clip Range:** 0.2
- **Adım Sayısı:** 250.000 adım

Eğitim tamamlandığında üretilen model `turret_fcs/rl_sim/models/turret_ppo_v14.zip` olarak kaydedilir. Bu modeli saf NumPy MLP (`.npz`) olarak dışa aktarmak için:

```bash
python analysis/export_rl_policy.py
```

---

## 5. P Kontrolcü vs PPO RL Benchmark Karşılaştırması

İki kontrolcünün aynı dinamik hedef yörüngesi üzerindeki başarımını karşılaştırmak ve karşılaştırma grafiğini üretmek için:

```bash
python turret_fcs/rl_sim/compare_p_vs_rl.py
```

Bu betik `analysis/figures/p_vs_rl_comparison.png` görselini üretir ve terminale pan/tilt RMS hatalarını, kilit oranlarını ve iyileşme yüzdesini basar.

---

## 6. Akademik Analiz Figürlerinin Üretilmesi

Tüm analitik modelleri, Kalman kestirim doğrulamalarını ve faz uzayı grafiklerini üretmek için:

```bash
# 1. 8 Panelli Dinamik Kararlılık Figürleri (fig1 - fig8)
python analysis/plot_tuned_system.py

# 2. Kalman Filtresi Hız ve Zemin Gerçeği Karşılaştırma Figürü
python analysis/generate_kalman_plots.py

# 3. P vs RL Webots Benchmark Görseli
python analysis/generate_benchmark_plots.py

# 4. Sistem Mimarisi SVG Şeması
python analysis/generate_architecture_svg.py
```
