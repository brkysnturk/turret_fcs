# Model Weights (Model Agirliklari)

Bu dizin, Turret FCS sistemi tarafindan kullanilan yapay zeka ve bilgisayarli goru modellerini barindirir:

- **`best.pt`**: Ozel veri setiyle (hava hedefleri, mikro-IHA ve balonlar) ince ayar yapilmis YOLOv8 modeli.
- **`yolov8n.pt`**: Ultralytics YOLOv8 Nano temel nesne tespit modeli (fallback).

## Depolama ve Indirme Stratejisi

Git reposunun boyutunu kucuk tutmak ve klonlama suresini hizlandirmak icin buyuk ikili `.pt` dosyalari Git gecmisinde saklanmaz. Bu dosyalar GitHub Releases uzerinden dagitilir:

1. **Otomatik Indirme:** `turret_fcs` calistirildiginda dosya yerel diskte yoksa [turret_fcs/weights.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/weights.py) uzerinden otomatik olarak en son GitHub Release asset'inden indirilir.
2. **Komut Satirindan Indirme:**
   ```bash
   python -m turret_fcs.weights
   ```
3. **Manuel Indirme:** [GitHub Releases](https://github.com/brkysnturk/turret_fcs/releases) sayfasindaki en son surumden `best.pt` ve `yolov8n.pt` dosyalarini indirip bu `weights/` dizinine yerlestirebilirsiniz.
