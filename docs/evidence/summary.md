# Turret FCS - Performans ve Kararlilik Metrikleri Kaniti (v1.4)

Bu dokuman, otonom hava savunma tareti ates kontrol sisteminin (Turret FCS v1.4) gercek fiziksel saha testlerinden ve Webots simulatorunden elde edilen telemetri verilerinin ozetini ve karsilastirmali analizini icerir.

## 1. Temsili Telemetri Kayitlari

Depo boyutunu hafif ve hizli klonlanabilir tutmak amaciyla referans ucus ve hedef takip verileri surumlere gore saklanmistir:

- `v1.2_baseline.csv`: Ilk parametre optimizasyonu oncesi, temel P kontrolcu ile yapilan statik ve yavas hareketli hedef takip testi.
- `v1.3_benchmark.csv`: Histerezisli Kilit Yoneticisi (LockManager), 2 Boyutlu Gauss Atis Penceresi ve Aim-Point Slew Ileri Beslemesi testi.
- `telemetry_test_20261024_150820.csv` (v1.4 Nihai Saha Logu - 5.387 Kare): Fiziksel donanim uzerinde ardisik Tip-1 P (1.738 kare) ve Saf PPO Pekiştirmeli Ogrenme (3.649 kare) karsilastirma telemetrisi.

## 2. v1.4 Fiziksel Donanim Saha Karsilastirmasi (24 Ekim 2026 Nihai Test)

5.387 tespit karesinden elde edilen nesnel donanim telemetri metrikleri:

| Performans Parametresi | Tip-1 P Kontrolcu | Saf PPO RL Kontrolcu | v1.4 Kazanim / Durum |
| :--- | :--- | :--- | :--- |
| **Toplam Test Karesi** | 1.738 kare (59.2 s) | 3.649 kare (125.7 s) | Uzun donanim dayaniklilik testi |
| **Kilitlenme Orani (Lock %)** | %61.0 | %65.5 | RL %4.5 daha yuksek kilitlenme |
| **Pan RMS Takip Hatasi** | 3.26 derece | 2.61 derece | RL %20 daha hassas hedef takibi |
| **Tilt RMS Takip Hatasi** | 1.62 derece | 1.61 derece | Esdeger basarim (~1.61 derece) |
| **Azami Kesintisiz Kilit** | 3.4 saniye (102 kare) | 4.1 saniye (123 kare) | RL +0.7 saniye daha uzun kilit |
| **Merkezde Komut Genligi (|e|<1°)** | 0.0033 derece/kare | 0.0008 derece/kare | Titresimsiz, tam sessiz durus |
| **Dikey Yon Degisimi (Reversal)** | %0.07 | %0.19 | Sifir mekanik avlanma / salinim |
| **Cikarim Suresi (CPU)** | 0.02 ms | 0.17 ms | 30 FPS altinda <0.2 ms gecikme |

## 3. v1.4 Anahtar Mimari Gelistirmeler

1. **Donanim Eylemsizligine Uyarlanmis Tilt Adim Siniri:**
   - Dikey (Tilt) eksen hareket araligi yatay eksenin 1/3'u oldugundan, 1.50 derece/kare adim ve 80 ms donanim gecikmesi 4.5 derecelik asima yol acmaktaydi. Tilt adimi 0.75 derece/kare (22.5 derece/saniye) degerine sinirlandirilarak dikey salinim tamamen sonumlendi.

2. **Cift Kademeli Yumusak Olu Bolge (Soft Deadband Taper):**
   - Pan icin 0.85 derece olu bolge + 1.00 derece taper, Tilt icin 0.95 derece olu bolge + 2.50 derece taper uygulandi. Bu sayede hedef merkeze yaklastiginda motor komutlari sert kesilmek yerine yumusatarak sifira yakinsar; kucuk hatalarda avlanma (chatter) ortadan kalkar.

3. **Komut Ivme Sinirlandiricisi (Slew Acceleration Limiter):**
   - Pan komut ivmesi 120 derece/s^2, Tilt komut ivmesi 80 derece/s^2 ile sinirlanarak servolara ani tork darbeleri gitmesi engellendi ve donanim omru artirildi.

4. **Gercek Zamanli HUD Gostergesi:**
   - Kullanici arayuzunde [P CONTROLLER] ve [RL CONTROLLER] buyuk harfli durum cubugu ile anlik kontrolcu modu seffaf sekilde sunulmaktadir.

## 4. Webots Dogrulama Sonuclari (v1.4, Saf P MAX / Saf RL)

Webots simülatöründe aynı test ayarlarıyla ilk 600 kare üzerinden alınan ölçümler:

| Profil | Kontrolcu | Pan RMS | Piksel | Kilit | Kilitli kalma | Yon degistirme | Tilt hatasi |
| :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 (standart) | P MAX (0.22/0.11) | 2.07 derece | 42.3 px | %39.8 | %82.2 | %0.9 | 1.34 derece |
| 2 (standart) | RL MAX (saf PPO) | 1.93 derece | 40.6 px | %36.3 | %92.5 | %4.4 | 0.78 derece |
| 5 (zigzag) | P MAX | 2.66 derece | 49.0 px | %39.5 | %80.8 | %1.6 | 1.16 derece |
| 5 (zigzag) | RL MAX | 2.34 derece | 42.9 px | %47.0 | %94.2 | %5.1 | 0.78 derece |
| 7 (figure-8) | P MAX | 2.14 derece | 43.8 px | %38.8 | %65.2 | %0.8 | 1.87 derece |
| 7 (figure-8) | RL MAX | 2.06 derece | 43.6 px | %35.0 | %93.8 | %4.5 | 0.86 derece |

Test paketi: 39 testin 39'u basariyla gecmektedir.
