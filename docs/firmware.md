# STM32F401RE Gömülü Donanım ve Firmware Kılavuzu

Bu doküman; Turret FCS projesinin gömülü denetleyici katmanı olan **STM32F401RE Nucleo-64** kartının firmware mimarisini, STM32CubeIDE kurulumunu, derleme adımlarını, pin bağlantılarını ve çok katmanlı güvenlik sistemini açıklamaktadır.

---

## 🛠️ Donanım Özellikleri

- **Mikrodenetleyici:** STM32F401RET6 (ARM Cortex-M4 @ 84 MHz, FPU destekli)
- **Hafıza:** 512 KB Flash, 96 KB SRAM
- **İletişim:** 2x USART (PC ile 115200 Baud ikili protokol, TF-Luna LiDAR ile 115200 Baud UART)
- **Zamanlayıcılar (Timers):** 50 Hz Servo PWM (TIM3), Volan PWM (TIM2), Hassas Darbe Zamanlayıcısı
- **Hata Güvenliği:** Donanımsal Bağımsız Bekçi Köpeği (IWDG) ve 300 ms Yazılımsal Heartbeat Watchdog

---

## 📌 Pin Bağlantı Tablosu (Pinout)

| Fonksiyon | STM32 Pini | Donanım Modülü | Açıklama |
| :--- | :--- | :--- | :--- |
| **Pan Servo Sinyali** | `PB4` (TIM3_CH1) | MG996R / DS3218 | 50 Hz PWM, 0.5 - 2.5 ms darbe genişliği (15° - 165°) |
| **Tilt Servo Sinyali** | `PB5` (TIM3_CH2) | MG996R / DS3218 | 50 Hz PWM, 0.5 - 2.5 ms darbe genişliği (-15° - +25°) |
| **Volan Sürücü (PWM)** | `PA0` (TIM2_CH1) | IRLZ44N N-MOSFET | Çift FF-180 fırçalı motor devir kontrolü |
| **Solenoid Tetikleme** | `PA1` (GPIO Out) | JF-0530B İtici | Optokuplör yalıtımlı 80 ms donanımsal darbe çıkışı |
| **TF-Luna LiDAR RX** | `PA10` (USART1_RX)| Benewake TF-Luna | 100 Hz ToF mesafe telemetrisi alımı |
| **TF-Luna LiDAR TX** | `PA9` (USART1_TX) | Benewake TF-Luna | LiDAR konfigürasyon hattı |
| **PC İletişim TX** | `PA2` (USART2_TX) | ST-Link VCP | PC'ye 10-baytlık durum ve LiDAR telemetrisi iletimi |
| **PC İletişim RX** | `PA3` (USART2_RX) | ST-Link VCP | PC'den 6-baytlık ikili hareket ve durum paketi alımı |
| **Kullanıcı Butonu** | `PC13` (EXTI13) | Mavi Buton (B1) | Manuel acil durdurma (E-Stop) kesmesi |
| **Durum LED'i** | `PA5` (GPIO Out) | Yeşil LED (LD2) | Sistem hazır/çalışma kalp atışı göstergesi |

---

## 💻 STM32CubeIDE ile Derleme ve Yükleme

1. **Projeyi İçe Aktarma (Import):**
   - STM32CubeIDE'yi açın: `File -> Import... -> General -> Existing Projects into Workspace`.
   - `Root Directory` olarak `stm32/turret_main/` dizinini seçin ve `Finish` butonuna tıklayın.

2. **CubeMX Yapılandırması (.ioc):**
   - `turret_main.ioc` dosyası mikrodenetleyicinin saat frekansını (84 MHz PLL), zamanlayıcıları ve UART çevre birimlerini barındırır.
   - Değişiklik yapıldığında `Device Configuration Tool -> Code Generation` ile C kodları yeniden üretilebilir.

3. **Derleme (Build):**
   - `Project -> Build Project` (veya `Ctrl+B`) seçeneği ile projeyi derleyin.
   - GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`) kullanarak `Debug/turret_main.elf` dosyasını üretecektir.

4. **Karta Yükleme (Flash):**
   - STM32 Nucleo kartını USB kablosuyla bilgisayara bağlayın.
   - `Run -> Run As -> STM32 Cortex-M C/C++ Application` seçin.
   - Dahili ST-Link programlayıcı kodu mikrodenetleyiciye flaşlayacak ve sistemi başlatacaktır.

---

## 🛡️ Gömülü Güvenlik Katmanları (Fail-Safe Protections)

1. **300 ms Heartbeat Watchdog:**
   - PC ana döngüsü (`main.py`) her karede STM32'ye komut paketi gönderir.
   - Seri iletişim koparsa veya PC yazılımı 300 ms boyunca yeni paket iletmezse STM32 acil duruma (`ESTOP`) geçer: volan motorları anında durdurulur ve taret güvenli mekanik merkezine (90°, 90°) kilitlenir.

2. **800 ms Donanımsal Bağımsız Bekçi Köpeği (IWDG):**
   - STM32 dahili 32 kHz LSI osilatörüyle çalışan IWDG bekçi köpeği aktiftir.
   - Firmware kilitlenirse veya ana döngü 800 ms aşarsa kart donanımsal reset atarak sistemi güvenli duruma döndürür.

3. **80 ms Kenar Tetiklemeli Solenoid Darbe Protokolü:**
   - Solenoid bobininin aşırı ısınıp yanmasını ve çift dart atılmasını önlemek için tetikleme donanımsal olarak 80 ms darbe ile sınırlandırılmıştır.

4. **Fiziksel Ateşleme İzolasyonu (Etik Güvenlik):**
   - Test ve simülasyon ortamında istenmeyen atışları engellemek için `servo_driver.h` içinde:
     ```c
     #define ENABLE_PHYSICAL_FIRE 0
     ```
   - Bu bayrak `0` olduğu sürece solenoid fiziksel olarak asla tetiklenmez.
