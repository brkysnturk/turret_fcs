# STM32F401RE Gömülü Donanım ve Firmware Kılavuzu

Bu doküman; Turret FCS projesinin gömülü denetleyici katmanı olan **STM32F401RE Nucleo-64** kartının firmware mimarisini, STM32CubeIDE kurulumunu, derleme adımlarını, doğrulanmış donanımsal pin bağlantılarını ve çok katmanlı güvenlik sistemini açıklamaktadır.

---

## 🛠️ Donanım Özellikleri

- **Mikrodenetleyici:** STM32F401RET6 (ARM Cortex-M4 @ 84 MHz, FPU destekli)
- **Hafıza:** 512 KB Flash, 96 KB SRAM
- **İletişim:** 2x USART (PC ile 115200 Baud ikili protokol, TF-Luna LiDAR ile 115200 Baud UART)
- **Zamanlayıcılar (Timers):** 50 Hz Servo PWM (TIM3_CH1, TIM3_CH2)
- **Hata Güvenliği:** Donanımsal Bağımsız Bekçi Köpeği (IWDG - 800 ms) ve Kademeli PC İletişim Bekçisi (300 ms fırlatıcı E-stop + 2000 ms servo merkezleme)

---

## 📌 Pin Bağlantı Tablosu (Pinout)

| Fonksiyon | STM32 Pini | Donanım Modülü | Açıklama |
| :--- | :--- | :--- | :--- |
| **Pan Servo Sinyali** | `PB4` (TIM3_CH1) | MG996R / DS3218 | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 15.0° - 165.0° (±75° hareket alanı) |
| **Tilt Servo Sinyali** | `PB5` (TIM3_CH2) | MG996R / DS3218 | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 65.0° - 115.0° (±25° hareket alanı) |
| **Volan Sürücü (MOSFET)** | `PB0` (GPIO Output) | IRLZ44N N-MOSFET | Çift FF-180 motor açma/kapama kontrolü (LOCKED/FIRING durumunda pre-spin aktif) |
| **Solenoid Tetikleme** | `PB1` (GPIO Output) | JF-0530B İtici | Optokuplör yalıtımlı solenoid darbe çıkışı (yazılımsal 80 ms itme + 150 ms geri çekilme) |
| **TF-Luna LiDAR RX** | `PA10` (USART1_RX)| Benewake TF-Luna | 100 Hz ToF mesafe telemetrisi alımı (115200 Baud UART) |
| **TF-Luna LiDAR TX** | `PA9` (USART1_TX) | Benewake TF-Luna | LiDAR konfigürasyon ve sorgulama hattı |
| **PC İletişim TX** | `PA2` (USART2_TX) | ST-Link VCP | PC'ye 10-baytlık durum ve LiDAR telemetrisi iletimi (`0x55` header) |
| **PC İletişim RX** | `PA3` (USART2_RX) | ST-Link VCP | PC'den 7-baytlık ikili komut paketi alımı (`0xAA` header + Pan + Tilt + State + CS) |
| **Kullanıcı Butonu (B1)** | `PC13` (GPIO Input)| Mavi Buton (B1) | CubeMX'te EXTI hattı olarak tanımlı kullanıcı butonu |
| **Durum LED'i (LD2)** | `PA5` (GPIO Output)| Yeşil LED (LD2) | Sistem hazır / ana döngü aktivite göstergesi |

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
   - GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`) kullanarak `Debug/turret_main.elf` dosyasını sıfır uyarı ile üretecektir.

4. **Karta Yükleme (Flash):**
   - STM32 Nucleo kartını USB kablosuyla bilgisayara bağlayın.
   - `Run -> Run As -> STM32 Cortex-M C/C++ Application` seçin.
   - Dahili ST-Link programlayıcı kodu mikrodenetleyiciye flaşlayacak ve sistemi başlatacaktır.

---

## 🛡️ Gömülü Güvenlik Katmanları (Fail-Safe Protections)

1. **Kademeli PC Heartbeat Watchdog (`main.c`):**
   - **300 ms Aşımı (`FCS_CMD_TIMEOUT_MS`):** PC komut akışı kesilirse `Launcher_EmergencyStop()` derhal çağrılır; volan motorları enerjisiz bırakılır ve solenoid bırakılır.
   - **2000 ms Aşımı (`SERVO_PARK_TIMEOUT_MS`):** PC kalıcı olarak kaybolursa servoların son konumda sürekli stall akımı çekip aşırı ısınmasını önlemek için taret `Servo_HomeStep()` fonksiyonu ile yumuşak adımlarla güvenli mekanik merkezine (90.0°, 90.0°) çekilir.

2. **800 ms Donanımsal Bağımsız Bekçi Köpeği (IWDG):**
   - STM32 dahili 32 kHz LSI osilatörü ve `/64` bölücüsü ile çalışan bağımsız IWDG bekçi köpeği aktiftir.
   - Firmware kilitlenirse veya ana döngü 800 ms aşarsa kart donanımsal reset atarak sistemi güvenli duruma döndürür.

3. **80 ms Yazılımsal Solenoid Durum Makinesi (`flywheel_solenoid.c`):**
   - Solenoid bobininin aşırı ısınıp yanmasını ve çift dart atılmasını önlemek için tetikleme `HAL_GetTick()` tabanlı durum makinesiyle (`PUSHING` 80 ms, `RELOADING` 150 ms) sınırlandırılmıştır.

4. **Fiziksel Ateşleme İzolasyonu (Etik Güvenlik):**
   - Test ve simülasyon ortamında istenmeyen atışları engellemek için `stm32/turret_main/Core/Src/main.c` içinde:
     ```c
     #ifndef ENABLE_PHYSICAL_FIRE
     #define ENABLE_PHYSICAL_FIRE 0
     #endif
     ```
   - Bu makro `0` olduğu sürece solenoid fiziksel olarak asla enerjilenmez; yalnızca simülasyon ve yazılımsal durum güncellenir.
