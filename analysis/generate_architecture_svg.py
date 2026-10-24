import xml.etree.ElementTree as ET
import urllib.parse
import html

# Refined generator for docs/assets/turret_fcs_architecture.svg
# Sleek, high-contrast dark theme matching turret_fcs_hardware.svg and turret_fcs_flowchart.svg
# 1460x1050 layout with spacious corridors and zero badge/line collisions

WIDTH = 1460
HEIGHT = 1050

LAYERS = [
    (
        "layer_perception",
        "1. ALGILAMA & SENSÖR FÜZYONU KATMANI (Perception & Sensor Fusion)",
        20, 20, 1420, 220,
        "#141020", "#8B5CF6", "#C4B5FD"
    ),
    (
        "layer_fcs_core",
        "2. FCS ÇEKİRDEK & HESAPLAMA KATMANI (PC / Host - fcs_manager.py)",
        20, 260, 1420, 265,
        "#0F172A", "#64748B", "#CBD5E1"
    ),
    (
        "layer_control",
        "3. ÇİFT MODLU AYRIK KONTROL KATMANI (Tip-1 P Kontrolcü & Saf PPO RL Kontrolcü)",
        20, 545, 1420, 220,
        "#191608", "#D97706", "#FCD34D"
    ),
    (
        "layer_actuation",
        "4. GÖMÜLÜ FCS DENETLEYİCİ, EYLEME & TAKTİKSEL HUD KATMANI",
        20, 785, 1420, 240,
        "#061E1C", "#14B8A6", "#5EEAD4"
    )
]

CARDS = [
    # Layer 1 Cards (y=70, h=150)
    (
        "card_c270",
        "Logitech C270 HD Web Kamerası",
        "(1280x720 @ 30 FPS | Yatay FOV: 69°, Dikey FOV: 42°)",
        [
            "camera_thread.py: Bağımsız arka plan iş parçacığı (QoS & Sıfır Kuyruk Gecikmesi)",
            "Bayat Kare Koruması (Stale-Frame Guard): En güncel görüntü karesi güvencesi",
            "Gerçek zamanlı optik merkez dönüşümü ve piksel-açı eşlemesi"
        ],
        50, 70, 390, 150,
        "#1C162C", "#8B5CF6", "#FFFFFF", 10.5, "#D8B4FE", "#C084FC"
    ),
    (
        "card_yolo",
        "YOLOv8n & ByteTrack Hedef Tespit ve Takip",
        "(Ultralytics YOLOv8 Nano + Çift Aşamalı ByteTrack Kalman Eşleme)",
        [
            "İki Kademeli Güven Eşiği: İlk Arama conf >= 0.28, Kilitli Takip conf >= 0.16",
            "TargetSelector: Hedef ID kararlılığı, uzamsal kapılama & 5-kare histerezis",
            "Boyut filtreleme: 15-400 px bounding box sınırları ile sahte hedefleri eleme"
        ],
        510, 70, 460, 150,
        "#1C162C", "#8B5CF6", "#FFFFFF", 10.5, "#D8B4FE", "#C084FC"
    ),
    (
        "card_luna",
        "TF-Luna LiDAR ToF Mesafe Ölçer",
        "(Benewake 850nm ToF | 100 Hz UART Telemetrisi | 0.2 m - 8.0 m Menzil)",
        [
            "Doğrudan STM32 USART Bağlantısı (Mikrosaniye gecikmesiz donanım okuma)",
            "3-Kademeli Menzil Kapılama (Range Gating): Ani sıçrama, geometri ve kararlılık",
            "Hedef mesafesi teyidi, dinamik uçuş süresi ve balistik düşüş hesabı girdisi"
        ],
        1030, 70, 380, 150,
        "#0F1E36", "#3B82F6", "#FFFFFF", 10.5, "#93C5FD", "#93C5FD"
    ),

    # Layer 2 Cards (y=300, h=205)
    (
        "card_kalman",
        "CV Kalman Filtresi & Ego-Hareket Telafisi",
        "([x, y, vx, vy] Durum Kestirimi | kalman_filter.py)",
        [
            "Joseph Form Kovaryans Güncellemesi (Sayısal kararlılık & simetri)",
            "1. Derece Aktüatör Gözlemcisi: Taret kendi dönüşünün hedef hızına bindirdiği optik etkiyi çıkarır (Ego-Motion Compensation)",
            "150 px Aykırı Değer Kapılama (Outlier Rejection Gate)",
            "30-60 px/s Dinamik Hız Ölü Bölgesi (Sensör gürültüsü sönümleme)"
        ],
        50, 300, 410, 205,
        "#0A241A", "#10B981", "#FFFFFF", 10.2, "#6EE7B7", "#6EE7B7"
    ),
    (
        "card_lead",
        "Kinematik Önleme (Lead) & Balistik Hesaplama",
        "(Dinamik Mermi Uçuş Süresi & Yerçekimi Telafisi)",
        [
            "Uçuş Süresi: t_flight = mesafe / 10.0 m/s (LiDAR menzili ile senkronize)",
            "Kinematik Önleme: lead_x = vx * t_flight, lead_y = vy * t_flight",
            "Zero-Lag Aim-Point Feedforward: delta_slew = lead - prev_lead türevsel adımı ile rampa takip hatası sıfırlanır",
            "Balistik Yerçekimi Süper-Elevasyonu: y_super = 0.5 * g * t_flight^2"
        ],
        520, 300, 440, 205,
        "#0A241A", "#10B981", "#FFFFFF", 10.2, "#6EE7B7", "#6EE7B7"
    ),
    (
        "card_lock",
        "Dinamik Kilit Yöneticisi (LockManager)",
        "(2B Gauss Hit Probability & Durum Kilit Histerezisi)",
        [
            "2B Gauss (ncx2) Kademeli Vuruş Olasılığı: Bounding box ve hedef hızına göre dinamik tolerans penceresi",
            "10-Kare Kesintisiz Kararlılık Histerezisi (Titreşim kaynaklı sahte kilitlenmeleri önleme)",
            "Atış Sonrası Kilit Muafiyeti (Post-Fire Grace: 0.75 s / 22 kare boyunca taret rezonansına tolerans)",
            "STM32 Donanım Durum Bildirimi (0: Arama, 1: Takip/Kilit, 2: Ateşleme)"
        ],
        1020, 300, 390, 205,
        "#261A0C", "#F59E0B", "#FFFFFF", 10.2, "#FDE68A", "#FDE68A"
    ),

    # Layer 3 Cards (y=585, h=160)
    (
        "card_p_ctrl",
        "Tip-1 P Kontrolcü Modu [P CONTROLLER]",
        "(Yumuşak Ölü Bölgeli & Slew İvme Sınırlamalı Oransal Takip)",
        [
            "v1.4 Taban Kazançlar: Kp_pan = 0.22, Kp_tilt = 0.11 (Faz Marjı: 45.1°, Kazanç Marjı: 7.2 dB)",
            "Slew İvme Sınırları: 120°/s² Pan, 80°/s² Tilt ile mekanik rezonans ve dişli aşınması önlenir",
            "Yumuşak Ölü Bölge Taper: Pan 0.85° - 1.00°, Tilt 0.95° - 2.50° ile merkezde sıfır titreme (0.030°)",
            "Tek EMA Filtresi (alpha = 0.85) ile ölçüm gürültüsü yumuşatma"
        ],
        50, 585, 430, 160,
        "#0A241A", "#10B981", "#FFFFFF", 10, "#6EE7B7", "#6EE7B7"
    ),
    (
        "card_mode_sel",
        "Ayrık A/B Mod Değişimi ([M] Tuşu)",
        "(Deterministik Operatör Anahtarlaması)",
        [
            "Operatör kontrollü [M] tuşu ile bağımsız mod geçişi",
            "Otomatik hibrit anahtarlamanın limit cycle riskleri engellendi",
            "Mod geçişinde eylem geçmişi sıfırlama & pürüzsüz devir",
            "Güvenlik Limitleri: Pan ±90°, Tilt -15° .. +30° kısıtlaması"
        ],
        540, 585, 340, 160,
        "#261A0C", "#F59E0B", "#FFFFFF", 10.2, "#FDE68A", "#FDE68A"
    ),
    (
        "card_rl_ctrl",
        "Saf PPO RL Kontrolcü Modu [RL CONTROLLER]",
        "(11-Boyutlu Durum Uzayı | turret_ppo_v14_policy.npz NumPy MLP)",
        [
            "11-B Gözlem: [hata_x, hata_y, d_err_x, d_err_y, vx, vy, mesafe, eylem_t-1, eylem_t-2]",
            "80 ms Taşıma Gecikmesi Telafisi: 2-adımlı eylem geçmişi Markovian süreci garanti eder",
            "0.75° Tilt Adımı (22.5°/s) & Yön Değiştirme Sönümü (0.50) ile rezonans önleme",
            "Açısal Yumuşak Ölü Bölge (Pan 0.85°, Tilt 0.95°): Hermite taper ile %65.5 kilit kararlılığı"
        ],
        940, 585, 470, 160,
        "#1C162C", "#8B5CF6", "#FFFFFF", 10, "#D8B4FE", "#C084FC"
    ),

    # Layer 4 Cards (y=825, h=180)
    (
        "card_stm32",
        "STM32F401RE Nucleo Gömülü Denetleyici",
        "(ARM Cortex-M4 @ 84 MHz | 115200 Baud UART | Donanımsal Timer)",
        [
            "50 Hz Donanımsal Timer PWM Üreteçleri (Servolar & TB6612FNG Sürücü)",
            "300 ms Heartbeat Watchdog & Mandallı E-Stop Güvenlik Protokolü",
            "Solenoid Fırlatma Zamanlayıcısı: 80 ms hassas yükselen darbe üretimi",
            "0.1° Çözünürlüklü Açısal Servo Telemetrisi ve Durum Geri Bildirimi"
        ],
        50, 825, 410, 180,
        "#0F1E36", "#3B82F6", "#FFFFFF", 10.2, "#93C5FD", "#93C5FD"
    ),
    (
        "card_actuators",
        "Fiziksel Eyleyiciler & Fırlatma Sistemi",
        "(2-DOF Gimbal + Çift FF-180 Volan + JF-0530B Solenoid Dart İtici)",
        [
            "MG996R / DS3218 Dijital Servolar (Metal Dişli, 45°/s Maksimum Slew Hızı)",
            "Çift FF-180 Motor: 23.500 RPM Devir ile 10.0 m/s Mermi Çıkış Hızı",
            "5-Aşamalı Atış Sekansı: 1.0s Spinup -> 0.15s Settle -> 80ms Darbe -> 0.35s Recover -> 1.50s Eval",
            "IRLZ44N Logic-Level N-MOSFET + Optokuplör ile Parazitsiz İzolasyon"
        ],
        520, 825, 440, 180,
        "#2A1014", "#EF4444", "#FFFFFF", 10, "#FCA5A5", "#FCA5A5"
    ),
    (
        "card_hud_telem",
        "Taktiksel MIL-STD Operatör HUD & Telemetri",
        "(turret_hud.py & telemetry_logger.py)",
        [
            "Taktiksel Nişangah, LEAD Reticle ve Kademeli Vuruş Olasılığı Göstergesi",
            "Büyük Harf [P CONTROLLER] / [RL CONTROLLER] Bannerı & Muharebe Sadeleştirmesi",
            "Operatör Onayı: Manuel Atış [Y Tuşu], Acil Durdurma [Space], Hedef Bırak [N]",
            "Telemetri V2: 30 Hz Senkronize JSON Metadata Sidecar & CSV Telemetri Kaydı"
        ],
        1010, 825, 400, 180,
        "#261A0C", "#F59E0B", "#FFFFFF", 10, "#FDE68A", "#FDE68A"
    )
]

# Connectors with collision-free paths and clean corridor labels
CONNECTORS = [
    # 1. C270 -> YOLO (direct horizontal connection with compact badge)
    (
        "edge_c270_yolo",
        "card_c270", "card_yolo",
        "RGB Video (30 FPS)",
        "#8B5CF6",
        [(440, 145), (510, 145)],
        (475, 133)
    ),
    # 2. YOLO -> Kalman (routed down in corridor between layer 1 and 2)
    (
        "edge_yolo_kalman",
        "card_yolo", "card_kalman",
        "Hedef BBox & Optik Sapma",
        "#8B5CF6",
        [(630, 220), (630, 245), (255, 245), (255, 300)],
        (442, 245)
    ),
    # 3. LiDAR -> Lead (routed cleanly from LiDAR down to Lead)
    (
        "edge_luna_lead",
        "card_luna", "card_lead",
        "100 Hz UART ToF Mesafe",
        "#3B82F6",
        [(1120, 220), (1120, 245), (740, 245), (740, 300)],
        (930, 245)
    ),
    # 4. Kalman -> Lead (direct horizontal)
    (
        "edge_kalman_lead",
        "card_kalman", "card_lead",
        "Filtrelenmiş Hız [vx, vy]",
        "#10B981",
        [(460, 402), (520, 402)],
        (490, 390)
    ),
    # 5. Lead -> LockManager (direct horizontal)
    (
        "edge_lead_lock",
        "card_lead", "card_lock",
        "Önleme & Nişan Noktası",
        "#10B981",
        [(960, 402), (1020, 402)],
        (990, 390)
    ),
    # 6. Kalman & Lead -> P Controller (routed down in corridor between layer 2 and 3)
    (
        "edge_lead_p",
        "card_lead", "card_p_ctrl",
        "Açı Hatası + Süper-Elevasyon",
        "#10B981",
        [(650, 505), (650, 530), (265, 530), (265, 585)],
        (457, 530)
    ),
    # 7. Lead & Kalman -> RL Controller (routed down to RL)
    (
        "edge_lead_rl",
        "card_lead", "card_rl_ctrl",
        "11-B Durum Gözlem Vektörü",
        "#8B5CF6",
        [(830, 505), (830, 530), (1175, 530), (1175, 585)],
        (1002, 530)
    ),
    # 8. Mode Selector -> P Controller
    (
        "edge_mode_p",
        "card_mode_sel", "card_p_ctrl",
        "[M] = P Modu",
        "#F59E0B",
        [(540, 665), (480, 665)],
        (510, 653)
    ),
    # 9. Mode Selector -> RL Controller
    (
        "edge_mode_rl",
        "card_mode_sel", "card_rl_ctrl",
        "[M] = RL Modu",
        "#F59E0B",
        [(880, 665), (940, 665)],
        (910, 653)
    ),
    # 10. P Controller -> STM32 (routed down in corridor between layer 3 and 4)
    (
        "edge_p_stm32",
        "card_p_ctrl", "card_stm32",
        "P Açısal Hız Komutu",
        "#10B981",
        [(215, 745), (215, 770), (215, 825)],
        (215, 785)
    ),
    # 11. RL Controller -> STM32 (routed cleanly under Layer 3 cards in the corridor at y=770)
    (
        "edge_rl_stm32",
        "card_rl_ctrl", "card_stm32",
        "RL Artımlı Açı Adımı (0.75°)",
        "#8B5CF6",
        [(1050, 745), (1050, 770), (300, 770), (300, 825)],
        (675, 770)
    ),
    # 12. STM32 -> Actuators (routed above card tops in corridor at y=808)
    (
        "edge_stm32_actuators",
        "card_stm32", "card_actuators",
        "50 Hz PWM + 80 ms Solenoid Darbesi",
        "#EF4444",
        [(460, 840), (485, 840), (485, 808), (640, 808), (640, 825)],
        (562, 808)
    ),
    # 13. LockManager -> HUD (routed cleanly down the right margin at x=1435 and along corridor)
    (
        "edge_lock_hud",
        "card_lock", "card_hud_telem",
        "Kilit Kararı & Vuruş Olasılığı",
        "#F59E0B",
        [(1215, 505), (1215, 535), (1435, 535), (1435, 770), (1210, 770), (1210, 825)],
        (1325, 770)
    ),
    # 14. STM32 -> Telemetry HUD (routed along bottom corridor)
    (
        "edge_stm32_telem",
        "card_stm32", "card_hud_telem",
        "115200 Baud Çift Yönlü UART Telemetrisi",
        "#3B82F6",
        [(255, 1005), (255, 1030), (1210, 1030), (1210, 1005)],
        (732, 1030)
    )
]

def xml_escape(text):
    if not text:
        return ""
    t = text.replace('&', '&amp;')
    t = t.replace('&amp;amp;', '&amp;').replace('&amp;lt;', '&lt;').replace('&amp;gt;', '&gt;')
    t = t.replace('>=', '&gt;=').replace('<=', '&lt;=')
    return t

# Build draw.io model
root_elem = ET.Element('mxfile', host='app.diagrams.net', scale='1', border='0')
diag_elem = ET.SubElement(root_elem, 'diagram', id='fcs_arch_diag', name='Turret FCS v1.4 Sistem Mimarisi')
model_elem = ET.SubElement(diag_elem, 'mxGraphModel', dx='1600', dy='1000', grid='1', gridSize='10',
                           guides='1', tooltips='1', connect='1', arrows='1', fold='1', page='1',
                           pageScale='1', pageWidth='1500', pageHeight='1100', math='0', shadow='0')
mx_root = ET.SubElement(model_elem, 'root')
ET.SubElement(mx_root, 'mxCell', id='0')
ET.SubElement(mx_root, 'mxCell', id='1', parent='0')

for lid, title, x, y, w, h, fill, stroke, title_col in LAYERS:
    style = (f"rounded=1;whiteSpace=wrap;html=1;arcSize=6;fillColor={fill};strokeColor={stroke};"
             f"strokeWidth=2;dashed=1;verticalAlign=top;align=left;spacingLeft=15;spacingTop=10;"
             f"fontStyle=1;fontSize=13;fontColor={title_col};")
    cell = ET.SubElement(mx_root, 'mxCell', id=lid, parent='1', style=style, value=f"&lt;b&gt;{html.escape(title)}&lt;/b&gt;", vertex='1')
    ET.SubElement(cell, 'mxGeometry', x=str(x), y=str(y), width=str(w), height=str(h), **{'as': 'geometry'})

for cid, title, subtitle, bullets, x, y, w, h, fill, stroke, text_col, fsize, key_col, sub_col in CARDS:
    style = (f"rounded=1;whiteSpace=wrap;html=1;arcSize=10;fillColor={fill};strokeColor={stroke};"
             f"strokeWidth=2;fontColor={text_col};align=center;verticalAlign=middle;")
    lines = [f"&lt;b&gt;{html.escape(title)}&lt;/b&gt;", f"&lt;font style=\"font-size:10px; color:{sub_col};\"&gt;{html.escape(subtitle)}&lt;/font&gt;"]
    b_html = "".join([f"&lt;br&gt;• {html.escape(b)}" for b in bullets])
    full_val = f"{lines[0]}&lt;br&gt;{lines[1]}&lt;br&gt;&lt;font style=\"font-size:{fsize}px;\"&gt;{b_html}&lt;/font&gt;"
    cell = ET.SubElement(mx_root, 'mxCell', id=cid, parent='1', style=style, value=full_val, vertex='1')
    ET.SubElement(cell, 'mxGeometry', x=str(x), y=str(y), width=str(w), height=str(h), **{'as': 'geometry'})

for eid, src, tgt, label, col, pts, lpos in CONNECTORS:
    style = (f"edgeStyle=orthogonalEdgeStyle;rounded=1;orthogonalLoop=1;jettySize=auto;html=1;"
             f"strokeColor={col};strokeWidth=2.2;fontColor={col};fontSize=9.5;fontStyle=1;")
    cell = ET.SubElement(mx_root, 'mxCell', id=eid, parent='1', source=src, target=tgt, style=style, value=html.escape(label), edge='1')
    geo = ET.SubElement(cell, 'mxGeometry', relative='1', **{'as': 'geometry'})
    if pts:
        arr = ET.SubElement(geo, 'Array', **{'as': 'points'})
        for px, py in pts:
            ET.SubElement(arr, 'mxPoint', x=str(px), y=str(py))

xml_raw = ET.tostring(root_elem, encoding='unicode')
encoded_content = urllib.parse.quote(xml_raw)

# Generate SVG in Dark Theme
svg_lines = []
svg_lines.append('<?xml version="1.0" encoding="UTF-8"?>')
svg_lines.append('<!-- Do not edit this file with editors other than draw.io -->')
svg_lines.append('<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">')
svg_lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" style="background: #121212; background-color: var(--ge-adaptive-bg, #121212); color-scheme: dark;" xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1" width="{WIDTH}px" height="{HEIGHT}px" viewBox="0 0 {WIDTH} {HEIGHT}" id="turret-fcs-arch-v14" content="{encoded_content}">')
svg_lines.append('  <style type="text/css">')
svg_lines.append('    #turret-fcs-arch-v14 { --ge-adaptive-bg: #121212; --ge-dark-color: #121212; }')
svg_lines.append('  </style>')
svg_lines.append('  <defs>')
colors = set(c[4] for c in CONNECTORS)
for col in colors:
    cid = col.replace('#', '')
    svg_lines.append(f'    <marker id="arrow-{cid}" markerWidth="10" markerHeight="8" refX="7" refY="4" orient="auto">')
    svg_lines.append(f'      <polygon points="0 1, 8 4, 0 7" fill="{col}"/>')
    svg_lines.append('    </marker>')
svg_lines.append('  </defs>')
svg_lines.append(f'  <rect fill="#121212" style="fill: var(--ge-adaptive-bg, #121212);" width="100%" height="100%" x="0" y="0"/>')
svg_lines.append('  <g>')
svg_lines.append('    <g data-cell-id="0">')
svg_lines.append('      <g data-cell-id="1">')

# Render Layers
for lid, title, x, y, w, h, fill, stroke, title_col in LAYERS:
    e_title = xml_escape(title)
    svg_lines.append(f'        <g data-cell-id="{lid}">')
    svg_lines.append(f'          <g><rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" ry="12" fill="{fill}" stroke="{stroke}" stroke-width="2" stroke-dasharray="6 6" pointer-events="all"/></g>')
    svg_lines.append('          <g><g><switch><foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">')
    svg_lines.append(f'            <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe flex-start; justify-content: unsafe flex-start; width: {w-30}px; height: 1px; padding-top: {y+12}px; margin-left: {x+15}px;">')
    svg_lines.append(f'              <div style="box-sizing: border-box; font-size: 0; text-align: left; color: {title_col};">')
    svg_lines.append(f'                <div style="display: inline-block; font-size: 13px; font-family: Segoe UI, Helvetica, Arial, sans-serif; color: {title_col}; line-height: 1.2; pointer-events: all; font-weight: bold; white-space: normal; word-wrap: normal;">{e_title}</div>')
    svg_lines.append('              </div>')
    svg_lines.append('            </div>')
    svg_lines.append('          </foreignObject>')
    svg_lines.append(f'          <text x="{x+15}" y="{y+25}" fill="{title_col}" font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="13px" font-weight="bold">{e_title}</text>')
    svg_lines.append('          </switch></g></g>')
    svg_lines.append('        </g>')

# Render Cards
for cid, title, subtitle, bullets, x, y, w, h, fill, stroke, text_col, fsize, key_col, sub_col in CARDS:
    e_title = xml_escape(title)
    e_sub = xml_escape(subtitle)
    svg_lines.append(f'        <g data-cell-id="{cid}">')
    svg_lines.append(f'          <g><rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" ry="10" fill="{fill}" stroke="{stroke}" stroke-width="2" pointer-events="all"/></g>')
    svg_lines.append('          <g><g><switch><foreignObject style="overflow: visible; text-align: left;" pointer-events="none" width="100%" height="100%" requiredFeatures="http://www.w3.org/TR/SVG11/feature#Extensibility">')
    svg_lines.append(f'            <div xmlns="http://www.w3.org/1999/xhtml" style="display: flex; align-items: unsafe center; justify-content: unsafe center; width: {w-16}px; height: 1px; padding-top: {y + h//2}px; margin-left: {x+8}px;">')
    svg_lines.append(f'              <div style="box-sizing: border-box; font-size: 0; text-align: center; color: #FFFFFF;">')
    svg_lines.append(f'                <div style="display: inline-block; font-size: 11px; font-family: Segoe UI, Helvetica, Arial, sans-serif; color: #FFFFFF; line-height: 1.35; pointer-events: all; white-space: normal; word-wrap: normal;">')
    svg_lines.append(f'                  <b style="color: #FFFFFF; font-size: 11.5px;">{e_title}</b><br />')
    svg_lines.append(f'                  <font style="font-size: 9.5px; color: {sub_col};">{e_sub}</font><br />')
    svg_lines.append(f'                  <div style="text-align: left; margin-top: 5px; font-size: {fsize}px; line-height: 1.38; color: #E2E8F0;">')
    for b in bullets:
        eb = xml_escape(b)
        if ':' in eb:
            part1, part2 = eb.split(':', 1)
            svg_lines.append(f'                    • <b style="color: {key_col};">{part1}:</b><span style="color: #E2E8F0;">{part2}</span><br />')
        else:
            svg_lines.append(f'                    • <span style="color: #E2E8F0;">{eb}</span><br />')
    svg_lines.append('                  </div>')
    svg_lines.append('                </div>')
    svg_lines.append('              </div>')
    svg_lines.append('            </div>')
    svg_lines.append('          </foreignObject>')
    svg_lines.append(f'          <text x="{x + w//2}" y="{y + 20}" fill="#FFFFFF" font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="11px" font-weight="bold" text-anchor="middle">{e_title}</text>')
    svg_lines.append('          </switch></g></g>')
    svg_lines.append('        </g>')

# Render Connectors & Labels
for eid, src, tgt, label, col, pts, lpos in CONNECTORS:
    cid = col.replace('#', '')
    e_label = xml_escape(label)
    d_path = f"M {pts[0][0]} {pts[0][1]} "
    for px, py in pts[1:]:
        d_path += f"L {px} {py} "
    
    mid_x, mid_y = lpos
    
    svg_lines.append(f'        <g data-cell-id="{eid}">')
    svg_lines.append(f'          <path d="{d_path.strip()}" fill="none" stroke="{col}" stroke-width="2.2" stroke-miterlimit="10" marker-end="url(#arrow-{cid})" pointer-events="stroke"/>')
    if label:
        lbl_w = len(label) * 6.3 + 20
        lbl_h = 20
        svg_lines.append(f'          <rect x="{mid_x - lbl_w//2}" y="{mid_y - lbl_h//2 - 2}" width="{lbl_w}" height="{lbl_h}" rx="6" ry="6" fill="#181B26" stroke="{col}" stroke-width="1.3" opacity="0.96"/>')
        svg_lines.append(f'          <text x="{mid_x}" y="{mid_y + 3.5}" fill="#F8FAFC" font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="9.5px" font-weight="bold" text-anchor="middle">{e_label}</text>')
    svg_lines.append('        </g>')

svg_lines.append('      </g>')
svg_lines.append('    </g>')
svg_lines.append('  </g>')
svg_lines.append('</svg>')

with open('docs/assets/turret_fcs_architecture.svg', 'w', encoding='utf-8') as f:
    f.write('\n'.join(svg_lines))

print("Regenerated docs/assets/turret_fcs_architecture.svg with dark background matching other assets!")
