/*
 * ==============================================================================
 * MODUL: PC <-> STM32 IKILI PROTOKOL SOZLESMESI (TEK KAYNAK)
 * ==============================================================================
 * Bu baslik, PC (Python) ve STM32 firmware'i arasindaki tum sabitleri tek
 * yerde toplar. Python tarafindaki karsiligi turret_fcs/protocol.py'dir;
 * iki dosya birlikte guncellenmelidir.
 *
 * Komut cercevesi (PC -> STM32, 7 bayt):
 *   [PC_TO_STM_HEADER][pan i16][tilt i16][state u8][checksum u8]
 * Telemetri cercevesi (STM32 -> PC, 10 bayt):
 *   [STM_TO_PC_HEADER][mesafe u16][sinyal u16][pan i16][tilt i16][checksum u8]
 *
 * Aci birimi: 0.1 derece (deger = aci * 10). Telemetrideki servo acilari
 * 0..180 derece araligindadir; PC tarafi 90 derece merkez offsetini cikarir.
 * ==============================================================================
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/* Cerceve basliklari */
#define PROTO_PC_TO_STM_HEADER   0xAAu
#define PROTO_STM_TO_PC_HEADER   0x55u
#define PROTO_LUNA_HEADER        0x59u

/* Cerceve uzunluklari (bayt) */
#define PROTO_CMD_FRAME_LEN      7u
#define PROTO_TLM_FRAME_LEN      10u
#define PROTO_LUNA_FRAME_LEN     9u

/* Aci olcegi: 0.1 derece/adim; servo merkez offseti PC tarafinda uygulanir */
#define PROTO_ANGLE_SCALE        10.0f
#define PROTO_ANGLE_OFFSET_DEG   90.0f

/* Durum bayti (state) kodlari */
#define PROTO_STATE_TRACKING     0u
#define PROTO_STATE_LOCKED       1u
#define PROTO_STATE_FIRING       2u
#define PROTO_STATE_ESTOP        3u

/* E-stop'un tek bozuk paketle tetiklenmemesi icin ust uste paket sayisi */
#define PROTO_ESTOP_CONFIRM_FRAMES 2u

#endif /* __PROTOCOL_H */
