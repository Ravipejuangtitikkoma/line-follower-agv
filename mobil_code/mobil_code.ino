// ====================================================================
// PROGRAM UTAMA: MOBIL AGV ROBOT (HIERARCHICAL FINITE STATE MACHINE)
// Board : Mappi32 (ESP32) by KMtek
// Arsitektur Modular:
//   - Config.h        : Konfigurasi Pin, Parameter Menggelir & Penetrasi Titik A/B
//   - Motor.h         : Penggerak Motor, Filter Akselerasi, & Soft-Start Ramp
//   - Sensor.h        : Line Follower 5-Ch TCRT5000, Ultrasonik HC-SR04
//   - NodeDetector.h  : Algoritma Deteksi Simpang & Kunci Garis Siku
//   - RoutePlans.h    : Tabel Rute Dinamis (RAM) & Preset Default
//   - GridGraph.h     : Graph Route Planner Arena 3m x 3m (Otomatis Stasiun A-E)
//   - RouteStorage.h  : Penyimpanan Permanen Flash ESP32 (NVS Preferences)
//   - NavigationFSM.h : Eksekutor 4-Fase Navigasi & Parser Perintah Serial/LoRa
// ====================================================================

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensor.h"
#include "NodeDetector.h"
#include "RoutePlans.h"
#include "GridGraph.h"
#include "RouteStorage.h"
#include "NavigationFSM.h"

// Variabel Pewaktu Telemetri
static unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.println("   AGV ROBOT: HIERARCHICAL FINITE STATE MACHINE (HFSM) ");
  Serial.println("   Mapping Dinamis via Serial Monitor + Flash NVS      ");
  Serial.println("   Board: Mappi32 (ESP32)                              ");
  Serial.println("=======================================================");

  // Inisialisasi Perangkat Keras & Muat Mapping Tersimpan dari Flash
  initMotor();
  initSensors();
  resetNodeDetector();
  muatMappingDariFlash();
  initNavigationFSM();
  tampilkanTabelMapping();

  if (!GUNAKAN_SENSOR_JARAK) {
    Serial.println(F("[INFO] Sensor Jarak HC-SR04: DINONAKTIFKAN (Mobil jalan terus tanpa halangan)."));
  }

  cetakBantuanSerial();

  if (runtimeAutoStart) {
    Serial.println("\n[INFO] Robot otomatis mulai dalam 2 detik... (Ketik perintah di Serial untuk batal & konfigurasi)\n");
    unsigned long tWait = millis();
    while (millis() - tWait < 2000) {
      if (Serial.available()) {
        statusMisiSaatIni = MISI_STANDBY;
        String cmd = Serial.readStringUntil('\n');
        prosesPerintah(cmd);
        break;
      }
      delay(10);
    }
  } else {
    Serial.println("\n[INFO] Mode AUTOSTART = OFF (STANDBY). Ketik 'START' untuk mulai atau 'HELP' untuk panduan.\n");
  }
}

void loop() {
  // 1. UPDATE RADAR KEAMANAN (HC-SR04 Tiap 60ms)
  updateRadarKeamanan();

  // 2. SAFETY INTERLOCK: JIKA ADA HALANGAN DEPAN, REM DARURAT
  if (statusHalangan) {
    berhentiTotal();
    if (millis() - lastLogTime >= 300) {
      lastLogTime = millis();
      Serial.print("[HALANGAN DEPAN!] Berhenti Darurat! Jarak: ");
      if (jarakSaatIni <= 2.0) {
        Serial.println("< 3 cm (Mepet Tembok / Titik Buta)");
      } else {
        Serial.print(jarakSaatIni, 1);
        Serial.println(" cm");
      }
    }
    delay(15);
    return;
  }

  // 3. MONITOR PERINTAH EKSTERNAL (SERIAL MONITOR / SIMULASI PAKET LORA)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    prosesPerintah(cmd);
  }

  // 4. PEMBACAAN 5 SENSOR GARIS (1 = Hitam, 0 = Spanduk/Kardus Putih)
  LineData line = bacaSensorGaris();

  // 5. EKSEKUSI DATA-DRIVEN PLAN ENGINE (HFSM)
  // Menjalankan Level 1 (Misi Status) & Level 2 (Eksekutor 4-Fase atas ruteAktif[]):
  //   - PHASE_FOLLOW -> PHASE_PRE_MOVE -> PHASE_PAUSE -> PHASE_ACTION
  updateNavigationFSM(line);

  // 6. TELEMETRI REAL-TIME SERIAL MONITOR (Setiap 250ms)
  if (millis() - lastLogTime >= 250) {
    lastLogTime = millis();
    tampilkanTelemetriFSM(line);
  }

  delay(15);
}