// ====================================================================
// PROGRAM PENGUJIAN & DIAGNOSTIK SENSOR GARIS 5-CHANNEL (MAPPI32 ESP32)
// Board : Mappi32 (ESP32-WROOM-32E by KMtek)
// Modul : 5-Channel Line Follower TCRT5000 / BFD-1000
// File  : test_sensor_garis.ino
// Baud  : 115200
//
// KONFIGURASI RESMI PIN MAPPI32 (PERSIS SAMA DENGAN Config.h AGV):
//   Sensor OUT1 -> GPIO 34 (Input Only / GPI) - S1
//   Sensor OUT2 -> GPIO 35 (Input Only / GPI) - S2
//   Sensor OUT3 -> GPIO 36 (Input Only / SENSOR_VP) - S3 (Tengah)
//   Sensor OUT4 -> GPIO 39 (Input Only / SENSOR_VN) - S4
//   Sensor OUT5 -> GPIO 23 (Digital GPIO) - S5
//
// PERINGATAN KERAS UNTUK BOARD MAPPI32:
//   - JANGAN gunakan GPIO 27 (terhubung ke LoRa DIO0 internal!)
//   - JANGAN gunakan GPIO 26 (terhubung ke Driver Motor Mundur Kiri!)
//   - JANGAN gunakan GPIO 32 & 33 (terhubung ke Encoder Roda Kiri!)
//   - JANGAN gunakan GPIO 0, 12, 13, 14, 15 (jalur SPI RFM95 LoRa Mappi32!)
// ====================================================================

#include <Arduino.h>

// ====================================================================
// [1] KONFIGURASI PIN SENSOR GARIS 5-CHANNEL (MAPPI32)
// ====================================================================
const int PIN_LINE1 = 34; // OUT1 (PCB)
const int PIN_LINE2 = 35; // OUT2 (PCB)
const int PIN_LINE3 = 36; // OUT3 (PCB - Pin VP / Tengah)
const int PIN_LINE4 = 39; // OUT4 (PCB - Pin VN)
const int PIN_LINE5 = 23; // OUT5 (PCB)

// ====================================================================
// [2] PARAMETER LOGIKA & URUTAN FISIK SENSOR
// ====================================================================
// Sesuai Config.h: kabel PCB fisik menghadap ke depan dengan OUT5 di Kiri dan OUT1 di Kanan
bool reverseSensorOrder = true; 

// Logika TCRT5000 umum:
//   LOW  (0) saat di atas garis HITAM
//   HIGH (1) saat di atas lantai PUTIH / SPANDUK / KARDUS
// Jika sensor Anda kebalikannya (HIGH di hitam), ubah activeLow = false (bisa tekan 'P' via Serial)
bool activeLow = false; // Active HIGH (1 = Garis Hitam, 0 = Lantai Putih)

// Mode Tampilan Tambahan (Bisa di-toggle via keyboard Serial):
bool showADCValues = false; // Tampilkan nilai ADC pin 34, 35, 36, 39

// Pewaktu Telemetri (100ms = 10 Hz live refresh)
unsigned long lastPrintTime = 0;
const unsigned long INTERVAL_UPDATE_MS = 100;

// ====================================================================
// [3] PROTOTIPE FUNGSI
// ====================================================================
void tampilkanBantuan();
void prosesPerintahSerial();

void setup() {
  // Inisialisasi Serial Monitor (Wajib set 115200 baud)
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi Seluruh Pin Sensor Garis pada Mappi32
  // Catatan: GPIO 34, 35, 36, 39 adalah Input-Only tanpa internal pullup
  pinMode(PIN_LINE1, INPUT);
  pinMode(PIN_LINE2, INPUT);
  pinMode(PIN_LINE3, INPUT);
  pinMode(PIN_LINE4, INPUT);
  pinMode(PIN_LINE5, INPUT);

  // Set ADC 10-bit (0 - 1023) untuk pembacaan analog jika diperlukan
  analogReadResolution(10);

  Serial.println(F("\n======================================================="));
  Serial.println(F("   DIAGNOSTIK & UJI SENSOR GARIS 5-CH (MAPPI32 ESP32)  "));
  Serial.println(F("======================================================="));
  Serial.println(F("[PANDUAN KABEL SENSOR KE MAPPI32]:"));
  Serial.println(F("  Sensor VCC  --> Pin 3.3V atau 5V Mappi32"));
  Serial.println(F("  Sensor GND  --> Pin GND Mappi32"));
  Serial.println(F("  Sensor OUT1 --> GPIO 34 (S1)"));
  Serial.println(F("  Sensor OUT2 --> GPIO 35 (S2)"));
  Serial.println(F("  Sensor OUT3 --> GPIO 36 / VP (S3 Tengah)"));
  Serial.println(F("  Sensor OUT4 --> GPIO 39 / VN (S4)"));
  Serial.println(F("  Sensor OUT5 --> GPIO 23 (S5)"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("[PROTEKSI PIN MAPPI32]:"));
  Serial.println(F("  Pin GPIO 27, 26, 32, 33, 14, 13, 12, 15, 0 AMAN &"));
  Serial.println(F("  TIDAK DIPAKAI oleh skrip uji sensor ini!"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("[STATUS AWAL]:"));
  Serial.print(F("  Reverse Order : ")); Serial.println(reverseSensorOrder ? F("TRUE (OUT5=Kiri, OUT1=Kanan)") : F("FALSE (OUT1=Kiri, OUT5=Kanan)"));
  Serial.print(F("  Polaritas     : ")); Serial.println(activeLow ? F("ACTIVE LOW (0 = Garis Hitam)") : F("ACTIVE HIGH (1 = Garis Hitam)"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("[TOMBOL KONTROL LIVE (Ketik di Serial Monitor)]:"));
  Serial.println(F("  'R' / 'r' -> Balik Urutan Kiri <-> Kanan (Reverse)"));
  Serial.println(F("  'P' / 'p' -> Ubah Polaritas (Active LOW <-> HIGH)"));
  Serial.println(F("  'A' / 'a' -> Toggle Tampilkan Nilai ADC Analog"));
  Serial.println(F("  'H' / 'h' -> Tampilkan Menu Bantuan Ini"));
  Serial.println(F("=======================================================\n"));

  delay(1500);
}

void loop() {
  // 1. Tangani input tombol kontrol dari Serial Monitor
  prosesPerintahSerial();

  // 2. Loop berkala pembacaan sensor
  if (millis() - lastPrintTime >= INTERVAL_UPDATE_MS) {
    lastPrintTime = millis();

    // A. BACA NILAI MENTAH (RAW DIGITAL) DARI 5 PIN MAPPI32
    int raw1 = digitalRead(PIN_LINE1);
    int raw2 = digitalRead(PIN_LINE2);
    int raw3 = digitalRead(PIN_LINE3);
    int raw4 = digitalRead(PIN_LINE4);
    int raw5 = digitalRead(PIN_LINE5);

    // B. KONVERSI POLARITAS (1 = Garis Hitam Terdeteksi, 0 = Lantai Terang)
    int s1 = activeLow ? (raw1 == LOW ? 1 : 0) : (raw1 == HIGH ? 1 : 0);
    int s2 = activeLow ? (raw2 == LOW ? 1 : 0) : (raw2 == HIGH ? 1 : 0);
    int s3 = activeLow ? (raw3 == LOW ? 1 : 0) : (raw3 == HIGH ? 1 : 0);
    int s4 = activeLow ? (raw4 == LOW ? 1 : 0) : (raw4 == HIGH ? 1 : 0);
    int s5 = activeLow ? (raw5 == LOW ? 1 : 0) : (raw5 == HIGH ? 1 : 0);

    // C. MAPPING KE SUSUNAN FISIK KIRI -> KANAN
    // Jika reverseSensorOrder = true: S5 = Kiri Luar, S1 = Kanan Luar (Sesuai fisik mobil AGV)
    int b1, b2, b3, b4, b5;
    if (!reverseSensorOrder) {
      b1 = s1; // Kiri Luar
      b2 = s2; // Kiri Tengah
      b3 = s3; // Tengah
      b4 = s4; // Kanan Tengah
      b5 = s5; // Kanan Luar
    } else {
      b1 = s5; // OUT5 jadi Kiri Luar
      b2 = s4; // OUT4 jadi Kiri Tengah
      b3 = s3; // OUT3 tetap Tengah
      b4 = s2; // OUT2 jadi Kanan Tengah
      b5 = s1; // OUT1 jadi Kanan Luar
    }

    int totalAktif = b1 + b2 + b3 + b4 + b5;

    // D. HITUNG KOREKSI ERROR POSISI (-2.0 s/d +2.0, persis Sensor.h)
    float error = 0.0;
    if (totalAktif > 0) {
      float bobot = (b1 * -2.0) + (b2 * -1.0) + (b3 * 0.0) + (b4 * 1.0) + (b5 * 2.0);
      error = bobot / (float)totalAktif;
    }

    // E. TAMPILKAN STATUS GPIO MENTAH
    Serial.print(F("GPIO [34:")); Serial.print(raw1);
    Serial.print(F(" 35:"));     Serial.print(raw2);
    Serial.print(F(" 36:"));     Serial.print(raw3);
    Serial.print(F(" 39:"));     Serial.print(raw4);
    Serial.print(F(" 23:"));     Serial.print(raw5);
    Serial.print(F("] "));

    // F. OPSIONAL: TAMPILKAN ADC ANALOG (0 - 1023)
    if (showADCValues) {
      int a1 = analogRead(PIN_LINE1);
      int a2 = analogRead(PIN_LINE2);
      int a3 = analogRead(PIN_LINE3);
      int a4 = analogRead(PIN_LINE4);
      Serial.print(F("| ADC: ["));
      Serial.print(a1); Serial.print(F(" "));
      Serial.print(a2); Serial.print(F(" "));
      Serial.print(a3); Serial.print(F(" "));
      Serial.print(a4); Serial.print(F(" -] "));
    }

    // G. VISUALISASI FISIK: [■] = Garis Hitam, [ ] = Lantai Terang
    Serial.print(F("| FISIK: | "));
    Serial.print(b1 ? F("[■]") : F("[ ]")); Serial.print(F(" "));
    Serial.print(b2 ? F("[■]") : F("[ ]")); Serial.print(F(" "));
    Serial.print(b3 ? F("[■]") : F("[ ]")); Serial.print(F(" "));
    Serial.print(b4 ? F("[■]") : F("[ ]")); Serial.print(F(" "));
    Serial.print(b5 ? F("[■]") : F("[ ]"));
    Serial.print(F(" | "));

    // H. TOTAL SENSOR AKTIF & NILAI ERROR
    Serial.print(F("Tot:"));
    Serial.print(totalAktif);
    Serial.print(F(" | Err:"));
    if (error >= 0.0) Serial.print(F("+"));
    Serial.print(error, 1);
    Serial.print(F(" | "));

    // I. INTERPRETASI STATUS DETEKSI (IDENTIK DENGAN LOGIKA NAVIGASI AGV)
    if (totalAktif == 0) {
      Serial.println(F("--> LEPAS GARIS (Putih/Terang Semua)"));
    } else if (totalAktif >= 4) {
      Serial.println(F("--> PERSIMPANGAN / GARIS MELINTANG [1 1 1 1 1]"));
    } else if (b3 == 1 && b1 == 0 && b5 == 0) {
      if (b2 == 1 && b4 == 1) {
        Serial.println(F("--> LURUS SEMPURNA [0 1 1 1 0] (Pas Tengah)"));
      } else if (b2 == 0 && b4 == 0) {
        Serial.println(F("--> LURUS TEPAT [0 0 1 0 0] (Tengah Pas)"));
      } else if (b4 == 1) {
        Serial.println(F("--> AGAK MENCONG KANAN [0 0 1 1 0]"));
      } else {
        Serial.println(F("--> AGAK MENCONG KIRI [0 1 1 0 0]"));
      }
    } else if (b5 == 1 && (b4 == 1 || b3 == 1) && b1 == 0) {
      Serial.println(F("--> TIKUNGAN / SIKU KANAN (Garis di Kanan)"));
    } else if (b1 == 1 && (b2 == 1 || b3 == 1) && b5 == 0) {
      Serial.println(F("--> TIKUNGAN / SIKU KIRI (Garis di Kiri)"));
    } else if (b1 == 1 && b5 == 1) {
      Serial.println(F("--> DETEKSI PEREMPATAN [1 ... 1]"));
    } else if (b5 == 1) {
      Serial.println(F("--> SENSOR PALING KANAN AKTIF [0 0 0 0 1]"));
    } else if (b1 == 1) {
      Serial.println(F("--> SENSOR PALING KIRI AKTIF [1 0 0 0 0]"));
    } else {
      Serial.println(F("--> DETEKSI GARIS"));
    }
  }
}

// ====================================================================
// [4] PENGENDALI INPUT SERIAL INTERAKTIF
// ====================================================================
void prosesPerintahSerial() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r' || cmd == 'R') {
      reverseSensorOrder = !reverseSensorOrder;
      Serial.println(F("\n-------------------------------------------------------"));
      Serial.print(F("[INFO] Reverse Sensor Order diubah ke: "));
      Serial.println(reverseSensorOrder ? F("TRUE (OUT5=Kiri, OUT1=Kanan)") : F("FALSE (OUT1=Kiri, OUT5=Kanan)"));
      Serial.println(F("-------------------------------------------------------\n"));
    } else if (cmd == 'p' || cmd == 'P') {
      activeLow = !activeLow;
      Serial.println(F("\n-------------------------------------------------------"));
      Serial.print(F("[INFO] Polaritas diubah ke: "));
      Serial.println(activeLow ? F("ACTIVE LOW (0 = Garis Hitam)") : F("ACTIVE HIGH (1 = Garis Hitam)"));
      Serial.println(F("-------------------------------------------------------\n"));
    } else if (cmd == 'a' || cmd == 'A') {
      showADCValues = !showADCValues;
      Serial.println(F("\n-------------------------------------------------------"));
      Serial.print(F("[INFO] Tampilan Nilai Analog ADC: "));
      Serial.println(showADCValues ? F("AKTIF (Pin 34, 35, 36, 39)") : F("NONAKTIF (Mode Cepat Digital Saja)"));
      Serial.println(F("-------------------------------------------------------\n"));
    } else if (cmd == 'h' || cmd == 'H') {
      tampilkanBantuan();
    }
  }
}

void tampilkanBantuan() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("                 MENU BANTUAN UJI SENSOR GARIS         "));
  Serial.println(F("======================================================="));
  Serial.println(F("Ketik huruf berikut di Serial Monitor lalu tekan Send/Enter:"));
  Serial.println(F("  'R' / 'r' -> Membalik urutan sensor Kiri <-> Kanan"));
  Serial.println(F("  'P' / 'p' -> Mengubah logika (0=Hitam <-> 1=Hitam)"));
  Serial.println(F("  'A' / 'a' -> Menampilkan nilai tegangan ADC pin 34,35,36,39"));
  Serial.println(F("  'H' / 'h' -> Menampilkan menu bantuan ini"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("Skema Pin Mappi32:"));
  Serial.println(F("  OUT1: GPIO 34 | OUT2: GPIO 35 | OUT3: GPIO 36"));
  Serial.println(F("  OUT4: GPIO 39 | OUT5: GPIO 23"));
  Serial.println(F("=======================================================\n"));
}
