// ====================================================================
// PROGRAM KHUSUS PENGUJIAN 1 SENSOR GARIS (SINGLE LINE SENSOR TEST)
// Board   : ESP32 (Mappi32 / ESP32 Dev Module / NodeMCU-32S)
// Sensor  : 1 Kanal Sensor Garis (TCRT5000 / ST188 / BFD-1000 / dsb.)
// File    : test_sensor_garis_tunggal.ino
// Baud    : 115200
// ===========a=========================================================
// DESKRIPSI:
// Program ini dibuat khusus untuk menguji SATU sensor garis saja.
// Sangat berguna untuk:
//   1. Memeriksa apakah modul sensor garis Anda masih hidup atau mati/rusak.
//   2. Mengkalibrasi trimpot (potensiometer sensitivitas) di atas lakban vs lantai.
//   3. Mengukur tegangan analog (ADC) dan logika digital secara real-time.
//   4. Mendiagnosis kabel putus, pin stuck HIGH / stuck LOW, atau noise.
// ====================================================================

#include <Arduino.h>

// ====================================================================
// [1] PENGATURAN PIN SENSOR (UBAH DI SINI)
// ====================================================================
// Masukkan pin GPIO ESP32 yang terhubung ke pin OUT sensor garis Anda.
//
// Rekomendasi Pin Mappi32 (Sesuai Konfigurasi Proyek AGV):
//   GPIO 34 -> Sensor 1 (OUT1) - Input Only (Support ADC)
//   GPIO 35 -> Sensor 2 (OUT2) - Input Only (Support ADC)
//   GPIO 36 -> Sensor 3 (OUT3 / VP) - Input Only (Support ADC)
//   GPIO 39 -> Sensor 4 (OUT4 / VN) - Input Only (Support ADC)
//   GPIO 23 -> Sensor 5 (OUT5) - GPIO Digital Standar
//
// Jika menggunakan ESP32 Dev Module biasa, Anda juga bisa mencoba GPIO 4, 18, 19, 21, dll.
const int PIN_SENSOR_GARIS = 34; // << GANTI PIN INI SESUAI COLOKAN ANDA

// ====================================================================
// [2] PENGATURAN POLARITAS LOGIKA
// ====================================================================
// Mayoritas sensor TCRT5000 / LM393:
//   - Di atas GARIS HITAM  : Output LOW  (0 Volt) -> LED modul MATI/MENYALA
//   - Di atas LANTAI TERANG: Output HIGH (3.3V)
// Jika sensor Anda ACTIVE LOW (0 saat di hitam), biarkan true.
// Jika kebalikannya (1 saat di hitam), ubah ke false (bisa tekan 'P' di Serial).
bool activeLow = false; // Active HIGH (1 = Garis Hitam, 0 = Lantai Putih)

// Ambang batas simulasi ADC jika menggunakan pin analog (Rentang 0 - 4095)
// Nilai di bawah threshold dianggap mendeteksi warna hitam/gelap
int thresholdADC = 1500;

// Variabel Diagnostik & Counter
unsigned long counterDeteksi = 0;
int lastStateDigital = -1;
unsigned long lastChangeTime = 0;
unsigned long lastPrintTime  = 0;
const unsigned long INTERVAL_PRINT_MS = 100; // Refresh tiap 100 ms (10 Hz)

// Cek apakah pin mendukung pembacaan Analog ADC pada ESP32
bool isPinADC(int pin) {
  // ADC1: GPIO 32, 33, 34, 35, 36, 39
  // ADC2: GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27
  return (pin == 32 || pin == 33 || pin == 34 || pin == 35 || pin == 36 || pin == 39 ||
          pin == 0  || pin == 2  || pin == 4  || pin == 12 || pin == 13 || pin == 14 || 
          pin == 15 || pin == 25 || pin == 26 || pin == 27);
}

void tampilkanBantuan() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("              PANDUAN PENGUJIAN 1 SENSOR               "));
  Serial.println(F("======================================================="));
  Serial.println(F("1. Buka Serial Monitor di 115200 Baud."));
  Serial.println(F("2. Letakkan sensor di atas LANTAI PUTIH/SPANDUK (~6-8 mm):"));
  Serial.println(F("   - Status harus terbaca: [  LANTAI PUTIH  ]"));
  Serial.println(F("   - Jika terbaca GARIS HITAM, putar trimpot biru kecil perlahan"));
  Serial.println(F("     sampai berubah menjadi LANTAI PUTIH."));
  Serial.println(F("3. Geser sensor ke atas LAKBAN HITAM:"));
  Serial.println(F("   - Status harus seketika berubah: [■■ GARIS HITAM ■■]"));
  Serial.println(F("4. Perhatikan 'Count': angka akan bertambah tiap kali"));
  Serial.println(F("   sensor mendeteksi garis hitam baru."));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("TOMBOL KONTROL SERIAL MONITOR:"));
  Serial.println(F("  'P' / 'p' -> Balik polaritas (Active LOW <-> HIGH)"));
  Serial.println(F("  'C' / 'c' -> Reset Counter deteksi jadi 0"));
  Serial.println(F("  'H' / 'h' -> Tampilkan panduan ini"));
  Serial.println(F("=======================================================\n"));
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi Pin Sensor
  // Jika pin adalah GPIO 34, 35, 36, 39 maka otomatis INPUT murni (tidak punya pull-up internal)
  pinMode(PIN_SENSOR_GARIS, INPUT);

  // Set resolusi ADC ESP32 ke 12-bit (0 - 4095)
  if (isPinADC(PIN_SENSOR_GARIS)) {
    analogReadResolution(12);
  }

  Serial.println(F("\n======================================================="));
  Serial.println(F("     UJI KHUSUS 1 SENSOR GARIS (ESP32 / MAPPI32)       "));
  Serial.println(F("======================================================="));
  Serial.print(F("Pin Sensor yang Diuji : GPIO "));
  Serial.println(PIN_SENSOR_GARIS);
  Serial.print(F("Tipe Kemampuan Pin    : "));
  if (isPinADC(PIN_SENSOR_GARIS)) {
    Serial.println(F("Digital I/O & Analog ADC (12-bit, 0-4095)"));
  } else {
    Serial.println(F("Digital I/O Saja"));
  }
  Serial.print(F("Polaritas Default     : "));
  Serial.println(activeLow ? F("ACTIVE LOW (0 = Garis Hitam)") : F("ACTIVE HIGH (1 = Garis Hitam)"));
  Serial.println(F("======================================================="));
  tampilkanBantuan();
  delay(1500);
}

void loop() {
  // 1. Terima Perintah Tombol dari Serial Monitor
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') {
      activeLow = !activeLow;
      Serial.print(F("\n[INFO] Polaritas diubah ke: "));
      Serial.println(activeLow ? F("ACTIVE LOW (0 = Hitam)") : F("ACTIVE HIGH (1 = Hitam)"));
    } else if (c == 'c' || c == 'C') {
      counterDeteksi = 0;
      Serial.println(F("\n[INFO] Counter deteksi di-reset ke 0."));
    } else if (c == 'h' || c == 'H') {
      tampilkanBantuan();
    }
  }

  // 2. Loop Pengujian Berkala (Tiap 100 ms)
  if (millis() - lastPrintTime >= INTERVAL_PRINT_MS) {
    lastPrintTime = millis();

    // A. Baca Sinyal Digital Mentah (0 atau 1)
    int rawDigital = digitalRead(PIN_SENSOR_GARIS);

    // B. Baca Sinyal Analog Mentah & Tegangan (Jika pin mendukung ADC)
    int rawADC = 0;
    float teganganVolt = 0.0;
    if (isPinADC(PIN_SENSOR_GARIS)) {
      rawADC = analogRead(PIN_SENSOR_GARIS);
      teganganVolt = (rawADC / 4095.0) * 3.3; // Estimasi voltase 0 - 3.3V
    }

    // C. Evaluasi Logika Garis Hitam
    bool isGarisHitam = activeLow ? (rawDigital == LOW) : (rawDigital == HIGH);

    // D. Hitung Transisi Deteksi Baru (Rising edge deteksi garis)
    if (isGarisHitam && (lastStateDigital == 0 || lastStateDigital == -1)) {
      counterDeteksi++;
      lastChangeTime = millis();
    }
    lastStateDigital = isGarisHitam ? 1 : 0;

    // E. Tampilkan Data Lengkap dalam 1 Baris Live
    
    // 1. Info Pin & Digital Raw
    Serial.print(F("Pin "));
    if (PIN_SENSOR_GARIS < 10) Serial.print(F(" "));
    Serial.print(PIN_SENSOR_GARIS);
    Serial.print(F(" | DIG: "));
    Serial.print(rawDigital);
    Serial.print(F(" ("));
    Serial.print(rawDigital == HIGH ? F("HIGH") : F("LOW "));
    Serial.print(F(") "));

    // 2. Info Analog ADC (Jika ada)
    if (isPinADC(PIN_SENSOR_GARIS)) {
      Serial.print(F("| ADC: "));
      if (rawADC < 1000) Serial.print(F(" "));
      if (rawADC < 100)  Serial.print(F(" "));
      if (rawADC < 10)   Serial.print(F(" "));
      Serial.print(rawADC);
      Serial.print(F(" ("));
      Serial.print(teganganVolt, 2);
      Serial.print(F("V) "));
    }

    // 3. Status Visual Deteksi
    Serial.print(F("| STATUS: "));
    if (isGarisHitam) {
      Serial.print(F("[■■ GARIS HITAM ■■]"));
    } else {
      Serial.print(F("[  LANTAI PUTIH  ]"));
    }

    // 4. Bar Gauge Grafik Visual
    Serial.print(F(" | BAR: ["));
    if (isPinADC(PIN_SENSOR_GARIS)) {
      // Grafik analog 10 blok
      int blok = map(rawADC, 0, 4095, 0, 10);
      for (int b = 0; b < 10; b++) {
        if (b < blok) Serial.print(F("#"));
        else Serial.print(F("."));
      }
    } else {
      // Grafik digital
      Serial.print(isGarisHitam ? F("##########") : F(".........."));
    }
    Serial.print(F("] "));

    // 5. Total Deteksi (Counter)
    Serial.print(F("| Total Deteksi: "));
    Serial.println(counterDeteksi);
  }
}
