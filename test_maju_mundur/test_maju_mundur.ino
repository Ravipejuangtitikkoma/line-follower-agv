// ====================================================================
// PROGRAM UJI SEDERHANA: MAJU & MUNDUR SAJA (KHUSUS BOARD MAPPI32 ESP32)
// File : test_maju_mundur.ino
// Board: Mappi32 (ESP32) / 2x Driver BTS7960
//
// PIN MAPPI32 RESMI PROYEK KITA:
//   - Roda KIRI Maju    : GPIO 25 (RPWM Kiri)
//   - Roda KIRI Mundur  : GPIO 26 (LPWM Kiri)
//   - Roda KANAN Maju   : GPIO 16 (RPWM Kanan - Solder IO16)
//   - Roda KANAN Mundur : GPIO 17 (LPWM Kanan - Solder IO17)
// ====================================================================

#include <Arduino.h>

// --- PIN DRIVER MOTOR MAPPI32 (ESP32) KITA ---
const int pinMajuKiri    = 25; // GPIO 25 Mappi32
const int pinMundurKiri  = 26; // GPIO 26 Mappi32
const int pinMajuKanan   = 16; // GPIO 16 Mappi32 (Solder IO16)
const int pinMundurKanan = 17; // GPIO 17 Mappi32 (Solder IO17)

// Kecepatan Dasar Motor (Rentang 0 - 255)
const int KECEPATAN_BASE = 100; 

// Tuning Penyeimbang Roda Asli (dari Config.h) agar jalan lurus simetris
const int TRIM_KIRI  = 22; // Roda kiri +22 PWM (mengatasi gesekan)
const int TRIM_KANAN = -6; // Roda kanan -6 PWM

// Fungsi Berhenti / Rem Total
void berhenti() {
  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, 0);
}

// Fungsi Bergerak Maju Lurus
void maju(int basePwm = KECEPATAN_BASE) {
  int pwmL = constrain(basePwm + TRIM_KIRI, 0, 255);
  int pwmR = constrain(basePwm + TRIM_KANAN, 0, 255);

  analogWrite(pinMajuKiri, pwmL);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, pwmR);
  analogWrite(pinMundurKanan, 0);
}

// Fungsi Bergerak Mundur Lurus
void mundur(int basePwm = KECEPATAN_BASE) {
  int pwmL = constrain(basePwm + TRIM_KIRI, 0, 255);
  int pwmR = constrain(basePwm + TRIM_KANAN, 0, 255);

  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, pwmL);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, pwmR);
}

void setup() {
  Serial.begin(115200);

  // Atur semua pin driver sebagai OUTPUT
  pinMode(pinMajuKiri, OUTPUT);
  pinMode(pinMundurKiri, OUTPUT);
  pinMode(pinMajuKanan, OUTPUT);
  pinMode(pinMundurKanan, OUTPUT);

  // Pastikan motor berhenti saat dinyalakan
  berhenti();

  Serial.println("\n=======================================================");
  Serial.println("       UJI COBA SEDERHANA: MAJU & MUNDUR               ");
  Serial.println("       Pin: Kiri(25, 26) | Kanan(16, 17)               ");
  Serial.println("=======================================================");
  Serial.println(">>> Pengujian otomatis dimulai dalam 3 detik...");
  Serial.println(">>> Pastikan roda mobil terangkat / ada ruang jalan!\n");

  delay(3000); // Jeda 3 detik waktu aman sebelum mulai
}

void loop() {
  // 1. MAJU LURUS
  Serial.println(">>> [1] MAJU (2 Detik)...");
  maju();
  delay(2000);

  // 2. BERHENTI SEMENTARA
  Serial.println(">>> STOP! (Jeda 1.5 Detik)");
  berhenti();
  delay(1500);

  // 3. MUNDUR LURUS
  Serial.println(">>> [2] MUNDUR (2 Detik)...");
  mundur();
  delay(2000);

  // 4. BERHENTI SEMENTARA SEBELUM MENGULANG
  Serial.println(">>> STOP! (Jeda 3 Detik sebelum mengulang siklus)\n");
  berhenti();
  delay(3000);
}
