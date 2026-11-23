// ====================================================================
// PROGRAM UJI OTOMATIS DRIVER MOTOR & ENCODER (ARDUINO UNO / NANO)
// TANPA INPUT SERIAL - LANGSUNG JALAN OTOMATIS BERURUTAN:
//   1. Motor KIRI MAJU   (2 Detik) -> STOP (1 Detik)
//   2. Motor KIRI MUNDUR (2 Detik) -> STOP (2 Detik)
//   3. Motor KANAN MAJU  (2 Detik) -> STOP (1 Detik)
//   4. Motor KANAN MUNDUR(2 Detik) -> STOP (3 Detik)
//   (Mengulang terus-menerus secara otomatis)
// ====================================================================

#include <Arduino.h>

// --- PIN PWM ARDUINO KE DRIVER BTS7960 ---
const int PIN_KIRI_MAJU    = 5;  // RPWM Driver Kiri
const int PIN_KIRI_MUNDUR  = 6;  // LPWM Driver Kiri
const int PIN_KANAN_MAJU   = 9;  // RPWM Driver Kanan
const int PIN_KANAN_MUNDUR = 10; // LPWM Driver Kanan

// --- PIN SENSOR ENCODER (Kabel Kuning & Hijau Roda Kiri) ---
const int PIN_ENCODER_A    = 2;  // Interrupt INT0 (Kabel Kuning)
const int PIN_ENCODER_B    = 3;  // Input Fase B   (Kabel Hijau)

// Variabel Encoder
volatile long encoderTicks = 0;

void isrEncoder() {
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// Fungsi Rem / Berhenti Semua Motor
void remTotal() {
  analogWrite(PIN_KIRI_MAJU, 0);
  analogWrite(PIN_KIRI_MUNDUR, 0);
  analogWrite(PIN_KANAN_MAJU, 0);
  analogWrite(PIN_KANAN_MUNDUR, 0);
}

void setup() {
  Serial.begin(9600);

  // Inisialisasi Pin Output Driver
  pinMode(PIN_KIRI_MAJU, OUTPUT);
  pinMode(PIN_KIRI_MUNDUR, OUTPUT);
  pinMode(PIN_KANAN_MAJU, OUTPUT);
  pinMode(PIN_KANAN_MUNDUR, OUTPUT);
  remTotal();

  // Inisialisasi Pin Encoder
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), isrEncoder, RISING);

  Serial.println(F("\n======================================================="));
  Serial.println(F("   PROGRAM UJI OTOMATIS DRIVER MOTOR (KIRI -> KANAN)   "));
  Serial.println(F("======================================================="));
  Serial.println(F(" [SAMBUNGAN PIN ARDUINO]:"));
  Serial.println(F("   - Driver Kiri  : RPWM -> D5 | LPWM -> D6"));
  Serial.println(F("   - Driver Kanan : RPWM -> D9 | LPWM -> D10"));
  Serial.println(F("   - R_EN & L_EN  : Jumper ke 5V Arduino"));
  Serial.println(F("   - VCC Driver   : 5V Arduino"));
  Serial.println(F("   - GND Driver   : GND Arduino & Negatif Baterai 12V"));
  Serial.println(F("   - Encoder Kiri : Kuning -> D2 | Hijau -> D3"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F(">>> Uji coba akan dimulai otomatis dalam 3 detik..."));
  Serial.println(F(">>> Pastikan roda robot terangkat dari meja!"));
  Serial.println(F("=======================================================\n"));

  delay(3000); // Waktu aman sebelum motor mulai berputar
}

void loop() {
  // -------------------------------------------------------------
  // TAHAP 1: UJI DRIVER KIRI
  // -------------------------------------------------------------
  Serial.println(F(">>> [1/4] MOTOR KIRI MAJU (PWM 120, 2 Detik)..."));
  long startTicks = encoderTicks;
  analogWrite(PIN_KIRI_MAJU, 120);
  analogWrite(PIN_KIRI_MUNDUR, 0);
  delay(2000);

  remTotal();
  long ticksMaju = encoderTicks - startTicks;
  Serial.print(F("    -> STOP! Encoder Ticks: "));
  Serial.println(ticksMaju);
  delay(1000); // Jeda 1 detik

  Serial.println(F(">>> [2/4] MOTOR KIRI MUNDUR (PWM 120, 2 Detik)..."));
  startTicks = encoderTicks;
  analogWrite(PIN_KIRI_MAJU, 0);
  analogWrite(PIN_KIRI_MUNDUR, 120);
  delay(2000);

  remTotal();
  long ticksMundur = encoderTicks - startTicks;
  Serial.print(F("    -> STOP! Encoder Ticks: "));
  Serial.println(ticksMundur);
  Serial.println(F(">>> Pengujian Sisi KIRI Selesai."));
  Serial.println(F("-------------------------------------------------------"));
  delay(2000); // Jeda 2 detik sebelum pindah ke sisi kanan

  // -------------------------------------------------------------
  // TAHAP 2: UJI DRIVER KANAN
  // -------------------------------------------------------------
  Serial.println(F(">>> [3/4] MOTOR KANAN MAJU (PWM 120, 2 Detik)..."));
  analogWrite(PIN_KANAN_MAJU, 120);
  analogWrite(PIN_KANAN_MUNDUR, 0);
  delay(2000);

  remTotal();
  Serial.println(F("    -> STOP!"));
  delay(1000); // Jeda 1 detik

  Serial.println(F(">>> [4/4] MOTOR KANAN MUNDUR (PWM 120, 2 Detik)..."));
  analogWrite(PIN_KANAN_MAJU, 0);
  analogWrite(PIN_KANAN_MUNDUR, 120);
  delay(2000);

  remTotal();
  Serial.println(F("    -> STOP!"));
  Serial.println(F(">>> Pengujian Sisi KANAN Selesai."));
  Serial.println(F("======================================================="));
  Serial.println(F(">>> Siklus Selesai! Mengulang lagi dalam 3 detik...\n"));
  delay(3000); // Jeda 3 detik sebelum mengulang siklus
}
