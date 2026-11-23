// ====================================================================
// PROGRAM PENGUJIAN & DIAGNOSTIK SENSOR ENCODER MOTOR (UNTUK ARDUINO)
// Board : Arduino Uno / Nano (ATmega328P)
// Modul : Sensor Rotary Encoder Motor DC (Hall Effect / Optical Quadrature)
// Tujuan: 
//   1. Menguji apakah sensor encoder motor (Fase A & Fase B) masih hidup / rusak.
//   2. Menghitung jumlah pulsa (ticks), arah putaran (maju/mundur), RPM, & jarak (cm).
//   3. Mendiagnosis apakah sinyal encoder stuck HIGH/LOW atau konslet.
// ====================================================================

#include <Arduino.h>

// --- 1. KONFIGURASI PIN ARDUINO & WARNA KABEL MOTOR ENCODER (6 KABEL) ---
// Tipe Standar JGB37-520 / JGA25-370 (Paling Umum di Indonesia):
//   1. Merah (Red)     : Motor Power (+) -> JANGAN COLOK KE ARDUINO! (Biar mengambang)
//   2. Hitam (Black)   : Encoder GND     -> Pin GND Arduino
//   3. Kuning (Yellow) : Encoder Fase A  -> Pin D2 Arduino (Hardware Interrupt INT0)
//   4. Hijau (Green)   : Encoder Fase B  -> Pin D3 Arduino
//   5. Biru (Blue)     : Encoder VCC     -> Pin 5V Arduino (Power sensor 3.3V - 5V)
//   6. Putih (White)   : Motor Power (-) -> JANGAN COLOK KE ARDUINO! (Biar mengambang)
//
// *CATATAN: Jika motor Anda tipe Pololu (Merah & Hitam = Motor):
//   Merah & Hitam = Motor (lepas), Biru = VCC, Hijau = GND, Kuning = Fase A, Putih = Fase B.
const int PIN_ENCODER_A = 2; // Interrupt INT0 (Kabel Kuning)
const int PIN_ENCODER_B = 3; // Input Fase B   (Kabel Hijau)

// --- 2. PARAMETER FISIK RODA MOBIL AGV ---
// Dari rumus.md: Diameter roda standar AGV = 6.5 cm
const float DIAMETER_RODA_CM  = 6.5; 
const float KELILING_RODA_CM  = 3.14159 * DIAMETER_RODA_CM; // ~20.42 cm per putaran penuh

// Estimasi Pulsa per Putaran Roda (PPR):
// Motor DC gearbox kuning/metal standar biasanya 11 PPR (sebelum gear) atau ~330-450 PPR (setelah gear).
// Nilai default 330 ticks per putaran (bisa disesuaikan setelah Anda putar 1 putaran penuh).
const float TICKS_PER_PUTARAN = 330.0; 

// --- 3. VARIABEL INTERRUPT ENCODER ---
volatile long  encoderCount  = 0;
volatile long  lastInterruptTime = 0;
volatile int   arahPutaran   = 0; // +1 = Maju, -1 = Mundur, 0 = Diam

// Pewaktu Telemetri Serial
unsigned long lastPrintTime  = 0;
const unsigned long INTERVAL_UPDATE_MS = 200; // Update tiap 200ms (5 Hz)
long lastEncoderForRPM = 0;

// --- 4. INTERRUPT SERVICE ROUTINE (ISR) ---
void bacaEncoderISR() {
  // Teknik Quadrature: baca status pin B saat pin A mengalami kenaikan (RISING)
  int valB = digitalRead(PIN_ENCODER_B);
  
  if (valB == LOW) {
    encoderCount++;
    arahPutaran = 1;  // Putar Maju / Searah Jarum Jam
  } else {
    encoderCount--;
    arahPutaran = -1; // Putar Mundur / Berlawanan Jarum Jam
  }
}

void setup() {
  Serial.begin(9600); // Baud rate standar Arduino Uno / Nano
  while (!Serial) {
    ; // Tunggu koneksi serial
  }

  // Gunakan INPUT_PULLUP untuk menjaga kestabilan sinyal open-collector Hall sensor
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);

  // Pasang hardware interrupt pada Pin D2 (RISING edge)
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), bacaEncoderISR, RISING);

  Serial.println(F("\n======================================================="));
  Serial.println(F("   DIAGNOSTIK & UJI SENSOR ENCODER MOTOR (ARDUINO)     "));
  Serial.println(F("======================================================="));
  Serial.println(F("[PANDUAN KABEL 6-PIN ENCODER MOTOR KE ARDUINO]:"));
  Serial.println(F("  1. BIRU   (VCC Sensor)  --> Pin 5V (atau 3.3V) Arduino"));
  Serial.println(F("  2. HITAM  (GND Sensor)  --> Pin GND Arduino"));
  Serial.println(F("  3. KUNING (Fase A)      --> Pin D2 Arduino (Hardware Interrupt)"));
  Serial.println(F("  4. HIJAU  (Fase B)      --> Pin D3 Arduino"));
  Serial.println(F("  5. MERAH & PUTIH (Motor)--> JANGAN DIHUBUNGKAN! (Biarkan lepas)"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("[CARA PENGUJIAN]:"));
  Serial.println(F("  1. Putar roda motor perlahan menggunakan tangan Anda."));
  Serial.println(F("  2. Perhatikan nilai 'Ticks' di bawah:"));
  Serial.println(F("     - Jika diputar MAJU: Ticks harus BERTAMBAH (+)."));
  Serial.println(F("     - Jika diputar MUNDUR: Ticks harus BERKURANG (-)."));
  Serial.println(F("  3. Jika roda diputar tapi Ticks TETAP 0: Sensor MATI/RUSAK."));
  Serial.println(F("-------------------------------------------------------\n"));
  delay(1500);
}

void loop() {
  if (millis() - lastPrintTime >= INTERVAL_UPDATE_MS) {
    unsigned long selangWaktuMs = millis() - lastPrintTime;
    lastPrintTime = millis();

    // Ambil nilai encoder secara atomik
    noInterrupts();
    long currentCount = encoderCount;
    int currentDir    = arahPutaran;
    interrupts();

    // Hitung Delta Ticks dalam 200ms terakhir untuk kecepatan & RPM
    long deltaTicks = currentCount - lastEncoderForRPM;
    lastEncoderForRPM = currentCount;

    // Hitung RPM & Kecepatan Linear (cm/detik)
    float rotasiPerDetik = ((float)deltaTicks / TICKS_PER_PUTARAN) / ((float)selangWaktuMs / 1000.0);
    float rpm = rotasiPerDetik * 60.0;
    float kecCmPerDetik = rotasiPerDetik * KELILING_RODA_CM;

    // Hitung Estimasi Total Jarak Tempuh (cm)
    float totalJarakCm = ((float)currentCount / TICKS_PER_PUTARAN) * KELILING_RODA_CM;

    // Baca status level logika mentah saat ini di pin A & B
    int levelA = digitalRead(PIN_ENCODER_A);
    int levelB = digitalRead(PIN_ENCODER_B);

    // Tampilkan Data ke Serial Monitor
    Serial.print(F("Pin A: ")); Serial.print(levelA);
    Serial.print(F(" | Pin B: ")); Serial.print(levelB);
    Serial.print(F(" | Ticks: ")); Serial.print(currentCount);
    Serial.print(F(" | Arah: "));
    if (deltaTicks > 0) {
      Serial.print(F("MAJU (CW)  "));
    } else if (deltaTicks < 0) {
      Serial.print(F("MUNDUR (CCW)"));
    } else {
      Serial.print(F("DIAM       "));
    }

    Serial.print(F(" | Kecepatan: "));
    Serial.print(kecCmPerDetik, 1);
    Serial.print(F(" cm/s ("));
    Serial.print(abs(rpm), 0);
    Serial.print(F(" RPM)"));

    Serial.print(F(" | Total Jarak: "));
    Serial.print(totalJarakCm, 1);
    Serial.print(F(" cm"));
    Serial.println();
  }
}
