// ====================================================================
// PROGRAM PENGUJIAN & DIAGNOSTIK SENSOR JARAK HC-SR04 (UNTUK ARDUINO)
// Board : Arduino Uno / Nano / Mega / Pro Mini (5V Logic)
// Modul : Sensor Ultrasonik HC-SR04
// Tujuan: 
//   1. Menguji fungsi pembacaan jarak (cm).
//   2. Mendiagnosis apakah sensor rusak, konslet, atau pin stuck HIGH/LOW
//      yang berpotensi menjadi penyebab Mappi32/ESP32 rusak/panas kemarin.
// ====================================================================

#include <Arduino.h>

// --- 1. KONFIGURASI PIN ARDUINO ---
// Hubungkan kabel sensor ke pin Arduino berikut:
//   VCC  -> Pin 5V Arduino
//   GND  -> Pin GND Arduino
//   TRIG -> Pin Digital D9 Arduino
//   ECHO -> Pin Digital D10 Arduino
const int PIN_TRIG = 9;
const int PIN_ECHO = 10;

// Konstanta Kecepatan Suara di Udara (343 m/s = 0.0343 cm/microsecond)
const float KONSTANTA_SUARA = 0.0343;

// Pewaktu
unsigned long lastMeasureTime = 0;
const unsigned long INTERVAL_UKUR_MS = 100; // Ukur tiap 100ms (10 Hz, responsif)

// Statistik Diagnostik
unsigned long totalSampel = 0;
unsigned long gagalTimeout = 0;
unsigned long berhasilUkur = 0;

void setup() {
  Serial.begin(9600); // Standar baud rate Arduino Uno / Nano
  while (!Serial) {
    ; // Tunggu Serial Monitor terbuka (khusus Leonardo/Micro)
  }

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Pastikan TRIG kondisi awal LOW
  digitalWrite(PIN_TRIG, LOW);
  delay(100);

  Serial.println(F("\n======================================================="));
  Serial.println(F("   DIAGNOSTIK & UJI KESEHATAN SENSOR JARAK HC-SR04     "));
  Serial.println(F("   Target Board: Arduino Uno / Nano (Tegangan 5V)      "));
  Serial.println(F("======================================================="));
  Serial.println(F("[PANDUAN KABEL KE ARDUINO]:"));
  Serial.println(F("  HC-SR04 VCC   --> Pin 5V Arduino"));
  Serial.println(F("  HC-SR04 GND   --> Pin GND Arduino"));
  Serial.println(F("  HC-SR04 TRIG  --> Pin D9 Arduino"));
  Serial.println(F("  HC-SR04 ECHO  --> Pin D10 Arduino"));
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("[UJI PERABAAN FISIK]:"));
  Serial.println(F("  Raba bagian belakang sensor HC-SR04 (terutama chip IC kecil)."));
  Serial.println(F("  Jika IC di belakang sensor terasa HANGAT/PANAS,"));
  Serial.println(F("  maka sensor tersebut KONSLET INTERNAL dan JANGAN DIPAKAI LAGI!"));
  Serial.println(F("-------------------------------------------------------\n"));
  delay(1000);
}

void loop() {
  if (millis() - lastMeasureTime >= INTERVAL_UKUR_MS) {
    lastMeasureTime = millis();
    totalSampel++;

    // 1. CEK ANOMALI PIN ECHO SEBELUM TRIGGER
    // Jika pin ECHO sudah bernilai HIGH sebelum diberi pulsa TRIG,
    // berarti jalur ECHO konslet/rusak (stuck HIGH).
    if (digitalRead(PIN_ECHO) == HIGH) {
      Serial.println(F("[BAHAYA] Pin ECHO STUCK HIGH! Sensor konslet internal atau kaki solderan menyatu!"));
      delay(500);
      return;
    }

    // 2. KIRIM PULSA TRIGGER 10 MICROSECONDS
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(4);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // 3. BACA DURASI PANTULAN DENGAN TIMEOUT 30ms (~5 meter)
    // pulseIn mengembalikan waktu tempuh pulsa HIGH dalam mikrodetik
    unsigned long durasiUs = pulseIn(PIN_ECHO, HIGH, 30000);

    // 4. ANALISIS HASIL DIAGNOSTIK
    if (durasiUs == 0) {
      // Timeout: Tidak ada pantulan kembali dalam 30ms
      gagalTimeout++;
      Serial.print(F("[#"));
      Serial.print(totalSampel);
      Serial.print(F("] "));
      Serial.print(F("Status: TIMEOUT / TIDAK TERDETEKSI "));
      Serial.print(F("| Durasi: 0 us "));
      Serial.print(F("| Keterangan: Objek terlalu jauh (>4m) atau transduser rusak"));
      Serial.println();
    } else {
      berhasilUkur++;
      // Hitung Jarak: Jarak = (Waktu * Kecepatan Suara) / 2
      float jarakCm = (durasiUs * KONSTANTA_SUARA) / 2.0;

      Serial.print(F("[#"));
      Serial.print(totalSampel);
      Serial.print(F("] "));
      Serial.print(F("Status: NORMAL OK "));
      Serial.print(F("| Jarak: "));
      Serial.print(jarakCm, 1);
      Serial.print(F(" cm "));
      Serial.print(F("| Pulsa: "));
      Serial.print(durasiUs);
      Serial.print(F(" us "));

      // Indikasi Visual Sederhana (Grafik Bar)
      Serial.print(F("| "));
      int barLength = constrain((int)(jarakCm / 5.0), 1, 20);
      for (int i = 0; i < barLength; i++) {
        Serial.print(F("="));
      }
      Serial.println(F(">"));
    }

    // Setiap 50 sampel (~5 detik), tampilkan ringkasan kesehatan sensor
    if (totalSampel % 50 == 0) {
      float persentaseSukses = ((float)berhasilUkur / (float)totalSampel) * 100.0;
      Serial.println(F("\n--- [RINGKASAN DIAGNOSTIK KESEHATAN SENSOR] ---"));
      Serial.print(F("Total Sampel   : ")); Serial.println(totalSampel);
      Serial.print(F("Sukses Terbaca : ")); Serial.print(berhasilUkur); 
      Serial.print(F(" (")); Serial.print(persentaseSukses, 1); Serial.println(F("%)"));
      Serial.print(F("Gagal Timeout  : ")); Serial.println(gagalTimeout);

      if (persentaseSukses >= 80.0) {
        Serial.println(F("KESIMPULAN     : SENSOR HC-SR04 DALAM KONDISI BAIK & SEHAT."));
      } else {
        Serial.println(F("KESIMPULAN     : SENSOR BERMASALAH (Sering timeout/putus-putus)."));
      }
      Serial.println(F("------------------------------------------------\n"));
    }
  }
}
