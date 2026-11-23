#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "Config.h"

// ====================================================================
// MODUL SENSOR (LINE FOLLOWER 5-CH, ULTRASONIK HC-SR04, ENCODER)
// File: Sensor.h
// ====================================================================

// --- STRUKTUR DATA PEMBACAAN GARIS ---
struct LineData {
  int b1; // Kiri Luar (S1)
  int b2; // Kiri Tengah (S2)
  int b3; // Tengah (S3)
  int b4; // Kanan Tengah (S4)
  int b5; // Kanan Luar (S5)
  int totalAktif; // Total sensor yang melihat garis hitam
  float error;    // Error posisi (-2.0 s/d +2.0)

  // Status Karakteristik Pola Jalur Sesuai Real Case
  bool isLurus;          // Pola [0 1 1 1 0] atau garis tengah tepat
  bool isTikunganKanan;  // Sensor kanan aktif tebal (S5=1, S4=1, S1=0)
  bool isTikunganKiri;   // Sensor kiri aktif tebal (S1=1, S2=1, S5=0)
  bool isPerempatan;     // Sensor terluar kanan dan kiri aktif bersamaan (S1=1, S5=1)
  bool isGarisHilang;    // Semua sensor membaca lantai putih/kardus [0 0 0 0 0]

  // Status Khusus Kelurusan Selesai Belok Kiri 90°:
  // Sesuai instruksi: sensor tengah (S3) DAN kanan sebelahnya sebelum akhir (S4)
  // Kondisi AND murni tanpa OR: [0 0 1 1 0]
  // Garis datang dari kanan menyapu S5 -> S4 -> S3. Begitu S3 & S4 aktif bersamaan (~85°),
  // robot langsung terkunci & rem aktif lawan arah sehingga inersia putar berhenti pas di 90° simetris,
  // TIDAK PERNAH over-rotate menyentuh S2 (< 90° anti menjorok ke kiri).
  bool isLurusBelokKiri() const {
    if (b1 != 0 || b5 != 0 || b2 != 0) return false;
    return (b3 == 1 && b4 == 1);
  }
  // Status Khusus Kelurusan Selesai Belok Kanan 90°:
  // Tetap deteksi 3 sensor dari tengah ke kanan (S3, S4, serta kombinasi tengah)
  // b1 dan b5 HARUS 0 (tidak berada di area persimpangan tebal).
  // Sensor tengah (S3) wajib aktif mengunci garis cabang timur: [0 0 1 1 0], [0 1 1 1 0], atau [0 0 1 0 0]
  bool isLurusBelokKanan() const {
    if (b1 != 0 || b5 != 0) return false;
    bool pola00110 = (b2 == 0 && b3 == 1 && b4 == 1);
    bool pola01110 = (b2 == 1 && b3 == 1 && b4 == 1);
    bool pola00100 = (b2 == 0 && b3 == 1 && b4 == 0);
    return (pola00110 || pola01110 || pola00100);
  }

  // Simpang Gelap Semua [1 1 1 1 1] (Simpang 3 Melintang atau Simpang 4)
  bool isGelapSemua() const {
    return (totalAktif >= AMBANG_SIMPANG_PENUH);
  }

  // Deteksi Cabang Simpang Kanan (S5 dan S4 aktif)
  bool isCabangKanan() const {
    return (b5 == 1 && b4 == 1);
  }

  // Deteksi Simpang Cabang Kanan Topologis Sejati:
  // Karakteristik T-Junction Cabang Kanan:
  // Jalur lurus tetap ada di depan (S3 = 1), DAN cabang kanan muncul (S4 = 1 dan S5 = 1)!
  // Total sensor aktif >= 3. Pola miring [0 0 0 1 1] di mana S3=0 akan DITOLAK MENTAH-MENTAH!
  bool isCabangKananTopologis() const {
    return (b3 == 1 && b4 == 1 && b5 == 1 && totalAktif >= 3);
  }
};

// Variabel Global Sensor
static volatile long          encoderTicks  = 0;
static volatile unsigned long encoderPulseA = 0;
static volatile unsigned long encoderPulseB = 0;
static volatile int64_t       lastIsrAUs    = 0;
static volatile int64_t       lastIsrBUs    = 0;
static portMUX_TYPE           encoderMux    = portMUX_INITIALIZER_UNLOCKED;
static float jarakValidTerakhir   = 999.0;
static float jarakSaatIni         = 999.0;
static bool statusHalangan        = false;
static unsigned long lastSonarTime     = 0;
static int arahGarisTerakhir           = 0; // -1 = Kiri, 0 = Lurus/Tengah, +1 = Kanan
static unsigned long lastLeftWingTime  = 0;
static unsigned long lastRightWingTime = 0;

// Reset timestamp sayap luar setiap selesai belok / transisi step agar tidak bocor
inline void resetWingTimestamps() {
  lastLeftWingTime  = 0;
  lastRightWingTime = 0;
}

// Interrupt ISR Fase A (GPIO 32): hitung pulsa tak-bertanda + quadrature arah
static void IRAM_ATTR bacaEncoderA_ISR() {
  int64_t nowUs = esp_timer_get_time();
  if (nowUs - lastIsrAUs < 100) return; // Filter EMI/bounce < 100us (aman s/d 10 kHz)
  lastIsrAUs = nowUs;

  int valB = gpio_get_level((gpio_num_t)pinEncoderB);
  portENTER_CRITICAL_ISR(&encoderMux);
  encoderPulseA++;
  if (valB == 1) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Interrupt ISR Fase B (GPIO 33): cadangan otomatis bila kabel Fase A lepas / tertukar
static void IRAM_ATTR bacaEncoderB_ISR() {
  int64_t nowUs = esp_timer_get_time();
  if (nowUs - lastIsrBUs < 100) return;
  lastIsrBUs = nowUs;

  portENTER_CRITICAL_ISR(&encoderMux);
  encoderPulseB++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

struct EncoderSnapshot {
  long          quadTicks;
  unsigned long pulseA;
  unsigned long pulseB;
};

// Ambil snapshot seluruh counter encoder secara atomik (thread/ISR-safe)
inline EncoderSnapshot bacaEncoderSnapshot() {
  EncoderSnapshot snap;
  portENTER_CRITICAL(&encoderMux);
  snap.quadTicks = encoderTicks;
  snap.pulseA    = encoderPulseA;
  snap.pulseB    = encoderPulseB;
  portEXIT_CRITICAL(&encoderMux);
  return snap;
}

// Pembacaan pulsa encoder quadrature secara atomik
inline long bacaEncoderTicks() {
  return bacaEncoderSnapshot().quadTicks;
}

// Reset seluruh pulsa encoder global ke 0
inline void resetEncoderTicks() {
  portENTER_CRITICAL(&encoderMux);
  encoderTicks  = 0;
  encoderPulseA = 0;
  encoderPulseB = 0;
  lastIsrAUs    = 0;
  lastIsrBUs    = 0;
  portEXIT_CRITICAL(&encoderMux);
}

// Hitung pulsa efektif gerak maju dalam 1 fase:
// - Jika quadrature sehat (|dQuad| >= 70% dA) -> pakai |dQuad|
// - Jika Fase B noisy/floating sehingga ++/-- saling menghapus -> otomatis pakai dA murni
// - Jika Fase A mati (dA == 0) namun Fase B aktif -> otomatis pakai dB
inline unsigned long hitungPulsaEfektif(const EncoderSnapshot& awal,
                                        const EncoderSnapshot& kini,
                                        bool& pakaiPulseAOnly,
                                        bool& pakaiPulseBCadangan) {
  long          dQuad = kini.quadTicks - awal.quadTicks;
  unsigned long dA    = kini.pulseA - awal.pulseA;
  unsigned long dB    = kini.pulseB - awal.pulseB;
  unsigned long absQ  = (unsigned long)labs(dQuad);

  pakaiPulseAOnly     = false;
  pakaiPulseBCadangan = false;

  if (dA > 0) {
    if (absQ * 10UL >= dA * 7UL) {
      return absQ;
    }
    pakaiPulseAOnly = true;
    return dA;
  }

  if (dB > 0) {
    pakaiPulseBCadangan = true;
    return dB;
  }

  return 0;
}

// Konversi selisih pulsa encoder (deltaTicks) ke jarak tempuh dalam cm.
// Menggunakan labs() agar kebal terhadap perbedaan polaritas kabel Fase A/B saat roda maju.
inline float ticksKeCm(long deltaTicks) {
  if (ENCODER_TICKS_PER_PUTARAN <= 0.0f) return 0.0f;
  return ((float)labs(deltaTicks) / ENCODER_TICKS_PER_PUTARAN) * KELILING_RODA_CM;
}

// Konversi target jarak (cm) ke perkiraan jumlah pulsa encoder (ticks)
inline long cmKeTicks(float cm) {
  if (KELILING_RODA_CM <= 0.0f) return 0;
  return (long)((cm / KELILING_RODA_CM) * ENCODER_TICKS_PER_PUTARAN + 0.5f);
}

// Inisialisasi Seluruh Sensor
inline void initSensors() {
  // Pin Encoder
  pinMode(pinEncoderA, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), bacaEncoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB), bacaEncoderB_ISR, RISING);

  // Pin Line Follower 5-Ch
  pinMode(pinLine1, INPUT);
  pinMode(pinLine2, INPUT);
  pinMode(pinLine3, INPUT);
  pinMode(pinLine4, INPUT);
  pinMode(pinLine5, INPUT);

  // Pin Ultrasonik HC-SR04 (Hanya diaktifkan jika GUNAKAN_SENSOR_JARAK = true)
  if (GUNAKAN_SENSOR_JARAK && pinTrig >= 0 && pinEcho >= 0) {
    pinMode(pinTrig, OUTPUT);
    pinMode(pinEcho, INPUT);
    digitalWrite(pinTrig, LOW);
  }
}

// Pembacaan 5 Sensor Garis (0 = Garis Hitam Aktif, 1 = Kardus Terang)
inline LineData bacaSensorGaris() {
  LineData data;

  // Baca status mentah (1 = Hitam, 0 = Terang/Putih)
  int raw1 = (digitalRead(pinLine1) == 1) ? 1 : 0;
  int raw2 = (digitalRead(pinLine2) == 1) ? 1 : 0;
  int raw3 = (digitalRead(pinLine3) == 1) ? 1 : 0;
  int raw4 = (digitalRead(pinLine4) == 1) ? 1 : 0;
  int raw5 = (digitalRead(pinLine5) == 1) ? 1 : 0;

  // Opsi Pembalikan Urutan jika Kabel S1 & S5 Tertukar
  if (!REVERSE_SENSOR_ORDER) {
    data.b1 = raw1; // S1 (Kiri Luar)
    data.b2 = raw2; // S2 (Kiri Tengah)
    data.b3 = raw3; // S3 (Tengah)
    data.b4 = raw4; // S4 (Kanan Tengah)
    data.b5 = raw5; // S5 (Kanan Luar)
  } else {
    data.b1 = raw5; // S5 jadi Kiri Luar
    data.b2 = raw4; // S4 jadi Kiri Tengah
    data.b3 = raw3; // S3 tetap Tengah
    data.b4 = raw2; // S2 jadi Kanan Tengah
    data.b5 = raw1; // S1 jadi Kanan Luar
  }

  data.totalAktif = data.b1 + data.b2 + data.b3 + data.b4 + data.b5;

  // 1. EVALUASI POLA KHUSUS SESUAI REAL CASE DI LAPANGAN
  // Saat lurus di atas lakban lebar: S2, S3, S4 aktif hitam [0 1 1 1 0]
  // Atau garis tipis tepat di tengah: S3 aktif, S2 dan S4 tidak aktif [0 0 1 0 0]
  // CATATAN PENTING: Pola [0 0 1 1 0] atau [0 1 1 0 0] BUKAN lurus tengah, tapi miring/offset!
  data.isLurus = (data.b1 == 0 && data.b5 == 0) &&
                 ((data.b2 == 1 && data.b3 == 1 && data.b4 == 1) ||
                  (data.b2 == 0 && data.b3 == 1 && data.b4 == 0));

  // Tikungan 90 Derajat ke KANAN: Sensor kanan luar & tengah aktif tebal, sensor kiri kosong
  data.isTikunganKanan = (data.b5 == 1 && (data.b4 == 1 || data.b3 == 1) && data.b1 == 0);

  // Tikungan 90 Derajat ke KIRI: Sensor kiri luar & tengah aktif tebal, sensor kanan kosong
  data.isTikunganKiri  = (data.b1 == 1 && (data.b2 == 1 || data.b3 == 1) && data.b5 == 0);

  // Toleransi Miring Perempatan (35ms):
  // Jika sayap kiri (S1) dan sayap kanan (S5) aktif bersamaan ATAU dalam selisih <= 35ms,
  // tetap dianggap perempatan sah (tahan posisi masuk miring, tetap kebal S-bend >150ms)
  unsigned long now = millis();
  bool leftWingNow  = (data.b1 == 1);
  bool rightWingNow = (data.b5 == 1);
  if (leftWingNow)  lastLeftWingTime  = now;
  if (rightWingNow) lastRightWingTime = now;

  bool crossWings = (leftWingNow && rightWingNow) ||
                    ((leftWingNow || rightWingNow) &&
                     (lastLeftWingTime > 0 && now - lastLeftWingTime <= TOLERANSI_MIRING_SIMPANG_MS) &&
                     (lastRightWingTime > 0 && now - lastRightWingTime <= TOLERANSI_MIRING_SIMPANG_MS));

  bool frontCenterAny = (data.b2 == 1 || data.b3 == 1 || data.b4 == 1);

  // Perempatan: Kedua sensor luar aktif (bersamaan atau toleransi miring 35ms) + garis tengah
  data.isPerempatan   = crossWings && (frontCenterAny || data.totalAktif >= 3);

  // Garis Hilang
  data.isGarisHilang   = (data.totalAktif == 0);

  // 2. HITUNG KOREKSI ERROR POSISI PUSAT MASSA (ANTI-GOYANG DI SIMPANG)
  // Prioritaskan sensor dalam (S2..S4) saat masih di garis agar sayap luar (S1/S5) di cabang T
  // tidak menarik hidung robot ke samping sebelum simpang terkonfirmasi.
  if (frontCenterAny) {
    int innerSum     = data.b2 + data.b3 + data.b4;
    float bobotDalam = (data.b2 * -1.0f) + (data.b3 * 0.0f) + (data.b4 * 1.0f);
    data.error       = bobotDalam / (float)innerSum;

    // Catat arah terakhir jika robot keluar jalur
    if (data.error > 0.3f) {
      arahGarisTerakhir = +1; // Terakhir cenderung di kanan
    } else if (data.error < -0.3f) {
      arahGarisTerakhir = -1; // Terakhir cenderung di kiri
    } else {
      arahGarisTerakhir = 0;  // Tepat di tengah
    }
  } else if (data.totalAktif > 0) {
    // Fallback ke sayap luar jika seluruh sensor dalam sudah lepas garis
    float bobotLuar = (data.b1 * -2.0f) + (data.b5 * 2.0f);
    data.error      = bobotLuar / (float)data.totalAktif;

    if (data.error > 0.3f) {
      arahGarisTerakhir = +1;
    } else if (data.error < -0.3f) {
      arahGarisTerakhir = -1;
    } else {
      arahGarisTerakhir = 0;
    }
  } else {
    // Garis hilang: Jangan paksa 0.0 agar tidak melaju buta lurus ke kardus kosong!
    data.error = 0.0f;
  }

  return data;
}

// Pembacaan Sensor Jarak HC-SR04 (Aman dari Titik Buta)
inline float bacaJarakUltrasonik() {
  if (!GUNAKAN_SENSOR_JARAK || pinTrig < 0 || pinEcho < 0) {
    return 999.0; // Sensor dinonaktifkan: jalan selalu dianggap bebas/kosong
  }

  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long durasi = pulseIn(pinEcho, HIGH, 6000); // Timeout 6ms (~1 meter, super responsif tanpa lag)

  if (durasi == 0) {
    return 999.0; // Jalan kosong di depan (timeout tidak ada halangan dekat)
  }

  float cm = durasi * 0.0343 / 2.0;
  if (cm < 3.0) {
    cm = 1.0; // Titik buta di bawah 3 cm
  }

  jarakValidTerakhir = cm;
  return cm;
}

// Update Radar Keamanan (Paced 60ms)
inline void updateRadarKeamanan() {
  if (!GUNAKAN_SENSOR_JARAK) {
    statusHalangan = false; // Selalu bebas halangan (mobil tidak akan berhenti)
    jarakSaatIni   = 999.0;
    return;
  }

  if (millis() - lastSonarTime >= 60) {
    lastSonarTime = millis();
    jarakSaatIni = bacaJarakUltrasonik();

    // Histeresis Keamanan Anti-Tabrak
    if (jarakSaatIni > 0 && jarakSaatIni <= JARAK_STOP_CM) {
      statusHalangan = true; // Rem Darurat
    } else if (jarakSaatIni > JARAK_LEPAS_CM) {
      statusHalangan = false; // Jalan Kembali
    }
  }
}
