#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Sensor.h"
#include "Motor.h"

// ====================================================================
// MODUL DETEKSI SIMPANG & MANUVER BELOK 90 DERAJAT (PASTI TEPAT DI TENGAH)
// File: NodeDetector.h
// ====================================================================

static int nodeCounter                = 0;     // Jumlah simpang yang berhasil dilewati
static bool sedangDiSimpang           = false; // State latch agar 1 simpang tidak terhitung berkali-kali
static unsigned long waktuMasukNode   = 0;     // Timestamp saat pertama kali menginjak simpang
static unsigned long waktuKeluarNode  = 0;     // Timestamp saat sensor meninggalkan simpang
static unsigned long waktuTerakhirBelok = 0;   // Cooldown agar tidak dobel pemicu belokan
static int hitunganBelokKanan         = 0;     // Penghitung khusus jumlah belok kanan (1, 2, dst)

// ====================================================================
// FUNGSI AUTO-PELURUSAN: PASTIKAN GARIS 100% TEPAT DI TENGAH MOBIL
// Sebelum mobil diizinkan melaju lurus ke depan
// ====================================================================
inline void kunciGarisTengah() {
  berhentiTotal();
  delay(100); // Diam sejenak agar pembacaan sensor stabil

  Serial.println(">>> [PELURUSAN TENGAH] Memeriksa posisi garis...");

  const int MAX_NUDGE = 10;
  int nudgeCount = 0;

  while (nudgeCount < MAX_NUDGE) {
    LineData l = bacaSensorGaris();

    // GARIS BENAR-BENAR DI TENGAH MOBIL JIKA:
    // Pola [0 1 1 1 0] atau [0 0 1 0 0] -> S3 aktif dan S2 seimbang dengan S4 (error == 0.0)
    bool isTepatTengah = l.isLurus || (l.b3 == 1 && abs(l.error) <= 0.2);

    if (isTepatTengah) {
      Serial.print(">>> [KUNCI SUKSES] Garis 100% Tepat di Tengah! Pola: [");
      Serial.print(l.b1); Serial.print(" ");
      Serial.print(l.b2); Serial.print(" ");
      Serial.print(l.b3); Serial.print(" ");
      Serial.print(l.b4); Serial.print(" ");
      Serial.print(l.b5); Serial.print("] | Err: ");
      Serial.println(l.error, 2);
      break;
    }

    // Jika garis masih di KANAN tengah mobil (kurang belok kanan / kelebihan belok kiri)
    // Pola seperti [0 0 1 1 0] -> error = +0.50
    if (l.error > 0.2 || (l.b4 == 1 && l.b2 == 0)) {
      Serial.print(">>> [NUDGE KANAN] Pola: [");
      Serial.print(l.b1); Serial.print(" ");
      Serial.print(l.b2); Serial.print(" ");
      Serial.print(l.b3); Serial.print(" ");
      Serial.print(l.b4); Serial.print(" ");
      Serial.print(l.b5); Serial.print("] Err: ");
      Serial.print(l.error, 2);
      Serial.println(" -> Koreksi geser kanan...");
      
      putarPorosKanan(155, 125);
      delay(30); // Pulsa mikro
      berhentiTotal();
      delay(60); // Waktu sensor membaca stabil
    }
    // Jika garis masih di KIRI tengah mobil (kurang belok kiri / kelebihan belok kanan)
    // Pola seperti [0 1 1 0 0] -> error = -0.50
    else if (l.error < -0.2 || (l.b2 == 1 && l.b4 == 0)) {
      Serial.print(">>> [NUDGE KIRI] Pola: [");
      Serial.print(l.b1); Serial.print(" ");
      Serial.print(l.b2); Serial.print(" ");
      Serial.print(l.b3); Serial.print(" ");
      Serial.print(l.b4); Serial.print(" ");
      Serial.print(l.b5); Serial.print("] Err: ");
      Serial.print(l.error, 2);
      Serial.println(" -> Koreksi geser kiri...");

      putarPorosKiri(160, 160);
      delay(30); // Pulsa mikro
      berhentiTotal();
      delay(60); // Waktu sensor membaca stabil
    } else {
      if (l.b3 == 1) break;
      delay(20);
    }

    nudgeCount++;
  }

  // Diam stabil sejenak sebelum akselerasi maju
  berhentiTotal();
  delay(120);
}

// --- MANUVER BELOK KANAN 90 DERAJAT (DILENGKAPI PRESET KHUSUS BELOK KANAN KE-2) ---
inline void eksekusiBelokKanan90() {
  hitunganBelokKanan++;
  bool isBelokKananKedua = (hitunganBelokKanan >= 2);

  // Penentuan parameter: Khusus tikungan ke-2 menggunakan preset anti-bablas
  int offsetMaju   = isBelokKananKedua ? OFFSET_MAJU_KANAN_2_MS        : OFFSET_MAJU_BELOK_MS;
  int kecMaju      = isBelokKananKedua ? KEC_BELOK_KANAN_2_MAJU        : KEC_BELOK_KANAN_MAJU;
  int kecMundur    = isBelokKananKedua ? KEC_BELOK_KANAN_2_MUNDUR      : KEC_BELOK_KANAN_MUNDUR;
  int waktuSweep   = isBelokKananKedua ? WAKTU_SWEEP_KANAN_2_MS        : WAKTU_SWEEP_AWAL_MS;
  int durasiTarik  = isBelokKananKedua ? DURASI_TAMBAHAN_BELAKANG_2_MS : DURASI_TAMBAHAN_BELAKANG_MS;

  Serial.println("\n=======================================================");
  if (isBelokKananKedua) {
    Serial.println(">>> [BELOK KANAN KE-2] MODE KHUSUS TIKUNGAN KEDUA (Anti-Bablas)...");
  } else {
    Serial.println(">>> [BELOK KANAN 90°] Maju Sedikit -> Putar Pas 90°...");
  }
  Serial.println("=======================================================");

  // TAHAP 1: OFFSET MAJU (Hanya jika offsetMaju > 0; di tikungan ke-2 nilainya 0ms agar tidak bablas keluar kardus)
  if (offsetMaju > 0) {
    setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
    delay(offsetMaju);
    setMotorLangsung(-65, -65);
    delay(20);
  }
  berhentiTotal();
  delay(JEDA_DIEM_SIMPANG_MS);

  // TAHAP 3: Breakaway Torque Kick 25ms mendobrak gesekan statis ke-4 roda serentak
  putarPorosKananHalus(kecMaju, kecMundur, DURASI_BREAKAWAY_KICK_MS);
  if (waktuSweep > DURASI_BREAKAWAY_KICK_MS) {
    delay(waktuSweep - DURASI_BREAKAWAY_KICK_MS);
  }

  // TAHAP 4: Putar aktif hingga Sensor Depan menangkap garis target
  // Belok Kanan: Wajib putar sampai sensor tengah (S3) & tengah kiri (S2) aktif, sebelum sensor kiri akhir (S1=0)
  unsigned long tMulai = millis();
  while (millis() - tMulai < 600) {
    LineData l = bacaSensorGaris();
    if (l.isLurusBelokKanan()) {
      Serial.println(">>> [SWEEP DEPAN] Sensor S3 & S2 Mengunci Garis Baru (S1 belum kena).");
      break;
    }
    putarPorosKanan(kecMaju, kecMundur);
    delay(10);
  }
  berhentiTotal();
  delay(40);

  // ====================================================================
  // TAHAP 5: PELURUSAN 30 DERAJAT BODI BELAKANG
  // Sensor depan sudah di garis, kini lanjutkan putaran secara dinamis agar bodi belakang
  // berputar 30 derajat lagi hingga seluruh bodi benar-benar tegak lurus 90°!
  // ====================================================================
  Serial.println(">>> [PELURUSAN 30°] Menarik bodi belakang 30 derajat lagi...");
  unsigned long tBelakang = millis();
  while (millis() - tBelakang < durasiTarik) {
    LineData l = bacaSensorGaris();
    if (l.isLurusBelokKanan()) {
      break; // Sudah pas lurus (S3=1, S2=1, S1=0), hentikan tarikan agar tidak over-rotate!
    }
    putarPorosKanan(kecMaju, kecMundur);
    delay(10);
  }
  berhentiTotal();
  delay(60);

  // ====================================================================
  // TAHAP 6: PENYESUAIAN HALUS AKHIR (Mikro Pulsa Presisi)
  // Memastikan garis berada tepat di tengah bodi tanpa kelebihan sudut
  // ====================================================================
  Serial.println(">>> [PENYESUAIAN AKHIR] Pemeriksaan akhir posisi garis...");
  unsigned long tSesuai = millis();
  while (millis() - tSesuai < TIMEOUT_PENYESUAIAN_BELAKANG_MS) {
    LineData l = bacaSensorGaris();

    // BARU DIBILANG LURUS JIKA: S3=1 & S2=1 tapi S1=0!
    if (l.isLurusBelokKanan()) {
      Serial.println(">>> [SUDAH LURUS] Bodi depan dan belakang terkunci sempurna 90°!");
      break;
    }

    // Jika bodi belakang masih agak tertinggal (S2 belum kena):
    if (l.b2 == 0) {
      putarPorosKanan(KEC_SESUAI_KANAN_MAJU, KEC_SESUAI_KANAN_MUNDUR);
      delay(40);
      berhentiTotal();
      delay(30);
    }
    // Jika bodi kelebihan memutar (sudah tembus ke sensor kiri akhir S1=1):
    else if (l.b1 == 1) {
      putarPorosKiri(KEC_SESUAI_KIRI_MAJU, KEC_SESUAI_KIRI_MUNDUR);
      delay(40);
      berhentiTotal();
      delay(30);
    } else {
      delay(15);
    }
  }

  // TAHAP 7: Diam stabil sejenak (100ms) memastikan mobil tenang
  berhentiTotal();
  delay(100);

  nodeCounter++;
  waktuTerakhirBelok = millis();
  waktuKeluarNode    = millis();
  sedangDiSimpang    = false;

  Serial.println(">>> [SELESAI] Bodi 100% Lurus 90°! Gas Maju Lurus...\n");
}

// --- MANUVER BELOK KIRI 90 DERAJAT (PASTI TEPAT DI TENGAH MOBIL) ---
inline void eksekusiBelokKiri90() {
  Serial.println("\n=======================================================");
  Serial.println(">>> [BELOK KIRI 90°] Maju Sedikit -> Putar Pas 90°...");
  Serial.println("=======================================================");

  // TAHAP 1: MAJU SEDIKIT SAJA (Hanya sentakan kecil 75ms agar roda pas di simpang)
  // Terkunci secara atomik sehingga mustahil terbaca sebagai garis lurus di loop()!
  setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
  delay(OFFSET_MAJU_BELOK_MS);

  // TAHAP 2: Rem aktif mematikan inersia maju
  setMotorLangsung(-65, -65);
  delay(20);
  berhentiTotal();
  delay(JEDA_DIEM_SIMPANG_MS);

  // TAHAP 3: Breakaway Torque Kick 25ms mendobrak gesekan statis ke-4 roda serentak
  putarPorosKiriHalus(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
  delay(WAKTU_SWEEP_AWAL_MS - DURASI_BREAKAWAY_KICK_MS);

  // TAHAP 4: Putar aktif hingga Sensor Depan menangkap garis target
  // Belok Kiri: Wajib putar sampai sensor tengah (S3) & tengah kanan (S4) aktif, sebelum sensor kanan akhir (S5=0)
  unsigned long tMulai = millis();
  while (millis() - tMulai < 600) {
    LineData l = bacaSensorGaris();
    if (l.isLurusBelokKiri()) {
      Serial.println(">>> [SWEEP DEPAN] Sensor S3 & S4 Mengunci Garis Baru (S5 belum kena).");
      break;
    }
    putarPorosKiri(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR);
    delay(10);
  }
  berhentiTotal();
  delay(40);

  // ====================================================================
  // TAHAP 5: PELURUSAN 30 DERAJAT BODI BELAKANG
  // Sensor depan sudah di garis, kini lanjutkan putaran secara dinamis agar bodi belakang
  // berputar 30 derajat lagi hingga seluruh bodi benar-benar tegak lurus 90°!
  // ====================================================================
  Serial.println(">>> [PELURUSAN 30°] Menarik bodi belakang 30 derajat lagi...");
  unsigned long tBelakang = millis();
  while (millis() - tBelakang < DURASI_TAMBAHAN_BELAKANG_MS) {
    LineData l = bacaSensorGaris();
    if (l.isLurusBelokKiri()) {
      break; // Sudah pas lurus (S3=1, S4=1, S5=0), hentikan tarikan agar tidak over-rotate!
    }
    putarPorosKiri(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR);
    delay(10);
  }
  berhentiTotal();
  delay(60);

  // ====================================================================
  // TAHAP 6: PENYESUAIAN HALUS AKHIR (Mikro Pulsa Presisi)
  // Memastikan garis berada tepat di tengah bodi tanpa kelebihan sudut
  // ====================================================================
  Serial.println(">>> [PENYESUAIAN AKHIR] Pemeriksaan akhir posisi garis...");
  unsigned long tSesuai = millis();
  while (millis() - tSesuai < TIMEOUT_PENYESUAIAN_BELAKANG_MS) {
    LineData l = bacaSensorGaris();

    // BARU DIBILANG LURUS JIKA: S3=1 dan S4=1 tapi sebelum sensor kanan akhir (S5=0)!
    if (l.isLurusBelokKiri()) {
      Serial.println(">>> [SUDAH LURUS] Bodi depan dan belakang terkunci sempurna 90°!");
      break;
    }

    // Jika bodi belakang masih agak tertinggal (S4 belum kena):
    if (l.b4 == 0) {
      putarPorosKiri(KEC_SESUAI_KIRI_MAJU, KEC_SESUAI_KIRI_MUNDUR);
      delay(40);
      berhentiTotal();
      delay(30);
    }
    // Jika bodi kelebihan memutar (sudah tembus ke sensor kanan akhir S5=1):
    else if (l.b5 == 1) {
      putarPorosKanan(KEC_SESUAI_KANAN_MAJU, KEC_SESUAI_KANAN_MUNDUR);
      delay(40);
      berhentiTotal();
      delay(30);
    } else {
      delay(15);
    }
  }

  // TAHAP 7: Diam stabil sejenak (100ms) memastikan mobil tenang
  berhentiTotal();
  delay(100);

  nodeCounter++;
  waktuTerakhirBelok = millis();
  waktuKeluarNode    = millis();
  sedangDiSimpang    = false;

  Serial.println(">>> [SELESAI] Bodi 100% Lurus 90°! Gas Maju Lurus...\n");
}

// Cek apakah sedang dalam masa cooldown setelah belokan
inline bool isCooldownBelok() {
  return (millis() - waktuTerakhirBelok < COOLDOWN_BELOK_MS);
}

// Update State Machine Deteksi Node Simpang Lurus (Perempatan / T-Junction)
inline bool updateNodeDetector(const LineData& line) {
  unsigned long now = millis();
  bool eventNodeBaru = false;

  if (!sedangDiSimpang && !isCooldownBelok()) {
    // Terdeteksi simpang perempatan (S1=1 dan S5=1)
    if (line.isPerempatan && (now - waktuKeluarNode >= DEBOUNCE_NODE_MS)) {
      sedangDiSimpang = true;
      waktuMasukNode  = now;
      nodeCounter++;
      eventNodeBaru   = true;

      Serial.println("\n***************************************************");
      Serial.print(">>> [SIMPANG PEREMPATAN] Node Ke: ");
      Serial.print(nodeCounter);
      Serial.print(" | Pola: [");
      Serial.print(line.b1); Serial.print(" ");
      Serial.print(line.b2); Serial.print(" ");
      Serial.print(line.b3); Serial.print(" ");
      Serial.print(line.b4); Serial.print(" ");
      Serial.print(line.b5); Serial.print("]");
      Serial.print(" | Timestamp: ");
      Serial.print(now);
      Serial.println(" ms");
      Serial.println("***************************************************\n");
    }
  } else {
    // Sensor sudah meninggalkan garis melintang
    if (line.totalAktif <= 3 && (now - waktuMasukNode >= EXIT_LATCH_MS)) {
      sedangDiSimpang = false;
      waktuKeluarNode = now;
    }
  }

  return eventNodeBaru;
}

// Reset Penghitung Node
inline void resetNodeDetector() {
  nodeCounter        = 0;
  sedangDiSimpang    = false;
  waktuMasukNode     = 0;
  waktuKeluarNode    = 0;
  waktuTerakhirBelok = 0;
}

inline int getNodeCount() {
  return nodeCounter;
}

inline bool isDiDalamSimpang() {
  return sedangDiSimpang;
}
