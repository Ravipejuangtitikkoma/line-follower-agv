#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensor.h"
#include "NodeDetector.h"
#include "RoutePlans.h"

// ====================================================================
// MODUL HIERARCHICAL FINITE STATE MACHINE & DATA-DRIVEN PLAN ENGINE
// File: NavigationFSM.h
// Deskripsi:
//   - Layer 1 (MisiStatus): STANDBY, JALAN_RUTE_REAL, EMERGENCY_STOP
//   - Layer 2 (StepPhase) : Eksekutor 4-Fase Generik membaca tabel
//                           RUTE_REAL_PP[] dari RoutePlans.h
// ====================================================================

// --- 1. LAYER 1: STATUS MISI GLOBAL (KENDALI LORA / OPERATOR) ---
enum MisiStatus {
  MISI_STANDBY,          // Diam di tempat, menunggu sinyal LoRa / Serial
  MISI_JALAN_RUTE_REAL,  // Misi Aktif: Menempuh rute peta riil (START -> A -> B -> START)
  MISI_EMERGENCY_STOP    // Berhenti darurat karena rintangan / tombol stop
};

// --- 2. LAYER 2: 4 FASE EKSEKUSI PER STEP DALAM PLAN ENGINE ---
enum StepPhase : uint8_t {
  PHASE_FOLLOW,    // Melacak garis PD + memantau trigger simpang step aktif
  PHASE_PRE_MOVE,  // Menjalankan offset maju (59ms) / penetrasi PD (495ms) + rem aktif
  PHASE_PAUSE,     // Berhenti diam sesuai durasi jeda (bisa di-bypass LANJUT / PULANG)
  PHASE_ACTION     // Menjalankan aksi utama (Belok Kiri/Kanan, Cross, U-Turn 180, Stop)
};

// Variabel Global Plan Engine
static MisiStatus statusMisiSaatIni     = MISI_JALAN_RUTE_REAL; // Default langsung jalan saat uji coba
static int        stepIdxSaatIni        = 0;
static StepPhase  faseSaatIni           = PHASE_FOLLOW;
static unsigned long waktuMasukFase     = 0;
static float      errSebelumnyaPD       = 0.0f;
static bool       sudahLepasGarisLama   = false;
static bool       sudahAutoAlignStart   = false;
static int        konfirmasiCabangKanan = 0;
static long       encoderTicksAwalFase  = 0;
static unsigned long lastWarnEncoderMs  = 0;

inline long getDeltaTicksFase() {
  return bacaEncoderTicks() - encoderTicksAwalFase;
}

inline float getJarakFaseCm() {
  return ticksKeCm(getDeltaTicksFase());
}

// Helper nama fase untuk log & telemetri
inline const char* namaFase(StepPhase fase) {
  switch (fase) {
    case PHASE_FOLLOW:   return "FOLLOW";
    case PHASE_PRE_MOVE: return "PRE_MOVE";
    case PHASE_PAUSE:    return "PAUSE";
    case PHASE_ACTION:   return "ACTION";
    default:             return "IDLE";
  }
}

// Helper transisi fase dalam step aktif
inline void gantiFase(StepPhase faseBaru, const char* alasan) {
  faseSaatIni           = faseBaru;
  waktuMasukFase        = millis();
  konfirmasiCabangKanan = 0;
  sudahLepasGarisLama   = false;

  // Jangan matikan motor saat masuk PHASE_PRE_MOVE agar momentum offset/penetrasi mulus
  if (faseBaru != PHASE_PRE_MOVE) {
    berhentiTotal();
  }

  if (faseBaru == PHASE_FOLLOW) {
    resetWingTimestamps();
    errSebelumnyaPD      = 0.0f;
    encoderTicksAwalFase = bacaEncoderTicks();
    lastWarnEncoderMs    = millis();
  }

  const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL)
                        ? RUTE_REAL_PP[stepIdxSaatIni].label
                        : "SELESAI";

  Serial.print("\n>>> [PLAN STEP ");
  Serial.print(stepIdxSaatIni);
  Serial.print("/");
  Serial.print(JUMLAH_STEP_RUTE_REAL - 1);
  Serial.print(" : ");
  Serial.print(label);
  Serial.print(" | ");
  Serial.print(namaFase(faseBaru));
  Serial.print("] Alasan: ");
  Serial.println(alasan);

  if (faseBaru == PHASE_FOLLOW &&
      stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL &&
      RUTE_REAL_PP[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
    float targetCm = RUTE_REAL_PP[stepIdxSaatIni].targetJarakCm;
    Serial.print(">>> [ENCODER START] Mulai hitung jarak -> Target: ");
    Serial.print(targetCm, 1);
    Serial.print(" cm (~");
    Serial.print(cmKeTicks(targetCm));
    Serial.println(" ticks)");
  }
}

// Helper berpindah ke step berikutnya di tabel PlanStep
inline void majuKeStepBerikutnya(const char* alasan) {
  stepIdxSaatIni++;
  if (stepIdxSaatIni >= JUMLAH_STEP_RUTE_REAL) {
    berhentiTotal();
    statusMisiSaatIni = MISI_STANDBY;
    Serial.println(">>> [PLAN ENGINE] Seluruh step pada rute telah selesai.");
    return;
  }

  if (RUTE_REAL_PP[stepIdxSaatIni].trigger == TRIG_IMMEDIATE) {
    gantiFase(PHASE_ACTION, alasan);
  } else {
    gantiFase(PHASE_FOLLOW, alasan);
  }
}

// Fungsi helper kontrol PD halus untuk pelacakan garis lurus & Hentakan Pemulihan (Anti-Bablas)
inline void jalankanTrackingPD(const LineData& line, int kecBase = KEC_BASE) {
  // -------------------------------------------------------------
  // KASUS 1: GARIS HILANG LEPAS TOTAL [0 0 0 0 0] (EMERGENCY HENTAKAN RECOVERY)
  // Faktor X melempar mobil keluar garis lurus. Hentakan instan bertenaga penuh!
  // -------------------------------------------------------------
  if (line.isGarisHilang) {
    if (arahGarisTerakhir == -1) {
      // Faktor X melempar mobil ke KANAN (garis ada di KIRI):
      // Hentakan penuh kedua roda KANAN MAJU (175 PWM) & roda KIRI MUNDUR (-85 PWM):
      setMotorLangsung(-PWM_RECOVERY_MUNDUR, PWM_RECOVERY_MAJU);
    } else if (arahGarisTerakhir == +1) {
      // Faktor X melempar mobil ke KIRI (garis ada di KANAN):
      // Hentakan penuh kedua roda KIRI MAJU (175 PWM) & roda KANAN MUNDUR (-85 PWM):
      setMotorLangsung(PWM_RECOVERY_MAJU, -PWM_RECOVERY_MUNDUR);
    } else {
      setMotorLangsung(0, 0);
    }
    return;
  }

  // -------------------------------------------------------------
  // KASUS 2: FAKTOR X MEMBUAT MOBIL MELENCENG KE KANAN (Garis di Kiri: S1 atau S2)
  // Roda kanan harus menghentak agar mobil masuk kembali ke jalur lurus!
  // -------------------------------------------------------------
  if (line.b1 == 1 && line.b3 == 0) {
    // Garis mepet di S1 (Kiri Luar) -> Hentakan kuat kedua roda kanan (165 PWM), kiri rem/mundur (-60 PWM)
    setMotorLangsung(-PWM_TEPI_MUNDUR, PWM_TEPI_MAJU);
    errSebelumnyaPD = -2.0f;
    return;
  }
  if (line.b2 == 1 && line.b3 == 0 && line.b4 == 0) {
    // Garis bergeser ke S2 (Kiri Tengah) -> Hentakan responsif kedua roda kanan (145 PWM), kiri pelan (15 PWM)
    setMotorLangsung(15, 145);
    errSebelumnyaPD = -1.0f;
    return;
  }

  // -------------------------------------------------------------
  // KASUS 3: FAKTOR X MEMBUAT MOBIL MELENCENG KE KIRI (Garis di Kanan: S5 atau S4)
  // Roda kiri harus menghentak agar mobil masuk kembali ke jalur lurus!
  // -------------------------------------------------------------
  if (line.b5 == 1 && line.b3 == 0) {
    // Garis mepet di S5 (Kanan Luar) -> Hentakan kuat kedua roda kiri (165 PWM), kanan rem/mundur (-60 PWM)
    setMotorLangsung(PWM_TEPI_MAJU, -PWM_TEPI_MUNDUR);
    errSebelumnyaPD = 2.0f;
    return;
  }
  if (line.b4 == 1 && line.b3 == 0 && line.b2 == 0) {
    // Garis bergeser ke S4 (Kanan Tengah) -> Hentakan responsif kedua roda kiri (145 PWM), kanan pelan (15 PWM)
    setMotorLangsung(145, 15);
    errSebelumnyaPD = 1.0f;
    return;
  }

  // -------------------------------------------------------------
  // KASUS 4: KONDISI LURUS DI JALUR (TUNING ASLI USER YANG SUDAH PAS)
  // Sensor tengah S3 membaca garis, mobil melaju mulus dengan tuning asli 100%
  // -------------------------------------------------------------
  float errorSaatIni = line.error;
  if (line.isLurus) {
    errorSaatIni = 0.0f;
  }

  float P = Kp * errorSaatIni;
  float D = Kd * (errorSaatIni - errSebelumnyaPD);
  float koreksi = P + D;
  errSebelumnyaPD = errorSaatIni;

  int vKiri  = kecBase + (int)koreksi + TRIM_KIRI;
  int vKanan = kecBase - (int)koreksi + TRIM_KANAN;
  setKecepatanMotor(vKiri, vKanan);
}

// ====================================================================
// EVALUATOR TRIGGER SIMPANG (TERMASUK LOCKOUT & FILTER PERSISTENSI)
// ====================================================================
inline bool evaluasiTrigger(const PlanStep& step, const LineData& line, unsigned long durasiDiFase) {
  if (step.trigger == TRIG_IMMEDIATE) {
    return true;
  }

  // TA Lockout Timer: kebal deteksi palsu sebelum jarak/waktu minimal terlampaui
  if (durasiDiFase < (unsigned long)step.lockoutMs) {
    konfirmasiCabangKanan = 0;
    return false;
  }

  switch (step.trigger) {
    case TRIG_T_KIRI:
      return (line.isGelapSemua() || (line.b1 == 1 && line.b2 == 1 && line.b3 == 1));

    case TRIG_CABANG_KANAN: {
      // Invarian Topologis: wajib ada garis lurus (S3=1 atau totalAktif>=3)
      // Pola oleng [0 0 0 1 1] ditolak 100% dan diperkuat 2-sample persistence filter
      bool isSimpangKananValid = (line.b5 == 1 && line.b4 == 1) &&
                                 (line.b3 == 1 || line.totalAktif >= 3 || line.isGelapSemua());
      if (isSimpangKananValid) {
        konfirmasiCabangKanan++;
      } else {
        konfirmasiCabangKanan = 0;
      }

      if (konfirmasiCabangKanan >= PERSISTENSI_CABANG_KANAN) {
        konfirmasiCabangKanan = 0;
        return true;
      }
      return false;
    }

    case TRIG_PEREMPATAN:
      return (line.isGelapSemua() || line.isPerempatan);

    case TRIG_ENCODER_JARAK: {
      if (step.targetJarakCm <= 0.0f) return false;
      long deltaTicks = getDeltaTicksFase();
      float jarakCm   = ticksKeCm(deltaTicks);

      // Peringatan jika mobil sudah berjalan > 1.5 detik namun encoder belum membaca pulsa
      if (durasiDiFase >= 1500 && deltaTicks == 0 && (millis() - lastWarnEncoderMs >= 1000)) {
        lastWarnEncoderMs = millis();
        Serial.println("[PERINGATAN ENCODER] Pulsa masih 0! Cek kabel encoder roda kiri (GPIO 32/33) & VCC sensor.");
      }

      if (jarakCm >= step.targetJarakCm) {
        Serial.print("\n>>> [ENCODER TARGET TERCAPAI] Jarak: ");
        Serial.print(jarakCm, 1);
        Serial.print(" / ");
        Serial.print(step.targetJarakCm, 1);
        Serial.print(" cm | Delta Ticks: ");
        Serial.println(deltaTicks);
        return true;
      }
      return false;
    }

    default:
      return false;
  }
}

// ====================================================================
// PROSEDUR ZERO-YAW AUTO-ALIGNMENT DI TITIK START (STEP 0)
// Mengembalikan true selama kalibrasi mikro awal masih berlangsung
// ====================================================================
inline bool eksekusiAutoAlignStart(const LineData& line, unsigned long durasiDiFase) {
  if (sudahAutoAlignStart) {
    return false;
  }

  if (line.isLurus || (line.b3 == 1 && line.b1 == 0 && line.b5 == 0 && abs(line.error) < 0.35f)) {
    sudahAutoAlignStart = true;
    berhentiTotal();
    delay(20);
    return false;
  }

  if (durasiDiFase < (unsigned long)WAKTU_AUTO_ALIGN_MAX_MS) {
    int pwmAktif = (durasiDiFase < (unsigned long)DURASI_AUTO_ALIGN_KICK_MS)
                     ? PWM_AUTO_ALIGN_KICK
                     : PWM_AUTO_ALIGN;

    if (line.error > 0.3f) {
      // Mencong ke kiri (garis di kanan) -> putar poros ke kanan
      putarPorosKanan(pwmAktif, pwmAktif);
    } else if (line.error < -0.3f) {
      // Mencong ke kanan (garis di kiri) -> putar poros ke kiri
      putarPorosKiri(pwmAktif, pwmAktif + 8);
    } else {
      // Sensor S3 sudah mengunci garis -> rem aktif lawan arah mikro
      if (line.error < 0) putarPorosKanan(130, 130);
      else putarPorosKiri(130, 130);
      delay(25);
      berhentiTotal();
      delay(20);
      sudahAutoAlignStart = true;
      return false;
    }
    return true;
  }

  sudahAutoAlignStart = true;
  berhentiTotal();
  delay(20);
  return false;
}

// ====================================================================
// EKSEKUTOR AKSI BELOK KIRI 90° (MODE BERANGKAT & MODE PULANG)
// ====================================================================
inline void eksekusiAksiBelokKiri(const LineData& line, unsigned long durasiDiFase, bool modePulang) {
  int kecMaju         = modePulang ? KEC_BELOK_KIRI_PULANG_MAJU   : KEC_BELOK_KIRI_MAJU;
  int kecMundur       = modePulang ? KEC_BELOK_KIRI_PULANG_MUNDUR : KEC_BELOK_KIRI_MUNDUR;
  int pwmKick         = modePulang ? PWM_BREAKAWAY_KIRI_PULANG    : PWM_BREAKAWAY_KICK;
  int kecSesuaiMaju   = modePulang ? KEC_SESUAI_KIRI_PULANG_MAJU  : KEC_SESUAI_KIRI_MAJU;
  int kecSesuaiMundur = modePulang ? KEC_SESUAI_KIRI_PULANG_MUNDUR: KEC_SESUAI_KIRI_MUNDUR;
  int pwmRem          = modePulang ? 145 : 140;

  if (durasiDiFase < (unsigned long)DURASI_RAMP_MS) {
    putarPorosKiriHalus(kecMaju, kecMundur, DURASI_BREAKAWAY_KICK_MS, pwmKick);
  } else {
    if (line.b4 == 1 && durasiDiFase >= (unsigned long)WAKTU_SWEEP_AWAL_MS) {
      putarPorosKiri(kecSesuaiMaju, kecSesuaiMundur);
    } else {
      putarPorosKiri(kecMaju, kecMundur);
    }
  }

  if (durasiDiFase >= (unsigned long)WAKTU_SWEEP_AWAL_MS) {
    if (line.isLurusBelokKiri() || (line.b3 == 1 && line.b4 == 1) || (line.b3 == 1 && line.b2 == 0)) {
      // Rem aktif lawan arah mematikan inersia putar tepat di 90° simetris
      putarPorosKanan(pwmRem, pwmRem);
      delay(DURASI_REM_BELOK_MS);
      berhentiTotal();
      delay(40);

      if (!modePulang && DURASI_MENGGELIR_KIRI_MS > 0) {
        unsigned long tG = millis();
        while (millis() - tG < DURASI_MENGGELIR_KIRI_MS) {
          LineData lg = bacaSensorGaris();
          if (lg.isLurusBelokKiri()) break;
          putarPorosKiri(kecSesuaiMaju, kecSesuaiMundur);
          delay(10);
        }
        putarPorosKanan(pwmRem, pwmRem);
        delay(20);
        berhentiTotal();
        delay(40);
      }

      majuKeStepBerikutnya("Target S3 & S4 Terkunci -> Rem Aktif Kanan Berhasil");
      return;
    }
  }

  if (durasiDiFase >= TIMEOUT_BELOK_MS) {
    berhentiTotal();
    delay(40);
    majuKeStepBerikutnya("Timeout Belok Kiri -> Lanjut Step Berikutnya");
  }
}

// ====================================================================
// EKSEKUTOR AKSI BELOK KANAN 90° (2-PHASE CLOSED-LOOP + STRICT BINARY LATCH)
// ====================================================================
inline void eksekusiAksiBelokKanan(const LineData& line, unsigned long durasiDiFase) {
  if (durasiDiFase < (unsigned long)DURASI_RAMP_MS) {
    putarPorosKananHalus(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
  } else {
    // Fase Radar: perlambat saat mendekati garis cabang 90° setelah lepas sudut lama
    if (sudahLepasGarisLama &&
        durasiDiFase >= (unsigned long)WAKTU_ABAIKAN_SENSOR_KANAN_MS &&
        (line.b1 == 1 || line.b2 == 1)) {
      putarPorosKanan(KEC_BELOK_KANAN_CARI, KEC_BELOK_KANAN_CARI + 6);
    } else {
      putarPorosKanan(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR);
    }
  }

  // FASE A: Pastikan bodi keluar dari sudut garis lama minimal 200ms / timeout 260ms
  if (!sudahLepasGarisLama) {
    if ((durasiDiFase >= (unsigned long)WAKTU_ABAIKAN_SENSOR_KANAN_MS && (line.totalAktif == 0 || line.isGarisHilang)) ||
        (durasiDiFase >= (unsigned long)WAKTU_LEPAS_SUDUT_MIN_MS)) {
      sudahLepasGarisLama = true;
    }
  }

  // FASE B: Strict Binary Latch mengunci garis cabang 90°
  if (sudahLepasGarisLama && durasiDiFase >= (unsigned long)WAKTU_ABAIKAN_SENSOR_KANAN_MS) {
    if (line.isLurusBelokKanan()) {
      putarPorosKiri(150, 150);
      delay(DURASI_REM_BELOK_MS);
      berhentiTotal();
      delay(40);

      if (DURASI_MENGGELIR_KANAN_MS > 0) {
        unsigned long tG = millis();
        while (millis() - tG < DURASI_MENGGELIR_KANAN_MS) {
          LineData lg = bacaSensorGaris();
          if (lg.isLurusBelokKanan()) break;
          putarPorosKanan(KEC_SESUAI_KANAN_MAJU, KEC_SESUAI_KANAN_MUNDUR);
          delay(10);
        }
        putarPorosKiri(150, 150);
        delay(20);
        berhentiTotal();
        delay(40);
      }

      majuKeStepBerikutnya("Target 90° Kanan Terkunci Presisi (Strict Binary)");
      return;
    }
  }

  if (durasiDiFase >= TIMEOUT_BELOK_MS) {
    berhentiTotal();
    delay(40);
    majuKeStepBerikutnya("Timeout Belok Kanan -> Lanjut Step Berikutnya");
  }
}

// ====================================================================
// EKSEKUTOR AKSI LINTAS LURUS MELAMPAUI SIMPANG 4
// ====================================================================
inline void eksekusiAksiLintasLurus(const LineData& line, unsigned long durasiDiFase) {
  if (durasiDiFase < 450) {
    setKecepatanMotor(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
  } else {
    jalankanTrackingPD(line, KEC_BASE);
  }

  if (durasiDiFase >= 450 && !line.isGelapSemua()) {
    majuKeStepBerikutnya("Simpang 4 Sukses Dilewati Lurus");
  }
}

// ====================================================================
// EKSEKUTOR AKSI PUTAR BALIK 180° HALUS DI TITIK B
// ====================================================================
inline void eksekusiAksiUTurn180(const LineData& line, unsigned long durasiDiFase) {
  if (durasiDiFase < (unsigned long)OFFSET_MAJU_PUTAR_180_MS) {
    jalankanTrackingPD(line, KEC_BASE);
  } else if (durasiDiFase < (unsigned long)(OFFSET_MAJU_PUTAR_180_MS + JEDA_REDAM_180_MS)) {
    berhentiTotal();
  } else {
    unsigned long durasiPutar = durasiDiFase - (OFFSET_MAJU_PUTAR_180_MS + JEDA_REDAM_180_MS);

    if (durasiPutar < (unsigned long)DURASI_RAMP_MS) {
      putarPorosKananHalus(KEC_PUTAR_180_MAJU, KEC_PUTAR_180_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
    } else {
      if (durasiPutar >= (unsigned long)WAKTU_ABAIKAN_180_MS && (line.b1 == 1 || line.b2 == 1)) {
        putarPorosKanan(KEC_PUTAR_180_CARI, KEC_PUTAR_180_CARI + 4);
      } else {
        putarPorosKanan(KEC_PUTAR_180_MAJU, KEC_PUTAR_180_MUNDUR);
      }
    }

    if (durasiPutar >= (unsigned long)WAKTU_ABAIKAN_180_MS) {
      if ((line.b3 == 1 && line.b1 == 0 && line.b5 == 0) || line.isLurus) {
        putarPorosKiri(PWM_REM_180, PWM_REM_180);
        delay(25);
        berhentiTotal();
        delay(40);
        majuKeStepBerikutnya("U-Turn 180° Halus Berhasil Kunci Garis -> Maju Pulang");
        return;
      }
    }

    if (durasiPutar >= TIMEOUT_PUTAR_180_MS) {
      berhentiTotal();
      delay(40);
      majuKeStepBerikutnya("Timeout U-Turn 180° -> Lanjut Maju Pulang");
    }
  }
}

// ====================================================================
// PROSES FASE PRE-MOVE (OFFSET LANGSUNG ATAU PENETRASI PD + REM AKTIF)
// ====================================================================
inline void prosesFasePreMove(const PlanStep& step, const LineData& line, unsigned long durasiDiFase) {
  if (step.preMoveUsePD) {
    jalankanTrackingPD(line, KEC_BASE);
    if (durasiDiFase >= (unsigned long)step.preMoveMs) {
      if (step.brakeMs > 0) {
        setMotorLangsung(-(int)step.brakePwm, -(int)step.brakePwm);
        delay(step.brakeMs);
      }
      berhentiTotal();
      if (step.pauseMs > 0) {
        gantiFase(PHASE_PAUSE, "Penetrasi PD & Rem Selesai -> Masuk Jeda");
      } else {
        gantiFase(PHASE_ACTION, "Penetrasi PD & Rem Selesai -> Masuk Aksi");
      }
    }
  } else {
    if (step.preMoveMs > 0) {
      setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
      delay(step.preMoveMs);
    }
    if (step.brakeMs > 0) {
      setMotorLangsung(-(int)step.brakePwm, -(int)step.brakePwm);
      delay(step.brakeMs);
    }
    berhentiTotal();
    if (step.pauseMs > 0) {
      gantiFase(PHASE_PAUSE, "Offset & Rem Aktif Selesai -> Masuk Jeda");
    } else {
      gantiFase(PHASE_ACTION, "Offset & Rem Aktif Selesai -> Masuk Aksi");
    }
  }
}

// Inisialisasi Plan Engine
inline void initNavigationFSM() {
  statusMisiSaatIni     = MISI_JALAN_RUTE_REAL;
  stepIdxSaatIni        = 0;
  faseSaatIni           = PHASE_FOLLOW;
  waktuMasukFase        = millis();
  sudahLepasGarisLama   = false;
  sudahAutoAlignStart   = false;
  konfirmasiCabangKanan = 0;
  errSebelumnyaPD       = 0.0f;
  resetEncoderTicks();
  encoderTicksAwalFase  = 0;
  lastWarnEncoderMs     = millis();
  resetWingTimestamps();
  Serial.println("[PLAN ENGINE] Rute Diinisialisasi: START -> N0(Kiri) -> N2(Kanan) -> A(Encoder 50cm, 3s) -> C(Encoder 200cm, 3s) -> Pulang.");
}

// Parser Perintah Operator (Serial Monitor / Paket LoRa)
inline void prosesPerintah(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  Serial.print("[COMMAND DITERIMA] -> ");
  Serial.println(cmd);

  if (cmd == "START" || cmd == "START_AB" || cmd == "START_AC" || cmd == "1") {
    statusMisiSaatIni   = MISI_JALAN_RUTE_REAL;
    stepIdxSaatIni      = 0;
    sudahAutoAlignStart = false;
    resetEncoderTicks();
    gantiFase(PHASE_FOLLOW, "Perintah Operator START (Misi START -> N0 -> N2 -> A -> C -> START)");
  }
  else if (cmd == "LANJUT" || cmd == "GO" || cmd == "NEXT") {
    if ((stepIdxSaatIni == IDX_STEP_CHECKPOINT_A || stepIdxSaatIni == IDX_STEP_CHECKPOINT_C) &&
        faseSaatIni == PHASE_PAUSE) {
      gantiFase(PHASE_ACTION, "Perintah Operator LANJUT (Bypass jeda 3 detik)");
    } else {
      Serial.println("[INFO] Perintah LANJUT diabaikan (Robot tidak sedang jeda di Checkpoint A / C).");
    }
  }
  else if (cmd == "PULANG" || cmd == "BALIK" || cmd == "RETURN") {
    statusMisiSaatIni = MISI_JALAN_RUTE_REAL;
    stepIdxSaatIni    = IDX_STEP_UTURN_C;
    gantiFase(PHASE_ACTION, "Perintah Operator PULANG (Rute C -> START Dimulai)");
  }
  else if (cmd == "STOP" || cmd == "ESTOP" || cmd == "0") {
    statusMisiSaatIni = MISI_EMERGENCY_STOP;
    berhentiTotal();
    Serial.println(">>> [E-STOP] REM DARURAT DIAKTIFKAN!");
  }
  else if (cmd == "STANDBY" || cmd == "PAUSE") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    Serial.println(">>> [STANDBY] Robot dijeda.");
  }
  else if (cmd == "RESET") {
    initNavigationFSM();
    Serial.println(">>> [RESET] Plan Engine & Encoder Di-reset ke Step Awal.");
  }
  else if (cmd == "RESET_ENC") {
    resetEncoderTicks();
    encoderTicksAwalFase = 0;
    Serial.println(">>> [RESET_ENC] Pulsa encoder roda kiri di-nolkan.");
  }
  else if (cmd == "TEST_ENCODER") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    resetEncoderTicks();
    encoderTicksAwalFase = 0;
    Serial.println("\n>>> [KALIBRASI ENCODER RODA KIRI (5 DETIK)]");
    Serial.println("Putar roda kiri 1 putaran penuh atau dorong robot 50 cm untuk cek nilai ticks...");
    for (int i = 0; i < 20; i++) {
      delay(250);
      long t = bacaEncoderTicks();
      Serial.print("  Encoder Ticks: ");
      Serial.print(t);
      Serial.print(" | Estimasi Jarak: ");
      Serial.print(ticksKeCm(t), 2);
      Serial.println(" cm");
    }
    Serial.println(">>> [SELESAI] Sesuaikan ENCODER_TICKS_PER_PUTARAN di Config.h bila perlu.\n");
  }
  else if (cmd == "STATUS") {
    const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL)
                          ? RUTE_REAL_PP[stepIdxSaatIni].label
                          : "SELESAI";
    Serial.println("\n--- [STATUS AGV DIAGNOSTIK] ---");
    Serial.print("Misi: ");
    switch (statusMisiSaatIni) {
      case MISI_STANDBY:          Serial.println("STANDBY"); break;
      case MISI_JALAN_RUTE_REAL:  Serial.println("JALAN RUTE REAL (START -> N0 -> N2 -> A -> C -> START)"); break;
      case MISI_EMERGENCY_STOP:   Serial.println("EMERGENCY STOP"); break;
    }
    Serial.print("Step: ");
    Serial.print(stepIdxSaatIni);
    Serial.print(" (");
    Serial.print(label);
    Serial.print(") | Fase: ");
    Serial.println(namaFase(faseSaatIni));
    Serial.print("Encoder Total: ");
    Serial.print(bacaEncoderTicks());
    Serial.print(" ticks | Delta Fase: ");
    Serial.print(getDeltaTicksFase());
    Serial.print(" ticks | Jarak Fase: ");
    Serial.print(getJarakFaseCm(), 1);
    Serial.print(" cm");
    if (stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL &&
        RUTE_REAL_PP[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
      Serial.print(" / Target: ");
      Serial.print(RUTE_REAL_PP[stepIdxSaatIni].targetJarakCm, 1);
      Serial.print(" cm");
    }
    Serial.println();
    Serial.println("--------------------------------\n");
  }
  else if (cmd == "TEST_KANAN") {
    Serial.println("\n>>> [DIAGNOSTIK] UJI PUTAR POROS KANAN (2 Detik)...");
    putarPorosKanan(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR);
    delay(2000);
    berhentiTotal();
    Serial.println(">>> [SELESAI] Cek: Roda kiri maju & roda kanan mundur!\n");
  }
  else if (cmd == "TEST_MUNDUR_KANAN") {
    Serial.println("\n>>> [DIAGNOSTIK] UJI KHUSUS RODA KANAN MUNDUR (Pin 17 / LPWM Driver Kanan)...");
    setMotorLangsung(0, -142);
    delay(2000);
    berhentiTotal();
    Serial.println(">>> [SELESAI] Jika roda kanan DIAM, cek kabel pin 17!\n");
  }
  else if (cmd == "TEST_RODA") {
    Serial.println("\n>>> [DIAGNOSTIK LENGKAP 4 ARAH MOTOR]");
    Serial.println("1. Roda Kiri MAJU (Pin 25) 1.5 detik...");
    setMotorLangsung(138, 0); delay(1500); berhentiTotal(); delay(500);
    Serial.println("2. Roda Kiri MUNDUR (Pin 26) 1.5 detik...");
    setMotorLangsung(-138, 0); delay(1500); berhentiTotal(); delay(500);
    Serial.println("3. Roda Kanan MAJU (Pin 16) 1.5 detik...");
    setMotorLangsung(0, 138); delay(1500); berhentiTotal(); delay(500);
    Serial.println("4. Roda Kanan MUNDUR (Pin 17) 1.5 detik...");
    setMotorLangsung(0, -142); delay(1500); berhentiTotal(); delay(500);
    Serial.println(">>> [SELESAI] Uji coba 4 arah motor selesai.\n");
  }
}

// Telemetri Serial Monitor Plan Engine
inline void tampilkanTelemetriFSM(const LineData& line) {
  const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL)
                        ? RUTE_REAL_PP[stepIdxSaatIni].label
                        : "SELESAI";

  Serial.print("PLAN [");
  Serial.print(stepIdxSaatIni);
  Serial.print("/");
  Serial.print(JUMLAH_STEP_RUTE_REAL - 1);
  Serial.print(":");
  Serial.print(label);
  Serial.print(" | ");
  Serial.print(namaFase(faseSaatIni));
  Serial.print("] | Line: [");
  Serial.print(line.b1); Serial.print(line.b2); Serial.print(line.b3); Serial.print(line.b4); Serial.print(line.b5);
  Serial.print("] | Err: ");
  Serial.print(line.error, 2);
  Serial.print(" | ");
  if (line.isGelapSemua()) Serial.print("[GELAP_SEMUA]");
  else if (line.isPerempatan) Serial.print("[PEREMPATAN]");
  else if (line.isLurus) Serial.print("[LURUS]");
  else if (line.isTikunganKanan) Serial.print("[TIK_KANAN]");
  else if (line.isTikunganKiri) Serial.print("[TIK_KIRI]");
  else Serial.print("[TRACK]");

  if (stepIdxSaatIni >= 0 && stepIdxSaatIni < JUMLAH_STEP_RUTE_REAL &&
      RUTE_REAL_PP[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
    Serial.print(" | Enc: ");
    Serial.print(getDeltaTicksFase());
    Serial.print("t (");
    Serial.print(getJarakFaseCm(), 1);
    Serial.print("/");
    Serial.print(RUTE_REAL_PP[stepIdxSaatIni].targetJarakCm, 0);
    Serial.print("cm)");
  } else {
    Serial.print(" | Enc: ");
    Serial.print(bacaEncoderTicks());
    Serial.print("t");
  }
  Serial.println();
}

// ====================================================================
// EKSEKUTOR UTAMA DATA-DRIVEN PLAN ENGINE (Dipanggil di loop())
// ====================================================================
inline void updateNavigationFSM(const LineData& line) {
  unsigned long durasiDiFase = millis() - waktuMasukFase;

  // LAYER 1: CEK STATUS MISI
  switch (statusMisiSaatIni) {
    case MISI_STANDBY:
    case MISI_EMERGENCY_STOP:
      berhentiTotal();
      return;

    case MISI_JALAN_RUTE_REAL:
      break;
  }

  if (stepIdxSaatIni < 0 || stepIdxSaatIni >= JUMLAH_STEP_RUTE_REAL) {
    berhentiTotal();
    statusMisiSaatIni = MISI_STANDBY;
    return;
  }

  const PlanStep& step = RUTE_REAL_PP[stepIdxSaatIni];

  // LAYER 2: MESIN EKSEKUTOR 4-FASE GENERIK
  switch (faseSaatIni) {
    case PHASE_FOLLOW: {
      // Kalibrasi mikro awal di titik START sebelum melaju pada Step 0
      if (stepIdxSaatIni == 0 && eksekusiAutoAlignStart(line, durasiDiFase)) {
        return;
      }

      if (step.trigger == TRIG_ENCODER_JARAK && (line.isGelapSemua() || line.isPerempatan)) {
        setKecepatanMotor(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
      } else {
        jalankanTrackingPD(line, KEC_BASE);
      }

      if (evaluasiTrigger(step, line, durasiDiFase)) {
        gantiFase(PHASE_PRE_MOVE, "Simpang/Trigger Step Terdeteksi");
        // Jika bukan penetrasi PD non-blocking, jalankan offset & rem seketika (0ms latency)
        if (!step.preMoveUsePD) {
          prosesFasePreMove(step, line, 0);
        }
      }
      break;
    }

    case PHASE_PRE_MOVE: {
      prosesFasePreMove(step, line, durasiDiFase);
      break;
    }

    case PHASE_PAUSE: {
      berhentiTotal();
      if (durasiDiFase >= (unsigned long)step.pauseMs) {
        gantiFase(PHASE_ACTION, "Jeda Selesai -> Eksekusi Aksi Step");
      }
      break;
    }

    case PHASE_ACTION: {
      switch (step.action) {
        case ACT_TURN_LEFT:
          eksekusiAksiBelokKiri(line, durasiDiFase, false);
          break;

        case ACT_TURN_LEFT_PULANG:
          eksekusiAksiBelokKiri(line, durasiDiFase, true);
          break;

        case ACT_TURN_RIGHT:
          eksekusiAksiBelokKanan(line, durasiDiFase);
          break;

        case ACT_CROSS_STRAIGHT:
          eksekusiAksiLintasLurus(line, durasiDiFase);
          break;

        case ACT_PAUSE_ONLY:
          if (stepIdxSaatIni == IDX_STEP_CHECKPOINT_C && !OTOMATIS_PULANG_SETELAH_C) {
            berhentiTotal();
            statusMisiSaatIni = MISI_STANDBY;
            Serial.println(">>> [CHECKPOINT C] Jeda 3 detik selesai. Robot STANDBY di C (ketik 'PULANG' untuk kembali ke START).");
          } else {
            majuKeStepBerikutnya("Jeda Parkir 3 Detik Selesai -> Lanjut Step Berikutnya");
          }
          break;

        case ACT_UTURN_180:
          eksekusiAksiUTurn180(line, durasiDiFase);
          break;

        case ACT_STOP_MISSION:
          berhentiTotal();
          statusMisiSaatIni = MISI_STANDBY;
          Serial.println("\n*******************************************************");
          Serial.println("  [SUKSES] AGV TELAH KEMBALI DI TITIK START SEMPURNA!  ");
          Serial.println("  Rute Pulang: C -> U-Turn 180° -> N10 -> N6 ->        ");
          Serial.println("               N2 (Belok Kiri) -> N0 (Belok Kanan) ->  ");
          Serial.println("               START                                   ");
          Serial.println("  Status: STANDBY (Misi 2 Arah Selesai Penuh)          ");
          Serial.println("*******************************************************\n");
          break;
      }
      break;
    }
  }
}
