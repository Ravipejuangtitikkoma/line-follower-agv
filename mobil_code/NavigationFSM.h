#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensor.h"
#include "NodeDetector.h"
#include "RoutePlans.h"
#include "GridGraph.h"
#include "RouteStorage.h"

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
static int             konfirmasiCabangKiri      = 0;
static EncoderSnapshot encoderAwalFase           = {0, 0, 0};
static bool            sudahLogModeEncoder       = false;
static unsigned long   lastWarnEncoderMs         = 0;
static bool            berhentiDiSimpangPengaman = false;
static bool            uturnOffsetSelesai        = false;
static unsigned long   waktuMulaiPutar180        = 0;

inline long getDeltaTicksFase() {
  return bacaEncoderSnapshot().quadTicks - encoderAwalFase.quadTicks;
}

inline unsigned long getPulsaEfektifFase(bool& pakaiAOnly, bool& pakaiBCadangan) {
  return hitungPulsaEfektif(encoderAwalFase, bacaEncoderSnapshot(), pakaiAOnly, pakaiBCadangan);
}

inline float getJarakFaseCm() {
  bool aOnly = false, bCad = false;
  unsigned long pulsa = getPulsaEfektifFase(aOnly, bCad);
  return ticksKeCm((long)pulsa);
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
  konfirmasiCabangKiri  = 0;
  sudahLepasGarisLama   = false;

  // Jangan matikan motor saat masuk PHASE_PRE_MOVE atau saat melintas lurus tanpa jeda (ACT_CROSS_STRAIGHT pause=0)
  bool lanjutTanpaRem = (faseBaru == PHASE_ACTION &&
                         stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif &&
                         ruteAktif[stepIdxSaatIni].action == ACT_CROSS_STRAIGHT &&
                         ruteAktif[stepIdxSaatIni].pauseMs == 0);
  if (faseBaru != PHASE_PRE_MOVE && !lanjutTanpaRem) {
    berhentiTotal();
  }

  if (faseBaru == PHASE_FOLLOW) {
    resetWingTimestamps();
    errSebelumnyaPD          = 0.0f;
    encoderAwalFase          = bacaEncoderSnapshot();
    sudahLogModeEncoder      = false;
    lastWarnEncoderMs        = millis();
    berhentiDiSimpangPengaman = false;
  }

  if (faseBaru == PHASE_ACTION) {
    uturnOffsetSelesai = false;
    waktuMulaiPutar180 = 0;
  }

  const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif)
                        ? ruteAktif[stepIdxSaatIni].label
                        : "SELESAI";

  Serial.print("\n>>> [PLAN STEP ");
  Serial.print(stepIdxSaatIni);
  Serial.print("/");
  Serial.print(jumlahStepAktif > 0 ? jumlahStepAktif - 1 : 0);
  Serial.print(" : ");
  Serial.print(label);
  Serial.print(" | ");
  Serial.print(namaFase(faseBaru));
  Serial.print("] Alasan: ");
  Serial.println(alasan);

  if (faseBaru == PHASE_FOLLOW &&
      stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif &&
      ruteAktif[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
    float targetCm = ruteAktif[stepIdxSaatIni].targetJarakCm;
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
  if (stepIdxSaatIni >= jumlahStepAktif) {
    berhentiTotal();
    statusMisiSaatIni = MISI_STANDBY;
    Serial.println(">>> [PLAN ENGINE] Seluruh step pada rute telah selesai.");
    return;
  }

  if (ruteAktif[stepIdxSaatIni].trigger == TRIG_IMMEDIATE) {
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
    // Garis bergeser ke S2 (Kiri Tengah) -> Counter-torque 4 roda aktif: kanan maju (+150 PWM), kiri mundur (-65 PWM)
    setMotorLangsung(-65, 150);
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
    // Garis bergeser ke S4 (Kanan Tengah) -> Counter-torque 4 roda aktif: kiri maju (+150 PWM), kanan mundur (-65 PWM)
    setMotorLangsung(150, -65);
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
    konfirmasiCabangKiri  = 0;
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

    case TRIG_CABANG_KIRI: {
      // Simetris dengan cabang kanan: wajib ada garis lurus (S3=1 atau totalAktif>=3)
      bool isSimpangKiriValid = (line.b1 == 1 && line.b2 == 1) &&
                                (line.b3 == 1 || line.totalAktif >= 3 || line.isGelapSemua());
      if (isSimpangKiriValid) {
        konfirmasiCabangKiri++;
      } else {
        konfirmasiCabangKiri = 0;
      }

      if (konfirmasiCabangKiri >= PERSISTENSI_CABANG_KIRI) {
        konfirmasiCabangKiri = 0;
        return true;
      }
      return false;
    }

    case TRIG_PEREMPATAN:
      return (line.isGelapSemua() || line.isPerempatan);

    case TRIG_ENCODER_JARAK: {
      if (step.targetJarakCm <= 0.0f) return false;

      EncoderSnapshot kini = bacaEncoderSnapshot();
      bool pakaiAOnly = false;
      bool pakaiBCad  = false;
      unsigned long pulsa = hitungPulsaEfektif(encoderAwalFase, kini, pakaiAOnly, pakaiBCad);
      long dQuad          = kini.quadTicks - encoderAwalFase.quadTicks;
      unsigned long dA    = kini.pulseA - encoderAwalFase.pulseA;
      unsigned long dB    = kini.pulseB - encoderAwalFase.pulseB;
      float jarakCm       = ticksKeCm((long)pulsa);

      if (!sudahLogModeEncoder && (pakaiAOnly || pakaiBCad)) {
        sudahLogModeEncoder = true;
        if (pakaiAOnly) {
          Serial.println("[INFO ENCODER] Fase B tidak sinkron/noisy -> otomatis pakai pulsa murni Fase A (anti-batal).");
        } else if (pakaiBCad) {
          Serial.println("[INFO ENCODER] Fase A (GPIO 32) = 0 -> otomatis pakai cadangan Fase B (GPIO 33).");
        }
      }

      // 1. TRIGGER UTAMA: Target jarak encoder tercapai
      if (pulsa > 0 && jarakCm >= step.targetJarakCm) {
        Serial.print("\n>>> [ENCODER TARGET TERCAPAI] Jarak: ");
        Serial.print(jarakCm, 1);
        Serial.print(" / ");
        Serial.print(step.targetJarakCm, 1);
        Serial.print(" cm | Pulsa: ");
        Serial.print(pulsa);
        Serial.print(" (Q:"); Serial.print(dQuad);
        Serial.print(" A:"); Serial.print(dA);
        Serial.print(" B:"); Serial.print(dB);
        Serial.println(")");
        return true;
      }

      // 2. FALLBACK OTOMATIS: Bila encoder belum terbaca (0 pulsa) setelah 600ms berjalan
      if (pulsa == 0 && durasiDiFase >= WAKTU_DETEKSI_ENCODER_MATI_MS) {
        float jarakWaktuCm = ((float)durasiDiFase / 1000.0f) * KECEPATAN_ESTIMASI_CM_PER_S;
        if (millis() - lastWarnEncoderMs >= 1000) {
          lastWarnEncoderMs = millis();
          Serial.print("[PERINGATAN ENCODER] Pulsa masih 0 (PinA=");
          Serial.print(digitalRead(pinEncoderA));
          Serial.print(" PinB=");
          Serial.print(digitalRead(pinEncoderB));
          Serial.print(")! Fallback estimasi waktu: ");
          Serial.print(jarakWaktuCm, 1);
          Serial.print("/");
          Serial.print(step.targetJarakCm, 1);
          Serial.println(" cm");
        }
        if (GUNAKAN_FALLBACK_WAKTU_ENCODER && jarakWaktuCm >= step.targetJarakCm) {
          Serial.print("\n>>> [FALLBACK WAKTU TERCAPAI] Encoder 0 pulsa -> stop di estimasi ");
          Serial.print(jarakWaktuCm, 1);
          Serial.print(" cm (");
          Serial.print(durasiDiFase);
          Serial.println(" ms). Cek kabel/daya encoder GPIO 32 & 33!");
          return true;
        }
      }

      // 3. PENGAMAN KERAS SIMPANG (mencegah bablas melewati N6 saat ke A atau melewati N14 saat ke C)
      if (step.stopJikaSimpang && durasiDiFase >= LOCKOUT_PENGAMAN_SIMPANG_MS &&
          (line.isGelapSemua() || line.isPerempatan)) {
        berhentiDiSimpangPengaman = true;
        const char* namaSimpang = (stepIdxSaatIni == IDX_STEP_CHECKPOINT_A) ? "N6" : "N14";
        float jarakSimpangCm    = (stepIdxSaatIni == IDX_STEP_CHECKPOINT_A) ? 100.0f : 40.0f;
        Serial.print("\n>>> [PENGAMAN SIMPANG ");
        Serial.print(namaSimpang);
        Serial.print("] Robot menyentuh ");
        Serial.print(namaSimpang);
        Serial.print(" sebelum target encoder! Pulsa: ");
        Serial.print(pulsa);
        Serial.print(" (Q:"); Serial.print(dQuad);
        Serial.print(" A:"); Serial.print(dA);
        Serial.print(" B:"); Serial.print(dB);
        Serial.print(") | Jarak terhitung: ");
        Serial.print(jarakCm, 1);
        Serial.println(" cm -> Berhenti pengaman!");
        if (pulsa > 0) {
          float saranPPR = ((float)pulsa / jarakSimpangCm) * KELILING_RODA_CM;
          Serial.print("    [KALIBRASI] Ketik SET_PPR ");
          Serial.print(saranPPR, 1);
          Serial.println(" (atau jalankan KALIBRASI_50CM) untuk akurasi penuh.");
        }
        return true;
      }

      // 4. PENGAMAN TIMEOUT MAKSIMAL (bila PPR motor asli lebih kecil dari 330 sehingga hitungan terlalu rendah)
      if (step.maxDurasiMs > 0 && durasiDiFase >= (unsigned long)step.maxDurasiMs) {
        Serial.print("\n>>> [TIMEOUT ENCODER] Durasi ");
        Serial.print(durasiDiFase);
        Serial.print(" ms tercapai (Jarak terhitung: ");
        Serial.print(jarakCm, 1);
        Serial.print("/");
        Serial.print(step.targetJarakCm, 1);
        Serial.print(" cm | Pulsa: ");
        Serial.print(pulsa);
        Serial.print(" Q:"); Serial.print(dQuad);
        Serial.print(" A:"); Serial.print(dA);
        Serial.print(" B:"); Serial.print(dB);
        Serial.println(") -> Berhenti pengaman!");
        if (pulsa > 0) {
          float saranPPR = ((float)pulsa * KELILING_RODA_CM) / step.targetJarakCm;
          Serial.print("    [KALIBRASI] Coba sesuaikan ENCODER_TICKS_PER_PUTARAN di Config.h ke ~= ");
          Serial.println(saranPPR, 1);
        }
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
    // Fase Radar: perlambat saat garis cabang mulai menyentuh sisi kanan (S4/S5) setelah lepas sudut lama
    if (sudahLepasGarisLama &&
        durasiDiFase >= (unsigned long)WAKTU_ABAIKAN_SENSOR_KANAN_MS &&
        (line.b4 == 1 || line.b5 == 1)) {
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
      delay(DURASI_REM_BELOK_KANAN_MS);
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
// EKSEKUTOR AKSI PUTAR BALIK 180° HALUS DI CHECKPOINT C (ANTI-BABLAS N14)
// ====================================================================
inline void eksekusiAksiUTurn180(const LineData& line, unsigned long durasiDiFase) {
  if (!uturnOffsetSelesai) {
    bool kenaUjungN14 = (durasiDiFase >= 40) && (line.isGelapSemua() || line.isPerempatan);
    if (berhentiDiSimpangPengaman || kenaUjungN14 || durasiDiFase >= (unsigned long)OFFSET_MAJU_PUTAR_180_MS) {
      if (berhentiDiSimpangPengaman || kenaUjungN14) {
        Serial.println(">>> [U-TURN PENGAMAN N14] Ujung N14 terdeteksi -> hentikan offset maju & langsung putar 180°!");
      }
      berhentiDiSimpangPengaman = false;
      uturnOffsetSelesai        = true;
      waktuMulaiPutar180        = millis();
      berhentiTotal();
      return;
    }
    jalankanTrackingPD(line, KEC_BASE);
    return;
  }

  unsigned long tSejakStop = millis() - waktuMulaiPutar180;
  if (tSejakStop < (unsigned long)JEDA_REDAM_180_MS) {
    berhentiTotal();
    return;
  }

  unsigned long durasiPutar = tSejakStop - (unsigned long)JEDA_REDAM_180_MS;

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
      if (step.brakeMs > 0 || step.pauseMs > 0) {
        berhentiTotal();
      }
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
    if (step.brakeMs > 0 || step.pauseMs > 0) {
      berhentiTotal();
    }
    if (step.pauseMs > 0) {
      gantiFase(PHASE_PAUSE, "Offset & Rem Aktif Selesai -> Masuk Jeda");
    } else {
      gantiFase(PHASE_ACTION, "Offset & Rem Aktif Selesai -> Masuk Aksi");
    }
  }
}

// Helper pecah argumen teks berdasar spasi/koma untuk editor mapping Serial Monitor
inline int pecahTokenArgumen(const String& s, String outTokens[], int maxTokens) {
  int count = 0;
  String cur = "";
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == ' ' || c == '\t' || c == ',') {
      if (cur.length() > 0) {
        if (count < maxTokens) outTokens[count++] = cur;
        cur = "";
      }
    } else {
      cur += c;
    }
  }
  if (cur.length() > 0 && count < maxTokens) {
    outTokens[count++] = cur;
  }
  return count;
}

// Cetak ringkasan bantuan perintah Serial Monitor
inline void cetakBantuanSerial() {
  Serial.println("\n================ [PANDUAN PERINTAH SERIAL MONITOR (115200)] ================");
  Serial.println(" [1] PILIH RUTE CEPAT ANTAR-GUDANG (LOOP MAJU TANPA U-TURN):");
  Serial.println("   RUTE A,C  (atau RUTE 1)  -> START -> Gudang A -> Gudang C -> Pulang START");
  Serial.println("   RUTE A,B  (atau RUTE 2)  -> START -> Gudang A -> Gudang B -> Pulang START");
  Serial.println("   RUTE E,D  (atau RUTE 3)  -> START -> Gudang E -> Gudang D -> Pulang START");
  Serial.println("   RUTE B,D  (atau RUTE 4)  -> START -> Gudang B -> Gudang D -> Pulang START");
  Serial.println("   RUTE <stasiun1>,<stasiun2> -> Susun rute PP ke stasiun manapun (contoh: RUTE B, RUTE E)");
  Serial.println("   RUTE_ONEWAY <stasiun>    -> Susun rute searah & berhenti di tujuan akhir");
  Serial.println("   CMD:GO:<stasiun>         -> Susun rute PP & langsung jalan (mode LoRa)");
  Serial.println("   MAP_SAVE                 -> SIMPAN PERMANEN ke Flash ESP32 (cabut USB tetap ingat!)");
  Serial.println(" [2] LIHAT & EDIT MAPPING MANUAL STEP-BY-STEP:");
  Serial.println("   MAP                      -> Tampilkan tabel mapping aktif");
  Serial.println("   MAP_CLEAR                -> Kosongkan seluruh tabel mapping");
  Serial.println("   MAP_ADD <TRIG> <CM> <PAUSE> <AKSI>       -> Tambah step di akhir");
  Serial.println("   MAP_SET <IDX> <TRIG> <CM> <PAUSE> <AKSI> -> Ubah step indeks ke-IDX");
  Serial.println("   MAP_DEL <IDX>            -> Hapus step indeks ke-IDX");
  Serial.println("     TRIG: LANGSUNG | TKIRI | CKANAN | CKIRI | S4 | ENC");
  Serial.println("     AKSI: KIRI | KIRI_P | KANAN | LURUS | JEDA | UTURN | STOP");
  Serial.println(" [3] KALIBRASI LIVE & SIMPAN PERMANEN KE FLASH ESP32:");
  Serial.println("   KALIBRASI_50CM / SIMPAN_50CM -> Kalibrasi PPR mudah (dorong 50cm) -> NVS");
  Serial.println("   SET_PPR <angka>              -> Setel & simpan permanen PPR ke NVS");
  Serial.println("   SET_JEDA <ms>            -> Ubah jeda berhenti di stasiun (default 3000)");
  Serial.println("   SET_AUTOSTART <0|1>      -> 1=auto jalan saat nyala, 0=tunggu START");
  Serial.println("   MAP_SAVE                 -> SIMPAN PERMANEN mapping & kalibrasi ke Flash");
  Serial.println("   MAP_LOAD / MAP_DEFAULT   -> Muat dari Flash / reset ke default A,C");
  Serial.println(" [4] KONTROL EKSEKUSI & DIAGNOSTIK:");
  Serial.println("   START | LANJUT | PULANG | STANDBY | STOP | RESET | STATUS");
  Serial.println("   TEST_ENCODER | RESET_ENC | TEST_RODA | TEST_KANAN | TEST_MUNDUR_KANAN");
  Serial.println("===========================================================================\n");
}

// Inisialisasi Plan Engine
inline void initNavigationFSM() {
  if (jumlahStepAktif <= 0) {
    muatRuteDefaultAC(false);
  }
  statusMisiSaatIni     = runtimeAutoStart ? MISI_JALAN_RUTE_REAL : MISI_STANDBY;
  stepIdxSaatIni        = 0;
  faseSaatIni           = PHASE_FOLLOW;
  waktuMasukFase        = millis();
  sudahLepasGarisLama   = false;
  sudahAutoAlignStart   = false;
  konfirmasiCabangKanan = 0;
  konfirmasiCabangKiri  = 0;
  errSebelumnyaPD       = 0.0f;
  resetEncoderTicks();
  encoderAwalFase           = {0, 0, 0};
  sudahLogModeEncoder       = false;
  lastWarnEncoderMs         = millis();
  berhentiDiSimpangPengaman = false;
  uturnOffsetSelesai        = false;
  waktuMulaiPutar180        = 0;
  resetWingTimestamps();
  Serial.print("[PLAN ENGINE] Rute Aktif: ");
  Serial.print(namaRuteAktif);
  Serial.print(" (");
  Serial.print(jumlahStepAktif);
  Serial.println(" step) siap dijalankan.");
}

// Parser Perintah Operator (Serial Monitor / Paket LoRa)
inline void prosesPerintah(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();
  Serial.print("[COMMAND DITERIMA] -> ");
  Serial.println(cmd);

  if (cmd == "START" || cmd == "START_AB" || cmd == "START_AC" || cmd == "1") {
    if (jumlahStepAktif <= 0) {
      Serial.println("[ERROR] Tabel mapping masih kosong! Ketik 'RUTE A,C' atau 'MAP_DEFAULT' dulu.");
      return;
    }
    statusMisiSaatIni   = MISI_JALAN_RUTE_REAL;
    stepIdxSaatIni      = 0;
    sudahAutoAlignStart = false;
    resetEncoderTicks();
    gantiFase(PHASE_FOLLOW, "Perintah Operator START");
  }
  else if (cmd == "LANJUT" || cmd == "GO" || cmd == "NEXT") {
    if (faseSaatIni == PHASE_PAUSE) {
      gantiFase(PHASE_ACTION, "Perintah Operator LANJUT (Bypass jeda stasiun)");
    } else {
      Serial.println("[INFO] Perintah LANJUT diabaikan (Robot tidak sedang berada di fase jeda PAUSE).");
    }
  }
  else if (cmd == "PULANG" || cmd == "BALIK" || cmd == "RETURN" || cmd == "CMD:HOME") {
    int idxUTurn = cariIdxStepUTurn();
    if (idxUTurn >= 0) {
      statusMisiSaatIni = MISI_JALAN_RUTE_REAL;
      stepIdxSaatIni    = idxUTurn;
      if (ruteAktif[idxUTurn].action == ACT_UTURN_180) {
        gantiFase(PHASE_ACTION, "Perintah Operator PULANG (Mulai U-Turn & Rute Pulang)");
      } else {
        gantiFase(PHASE_FOLLOW, "Perintah Operator PULANG (Mulai Rute Pulang Loop)");
      }
    } else {
      Serial.println("[INFO] Rute aktif saat ini tidak memiliki step pulang (ketik 'MAP_DEFAULT' atau 'RUTE A,C' untuk PP).");
    }
  }
  else if (cmd == "STOP" || cmd == "ESTOP" || cmd == "0" || cmd == "CMD:STOP") {
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
  else if (cmd == "HELP" || cmd == "?") {
    cetakBantuanSerial();
  }
  else if (cmd == "MAP" || cmd == "LIHAT_MAP") {
    tampilkanTabelMapping();
  }
  else if (cmd == "RUTE 1" || cmd == "RUTE_AC" || cmd == "RUTE AC") {
    if (bangunRuteStasiun("A,C", true)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      simpanMappingKeFlash(); // Otomatis simpan permanen ke Flash ESP32
      tampilkanTabelMapping();
      Serial.println(">>> [SUKSES TERSIMPAN] RUTE A ke C aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.");
    }
  }
  else if (cmd == "RUTE 2" || cmd == "RUTE_AB" || cmd == "RUTE AB") {
    if (bangunRuteStasiun("A,B", true)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      simpanMappingKeFlash(); // Otomatis simpan permanen ke Flash ESP32
      tampilkanTabelMapping();
      Serial.println(">>> [SUKSES TERSIMPAN] RUTE A ke B aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.");
    }
  }
  else if (cmd == "RUTE 3" || cmd == "RUTE_ED" || cmd == "RUTE ED") {
    if (bangunRuteStasiun("E,D", true)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      simpanMappingKeFlash(); // Otomatis simpan permanen ke Flash ESP32
      tampilkanTabelMapping();
      Serial.println(">>> [SUKSES TERSIMPAN] RUTE E ke D aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.");
    }
  }
  else if (cmd == "RUTE 4" || cmd == "RUTE_BD" || cmd == "RUTE BD") {
    if (bangunRuteStasiun("B,D", true)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      simpanMappingKeFlash(); // Otomatis simpan permanen ke Flash ESP32
      tampilkanTabelMapping();
      Serial.println(">>> [SUKSES TERSIMPAN] RUTE B ke D aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.");
    }
  }
  else if (cmd.startsWith("RUTE ")) {
    String arg = cmd.substring(5);
    if (bangunRuteStasiun(arg, true)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      simpanMappingKeFlash(); // Otomatis simpan permanen ke Flash ESP32
      tampilkanTabelMapping();
      Serial.println(">>> [SUKSES TERSIMPAN] Rute aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.");
    }
  }
  else if (cmd.startsWith("RUTE_ONEWAY ")) {
    String arg = cmd.substring(12);
    if (bangunRuteStasiun(arg, false)) {
      statusMisiSaatIni = MISI_STANDBY;
      berhentiTotal();
      stepIdxSaatIni = 0;
      tampilkanTabelMapping();
      Serial.println(">>> Ketik 'MAP_SAVE' untuk simpan permanen ke Flash, atau 'START' untuk langsung jalan.");
    }
  }
  else if (cmd.startsWith("CMD:GO:")) {
    String arg = cmd.substring(7);
    if (bangunRuteStasiun(arg, true)) {
      tampilkanTabelMapping();
      statusMisiSaatIni   = MISI_JALAN_RUTE_REAL;
      stepIdxSaatIni      = 0;
      sudahAutoAlignStart = false;
      resetEncoderTicks();
      gantiFase(PHASE_FOLLOW, "Perintah Misi CMD:GO (Siap LoRa/Serial)");
    }
  }
  else if (cmd == "MAP_CLEAR") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    jumlahStepAktif = 0;
    stepIdxSaatIni  = 0;
    strncpy(namaRuteAktif, "MANUAL_CUSTOM", sizeof(namaRuteAktif) - 1);
    namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';
    Serial.println(">>> [MAP_CLEAR] Tabel mapping dikosongkan. Gunakan MAP_ADD untuk menambah langkah.");
  }
  else if (cmd.startsWith("MAP_ADD ")) {
    if (jumlahStepAktif >= MAX_PLAN_STEPS) {
      Serial.println("[ERROR] Kapasitas MAX_PLAN_STEPS penuh!");
      return;
    }
    String tok[4];
    int n = pecahTokenArgumen(cmd.substring(8), tok, 4);
    if (n < 4) {
      Serial.println("[ERROR] Format: MAP_ADD <TRIG> <CM> <PAUSE_MS> <AKSI> (contoh: MAP_ADD ENC 50 3000 JEDA)");
      return;
    }
    StepTrigger trig;
    StepAction  act;
    if (!parseTriggerToken(tok[0], trig)) {
      Serial.println("[ERROR] Trigger tidak dikenal! Pilihan: LANGSUNG | TKIRI | CKANAN | CKIRI | S4 | ENC");
      return;
    }
    if (!parseAksiToken(tok[3], act)) {
      Serial.println("[ERROR] Aksi tidak dikenal! Pilihan: KIRI | KIRI_P | KANAN | LURUS | JEDA | UTURN | STOP");
      return;
    }
    float targetCm   = tok[1].toFloat();
    uint16_t pauseMs = (uint16_t)constrain((long)tok[2].toInt(), 0L, 60000L);
    char lbl[20];
    snprintf(lbl, sizeof(lbl), "MANUAL_%d", jumlahStepAktif);
    isiDefaultStepDariAksi(ruteAktif[jumlahStepAktif], trig, targetCm, pauseMs, act, lbl);
    jumlahStepAktif++;
    strncpy(namaRuteAktif, "MANUAL_CUSTOM", sizeof(namaRuteAktif) - 1);
    namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    Serial.print(">>> [MAP_ADD] Step "); Serial.print(jumlahStepAktif - 1); Serial.println(" ditambahkan.");
    tampilkanTabelMapping();
  }
  else if (cmd.startsWith("MAP_SET ")) {
    String tok[5];
    int n = pecahTokenArgumen(cmd.substring(8), tok, 5);
    if (n < 5) {
      Serial.println("[ERROR] Format: MAP_SET <IDX> <TRIG> <CM> <PAUSE_MS> <AKSI>");
      return;
    }
    int idx = tok[0].toInt();
    if (idx < 0 || idx >= jumlahStepAktif) {
      Serial.println("[ERROR] Indeks step di luar rentang!");
      return;
    }
    StepTrigger trig;
    StepAction  act;
    if (!parseTriggerToken(tok[1], trig) || !parseAksiToken(tok[4], act)) {
      Serial.println("[ERROR] Trigger atau Aksi tidak dikenal! Ketik 'HELP' untuk daftar.");
      return;
    }
    float targetCm   = tok[2].toFloat();
    uint16_t pauseMs = (uint16_t)constrain((long)tok[3].toInt(), 0L, 60000L);
    char lblLama[20];
    strncpy(lblLama, ruteAktif[idx].label, sizeof(lblLama) - 1);
    lblLama[sizeof(lblLama) - 1] = '\0';
    isiDefaultStepDariAksi(ruteAktif[idx], trig, targetCm, pauseMs, act, lblLama);
    strncpy(namaRuteAktif, "MANUAL_CUSTOM", sizeof(namaRuteAktif) - 1);
    namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    Serial.print(">>> [MAP_SET] Step "); Serial.print(idx); Serial.println(" diperbarui.");
    tampilkanTabelMapping();
  }
  else if (cmd.startsWith("MAP_DEL ")) {
    String arg = cmd.substring(8);
    arg.trim();
    if (arg.length() == 0) {
      Serial.println("[ERROR] Format: MAP_DEL <IDX>");
      return;
    }
    int idx = arg.toInt();
    if (idx < 0 || idx >= jumlahStepAktif) {
      Serial.println("[ERROR] Indeks step di luar rentang!");
      return;
    }
    for (int i = idx; i < jumlahStepAktif - 1; i++) {
      ruteAktif[i] = ruteAktif[i + 1];
    }
    jumlahStepAktif--;
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    Serial.print(">>> [MAP_DEL] Step "); Serial.print(idx); Serial.println(" dihapus.");
    tampilkanTabelMapping();
  }
  else if (cmd == "MAP_SAVE") {
    simpanMappingKeFlash();
  }
  else if (cmd == "MAP_LOAD") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    stepIdxSaatIni = 0;
    muatMappingDariFlash();
    tampilkanTabelMapping();
  }
  else if (cmd == "MAP_DEFAULT") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    stepIdxSaatIni = 0;
    hapusMappingFlash();
    tampilkanTabelMapping();
  }
  else if (cmd.startsWith("SET_PPR ")) {
    float ppr = cmd.substring(8).toFloat();
    if (simpanKalibrasiEncoderNVS(ppr)) {
      Serial.print(">>> [SET_PPR] PPR encoder diatur & disimpan ke NVS: ");
      Serial.print(runtimeTicksPerPutaran, 1);
      Serial.println(" ticks/putaran (ketik 'MAP_SAVE' untuk simpan ke mapping Flash).");
    }
  }
  else if (cmd.startsWith("SET_JEDA ")) {
    long jeda = constrain((long)cmd.substring(9).toInt(), 0L, 60000L);
    runtimeJedaStasiunMs = (unsigned long)jeda;
    for (int i = 0; i < jumlahStepAktif; i++) {
      if (ruteAktif[i].trigger == TRIG_ENCODER_JARAK) {
        ruteAktif[i].pauseMs = (uint16_t)jeda;
      }
    }
    Serial.print(">>> [SET_JEDA] Jeda stasiun diatur ke ");
    Serial.print(runtimeJedaStasiunMs);
    Serial.println(" ms (ketik 'MAP_SAVE' untuk simpan permanen).");
  }
  else if (cmd.startsWith("SET_AUTOSTART ")) {
    int val = cmd.substring(14).toInt();
    runtimeAutoStart = (val != 0);
    Serial.print(">>> [SET_AUTOSTART] Auto-jalan saat nyala: ");
    Serial.println(runtimeAutoStart ? "ON (1)" : "OFF (0)");
    Serial.println("    (Ketik 'MAP_SAVE' untuk simpan permanen ke Flash).");
  }
  else if (cmd == "RESET_ENC") {
    resetEncoderTicks();
    encoderAwalFase     = {0, 0, 0};
    sudahLogModeEncoder = false;
    Serial.println(">>> [RESET_ENC] Pulsa encoder roda kiri di-nolkan.");
  }
  else if (cmd == "TEST_ENCODER") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    resetEncoderTicks();
    encoderAwalFase     = {0, 0, 0};
    sudahLogModeEncoder = false;
    Serial.println("\n>>> [KALIBRASI ENCODER RODA KIRI (5 DETIK)]");
    Serial.print("PPR aktif: ");
    Serial.print(runtimeTicksPerPutaran, 1);
    Serial.println(encoderSudahKalibrasiNVS ? " (NVS Tersimpan)" : " (Default Belum Kalibrasi)");
    Serial.println("Putar roda kiri 1 putaran penuh atau dorong robot 50 cm untuk cek nilai ticks...");
    for (int i = 0; i < 20; i++) {
      delay(250);
      EncoderSnapshot snap = bacaEncoderSnapshot();
      bool pakaiAOnly = false, pakaiBCad = false;
      unsigned long ef = hitungPulsaEfektif(encoderAwalFase, snap, pakaiAOnly, pakaiBCad);
      Serial.print("  PinA="); Serial.print(digitalRead(pinEncoderA));
      Serial.print(" PinB="); Serial.print(digitalRead(pinEncoderB));
      Serial.print(" | Quad:"); Serial.print(snap.quadTicks);
      Serial.print(" A:"); Serial.print(snap.pulseA);
      Serial.print(" B:"); Serial.print(snap.pulseB);
      Serial.print(" | Estimasi Jarak: ");
      Serial.print(ticksKeCm((long)ef), 2);
      Serial.println(" cm");
    }
    Serial.println(">>> [SELESAI] Gunakan 'SET_PPR <nilai>' / 'KALIBRASI_50CM' (-> NVS), atau 'MAP_SAVE'!\n");
  }
  else if (cmd == "KALIBRASI_50CM") {
    statusMisiSaatIni = MISI_STANDBY;
    berhentiTotal();
    resetEncoderTicks();
    encoderAwalFase     = {0, 0, 0};
    sudahLogModeEncoder = false;
    Serial.println("\n>>> [MODE KALIBRASI JARAK 50 CM]");
    Serial.println("1. Pastikan posisi start ditandai dengan penggaris/lakban.");
    Serial.println("2. Dorong robot LURUS ke depan sejauh tepat 50.0 cm.");
    Serial.println("3. Setelah sampai 50 cm, ketik 'SIMPAN_50CM' di Serial Monitor.\n");
  }
  else if (cmd == "SIMPAN_50CM") {
    EncoderSnapshot snap = bacaEncoderSnapshot();
    bool pakaiAOnly = false, pakaiBCad = false;
    unsigned long pulsa = hitungPulsaEfektif(encoderAwalFase, snap, pakaiAOnly, pakaiBCad);
    if (pulsa < 10) {
      Serial.println("[ERROR] Pulsa encoder terlalu kecil (<10)! Pastikan roda kiri berputar saat didorong.");
    } else {
      float pprHitung = ((float)pulsa * KELILING_RODA_CM) / 50.0f;
      Serial.print(">>> [KALIBRASI 50CM] Pulsa terbaca: ");
      Serial.print(pulsa);
      Serial.print(" | PPR terhitung: ");
      Serial.println(pprHitung, 1);
      simpanKalibrasiEncoderNVS(pprHitung);
    }
  }
  else if (cmd == "STATUS") {
    const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif)
                          ? ruteAktif[stepIdxSaatIni].label
                          : "SELESAI";
    Serial.println("\n--- [STATUS AGV DIAGNOSTIK] ---");
    Serial.print("Rute Aktif: "); Serial.println(namaRuteAktif);
    Serial.print("Misi: ");
    switch (statusMisiSaatIni) {
      case MISI_STANDBY:          Serial.println("STANDBY"); break;
      case MISI_JALAN_RUTE_REAL:  Serial.println("JALAN RUTE AKTIF"); break;
      case MISI_EMERGENCY_STOP:   Serial.println("EMERGENCY STOP"); break;
    }
    Serial.print("Step: ");
    Serial.print(stepIdxSaatIni);
    Serial.print("/");
    Serial.print(jumlahStepAktif > 0 ? jumlahStepAktif - 1 : 0);
    Serial.print(" (");
    Serial.print(label);
    Serial.print(") | Fase: ");
    Serial.println(namaFase(faseSaatIni));
    Serial.print("Kalibrasi PPR: ");
    Serial.print(runtimeTicksPerPutaran, 1);
    Serial.println(encoderSudahKalibrasiNVS ? " (Tersimpan di NVS)" : " (DEFAULT - Belum dikalibrasi!)");
    EncoderSnapshot kini = bacaEncoderSnapshot();
    bool pakaiAOnly = false, pakaiBCad = false;
    unsigned long pulsa = hitungPulsaEfektif(encoderAwalFase, kini, pakaiAOnly, pakaiBCad);
    long dQuad          = kini.quadTicks - encoderAwalFase.quadTicks;
    unsigned long dA    = kini.pulseA - encoderAwalFase.pulseA;
    unsigned long dB    = kini.pulseB - encoderAwalFase.pulseB;
    Serial.print("Encoder Total Q:");
    Serial.print(kini.quadTicks);
    Serial.print(" A:");
    Serial.print(kini.pulseA);
    Serial.print(" B:");
    Serial.print(kini.pulseB);
    Serial.print(" | Fase Efektif: ");
    Serial.print(pulsa);
    Serial.print(" (dQ:"); Serial.print(dQuad);
    Serial.print(" dA:"); Serial.print(dA);
    Serial.print(" dB:"); Serial.print(dB);
    Serial.print(") -> ");
    Serial.print(ticksKeCm((long)pulsa), 1);
    Serial.print(" cm");
    if (stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif &&
        ruteAktif[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
      Serial.print(" / Target: ");
      Serial.print(ruteAktif[stepIdxSaatIni].targetJarakCm, 1);
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
  else {
    Serial.println("[INFO] Perintah tidak dikenal. Ketik 'HELP' untuk melihat semua perintah.");
  }
}

// Telemetri Serial Monitor Plan Engine
inline void tampilkanTelemetriFSM(const LineData& line) {
  const char* label = (stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif)
                        ? ruteAktif[stepIdxSaatIni].label
                        : "SELESAI";

  Serial.print("PLAN [");
  Serial.print(stepIdxSaatIni);
  Serial.print("/");
  Serial.print(jumlahStepAktif > 0 ? jumlahStepAktif - 1 : 0);
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

  if (stepIdxSaatIni >= 0 && stepIdxSaatIni < jumlahStepAktif &&
      ruteAktif[stepIdxSaatIni].trigger == TRIG_ENCODER_JARAK) {
    EncoderSnapshot kini = bacaEncoderSnapshot();
    bool pakaiAOnly = false, pakaiBCad = false;
    unsigned long pulsa = hitungPulsaEfektif(encoderAwalFase, kini, pakaiAOnly, pakaiBCad);
    long dQuad          = kini.quadTicks - encoderAwalFase.quadTicks;
    unsigned long dA    = kini.pulseA - encoderAwalFase.pulseA;
    unsigned long dB    = kini.pulseB - encoderAwalFase.pulseB;
    Serial.print(" | Enc: ");
    Serial.print(pulsa);
    Serial.print("t (Q:"); Serial.print(dQuad);
    Serial.print(" A:"); Serial.print(dA);
    Serial.print(" B:"); Serial.print(dB);
    Serial.print(" | ");
    Serial.print(ticksKeCm((long)pulsa), 1);
    if (pulsa == 0 && GUNAKAN_FALLBACK_WAKTU_ENCODER && faseSaatIni == PHASE_FOLLOW) {
      float estCm = ((float)(millis() - waktuMasukFase) / 1000.0f) * KECEPATAN_ESTIMASI_CM_PER_S;
      Serial.print("cm/Fb:");
      Serial.print(estCm, 1);
    }
    Serial.print("/");
    Serial.print(ruteAktif[stepIdxSaatIni].targetJarakCm, 0);
    Serial.print("cm)");
  } else {
    EncoderSnapshot kini = bacaEncoderSnapshot();
    Serial.print(" | Enc Q:");
    Serial.print(kini.quadTicks);
    Serial.print(" A:");
    Serial.print(kini.pulseA);
    Serial.print(" B:");
    Serial.print(kini.pulseB);
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

  if (stepIdxSaatIni < 0 || stepIdxSaatIni >= jumlahStepAktif) {
    berhentiTotal();
    statusMisiSaatIni = MISI_STANDBY;
    return;
  }

  const PlanStep& step = ruteAktif[stepIdxSaatIni];

  // LAYER 2: MESIN EKSEKUTOR 4-FASE GENERIK
  switch (faseSaatIni) {
    case PHASE_FOLLOW: {
      // Kalibrasi mikro awal di titik START sebelum melaju pada Step 0
      if (stepIdxSaatIni == 0 && eksekusiAutoAlignStart(line, durasiDiFase)) {
        return;
      }

      if (step.trigger == TRIG_ENCODER_JARAK && !step.stopJikaSimpang && (line.isGelapSemua() || line.isPerempatan)) {
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

        case ACT_PAUSE_ONLY: {
          bool adaStepBerikutnya         = (stepIdxSaatIni + 1 < jumlahStepAktif);
          StepAction aksiBerikutnya      = adaStepBerikutnya ? ruteAktif[stepIdxSaatIni + 1].action : ACT_STOP_MISSION;
          bool stepBerikutnyaLintasLurus = adaStepBerikutnya &&
                                           (aksiBerikutnya == ACT_CROSS_STRAIGHT) &&
                                           (ruteAktif[stepIdxSaatIni + 1].pauseMs == 0);
          bool stepBerikutnyaBelok       = adaStepBerikutnya &&
                                           (aksiBerikutnya == ACT_TURN_RIGHT ||
                                            aksiBerikutnya == ACT_TURN_LEFT ||
                                            aksiBerikutnya == ACT_TURN_LEFT_PULANG);
          bool stepBerikutnyaPulang      = adaStepBerikutnya &&
                                           (aksiBerikutnya == ACT_UTURN_180 ||
                                            strncmp(ruteAktif[stepIdxSaatIni + 1].label, "PULANG_", 7) == 0);

          if (stepBerikutnyaPulang && !runtimePulangOtomatis) {
            berhentiTotal();
            statusMisiSaatIni = MISI_STANDBY;
            Serial.println(">>> [TUJUAN SELESAI] Jeda stasiun selesai. Robot STANDBY (ketik 'PULANG' untuk kembali ke START).");
          } else if (berhentiDiSimpangPengaman && stepBerikutnyaLintasLurus) {
            // Pengaman simpang sempat menghentikan robot tepat di garis simpang berikutnya (misal N6):
            // langsung jalankan PHASE_ACTION step lintasi-lurus berikutnya agar referensi tetap sinkron.
            stepIdxSaatIni++;
            berhentiDiSimpangPengaman = false;
            gantiFase(PHASE_ACTION, "Pengaman Simpang Terpicu -> Langsung Melintasi Simpang Berikutnya");
          } else if (berhentiDiSimpangPengaman && stepBerikutnyaBelok) {
            // Pengaman simpang menghentikan robot tepat di simpang belok berikutnya (misal N14):
            // lewati PHASE_FOLLOW step belok agar tidak bablas melewati ujung N14 saat lockout.
            stepIdxSaatIni++;
            berhentiDiSimpangPengaman = false;
            gantiFase(PHASE_PRE_MOVE, "Pengaman Simpang Terpicu -> Langsung Pre-Move Belok di Simpang");
            if (!ruteAktif[stepIdxSaatIni].preMoveUsePD) {
              prosesFasePreMove(ruteAktif[stepIdxSaatIni], line, 0);
            }
          } else {
            majuKeStepBerikutnya("Jeda Parkir Selesai -> Lanjut Step Berikutnya");
          }
          break;
        }

        case ACT_UTURN_180:
          eksekusiAksiUTurn180(line, durasiDiFase);
          break;

        case ACT_STOP_MISSION:
          berhentiTotal();
          statusMisiSaatIni = MISI_STANDBY;
          Serial.println("\n*******************************************************");
          Serial.println("  [SUKSES] MISI NAVIGASI AGV SELESAI SEMPURNA!         ");
          Serial.print("  Rute: "); Serial.println(namaRuteAktif);
          Serial.println("  Status: STANDBY                                      ");
          Serial.println("*******************************************************\n");
          break;
      }
      break;
    }
  }
}
