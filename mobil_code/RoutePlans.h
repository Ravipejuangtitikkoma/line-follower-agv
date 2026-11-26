#pragma once
#include <Arduino.h>
#include <string.h>
#include "Config.h"
#include "Sensor.h"

// ====================================================================
// MODUL TABEL RUTE DINAMIS (DATA-DRIVEN PLAN ENGINE + NVS READY)
// File: RoutePlans.h
// Deskripsi: Menyimpan struktur langkah navigasi dalam RAM (ruteAktif[])
//            agar mapping dapat diubah via Serial Monitor / LoRa dan
//            disimpan permanen ke Flash ESP32 tanpa upload ulang kode.
// ====================================================================

// Jenis kondisi simpang pemicu transisi ke fase aksi
enum StepTrigger : uint8_t {
  TRIG_IMMEDIATE,     // Langsung eksekusi tanpa menunggu sensor
  TRIG_T_KIRI,        // Simpang T melintang / kiri: gelap semua [11111] atau (b1 && b2 && b3)
  TRIG_CABANG_KANAN,  // Cabang kanan topologis: (b5 && b4) && (b3 || totalAktif>=3 || gelapSemua)
  TRIG_CABANG_KIRI,   // Cabang kiri topologis: (b1 && b2) && (b3 || totalAktif>=3 || gelapSemua)
  TRIG_PEREMPATAN,    // Simpang 4 / garis melintang: gelap semua atau isPerempatan (toleransi 35ms)
  TRIG_ENCODER_JARAK  // Target jarak tempuh encoder roda kiri (targetJarakCm) tercapai
};

// Jenis aksi utama setelah fase follow + pre-move + pause
enum StepAction : uint8_t {
  ACT_TURN_LEFT,         // Belok kiri 90° (torsi berangkat)
  ACT_TURN_LEFT_PULANG,  // Belok kiri 90° (torsi khusus rute pulang 156/158 PWM)
  ACT_TURN_RIGHT,        // Belok kanan 90° (2-phase closed-loop + strict binary latch)
  ACT_CROSS_STRAIGHT,    // Dorong lurus melintasi garis perempatan (450ms + tunggu lepas gelap)
  ACT_PAUSE_ONLY,        // Hanya berhenti/parkir sesuai pauseMs lalu lanjut step berikutnya
  ACT_UTURN_180,         // Maju PD 765ms + jeda redam 90ms + putar balik 180° halus
  ACT_STOP_MISSION       // Parkir selesai misi & kembali ke MISI_STANDBY
};

// Struktur Plain-Old-Data (POD) 1 langkah rute AGV (aman disimpan ke Flash ESP32 NVS)
struct PlanStep {
  char        label[20];      // Nama step untuk telemetri Serial Monitor (maks 19 karakter)
  StepTrigger trigger;        // Kondisi simpang / jarak pemicu
  uint16_t    lockoutMs;      // Waktu minimal jalan sebelum trigger aktif (TA Lockout Timer)
  float       targetJarakCm;  // Target jarak encoder dalam cm (hanya dipakai jika TRIG_ENCODER_JARAK)
  uint16_t    maxDurasiMs;    // Batas waktu pengaman step encoder (0 = nonaktif)
  bool        stopJikaSimpang;// true = hentikan paksa jika simpang terinjak (khusus segmen menuju stasiun)
  uint16_t    preMoveMs;      // Durasi offset maju (59ms) atau penetrasi PD sebelum rem
  bool        preMoveUsePD;   // false = offset maju langsung, true = penetrasi tracking PD
  uint8_t     brakePwm;       // Daya rem aktif berlawanan arah (PWM)
  uint16_t    brakeMs;        // Durasi rem aktif (ms)
  uint16_t    pauseMs;        // Jeda diam di stasiun/simpang sebelum aksi (ms)
  StepAction  action;         // Aksi utama yang dijalankan
};

// Indeks step khusus pada preset default (referensi pengaman simpang N6 / N14 & rute pulang)
const int IDX_STEP_CHECKPOINT_A = 2; // Step N2 -> Checkpoint A (Encoder ~50 cm + Jeda 3 detik)
const int IDX_STEP_LEWAT_N6     = 3; // Step melintasi N6 lurus menuju N10 (reset referensi posisi)
const int IDX_STEP_LEWAT_N10    = 4; // Step melintasi N10 lurus sebelum ukur 40 cm ke C
const int IDX_STEP_CHECKPOINT_C = 5; // Step N10 -> Checkpoint C (Encoder ~40 cm + Pengaman N14)
const int IDX_STEP_PULANG_MULAI = 6; // Step awal rute pulang (C -> N14 Belok Kanan -> N15 -> ... -> START)
const int IDX_STEP_UTURN_C      = IDX_STEP_PULANG_MULAI;
const int IDX_STEP_TITIK_A      = IDX_STEP_CHECKPOINT_A;
const int IDX_STEP_UTURN_B      = IDX_STEP_PULANG_MULAI;

// ====================================================================
// TABEL PRESET DEFAULT LOOP (START -> N0 -> N2 -> A -> N6 -> N10 -> C -> N14 -> N15 -> N11 -> N7 -> N3 -> N0 -> START)
// ====================================================================
const PlanStep RUTE_DEFAULT_AC[] = {
  // label,               trigger,             lockout,                    targetCm,         maxDurMs,             stopNode, preMove,                     usePD, brkPwm, brkMs, pause,                    action
  { "N0_BELOK_KIRI",      TRIG_T_KIRI,         250,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 0
  { "N2_BELOK_KANAN",     TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 1
  { "CHECKPOINT_A",       TRIG_ENCODER_JARAK,  100,                        JARAK_N2_KE_A_CM, TIMEOUT_ENCODER_A_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 2
  { "LEWAT_N6",           TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 3
  { "LEWAT_N10",          TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 4
  { "CHECKPOINT_C",       TRIG_ENCODER_JARAK,  100,                        JARAK_N10_KE_C_CM,TIMEOUT_ENCODER_C_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_C_MS,   ACT_PAUSE_ONLY       }, // 5
  { "PULANG_N14_KANAN",   TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 6
  { "PULANG_N15_KANAN",   TRIG_CABANG_KANAN,   400,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 7
  { "PULANG_N11",         TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 8
  { "PULANG_N7",          TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 9
  { "PULANG_N3_KANAN",    TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 10
  { "PULANG_N0_KIRI",     TRIG_CABANG_KIRI,    WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 11
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    0,                           false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 12
};

const int JUMLAH_STEP_DEFAULT_AC = sizeof(RUTE_DEFAULT_AC) / sizeof(RUTE_DEFAULT_AC[0]);

// ====================================================================
// TABEL PRESET GUDANG A KE B (LOOP TANPA U-TURN):
// START -> N0 -> N2 -> [A] -> N6 -> N7 -> [B] -> N11 -> N10 -> N6 -> N2 -> N0 -> START
// ====================================================================
const PlanStep RUTE_PRESET_AB[] = {
  // label,               trigger,             lockout,                    targetCm,         maxDurMs,             stopNode, preMove,                     usePD, brkPwm, brkMs, pause,                    action
  { "N0_BELOK_KIRI",      TRIG_T_KIRI,         250,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 0
  { "N2_BELOK_KANAN",     TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 1
  { "CHECKPOINT_A",       TRIG_ENCODER_JARAK,  100,                        JARAK_N2_KE_A_CM, TIMEOUT_ENCODER_A_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 2
  { "N6_BELOK_KANAN",     TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 3
  { "N7_BELOK_KIRI",      TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 4
  { "CHECKPOINT_B",       TRIG_ENCODER_JARAK,  100,                        50.0f,            TIMEOUT_ENCODER_A_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 5
  { "PULANG_N11_KIRI",    TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 6
  { "PULANG_N10_KIRI",    TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 7
  { "PULANG_N6_LURUS",    TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 8
  { "PULANG_N2_KIRI",     TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 9
  { "PULANG_N0_KANAN",    TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 10
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    0,                           false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 11
};

// ====================================================================
// TABEL PRESET GUDANG E KE D (LOOP TANPA U-TURN):
// START -> N0 -> N3 -> N4 -> [E] -> N8 -> N12 -> [D] -> N16 -> N15 -> N11 -> N7 -> N3 -> N0 -> START
// ====================================================================
const PlanStep RUTE_PRESET_ED[] = {
  // label,               trigger,             lockout,                    targetCm,         maxDurMs,             stopNode, preMove,                     usePD, brkPwm, brkMs, pause,                    action
  { "N0_BELOK_KANAN",     TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 0
  { "LEWAT_N3",           TRIG_CABANG_KIRI,    350,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 1
  { "N4_BELOK_KIRI",      TRIG_T_KIRI,         350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 2
  { "CHECKPOINT_E",       TRIG_ENCODER_JARAK,  100,                        50.0f,            TIMEOUT_ENCODER_A_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 3
  { "LEWAT_N8",           TRIG_CABANG_KIRI,    250,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 4
  { "LEWAT_N12",          TRIG_CABANG_KIRI,    400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 5
  { "CHECKPOINT_D",       TRIG_ENCODER_JARAK,  100,                        JARAK_N10_KE_C_CM,TIMEOUT_ENCODER_C_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_C_MS,   ACT_PAUSE_ONLY       }, // 6
  { "PULANG_N16_KIRI",    TRIG_T_KIRI,         250,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 7
  { "PULANG_N15_KIRI",    TRIG_T_KIRI,         350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 8
  { "PULANG_N11",         TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 9
  { "PULANG_N7",          TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 10
  { "PULANG_N3_KANAN",    TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 11
  { "PULANG_N0_KIRI",     TRIG_CABANG_KIRI,    WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 12
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    0,                           false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 13
};

// ====================================================================
// TABEL PRESET GUDANG B KE D (LOOP TANPA U-TURN):
// START -> N0 -> N3 -> N7 -> [B] -> N11 -> N12 -> [D] -> N16 -> N15 -> N11 -> N7 -> N3 -> N0 -> START
// ====================================================================
const PlanStep RUTE_PRESET_BD[] = {
  // label,               trigger,             lockout,                    targetCm,         maxDurMs,             stopNode, preMove,                     usePD, brkPwm, brkMs, pause,                    action
  { "N0_BELOK_KANAN",     TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 0
  { "N3_BELOK_KIRI",      TRIG_CABANG_KIRI,    350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 1
  { "LEWAT_N7",           TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 2
  { "CHECKPOINT_B",       TRIG_ENCODER_JARAK,  100,                        50.0f - OFFSET_CROSS_SIMPANG_CM, TIMEOUT_ENCODER_A_MS, true, 0,                  false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 3
  { "N11_BELOK_KANAN",    TRIG_PEREMPATAN,     250,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 4
  { "N12_BELOK_KIRI",     TRIG_CABANG_KIRI,    350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 5
  { "CHECKPOINT_D",       TRIG_ENCODER_JARAK,  100,                        50.0f,            TIMEOUT_ENCODER_C_MS, true,     0,                           false, 85,     35,    DURASI_JEDA_TITIK_C_MS,   ACT_PAUSE_ONLY       }, // 6
  { "PULANG_N16_KIRI",    TRIG_T_KIRI,         250,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 7
  { "PULANG_N15_KIRI",    TRIG_T_KIRI,         350,                        0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 8
  { "PULANG_N11",         TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 9
  { "PULANG_N7",          TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    0,                           false, 0,      0,     0,                        ACT_CROSS_STRAIGHT   }, // 10
  { "PULANG_N3_KANAN",    TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,    OFFSET_MAJU_SIMPANG_KANAN_MS,false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 11
  { "PULANG_N0_KIRI",     TRIG_CABANG_KIRI,    WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,    OFFSET_MAJU_BELOK_MS,        false, 105,    45,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 12
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,    0,                           false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 13
};

// ====================================================================
// BUFFER MAPPING AKTIF DI RAM (DIUBAH VIA SERIAL / LORA / FLASH NVS)
// ====================================================================
static PlanStep      ruteAktif[MAX_PLAN_STEPS];
static int           jumlahStepAktif       = 0;
static char          namaRuteAktif[32]     = "DEFAULT_AC_LOOP";
static unsigned long runtimeJedaStasiunMs  = DURASI_JEDA_TITIK_A_MS;
static bool          runtimePulangOtomatis = OTOMATIS_PULANG_SETELAH_C;
static bool          runtimeAutoStart      = AUTO_START_SAAT_NYALA;

// Helper konstruktor PlanStep aman-ukuran
inline PlanStep buatPlanStep(const char* lbl,
                             StepTrigger trig,
                             uint16_t lockout,
                             float targetCm,
                             uint16_t maxDur,
                             bool stopN6,
                             uint16_t preMove,
                             bool usePD,
                             uint8_t brkPwm,
                             uint16_t brkMs,
                             uint16_t pause,
                             StepAction act) {
  PlanStep s;
  memset(&s, 0, sizeof(s));
  strncpy(s.label, lbl, sizeof(s.label) - 1);
  s.trigger         = trig;
  s.lockoutMs       = lockout;
  s.targetJarakCm   = targetCm;
  s.maxDurasiMs     = maxDur;
  s.stopJikaSimpang = stopN6;
  s.preMoveMs       = preMove;
  s.preMoveUsePD    = usePD;
  s.brakePwm        = brkPwm;
  s.brakeMs         = brkMs;
  s.pauseMs         = pause;
  s.action          = act;
  return s;
}

// Muat preset rute pilihan ke RAM (AC, AB, ED, BD)
inline bool muatPresetRute(PresetRuteId idPreset, bool resetParam = false) {
  if (resetParam) {
    if (!encoderSudahKalibrasiNVS) {
      runtimeTicksPerPutaran = ENCODER_TICKS_PER_PUTARAN;
    }
    runtimeJedaStasiunMs   = DURASI_JEDA_TITIK_A_MS;
    runtimePulangOtomatis  = true;
    runtimeAutoStart       = AUTO_START_SAAT_NYALA;
  }

  const PlanStep* srcPreset = nullptr;
  int countPreset = 0;
  const char* nama = "DEFAULT_AC_LOOP";

  switch (idPreset) {
    case RUTE_AB:
      srcPreset   = RUTE_PRESET_AB;
      countPreset = sizeof(RUTE_PRESET_AB) / sizeof(RUTE_PRESET_AB[0]);
      nama        = "RUTE_A_B_LOOP";
      break;
    case RUTE_ED:
      srcPreset   = RUTE_PRESET_ED;
      countPreset = sizeof(RUTE_PRESET_ED) / sizeof(RUTE_PRESET_ED[0]);
      nama        = "RUTE_E_D_LOOP";
      break;
    case RUTE_BD:
      srcPreset   = RUTE_PRESET_BD;
      countPreset = sizeof(RUTE_PRESET_BD) / sizeof(RUTE_PRESET_BD[0]);
      nama        = "RUTE_B_D_LOOP";
      break;
    case RUTE_AC:
    default:
      srcPreset   = RUTE_DEFAULT_AC;
      countPreset = sizeof(RUTE_DEFAULT_AC) / sizeof(RUTE_DEFAULT_AC[0]);
      nama        = "RUTE_A_C_LOOP";
      break;
  }

  jumlahStepAktif = (countPreset <= MAX_PLAN_STEPS) ? countPreset : MAX_PLAN_STEPS;
  for (int i = 0; i < jumlahStepAktif; i++) {
    ruteAktif[i] = srcPreset[i];
    if (ruteAktif[i].trigger == TRIG_ENCODER_JARAK) {
      ruteAktif[i].pauseMs = (uint16_t)constrain((long)runtimeJedaStasiunMs, 0L, 60000L);
    }
  }
  runtimePulangOtomatis = true;
  strncpy(namaRuteAktif, nama, sizeof(namaRuteAktif) - 1);
  namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';
  return true;
}

// Muat preset default bawaan pabrik ke RAM (sesuai PILIHAN_RUTE_DEFAULT di Config.h)
inline void muatRuteDefaultAC(bool resetParam = false) {
  muatPresetRute(PILIHAN_RUTE_DEFAULT, resetParam);
}

// Cari indeks step awal rute pulang (U-Turn atau label PULANG_/PLG_, atau -1 bila tidak ada)
inline int cariIdxStepUTurn() {
  for (int i = 0; i < jumlahStepAktif; i++) {
    if (ruteAktif[i].action == ACT_UTURN_180) {
      return i;
    }
  }
  for (int i = 0; i < jumlahStepAktif; i++) {
    if (strncmp(ruteAktif[i].label, "PULANG_", 7) == 0 ||
        strncmp(ruteAktif[i].label, "PLG_", 4) == 0) {
      return i;
    }
  }
  return -1;
}

// Nama singkat trigger untuk tampilan & editor Serial Monitor
inline const char* namaTrigger(StepTrigger t) {
  switch (t) {
    case TRIG_IMMEDIATE:     return "LANGSUNG";
    case TRIG_T_KIRI:        return "TKIRI";
    case TRIG_CABANG_KANAN:  return "CKANAN";
    case TRIG_CABANG_KIRI:   return "CKIRI";
    case TRIG_PEREMPATAN:    return "S4";
    case TRIG_ENCODER_JARAK: return "ENC";
    default:                 return "?";
  }
}

// Nama singkat aksi untuk tampilan & editor Serial Monitor
inline const char* namaAksi(StepAction a) {
  switch (a) {
    case ACT_TURN_LEFT:        return "KIRI";
    case ACT_TURN_LEFT_PULANG: return "KIRI_P";
    case ACT_TURN_RIGHT:       return "KANAN";
    case ACT_CROSS_STRAIGHT:   return "LURUS";
    case ACT_PAUSE_ONLY:       return "JEDA";
    case ACT_UTURN_180:        return "UTURN";
    case ACT_STOP_MISSION:     return "STOP";
    default:                   return "?";
  }
}

// Parser token trigger dari perintah teks Serial Monitor
inline bool parseTriggerToken(String tok, StepTrigger& out) {
  tok.trim();
  tok.toUpperCase();
  if (tok == "LANGSUNG" || tok == "IMM" || tok == "IMMEDIATE") { out = TRIG_IMMEDIATE;     return true; }
  if (tok == "TKIRI"    || tok == "T_KIRI")                    { out = TRIG_T_KIRI;        return true; }
  if (tok == "CKANAN"   || tok == "CAB_KANAN")                 { out = TRIG_CABANG_KANAN;  return true; }
  if (tok == "CKIRI"    || tok == "CAB_KIRI")                  { out = TRIG_CABANG_KIRI;   return true; }
  if (tok == "S4"       || tok == "PEREMPATAN")                { out = TRIG_PEREMPATAN;    return true; }
  if (tok == "ENC"      || tok == "ENCODER" || tok == "JARAK") { out = TRIG_ENCODER_JARAK; return true; }
  return false;
}

// Parser token aksi dari perintah teks Serial Monitor
inline bool parseAksiToken(String tok, StepAction& out) {
  tok.trim();
  tok.toUpperCase();
  if (tok == "KIRI"   || tok == "LEFT")       { out = ACT_TURN_LEFT;        return true; }
  if (tok == "KIRI_P" || tok == "KIRI_PULANG"){ out = ACT_TURN_LEFT_PULANG; return true; }
  if (tok == "KANAN"  || tok == "RIGHT")      { out = ACT_TURN_RIGHT;       return true; }
  if (tok == "LURUS"  || tok == "CROSS")      { out = ACT_CROSS_STRAIGHT;   return true; }
  if (tok == "JEDA"   || tok == "PAUSE")      { out = ACT_PAUSE_ONLY;       return true; }
  if (tok == "UTURN"  || tok == "BALIK")      { out = ACT_UTURN_180;        return true; }
  if (tok == "STOP"   || tok == "SELESAI")    { out = ACT_STOP_MISSION;     return true; }
  return false;
}

// Isi field detail (lockout, offset maju, rem aktif, timeout) otomatis agar editor MAP_ADD/MAP_SET cukup 4 argumen
inline void isiDefaultStepDariAksi(PlanStep& s,
                                   StepTrigger trig,
                                   float targetCm,
                                   uint16_t pauseMs,
                                   StepAction act,
                                   const char* lbl) {
  memset(&s, 0, sizeof(s));
  strncpy(s.label, lbl, sizeof(s.label) - 1);
  s.trigger       = trig;
  s.targetJarakCm = (trig == TRIG_ENCODER_JARAK && targetCm > 0.0f) ? targetCm : 0.0f;
  s.pauseMs       = pauseMs;
  s.action        = act;

  if (trig == TRIG_IMMEDIATE) {
    s.lockoutMs = 0;
    s.brakePwm  = 0;
    s.brakeMs   = 0;
  } else if (trig == TRIG_ENCODER_JARAK) {
    s.lockoutMs       = 100;
    s.brakePwm        = 85;
    s.brakeMs         = 35;
    s.stopJikaSimpang = (s.targetJarakCm > 0.0f && s.targetJarakCm <= 95.0f);
    if (s.targetJarakCm > 0.0f) {
      long estMs = (long)((s.targetJarakCm / KECEPATAN_ESTIMASI_CM_PER_S) * 1500.0f) + 500L;
      s.maxDurasiMs = (uint16_t)constrain(estMs, 1500L, 60000L);
    }
  } else {
    s.lockoutMs = 300;
    s.brakePwm  = 85;
    s.brakeMs   = 35;
  }

  if (act == ACT_TURN_LEFT || act == ACT_TURN_LEFT_PULANG) {
    s.preMoveMs = OFFSET_MAJU_BELOK_MS;
  } else if (act == ACT_TURN_RIGHT) {
    s.preMoveMs = OFFSET_MAJU_SIMPANG_KANAN_MS;
    if (trig == TRIG_CABANG_KANAN) {
      s.lockoutMs = WAKTU_MINIMAL_RUAS2_MS;
    }
  } else if (act == ACT_CROSS_STRAIGHT && pauseMs == 0) {
    s.brakePwm = 0;
    s.brakeMs  = 0;
  }
}

// Cetak tabel mapping aktif lengkap ke Serial Monitor
inline void tampilkanTabelMapping() {
  Serial.println("\n================ [TABEL MAPPING RUTE AKTIF] ================");
  Serial.print("Nama Rute   : "); Serial.println(namaRuteAktif);
  Serial.print("Jumlah Step : "); Serial.print(jumlahStepAktif);
  Serial.print(" / ");            Serial.println(MAX_PLAN_STEPS);
  Serial.print("Kalibrasi   : PPR="); Serial.print(runtimeTicksPerPutaran, 1);
  Serial.print(" | Jeda=");       Serial.print(runtimeJedaStasiunMs);
  Serial.print("ms | Pulang=");   Serial.print(runtimePulangOtomatis ? "AUTO" : "STANDBY");
  Serial.print(" | AutoStart=");  Serial.println(runtimeAutoStart ? "ON" : "OFF");
  Serial.println("------------------------------------------------------------");
  Serial.println("Idx | Label               | Trig     | JarakCm | PauseMs | Aksi");
  Serial.println("----+---------------------+----------+---------+---------+-------");

  for (int i = 0; i < jumlahStepAktif; i++) {
    const PlanStep& s = ruteAktif[i];
    if (i < 10) Serial.print(" ");
    Serial.print(i);
    Serial.print("  | ");

    Serial.print(s.label);
    int lenLbl = strlen(s.label);
    for (int p = lenLbl; p < 19; p++) Serial.print(" ");
    Serial.print(" | ");

    const char* nt = namaTrigger(s.trigger);
    Serial.print(nt);
    for (int p = strlen(nt); p < 8; p++) Serial.print(" ");
    Serial.print(" | ");

    if (s.trigger == TRIG_ENCODER_JARAK) {
      Serial.print(s.targetJarakCm, 1);
      if (s.targetJarakCm < 10.0f)       Serial.print("    ");
      else if (s.targetJarakCm < 100.0f) Serial.print("   ");
      else                               Serial.print("  ");
    } else {
      Serial.print("-      ");
    }
    Serial.print(" | ");

    Serial.print(s.pauseMs);
    if (s.pauseMs < 10)        Serial.print("      ");
    else if (s.pauseMs < 100)  Serial.print("     ");
    else if (s.pauseMs < 1000) Serial.print("    ");
    else                       Serial.print("   ");
    Serial.print(" | ");

    Serial.println(namaAksi(s.action));
  }
  Serial.println("============================================================\n");
}
