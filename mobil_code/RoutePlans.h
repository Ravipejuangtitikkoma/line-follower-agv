#pragma once
#include <Arduino.h>
#include "Config.h"

// ====================================================================
// MODUL TABEL RUTE DEKLARATIF (DATA-DRIVEN PLAN ENGINE)
// File: RoutePlans.h
// Deskripsi: Memisahkan definisi urutan rute (data) dari mesin eksekutor
//            navigasi agar penambahan/perubahan rute tidak perlu mengubah
//            logika inti di NavigationFSM.h.
// ====================================================================

// Jenis kondisi simpang pemicu transisi ke fase aksi
enum StepTrigger : uint8_t {
  TRIG_IMMEDIATE,     // Langsung eksekusi tanpa menunggu sensor (seperti SENS_NONE_MOTION)
  TRIG_T_KIRI,        // Simpang T melintang / kiri: gelap semua [11111] atau (b1 && b2 && b3)
  TRIG_CABANG_KANAN,  // Cabang kanan topologis: (b5 && b4) && (b3 || totalAktif>=3 || gelapSemua)
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

// Struktur deklaratif 1 langkah rute AGV
struct PlanStep {
  const char* label;          // Nama step untuk telemetri Serial Monitor
  StepTrigger trigger;        // Kondisi simpang / jarak pemicu
  uint16_t    lockoutMs;      // Waktu minimal jalan sebelum trigger aktif (TA Lockout Timer)
  float       targetJarakCm;  // Target jarak encoder dalam cm (hanya dipakai jika TRIG_ENCODER_JARAK)
  uint16_t    maxDurasiMs;    // Batas waktu pengaman step encoder (0 = nonaktif)
  bool        stopJikaSimpang;// true = hentikan paksa jika simpang N6 terinjak (khusus CHECKPOINT_A)
  uint16_t    preMoveMs;      // Durasi offset maju (59ms) atau penetrasi PD sebelum rem
  bool        preMoveUsePD;   // false = offset maju langsung, true = penetrasi tracking PD
  uint8_t     brakePwm;       // Daya rem aktif berlawanan arah (PWM)
  uint16_t    brakeMs;        // Durasi rem aktif (ms)
  uint16_t    pauseMs;        // Jeda diam di stasiun/simpang sebelum aksi (ms)
  StepAction  action;         // Aksi utama yang dijalankan
};

// Indeks step khusus untuk perintah cepat operator (LANJUT / PULANG)
const int IDX_STEP_CHECKPOINT_A = 2; // Step N2 -> Checkpoint A (Encoder ~50 cm + Jeda 3 detik)
const int IDX_STEP_CHECKPOINT_C = 3; // Step A -> Checkpoint C (Encoder ~200 cm + Jeda 3 detik)
const int IDX_STEP_UTURN_C      = 4; // Step awal rute pulang (U-Turn 180° setelah C)
const int IDX_STEP_TITIK_A      = IDX_STEP_CHECKPOINT_A;
const int IDX_STEP_UTURN_B      = IDX_STEP_UTURN_C;

// ====================================================================
// TABEL RUTE PETA RIIL PULANG-PERGI (START -> N0 -> N2 -> A -> C -> START)
// ====================================================================
const PlanStep RUTE_REAL_PP[] = {
  // label,               trigger,             lockout,                    targetCm,         maxDurMs,             stopN6, preMove,                    usePD, brkPwm, brkMs, pause,                    action
  { "N0_BELOK_KIRI",      TRIG_T_KIRI,         250,                        0.0f,             0,                    false,  OFFSET_MAJU_BELOK_MS,       false, 85,     35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 0
  { "N2_BELOK_KANAN",     TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,  OFFSET_MAJU_SIMPANG_KANAN_MS, false, 85,   35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 1
  { "CHECKPOINT_A",       TRIG_ENCODER_JARAK,  100,                        JARAK_N2_KE_A_CM, TIMEOUT_ENCODER_A_MS, true,   0,                          false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_PAUSE_ONLY       }, // 2
  { "CHECKPOINT_C",       TRIG_ENCODER_JARAK,  100,                        JARAK_A_KE_C_CM,  TIMEOUT_ENCODER_C_MS, false,  0,                          false, 85,     35,    DURASI_JEDA_TITIK_C_MS,   ACT_PAUSE_ONLY       }, // 3
  { "UTURN_180_C",        TRIG_IMMEDIATE,      0,                          0.0f,             0,                    false,  0,                          false, 0,      0,     0,                        ACT_UTURN_180        }, // 4
  { "PULANG_N10",         TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,  0,                          false, 85,     35,    DURASI_STOP_S4_PULANG_MS, ACT_CROSS_STRAIGHT   }, // 5
  { "PULANG_N6",          TRIG_PEREMPATAN,     400,                        0.0f,             0,                    false,  0,                          false, 85,     35,    DURASI_STOP_S4_PULANG_MS, ACT_CROSS_STRAIGHT   }, // 6
  { "PULANG_N2_KIRI",     TRIG_T_KIRI,         350,                        0.0f,             0,                    false,  OFFSET_MAJU_BELOK_MS,       false, 85,     35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 7
  { "PULANG_N0_KANAN",    TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     0.0f,             0,                    false,  OFFSET_MAJU_SIMPANG_KANAN_MS, false, 85,   35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 8
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0.0f,             0,                    false,  0,                          false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 9
};

const int JUMLAH_STEP_RUTE_REAL = sizeof(RUTE_REAL_PP) / sizeof(RUTE_REAL_PP[0]);
