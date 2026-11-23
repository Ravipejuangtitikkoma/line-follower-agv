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
  TRIG_PEREMPATAN     // Simpang 4 / garis melintang: gelap semua atau isPerempatan (toleransi 35ms)
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
  StepTrigger trigger;        // Kondisi simpang pemicu
  uint16_t    lockoutMs;      // Waktu minimal jalan sebelum trigger aktif (TA Lockout Timer)
  uint16_t    preMoveMs;      // Durasi offset maju (59ms) atau penetrasi PD (495ms) sebelum rem
  bool        preMoveUsePD;   // false = offset maju langsung, true = penetrasi tracking PD
  uint8_t     brakePwm;       // Daya rem aktif berlawanan arah (PWM)
  uint16_t    brakeMs;        // Durasi rem aktif (ms)
  uint16_t    pauseMs;        // Jeda diam di stasiun/simpang sebelum aksi (ms)
  StepAction  action;         // Aksi utama yang dijalankan
};

// Indeks step khusus untuk perintah cepat operator (LANJUT / PULANG)
const int IDX_STEP_TITIK_A = 2; // Step jeda 3 detik di Titik A sebelum melintasi S4-1
const int IDX_STEP_UTURN_B = 4; // Step awal rute pulang (U-Turn 180° di Titik B)

// ====================================================================
// TABEL RUTE PETA RIIL PULANG-PERGI (START -> A -> B -> START)
// Pemetaan 1:1 terhadap 25 State FSM sebelumnya:
//   Idx 0: State 1..3   | Idx 1: State 4..6   | Idx 2: State 7..9
//   Idx 3: State 10..12 | Idx 4: State 13     | Idx 5: State 14..16
//   Idx 6: State 17..19 | Idx 7: State 20..21 | Idx 8: State 22..23
//   Idx 9: State 24..25
// ====================================================================
const PlanStep RUTE_REAL_PP[] = {
  // label,               trigger,             lockout,                    preMove,                    usePD, brkPwm, brkMs, pause,                    action
  { "S1_BELOK_KIRI",      TRIG_T_KIRI,         250,                        OFFSET_MAJU_BELOK_MS,       false, 85,     35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT        }, // 0
  { "S2_BELOK_KANAN",     TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     OFFSET_MAJU_SIMPANG_KANAN_MS, false, 85,   35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 1
  { "TITIK_A_S4_1",       TRIG_PEREMPATAN,     350,                        0,                          false, 85,     35,    DURASI_JEDA_TITIK_A_MS,   ACT_CROSS_STRAIGHT   }, // 2
  { "TITIK_B_S4_2",       TRIG_PEREMPATAN,     500,                        DURASI_PENETRASI_TITIK_B_MS, true, 65,     30,    DURASI_JEDA_TITIK_B_MS,   ACT_PAUSE_ONLY       }, // 3
  { "UTURN_180_B",        TRIG_IMMEDIATE,      0,                          0,                          false, 0,      0,     0,                        ACT_UTURN_180        }, // 4
  { "PULANG_S4_1",        TRIG_PEREMPATAN,     350,                        0,                          false, 85,     35,    DURASI_STOP_S4_PULANG_MS, ACT_CROSS_STRAIGHT   }, // 5
  { "PULANG_S4_2",        TRIG_PEREMPATAN,     400,                        0,                          false, 85,     35,    DURASI_STOP_S4_PULANG_MS, ACT_CROSS_STRAIGHT   }, // 6
  { "PULANG_T_KIRI",      TRIG_T_KIRI,         350,                        OFFSET_MAJU_BELOK_MS,       false, 85,     35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_LEFT_PULANG }, // 7
  { "PULANG_CAB_KANAN",   TRIG_CABANG_KANAN,   WAKTU_MINIMAL_RUAS2_MS,     OFFSET_MAJU_SIMPANG_KANAN_MS, false, 85,   35,    JEDA_DIEM_SIMPANG_MS,     ACT_TURN_RIGHT       }, // 8
  { "FINISH_START",       TRIG_PEREMPATAN,     350,                        0,                          false, 85,     35,    50,                       ACT_STOP_MISSION     }  // 9
};

const int JUMLAH_STEP_RUTE_REAL = sizeof(RUTE_REAL_PP) / sizeof(RUTE_REAL_PP[0]);
