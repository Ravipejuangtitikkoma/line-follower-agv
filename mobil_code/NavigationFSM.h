#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Sensor.h"
#include "NodeDetector.h"

// ====================================================================
// MODUL HIERARCHICAL FINITE STATE MACHINE (HFSM) NAVIGASI AGV
// File: NavigationFSM.h
// Rute Peta Riil: START -> Simpang 3 Kiri -> Simpang 3 Kanan -> Titik A ->
//                 Simpang 4 Ke-1 (Lurus) -> Simpang 4 Ke-2 (Titik B)
// ====================================================================

// --- 1. LAYER 1: STATUS MISI GLOBAL (KENDALI LORA / OPERATOR) ---
enum MisiStatus {
  MISI_STANDBY,          // Diam di tempat, menunggu sinyal LoRa / Serial
  MISI_JALAN_RUTE_REAL,  // Misi Aktif: Menempuh rute peta riil (START -> A -> B)
  MISI_EMERGENCY_STOP    // Berhenti darurat karena rintangan / tombol stop
};

// --- 2. LAYER 2: STATE NAVIGASI DETAIL MISI RUTE REAL (24 STATE PP) ---
enum NavState {
  STATE_IDLE_PARKIR,                 // 0: Mobil diam di tempat
  STATE_1_START_MAJU_S1,             // 1: Maju lurus PD sampai deteksi Simpang 3 Pertama [1 1 1 1 1]
  STATE_2_BELOK_KIRI_SWEEP,          // 2: Putar poros kiri sampai sensor mulai menangkap garis baru
  STATE_3_LURUSKAN_BELAKANG_KIRI,    // 3: Menggelir bodi belakang kiri (menyelaraskan bodi 25cm & ban lebar)
  STATE_4_MAJU_MENUJU_SIMPANG_KANAN, // 4: Maju lurus PD sampai sensor kanan deteksi cabang simpang
  STATE_5_BELOK_KANAN_SIMPANG_3,     // 5: Putar poros kanan sampai mengunci garis cabang simpang 3
  STATE_6_LURUSKAN_BELAKANG_KANAN,   // 6: Menggelir bodi belakang kanan sebentar
  STATE_7_MAJU_PENETRASI_TITIK_A,    // 7: Maju sedikit memasuki zona stasiun A
  STATE_8_STOP_TITIK_A,              // 8: Berhenti 3 detik di Titik A (jeda bongkar/muat)
  STATE_9_MAJU_LEWATI_SIMPANG4_1,    // 9: Maju lewati Simpang 4 pertama [1 1 1 1 1] secara lurus
  STATE_10_MAJU_MENUJU_SIMPANG4_2,   // 10: Maju menuju Simpang 4 kedua [1 1 1 1 1]
  STATE_11_MAJU_LURUS_STOP_TITIK_B,  // 11: Melintas lurus sedikit lalu berhenti sempurna di Titik B
  STATE_12_PARKIR_TITIK_B,           // 12: Parkir di Titik B (jeda 3 detik atau tunggu perintah PULANG)
  STATE_13_PUTAR_180_TITIK_B,        // 13: Maju 40ms, putar 180° hingga S3 kunci garis tengah kembali
  STATE_14_MAJU_PULANG_KE_S4_1,      // 14: Maju PD menyusuri Ruas 4 menuju Simpang 4-1 jalur pulang
  STATE_15_STOP_S4_1_PULANG,         // 15: Berhenti sejenak 1.5 detik di Simpang 4-1
  STATE_16_LEWATI_S4_1_PULANG,       // 16: Maju lurus melintasi Simpang 4-1
  STATE_17_MAJU_PULANG_KE_S4_2,      // 17: Maju PD menuju Simpang 4-2 jalur pulang
  STATE_18_STOP_S4_2_PULANG,         // 18: Berhenti sejenak 1.5 detik di Simpang 4-2
  STATE_19_LEWATI_S4_2_PULANG,       // 19: Maju lurus melintasi Simpang 4-2
  STATE_20_MAJU_KE_SIMPANG_T_KIRI,   // 20: Maju PD menuju Simpang T melintang
  STATE_21_BELOK_KIRI_SIMPANG_T,     // 21: Deteksi Full Hitam [11111] -> Offset 59ms -> Putar Poros Kiri 90°
  STATE_22_MAJU_KE_CABANG_KANAN,     // 22: Maju PD menyusuri Ruas 2 menuju cabang kanan ke START
  STATE_23_BELOK_KANAN_KE_START,     // 23: Deteksi Cabang Kanan -> Offset 59ms -> Putar Poros Kanan 90°
  STATE_24_MAJU_FINISH_START,        // 24: Maju menyusuri garis menuju titik START
  STATE_SELESAI_MISI                 // 25: Misi Pulang-Pergi Selesai Penuh! Parkir di START
};

// Variabel Global FSM
static MisiStatus statusMisiSaatIni = MULAI_DENGAN_STANDBY ? MISI_STANDBY : MISI_JALAN_RUTE_REAL;
static NavState   stateNavSaatIni   = STATE_1_START_MAJU_S1;
static unsigned long waktuMasukState        = 0;
static unsigned long waktuSimpangTerlewati  = 0;
static unsigned long lastPdUpdateTime       = 0;
static float errSebelumnyaPD                = 0.0;
static bool simpangPernahTerdeteksi        = false;
static bool sudahLepasGarisLama            = false;
static bool sudahAutoAlignStart            = false;
static bool modeTesBelokKananSaja          = false;
static int  konfirmasiCabangKanan          = 0;

// Fungsi helper pencatat transisi state
inline void gantiState(NavState stateBaru, const char* alasan) {
  stateNavSaatIni  = stateBaru;
  waktuMasukState  = millis();
  lastPdUpdateTime = 0;
  berhentiTotal();
  Serial.print("\n>>> [FSM TRANSISI] -> ");
  switch (stateBaru) {
    case STATE_1_START_MAJU_S1:             Serial.print("[STATE 1] Maju Start ke Simpang 3 Kiri"); break;
    case STATE_2_BELOK_KIRI_SWEEP:          Serial.print("[STATE 2] Belok Kiri Sweep (Target: S3 & S4)"); break;
    case STATE_3_LURUSKAN_BELAKANG_KIRI:    Serial.print("[STATE 3] Menggelir Bodi Belakang Kiri (25cm)"); break;
    case STATE_4_MAJU_MENUJU_SIMPANG_KANAN: Serial.print("[STATE 4] Maju Menuju Simpang Cabang Kanan"); break;
    case STATE_5_BELOK_KANAN_SIMPANG_3:     Serial.print("[STATE 5] Belok Kanan Kunci Simpang 3"); break;
    case STATE_6_LURUSKAN_BELAKANG_KANAN:   Serial.print("[STATE 6] Luruskan Bodi Belakang Kanan Sebentar"); break;
    case STATE_7_MAJU_PENETRASI_TITIK_A:    Serial.print("[STATE 7] Maju Penetrasi Sedikit ke Titik A"); break;
    case STATE_8_STOP_TITIK_A:              Serial.print("[STATE 8] Berhenti 3 Detik di Titik A"); break;
    case STATE_9_MAJU_LEWATI_SIMPANG4_1:    Serial.print("[STATE 9] Maju Lewati Simpang 4 Ke-1 (Lurus)"); break;
    case STATE_10_MAJU_MENUJU_SIMPANG4_2:   Serial.print("[STATE 10] Maju Menuju Simpang 4 Ke-2 (Titik B)"); break;
    case STATE_11_MAJU_LURUS_STOP_TITIK_B:  Serial.print("[STATE 11] Maju Sedikit & Stop Total di Titik B"); break;
    case STATE_12_PARKIR_TITIK_B:           Serial.print("[STATE 12] Parkir Jeda di Titik B"); break;
    case STATE_13_PUTAR_180_TITIK_B:        Serial.print("[STATE 13] Putar Balik 180° di Titik B"); break;
    case STATE_14_MAJU_PULANG_KE_S4_1:      Serial.print("[STATE 14] Maju Pulang ke Simpang 4-1"); break;
    case STATE_15_STOP_S4_1_PULANG:         Serial.print("[STATE 15] Stop Sejenak di Simpang 4-1"); break;
    case STATE_16_LEWATI_S4_1_PULANG:       Serial.print("[STATE 16] Maju Lurus Melintasi Simpang 4-1"); break;
    case STATE_17_MAJU_PULANG_KE_S4_2:      Serial.print("[STATE 17] Maju Pulang ke Simpang 4-2"); break;
    case STATE_18_STOP_S4_2_PULANG:         Serial.print("[STATE 18] Stop Sejenak di Simpang 4-2"); break;
    case STATE_19_LEWATI_S4_2_PULANG:       Serial.print("[STATE 19] Maju Lurus Melintasi Simpang 4-2"); break;
    case STATE_20_MAJU_KE_SIMPANG_T_KIRI:   Serial.print("[STATE 20] Maju Pulang ke Simpang T (Full Hitam)"); break;
    case STATE_21_BELOK_KIRI_SIMPANG_T:     Serial.print("[STATE 21] Belok Kiri di Simpang T Full Hitam"); break;
    case STATE_22_MAJU_KE_CABANG_KANAN:     Serial.print("[STATE 22] Maju Ruas 2 ke Cabang Kanan"); break;
    case STATE_23_BELOK_KANAN_KE_START:     Serial.print("[STATE 23] Belok Kanan ke Arah START"); break;
    case STATE_24_MAJU_FINISH_START:        Serial.print("[STATE 24] Maju Menuju Garis START"); break;
    case STATE_SELESAI_MISI:                Serial.print("[SELESAI] Misi Pulang-Pergi Selesai! Parkir di START"); break;
    default: Serial.print("[IDLE]"); break;
  }
  Serial.print(" | Alasan: ");
  Serial.println(alasan);
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
    errSebelumnyaPD = -2.0;
    return;
  }
  if (line.b2 == 1 && line.b3 == 0 && line.b4 == 0) {
    // Garis bergeser ke S2 (Kiri Tengah) -> Hentakan responsif kedua roda kanan (145 PWM), kiri pelan (15 PWM)
    setMotorLangsung(15, 145);
    errSebelumnyaPD = -1.0;
    return;
  }

  // -------------------------------------------------------------
  // KASUS 3: FAKTOR X MEMBUAT MOBIL MELENCENG KE KIRI (Garis di Kanan: S5 atau S4)
  // Roda kiri harus menghentak agar mobil masuk kembali ke jalur lurus!
  // -------------------------------------------------------------
  if (line.b5 == 1 && line.b3 == 0) {
    // Garis mepet di S5 (Kanan Luar) -> Hentakan kuat kedua roda kiri (165 PWM), kanan rem/mundur (-60 PWM)
    setMotorLangsung(PWM_TEPI_MAJU, -PWM_TEPI_MUNDUR);
    errSebelumnyaPD = 2.0;
    return;
  }
  if (line.b4 == 1 && line.b3 == 0 && line.b2 == 0) {
    // Garis bergeser ke S4 (Kanan Tengah) -> Hentakan responsif kedua roda kiri (145 PWM), kanan pelan (15 PWM)
    setMotorLangsung(145, 15);
    errSebelumnyaPD = 1.0;
    return;
  }

  // -------------------------------------------------------------
  // KASUS 4: KONDISI LURUS DI JALUR (TUNING ASLI USER YANG SUDAH PAS)
  // Sensor tengah S3 membaca garis, mobil melaju mulus dengan tuning asli 100%
  // Dipacing 15ms agar loop cepat 5ms untuk deteksi simpang tidak mengubah respon PD/smoothing
  // -------------------------------------------------------------
  unsigned long nowPd = millis();
  if (nowPd - lastPdUpdateTime < 15) {
    return;
  }
  lastPdUpdateTime = nowPd;

  float errorSaatIni = line.error;
  if (line.isLurus) {
    errorSaatIni = 0.0;
  }

  float P = Kp * errorSaatIni;
  float D = Kd * (errorSaatIni - errSebelumnyaPD);
  float koreksi = P + D;
  errSebelumnyaPD = errorSaatIni;

  int vKiri  = kecBase + (int)koreksi + TRIM_KIRI;
  int vKanan = kecBase - (int)koreksi + TRIM_KANAN;
  setKecepatanMotor(vKiri, vKanan);
}

// Inisialisasi FSM
inline void initNavigationFSM() {
  statusMisiSaatIni       = MULAI_DENGAN_STANDBY ? MISI_STANDBY : MISI_JALAN_RUTE_REAL;
  stateNavSaatIni         = STATE_1_START_MAJU_S1;
  waktuMasukState         = millis();
  lastPdUpdateTime        = 0;
  modeTesBelokKananSaja   = false;
  simpangPernahTerdeteksi = false;
  sudahLepasGarisLama     = false;
  sudahAutoAlignStart     = false;
  konfirmasiCabangKanan   = 0;
  waktuSimpangTerlewati   = 0;
  errSebelumnyaPD         = 0.0;
  berhentiTotal();
  Serial.println("[FSM] Modul Navigasi FSM 24-State Peta Riil Diinisialisasi.");
}

// Parser Perintah Operator (Serial Monitor / Paket LoRa)
inline void prosesPerintah(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  Serial.print("[COMMAND DITERIMA] -> ");
  Serial.println(cmd);

  if (cmd == "START" || cmd == "START_AB" || cmd == "1") {
    statusMisiSaatIni       = MISI_JALAN_RUTE_REAL;
    modeTesBelokKananSaja   = false;
    simpangPernahTerdeteksi = false;
    sudahAutoAlignStart     = false;
    sudahLepasGarisLama     = false;
    konfirmasiCabangKanan   = 0;
    gantiState(STATE_1_START_MAJU_S1, "Perintah Operator START (Misi START -> A -> B)");
  }
  else if (cmd == "LANJUT" || cmd == "GO" || cmd == "NEXT") {
    if (stateNavSaatIni == STATE_8_STOP_TITIK_A) {
      gantiState(STATE_9_MAJU_LEWATI_SIMPANG4_1, "Perintah Operator LANJUT dari Titik A ke Titik B");
    } else {
      Serial.println("[INFO] Perintah LANJUT diabaikan (Robot tidak sedang di Titik A).");
    }
  }
  else if (cmd == "PULANG" || cmd == "BALIK" || cmd == "RETURN") {
    statusMisiSaatIni     = MISI_JALAN_RUTE_REAL;
    modeTesBelokKananSaja = false;
    gantiState(STATE_13_PUTAR_180_TITIK_B, "Perintah Operator PULANG (Rute B -> START Dimulai)");
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
    Serial.println(">>> [RESET] FSM Di-reset ke State Awal.");
  }
  else if (cmd == "STATUS") {
    Serial.println("\n--- [STATUS AGV DIAGNOSTIK] ---");
    Serial.print("Misi: ");
    switch (statusMisiSaatIni) {
      case MISI_STANDBY:          Serial.println("STANDBY"); break;
      case MISI_JALAN_RUTE_REAL:  Serial.println("JALAN RUTE REAL (START -> A -> B)"); break;
      case MISI_EMERGENCY_STOP:   Serial.println("EMERGENCY STOP"); break;
    }
    Serial.print("State Nav: ");
    Serial.println(stateNavSaatIni);
    Serial.println("--------------------------------\n");
  }
  else if (cmd == "TEST_KANAN") {
    Serial.println("\n>>> [DIAGNOSTIK] UJI PUTAR POROS KANAN (2 Detik)...");
    putarPorosKanan(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR);
    delay(2000);
    berhentiTotal();
    Serial.println(">>> [SELESAI] Cek: Roda kiri maju & roda kanan mundur!\n");
  }
  else if (cmd == "TEST_KIRI") {
    berhentiTotal();
    Serial.println("\n>>> [DIAGNOSTIK] UJI PUTAR POROS KIRI (2 Detik)...");
    putarPorosKiri(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR);
    delay(2000);
    berhentiTotal();
    Serial.println(">>> [SELESAI] Cek: Roda kanan maju (Pin 16) & roda kiri mundur (Pin 26)!\n");
  }
  else if (cmd == "TEST_BELOK_KANAN") {
    statusMisiSaatIni     = MISI_JALAN_RUTE_REAL;
    modeTesBelokKananSaja = true;
    konfirmasiCabangKanan = 0;
    sudahLepasGarisLama   = false;
    gantiState(STATE_4_MAJU_MENUJU_SIMPANG_KANAN, "Mode Tes Khusus: Maju Lurus -> Deteksi Simpang Kanan -> Belok Kanan 90°");
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

// Telemetri Serial Monitor FSM
inline void tampilkanTelemetriFSM(const LineData& line) {
  Serial.print("FSM: [");
  switch (stateNavSaatIni) {
    case STATE_1_START_MAJU_S1:             Serial.print("1:MAJU_S1"); break;
    case STATE_2_BELOK_KIRI_SWEEP:          Serial.print("2:BELOK_KIRI"); break;
    case STATE_3_LURUSKAN_BELAKANG_KIRI:    Serial.print("3:GELIR_KIRI"); break;
    case STATE_4_MAJU_MENUJU_SIMPANG_KANAN: Serial.print("4:MAJU_RUAS2"); break;
    case STATE_5_BELOK_KANAN_SIMPANG_3:     Serial.print("5:BELOK_KANAN"); break;
    case STATE_6_LURUSKAN_BELAKANG_KANAN:   Serial.print("6:GELIR_KANAN"); break;
    case STATE_7_MAJU_PENETRASI_TITIK_A:    Serial.print("7:MAJU_A"); break;
    case STATE_8_STOP_TITIK_A:              Serial.print("8:STOP_A_3S"); break;
    case STATE_9_MAJU_LEWATI_SIMPANG4_1:    Serial.print("9:LEWAT_S4_1"); break;
    case STATE_10_MAJU_MENUJU_SIMPANG4_2:   Serial.print("10:MAJU_S4_2"); break;
    case STATE_11_MAJU_LURUS_STOP_TITIK_B:  Serial.print("11:STOP_B"); break;
    case STATE_12_PARKIR_TITIK_B:           Serial.print("12:PARK_B"); break;
    case STATE_13_PUTAR_180_TITIK_B:        Serial.print("13:U_TURN"); break;
    case STATE_14_MAJU_PULANG_KE_S4_1:      Serial.print("14:PULANG_S4_1"); break;
    case STATE_15_STOP_S4_1_PULANG:         Serial.print("15:STOP_S4_1"); break;
    case STATE_16_LEWATI_S4_1_PULANG:       Serial.print("16:LEWAT_S4_1"); break;
    case STATE_17_MAJU_PULANG_KE_S4_2:      Serial.print("17:PULANG_S4_2"); break;
    case STATE_18_STOP_S4_2_PULANG:         Serial.print("18:STOP_S4_2"); break;
    case STATE_19_LEWATI_S4_2_PULANG:       Serial.print("19:LEWAT_S4_2"); break;
    case STATE_20_MAJU_KE_SIMPANG_T_KIRI:   Serial.print("20:PULANG_T"); break;
    case STATE_21_BELOK_KIRI_SIMPANG_T:     Serial.print("21:BELOK_KIRI_T"); break;
    case STATE_22_MAJU_KE_CABANG_KANAN:     Serial.print("22:PULANG_CAB"); break;
    case STATE_23_BELOK_KANAN_KE_START:     Serial.print("23:BELOK_KANAN_START"); break;
    case STATE_24_MAJU_FINISH_START:        Serial.print("24:MAJU_START"); break;
    case STATE_SELESAI_MISI:                Serial.print("SELESAI"); break;
    default: Serial.print("IDLE"); break;
  }
  Serial.print("] | Line: [");
  Serial.print(line.b1); Serial.print(line.b2); Serial.print(line.b3); Serial.print(line.b4); Serial.print(line.b5);
  Serial.print("] | Err: ");
  Serial.print(line.error, 2);
  Serial.print(" | ");
  if (line.isGelapSemua()) Serial.print("[GELAP_SEMUA]");
  else if (line.isLurus) Serial.print("[LURUS]");
  else if (line.isTikunganKanan) Serial.print("[TIK_KANAN]");
  else if (line.isTikunganKiri) Serial.print("[TIK_KIRI]");
  else Serial.print("[TRACK]");
  Serial.println();
}

// Eksekusi Finite State Machine Navigasi Utama (Dipanggil di loop())
inline void updateNavigationFSM(const LineData& line) {
  unsigned long durasiDiState = millis() - waktuMasukState;

  // LAYER 1: CEK STATUS MISI
  switch (statusMisiSaatIni) {
    case MISI_STANDBY:
      berhentiTotal();
      return;

    case MISI_EMERGENCY_STOP:
      berhentiTotal();
      return;

    case MISI_JALAN_RUTE_REAL:
      break; // Lanjut ke Layer 2 di bawah
  }

  // LAYER 2: EKSEKUSI 11 STATE NAVIGASI MISI PETA RIIL (START -> A -> B)
  switch (stateNavSaatIni) {

    // ==================================================================
    // STATE 1: MAJU LURUS SAMPAI DETEKSI SIMPANG 3 PERTAMA GELAP SEMUA [1 1 1 1 1]
    // Dilengkapi Pre-Flight Zero-Yaw Auto-Centering (Mengoreksi peletakan awal miring)
    // ==================================================================
    case STATE_1_START_MAJU_S1: {
      // PROSEDUR OUT-OF-THE-BOX: ZERO-YAW AUTO-ALIGNMENT DI TEMPAT
      if (!sudahAutoAlignStart) {
        if (line.isLurus || (line.b3 == 1 && line.b1 == 0 && line.b5 == 0 && abs(line.error) < 0.35)) {
          sudahAutoAlignStart = true;
          berhentiTotal();
          delay(20);
        } else if (durasiDiState < (unsigned long)WAKTU_AUTO_ALIGN_MAX_MS) {
          // 25ms pertama gunakan PWM Kick (160) agar roda tidak mematung, lalu PWM 138 stabil
          int pwmAktif = (durasiDiState < (unsigned long)DURASI_AUTO_ALIGN_KICK_MS) ? PWM_AUTO_ALIGN_KICK : PWM_AUTO_ALIGN;

          if (line.error > 0.3) {
            // Mencong ke kiri (garis di kanan) -> putar poros ke kanan (roda kiri maju, kanan mundur)
            putarPorosKanan(pwmAktif, pwmAktif);
          } else if (line.error < -0.3) {
            // Mencong ke kanan (garis di kiri) -> putar poros ke kiri (roda kanan maju, kiri mundur)
            putarPorosKiri(pwmAktif, pwmAktif + 8);
          } else {
            // Sensor S3 sudah mengunci garis -> Rem aktif lawan arah mikro mematikan inersia
            if (line.error < 0) putarPorosKanan(130, 130);
            else putarPorosKiri(130, 130);
            delay(25);
            berhentiTotal();
            delay(20);
            sudahAutoAlignStart = true;
          }
          return; // Tunggu kalibrasi mikro selesai sebelum gas maju
        } else {
          sudahAutoAlignStart = true;
          berhentiTotal();
          delay(20);
        }
      }

      jalankanTrackingPD(line, KEC_BASE);

      // Kondisi Simpang 3 Pertama: Sensor membaca gelap semua atau 3 sensor kiri-tengah aktif
      bool simpang1Ditemukan = (durasiDiState > 250) && 
                               (line.isGelapSemua() || (line.b1 == 1 && line.b2 == 1 && line.b3 == 1));

      if (simpang1Ditemukan) {
        // Jika OFFSET_MAJU_BELOK_MS > 0, maju sedikit melewati garis (59ms) agar poros roda pas di pusat simpang
        if (OFFSET_MAJU_BELOK_MS > 0) {
          setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
          delay(OFFSET_MAJU_BELOK_MS);
        }

        // Rem aktif keras mematikan inersia maju seketika agar sensor tidak berlebihan/bablas
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(JEDA_DIEM_SIMPANG_MS);

        sudahLepasGarisLama = false;
        gantiState(STATE_2_BELOK_KIRI_SWEEP, "Simpang 3 Kiri Terdeteksi -> Mulai Belok Kiri");
      }
      break;
    }

    // ==================================================================
    // STATE 2: BELOK KIRI TAHAP 1 - SWEEP 2-FASE (CLOSED-LOOP ANTI FALSE-LOCK)
    // Fase A: Wajib lepas dari garis lurus lama (180ms + putih atau batas 240ms)
    // Fase B: Baru mengunci garis baru saat sensor menyapu dari kanan ke tengah (S3 & S4)
    // ==================================================================
    case STATE_2_BELOK_KIRI_SWEEP: {
      // 1. Putar kiri: Kick 185 PWM (30ms) lalu stabil di PWM 134
      if (durasiDiState < DURASI_RAMP_MS) {
        putarPorosKiriHalus(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
      } else {
        // FASE RADAR: Hanya aktif setelah bodi berputar signifikan & lepas sudut lama
        if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS && line.b4 == 1) {
          putarPorosKiri(KEC_SESUAI_KIRI_MAJU, KEC_SESUAI_KIRI_MUNDUR);
        } else {
          putarPorosKiri(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KIRI_MUNDUR);
        }
      }

      // 2. FASE A: DETEKSI CLOSED-LOOP LEPAS DARI GARIS LAMA (Keluar ke Lantai Putih)
      if (!sudahLepasGarisLama) {
        if ((durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS && (line.totalAktif == 0 || line.isGarisHilang)) ||
            (durasiDiState >= WAKTU_LEPAS_SUDUT_KIRI_MS)) {
          sudahLepasGarisLama = true;
        }
      }

      // 3. FASE B: KUNCI GARIS BARU 90° (Hanya aktif setelah lepas garis lama & minimal 180ms)
      if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS) {
        if (line.isLurusBelokKiri() || (line.b3 == 1 && line.b4 == 1) || (line.b3 == 1 && line.b2 == 0 && line.b1 == 0 && line.b5 == 0)) {
          // Rem aktif lawan arah ke kanan (PWM 140, 35ms) mematikan inersia putar seketika!
          // Menghentikan rotasi tepat saat menyentuh S3 & S4 sehingga bodi berhenti pas di 90° simetris
          putarPorosKanan(140, 140);
          delay(DURASI_REM_BELOK_MS);
          berhentiTotal();
          delay(40);
          gantiState(STATE_3_LURUSKAN_BELAKANG_KIRI, "Target S3 & S4 Terkunci -> Rem Aktif Kanan Berhasil");
          break;
        }
      }

      // Timeout proteksi jika sensor terlewat
      if (durasiDiState >= TIMEOUT_BELOK_MS) {
        berhentiTotal();
        delay(40);
        gantiState(STATE_3_LURUSKAN_BELAKANG_KIRI, "Timeout Belok Kiri -> Lanjut State 3");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 3: PELURUSAN PASCA BELOK KIRI (STRUKTUR SIMETRIS ANTI OVER-ROTATE)
    // Sinyal 3 & 4 sudah tertangkap dan inersia sudah direm aktif di State 2.
    // Menghilangkan putaran buta open-loop (DURASI_MENGGELIR_KIRI_MS = 0) agar mobil
    // tidak terdorong menjorok ke kiri, identik dengan State 6 Belok Kanan.
    // ==================================================================
    case STATE_3_LURUSKAN_BELAKANG_KIRI: {
      if (DURASI_MENGGELIR_KIRI_MS > 0 && !line.isLurusBelokKiri()) {
        putarPorosKiri(KEC_SESUAI_KIRI_MAJU, KEC_SESUAI_KIRI_MUNDUR);
      }

      if (durasiDiState >= DURASI_MENGGELIR_KIRI_MS || line.isLurusBelokKiri() || (line.b3 == 1 && line.b2 == 0)) {
        if (DURASI_MENGGELIR_KIRI_MS > 0 && !line.isLurusBelokKiri()) {
          // Rem aktif lawan arah mematikan inersia bodi
          putarPorosKanan(140, 140);
          delay(20);
          berhentiTotal();
          delay(40);
        }
        simpangPernahTerdeteksi = false;
        gantiState(STATE_4_MAJU_MENUJU_SIMPANG_KANAN, "Bodi 90° Sempurna Simetris -> Gas Lurus PD Tracking Ruas 2");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 4: MAJU LURUS MENYUSURI RUAS SAMPAI SENSOR KANAN DETEKSI SIMPANG CABANG
    // Invarian Topologis Sejati Simpang 2: S4 & S5 aktif DAN (S3 aktif atau totalAktif >= 3)
    // 2-Sample Persistence Filter: Mencegah trigger akibat ayunan garis tunggal [0 0 0 1 1]
    // ==================================================================
    case STATE_4_MAJU_MENUJU_SIMPANG_KANAN: {
      jalankanTrackingPD(line, KEC_BASE);

      // Invarian Topologis: Simpang sejati wajib memiliki jalur lurus (S3=1 atau totalAktif>=3)
      // Pola oleng garis tunggal [0 0 0 1 1] di mana S3=0 dan totalAktif=2 DITOLAK 100%!
      bool isSimpangKananValid = (line.b5 == 1 && line.b4 == 1) && 
                                 (line.b3 == 1 || line.totalAktif >= 3 || line.isGelapSemua());

      // Filter Persistensi 2-Sampel berturut-turut (~10-12ms) setelah jeda aman
      unsigned long waktuMinimalRuas = modeTesBelokKananSaja ? 100 : WAKTU_MINIMAL_RUAS2_MS;
      if (durasiDiState >= waktuMinimalRuas && isSimpangKananValid) {
        konfirmasiCabangKanan++;
      } else {
        konfirmasiCabangKanan = 0;
      }

      if (konfirmasiCabangKanan >= 2) {
        konfirmasiCabangKanan = 0; // Reset counter konfirmasi

        // Maju presisi 59ms (identik dengan Belok Kiri di Simpang 1)
        if (OFFSET_MAJU_SIMPANG_KANAN_MS > 0) {
          setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
          delay(OFFSET_MAJU_SIMPANG_KANAN_MS);
        }

        // Rem aktif keras mematikan inersia maju seketika
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(JEDA_DIEM_SIMPANG_MS);

        sudahLepasGarisLama = false; // Reset flag deteksi garis lama
        gantiState(STATE_5_BELOK_KANAN_SIMPANG_3, "Simpang 2 Cabang Kanan Sejati Terkonfirmasi -> Mulai Belok Kanan Otomatis");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 5: BELOK KANAN TAHAP 1 - SWEEP OTOMATIS BERBASIS SENSOR (CLOSED-LOOP)
    // 2-Phase FSM: 
    // Fase A: Lepas dari sudut persimpangan (Sensor konfirmasi lantai putih/kardus totalAktif == 0 atau timeout 240ms)
    // Fase B: Radar S1/S2 Melambat + Strict Binary Latch (S3=1, S1=0, S5=0) Kunci Garis Cabang 90 Derajat
    // ==================================================================
    case STATE_5_BELOK_KANAN_SIMPANG_3: {
      // 1. Torsi awal: Kick 180 PWM (30ms) agar 4 roda lepas gesekan statis serempak
      if (durasiDiState < DURASI_RAMP_MS) {
        putarPorosKananHalus(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
      } else {
        // FASE RADAR: Hanya aktif setelah bodi berputar signifikan (>= WAKTU_ABAIKAN_SENSOR_KANAN_MS) & lepas sudut lama:
        // Perlambat putaran ke KEC_BELOK_KANAN_CARI agar pendaratan di target cabang 90° mulus!
        if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS && (line.b1 == 1 || line.b2 == 1)) {
          putarPorosKanan(KEC_BELOK_KANAN_CARI, KEC_BELOK_KANAN_CARI + 6);
        } else {
          putarPorosKanan(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR);
        }
      }

      // 2. FASE A: DETEKSI CLOSED-LOOP LEPAS DARI SUDUT PERSIMPANGAN (Keluar ke Lantai Putih):
      // Garis simpang kanan ada di sudut 0°-25°. Sensor depan baru diizinkan mengonfirmasi lepas garis
      // setelah mobil berputar minimal WAKTU_ABAIKAN_SENSOR_KANAN_MS (200ms ~ 35°) saat menyentuh
      // lantai putih/kardus bersih (totalAktif == 0 atau isGarisHilang), ATAU batas aman WAKTU_LEPAS_SUDUT_MIN_MS (260ms ~ 40°).
      // Ini 100% memblokir trigger palsu akibat celah kardus/flicker sensor di awal putaran!
      if (!sudahLepasGarisLama) {
        if ((durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS && (line.totalAktif == 0 || line.isGarisHilang)) ||
            (durasiDiState >= WAKTU_LEPAS_SUDUT_MIN_MS)) {
          sudahLepasGarisLama = true;
        }
      }

      // 3. FASE B: STRICT BINARY LATCH (HANYA AKTIF SETELAH LEPAS SUDUT & MINIMAL 200ms):
      // Fisik mobil sudah berada di sudut > 35°, sensor mencari garis cabang 90°.
      if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS) {
        // Kunci lurus garis ketat (b1=0, b5=0 dan pola [00110] / [01110] / [00100])
        if (line.isLurusBelokKanan()) {
          // Rem aktif lawan arah ke kiri (PWM 150, 35ms) mematikan inersia putar seketika!
          putarPorosKiri(150, 150);
          delay(DURASI_REM_BELOK_MS);
          berhentiTotal();
          delay(40);
          gantiState(STATE_6_LURUSKAN_BELAKANG_KANAN, "Target 90° Terkunci Presisi (Strict Binary) -> Selesai Belok");
          break;
        }
      }

      // Timeout proteksi jika garis terlewat
      if (durasiDiState >= TIMEOUT_BELOK_MS) {
        berhentiTotal();
        delay(40);
        gantiState(STATE_6_LURUSKAN_BELAKANG_KANAN, "Timeout Belok Kanan -> Lanjut Pelurusan Belakang");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 6: BELOK KANAN TAHAP 2 - PELURUSAN TERKONTROL (TANPA PUTARAN BUTA)
    // Menghilangkan over-rotation dengan meniadakan putaran buta open-loop jika sudah lurus
    // ==================================================================
    case STATE_6_LURUSKAN_BELAKANG_KANAN: {
      if (DURASI_MENGGELIR_KANAN_MS > 0 && !line.isLurusBelokKanan()) {
        putarPorosKanan(KEC_SESUAI_KANAN_MAJU, KEC_SESUAI_KANAN_MUNDUR);
      }

      if (durasiDiState >= DURASI_MENGGELIR_KANAN_MS || line.isLurusBelokKanan()) {
        if (DURASI_MENGGELIR_KANAN_MS > 0 && !line.isLurusBelokKanan()) {
          // Rem aktif lawan arah mematikan inersia bodi
          putarPorosKiri(150, 150);
          delay(20);
          berhentiTotal();
          delay(40);
        }
        gantiState(STATE_7_MAJU_PENETRASI_TITIK_A, "Bodi 90° Sempurna -> Maju Penetrasi PD ke Titik A");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 7: MAJU MENYUSURI RUAS MENUJU TITIK A (TEPAT SEBELUM SIMPANG 4 PERTAMA)
    // Menggunakan PD Tracking aktif hingga sensor mendeteksi garis Simpang 4 pertama [11111]
    // ==================================================================
    case STATE_7_MAJU_PENETRASI_TITIK_A: {
      jalankanTrackingPD(line, KEC_BASE);

      // Jika sedang dalam mode uji khusus TEST_BELOK_KANAN: maju 600ms untuk verifikasi lurus, lalu standby
      if (modeTesBelokKananSaja && durasiDiState >= 600) {
        berhentiTotal();
        modeTesBelokKananSaja = false;
        statusMisiSaatIni = MISI_STANDBY;
        Serial.println("\n>>> [TEST_BELOK_KANAN SELESAI] Robot berhasil belok kanan 90° & melaju lurus -> STANDBY.\n");
        break;
      }

      // Beri jeda 350ms agar aman meninggalkan area Simpang 2
      // Deteksi garis Simpang 4 pertama: Gelap semua [1 1 1 1 1] atau sensor perempatan S1 & S5
      bool tibaDiSimpang4_1 = (durasiDiState > 350) && (line.isGelapSemua() || line.isPerempatan);

      if (tibaDiSimpang4_1) {
        // Rem aktif keras seketika agar bodi dan roda berhenti tepat SEBELUM garis simpang 4
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(50);

        gantiState(STATE_8_STOP_TITIK_A, "Tiba di Titik A (Tepat Sebelum Simpang 4-1) -> Parkir 3 Detik");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 8: BERHENTI DI TITIK A (JEDA 3 DETIK BONGKAR/MUAT ATAU PERINTAH LANJUT)
    // ==================================================================
    case STATE_8_STOP_TITIK_A: {
      berhentiTotal();

      // Jeda 3 detik selesai: otomatis melanjutkan perjalanan melintasi Simpang 4 pertama
      if (durasiDiState >= DURASI_JEDA_TITIK_A_MS) {
        simpangPernahTerdeteksi = false;
        waktuSimpangTerlewati   = 0;
        gantiState(STATE_9_MAJU_LEWATI_SIMPANG4_1, "Jeda 3 Detik di Titik A Selesai -> Lanjut Maju Melintasi Simpang 4-1");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 9: MAJU DARI TITIK A, MELINTASI SIMPANG 4 KE-1 [11111] LURUS TERUS
    // ==================================================================
    case STATE_9_MAJU_LEWATI_SIMPANG4_1: {
      // 450ms pertama: dorong lurus stabil melangkahi garis persimpangan yang lebar
      if (durasiDiState < 450) {
        setKecepatanMotor(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
      } else {
        jalankanTrackingPD(line, KEC_BASE);
      }

      // Robot telah berhasil melintasi garis perempatan pertama (sensor kembali membaca garis normal)
      if (durasiDiState >= 450 && !line.isGelapSemua()) {
        gantiState(STATE_10_MAJU_MENUJU_SIMPANG4_2, "Simpang 4-1 Sukses Dilewati Lurus -> Menuju Simpang 4-2 (Titik B)");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 10: MAJU LURUS MENYUSURI JALUR SAMPAI KETEMU SIMPANG 4 KE-2 [11111]
    // ==================================================================
    case STATE_10_MAJU_MENUJU_SIMPANG4_2: {
      jalankanTrackingPD(line, KEC_BASE);

      // Debounce dari simpang sebelumnya
      bool simpang4KeDuaDitemukan = (durasiDiState > 500) && line.isGelapSemua();

      if (simpang4KeDuaDitemukan) {
        gantiState(STATE_11_MAJU_LURUS_STOP_TITIK_B, "Simpang 4 Ke-2 Terdeteksi [11111] -> Maju Sedikit & Berhenti di B");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 11: MAJU LURUS SEDIKIT LALU BERHENTI SEMPURNA DI TITIK B
    // Penetrasi lebih dalam (350ms) dengan tracking PD agar mobil masuk penuh ke zona Titik B
    // ==================================================================
    case STATE_11_MAJU_LURUS_STOP_TITIK_B: {
      jalankanTrackingPD(line, KEC_BASE);

      if (durasiDiState >= DURASI_PENETRASI_TITIK_B_MS) {
        // Rem aktif keras
        setMotorLangsung(-65, -65);
        delay(30);
        berhentiTotal();
        gantiState(STATE_12_PARKIR_TITIK_B, "Tiba di Titik B -> Parkir Jeda Sebelum Putar Balik Pulang");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 12: PARKIR DI TITIK B (JEDA 3 DETIK BONGKAR/MUAT ATAU PERINTAH PULANG)
    // ==================================================================
    case STATE_12_PARKIR_TITIK_B: {
      berhentiTotal();

      // Jeda di Titik B selesai -> otomatis mulai U-Turn 180° untuk perjalanan pulang
      if (durasiDiState >= DURASI_JEDA_TITIK_B_MS) {
        gantiState(STATE_13_PUTAR_180_TITIK_B, "Jeda di Titik B Selesai -> Mulai U-Turn 180° Pulang");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 13: PUTAR BALIK 180 DERAJAT DI TITIK B (U-TURN CLOSED-LOOP HALUS)
    // Maju tracking PD 765ms (dikurangi 10% agar posisi putar pas), lalu jeda redam
    // inersia 90ms, dilanjutkan U-Turn 180° berkecepatan halus & pendaratan lembut.
    // ==================================================================
    case STATE_13_PUTAR_180_TITIK_B: {
      if (durasiDiState < (unsigned long)OFFSET_MAJU_PUTAR_180_MS) {
        // 1. Maju lurus tracking PD ke depan (jarak pas, mundur 10% dari sebelumnya)
        jalankanTrackingPD(line, KEC_BASE);
      } else if (durasiDiState < (unsigned long)(OFFSET_MAJU_PUTAR_180_MS + JEDA_REDAM_180_MS)) {
        // 2. Transisi Halus: Redam inersia maju sejenak sebelum mulai berputar poros (anti-tersentak)
        berhentiTotal();
      } else {
        unsigned long durasiPutar = durasiDiState - (OFFSET_MAJU_PUTAR_180_MS + JEDA_REDAM_180_MS);

        // 3. Torsi putar 180° terkendali & halus
        if (durasiPutar < (unsigned long)DURASI_RAMP_MS) {
          putarPorosKananHalus(KEC_PUTAR_180_MAJU, KEC_PUTAR_180_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
        } else {
          // Fase Radar saat mendekati 180° (setelah blind-window 350ms): Perlambat halus
          if (durasiPutar >= (unsigned long)WAKTU_ABAIKAN_180_MS && (line.b1 == 1 || line.b2 == 1)) {
            putarPorosKanan(KEC_PUTAR_180_CARI, KEC_PUTAR_180_CARI + 4);
          } else {
            putarPorosKanan(KEC_PUTAR_180_MAJU, KEC_PUTAR_180_MUNDUR);
          }
        }

        // 4. Deteksi Penguncian Garis 180°: Pendaratan Lembut Presisi
        if (durasiPutar >= (unsigned long)WAKTU_ABAIKAN_180_MS) {
          if ((line.b3 == 1 && line.b1 == 0 && line.b5 == 0) || line.isLurus) {
            // Rem lawan arah lembut proporsional agar berhenti sangat mulus tanpa hentakan
            putarPorosKiri(PWM_REM_180, PWM_REM_180);
            delay(25);
            berhentiTotal();
            delay(40);
            gantiState(STATE_14_MAJU_PULANG_KE_S4_1, "U-Turn 180° Halus Berhasil Kunci Garis -> Maju Pulang ke Simpang 4-1");
            break;
          }
        }

        // Timeout proteksi jika garis terlewat
        if (durasiPutar >= TIMEOUT_PUTAR_180_MS) {
          berhentiTotal();
          delay(40);
          gantiState(STATE_14_MAJU_PULANG_KE_S4_1, "Timeout U-Turn 180° -> Lanjut Maju Pulang");
          break;
        }
      }
      break;
    }

    // ==================================================================
    // STATE 14: MAJU MENYUSURI RUAS 4 MENUJU SIMPANG 4 PERTAMA (RUTE PULANG)
    // Sesuai instruksi: "simpang satu simpang 4 dia berhenti kemudian lurus"
    // ==================================================================
    case STATE_14_MAJU_PULANG_KE_S4_1: {
      jalankanTrackingPD(line, KEC_BASE);

      // Debounce dari U-Turn (minimal 350ms)
      bool tibaDiS4_1 = (durasiDiState > 350) && (line.isGelapSemua() || line.isPerempatan);

      if (tibaDiS4_1) {
        // Rem aktif keras seketika sesuai instruksi: berhenti di simpang 4
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(40);
        gantiState(STATE_15_STOP_S4_1_PULANG, "Tiba di Simpang 4-1 Pulang -> Berhenti 1.5 Detik");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 15: BERHENTI DI SIMPANG 4 PERTAMA (JALUR PULANG)
    // ==================================================================
    case STATE_15_STOP_S4_1_PULANG: {
      berhentiTotal();

      if (durasiDiState >= DURASI_STOP_S4_PULANG_MS) {
        gantiState(STATE_16_LEWATI_S4_1_PULANG, "Jeda Berhenti S4-1 Selesai -> Maju Lurus Melintas");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 16: MAJU LURUS MELINTASI SIMPANG 4 PERTAMA (RUTE PULANG)
    // ==================================================================
    case STATE_16_LEWATI_S4_1_PULANG: {
      if (durasiDiState < 450) {
        setKecepatanMotor(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
      } else {
        jalankanTrackingPD(line, KEC_BASE);
      }

      if (durasiDiState >= 450 && !line.isGelapSemua()) {
        gantiState(STATE_17_MAJU_PULANG_KE_S4_2, "Simpang 4-1 Sukses Dilewati Lurus -> Menuju Simpang 4-2 Pulang");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 17: MAJU MENYUSURI RUAS 3 MENUJU SIMPANG 4 KEDUA (RUTE PULANG)
    // Sesuai instruksi: "simpang kedua simpang 4 dia berhenti kemudian lurus"
    // ==================================================================
    case STATE_17_MAJU_PULANG_KE_S4_2: {
      jalankanTrackingPD(line, KEC_BASE);

      // Debounce dari Simpang 4 pertama
      bool tibaDiS4_2 = (durasiDiState > 400) && (line.isGelapSemua() || line.isPerempatan);

      if (tibaDiS4_2) {
        // Rem aktif keras seketika sesuai instruksi: berhenti di simpang 4 kedua
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(40);
        gantiState(STATE_18_STOP_S4_2_PULANG, "Tiba di Simpang 4-2 Pulang -> Berhenti 1.5 Detik");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 18: BERHENTI DI SIMPANG 4 KEDUA (JALUR PULANG)
    // ==================================================================
    case STATE_18_STOP_S4_2_PULANG: {
      berhentiTotal();

      if (durasiDiState >= DURASI_STOP_S4_PULANG_MS) {
        gantiState(STATE_19_LEWATI_S4_2_PULANG, "Jeda Berhenti S4-2 Selesai -> Maju Lurus Melintas");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 19: MAJU LURUS MELINTASI SIMPANG 4 KEDUA (RUTE PULANG)
    // ==================================================================
    case STATE_19_LEWATI_S4_2_PULANG: {
      if (durasiDiState < 450) {
        setKecepatanMotor(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
      } else {
        jalankanTrackingPD(line, KEC_BASE);
      }

      if (durasiDiState >= 450 && !line.isGelapSemua()) {
        gantiState(STATE_20_MAJU_KE_SIMPANG_T_KIRI, "Simpang 4-2 Sukses Dilewati Lurus -> Menuju Simpang T (Full Hitam)");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 20: MAJU MENUJU SIMPANG T KETIGA (DETEKSI FULL HITAM)
    // Sesuai instruksi: "simpang 3 t ketiga, dia bakal deteksi full hitam baru belok kiri"
    // ==================================================================
    case STATE_20_MAJU_KE_SIMPANG_T_KIRI: {
      jalankanTrackingPD(line, KEC_BASE);

      // Deteksi persimpangan T melintang penuh di depan moncong
      bool simpangTDitemukan = (durasiDiState > 350) && 
                               (line.isGelapSemua() || (line.b1 == 1 && line.b2 == 1 && line.b3 == 1));

      if (simpangTDitemukan) {
        // Offset maju sedikit melangkahi garis (59ms) membawa as roda ke pusat simpang
        if (OFFSET_MAJU_BELOK_MS > 0) {
          setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
          delay(OFFSET_MAJU_BELOK_MS);
        }

        // Rem aktif keras seketika mematikan inersia maju
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(JEDA_DIEM_SIMPANG_MS);

        sudahLepasGarisLama = false;
        gantiState(STATE_21_BELOK_KIRI_SIMPANG_T, "Simpang T Full Hitam Terdeteksi -> Mulai Belok Kiri 90°");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 21: BELOK KIRI 90 DERAJAT DI SIMPANG T MELINTANG (RUTE PULANG)
    // Torsi khusus rute pulang ditingkatkan + 2-Fase anti false-lock di garis lama
    // ==================================================================
    case STATE_21_BELOK_KIRI_SIMPANG_T: {
      // 1. Putar kiri bertenaga khusus rute pulang: Kick 195 PWM lalu stabil di 156/158 PWM
      if (durasiDiState < DURASI_RAMP_MS) {
        putarPorosKiriHalus(KEC_BELOK_KIRI_PULANG_MAJU, KEC_BELOK_KIRI_PULANG_MUNDUR, DURASI_BREAKAWAY_KICK_MS, PWM_BREAKAWAY_KIRI_PULANG);
      } else {
        if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS && line.b4 == 1) {
          putarPorosKiri(KEC_SESUAI_KIRI_PULANG_MAJU, KEC_SESUAI_KIRI_PULANG_MUNDUR);
        } else {
          putarPorosKiri(KEC_BELOK_KIRI_PULANG_MAJU, KEC_BELOK_KIRI_PULANG_MUNDUR);
        }
      }

      // 2. FASE A: Lepas garis lama sebelum scan garis baru
      if (!sudahLepasGarisLama) {
        if ((durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS && (line.totalAktif == 0 || line.isGarisHilang)) ||
            (durasiDiState >= WAKTU_LEPAS_SUDUT_KIRI_MS)) {
          sudahLepasGarisLama = true;
        }
      }

      // 3. FASE B: Kunci garis target 90°: sensor tengah (S3=1) dan kanan (S4=1)
      if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KIRI_MS) {
        if (line.isLurusBelokKiri() || (line.b3 == 1 && line.b4 == 1) || (line.b3 == 1 && line.b2 == 0 && line.b1 == 0 && line.b5 == 0)) {
          putarPorosKanan(145, 145);
          delay(DURASI_REM_BELOK_MS);
          berhentiTotal();
          delay(40);
          gantiState(STATE_22_MAJU_KE_CABANG_KANAN, "Target S3 & S4 Terkunci -> Maju Ruas 2 ke Cabang Kanan");
          break;
        }
      }

      // Timeout proteksi jika sensor terlewat
      if (durasiDiState >= TIMEOUT_BELOK_MS) {
        berhentiTotal();
        delay(40);
        gantiState(STATE_22_MAJU_KE_CABANG_KANAN, "Timeout Belok Kiri Pulang -> Lanjut Ruas 2");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 22: MAJU MENYUSURI RUAS 2 SAMPAI DETEKSI SIMPANG CABANG KANAN
    // Sesuai instruksi: "abis tu maju sampai simpang kanan belok kanan"
    // Invarian Topologis: S4 & S5 aktif DAN (S3 aktif atau totalAktif >= 3)
    // ==================================================================
    case STATE_22_MAJU_KE_CABANG_KANAN: {
      jalankanTrackingPD(line, KEC_BASE);

      bool isSimpangKananValid = (line.b5 == 1 && line.b4 == 1) && 
                                 (line.b3 == 1 || line.totalAktif >= 3 || line.isGelapSemua());

      // Filter Persistensi 2-Sampel
      if (durasiDiState >= WAKTU_MINIMAL_RUAS2_MS && isSimpangKananValid) {
        konfirmasiCabangKanan++;
      } else {
        konfirmasiCabangKanan = 0;
      }

      if (konfirmasiCabangKanan >= 2) {
        konfirmasiCabangKanan = 0;

        if (OFFSET_MAJU_SIMPANG_KANAN_MS > 0) {
          setMotorLangsung(KEC_MAJU_OFFSET_KIRI, KEC_MAJU_OFFSET_KANAN);
          delay(OFFSET_MAJU_SIMPANG_KANAN_MS);
        }

        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(JEDA_DIEM_SIMPANG_MS);

        sudahLepasGarisLama = false;
        gantiState(STATE_23_BELOK_KANAN_KE_START, "Simpang Cabang Kanan Terkonfirmasi -> Mulai Belok Kanan ke START");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 23: BELOK KANAN 90 DERAJAT MENUJU JALUR START
    // Menggunakan blind window 200ms dan strict binary latch anti-stall
    // ==================================================================
    case STATE_23_BELOK_KANAN_KE_START: {
      // 1. Torsi awal: Kick 180 PWM (30ms)
      if (durasiDiState < DURASI_RAMP_MS) {
        putarPorosKananHalus(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR, DURASI_BREAKAWAY_KICK_MS);
      } else {
        if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS && (line.b1 == 1 || line.b2 == 1)) {
          putarPorosKanan(KEC_BELOK_KANAN_CARI, KEC_BELOK_KANAN_CARI + 6);
        } else {
          putarPorosKanan(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KANAN_MUNDUR);
        }
      }

      // 2. FASE A: Lepas sudut minimal 200ms
      if (!sudahLepasGarisLama) {
        if ((durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS && (line.totalAktif == 0 || line.isGarisHilang)) ||
            (durasiDiState >= WAKTU_LEPAS_SUDUT_MIN_MS)) {
          sudahLepasGarisLama = true;
        }
      }

      // 3. FASE B: Strict Binary Latch
      if (sudahLepasGarisLama && durasiDiState >= WAKTU_ABAIKAN_SENSOR_KANAN_MS) {
        if (line.isLurusBelokKanan()) {
          putarPorosKiri(150, 150);
          delay(DURASI_REM_BELOK_MS);
          berhentiTotal();
          delay(40);
          gantiState(STATE_24_MAJU_FINISH_START, "Target 90° START Terkunci -> Maju ke Garis START");
          break;
        }
      }

      if (durasiDiState >= TIMEOUT_BELOK_MS) {
        berhentiTotal();
        delay(40);
        gantiState(STATE_24_MAJU_FINISH_START, "Timeout Belok Kanan -> Lanjut Maju START");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE 24: MAJU MENYUSURI RUAS MENUJU GARIS START
    // ==================================================================
    case STATE_24_MAJU_FINISH_START: {
      jalankanTrackingPD(line, KEC_BASE);

      // Garis START adalah simpang melintang [1 1 1 1 1] atau perempatan
      bool tibaDiStart = (durasiDiState > 350) && (line.isGelapSemua() || line.isPerempatan);

      if (tibaDiStart) {
        setMotorLangsung(-85, -85);
        delay(35);
        berhentiTotal();
        delay(50);

        gantiState(STATE_SELESAI_MISI, "Tiba di Garis START -> Misi Pulang-Pergi Sukses Penuh!");
        break;
      }
      break;
    }

    // ==================================================================
    // STATE SELESAI: PARKIR DAN LAPORKAN STATUS SUKSES LENGKAP
    // ==================================================================
    case STATE_SELESAI_MISI: {
      berhentiTotal();
      statusMisiSaatIni = MISI_STANDBY;
      Serial.println("\n*******************************************************");
      Serial.println("  [SUKSES] AGV TELAH KEMBALI DI TITIK START SEMPURNA!  ");
      Serial.println("  Rute Pulang: B -> U-Turn 180° -> S4-1 (Stop) ->     ");
      Serial.println("               S4-2 (Stop) -> Simpang T (Belok Kiri)-> ");
      Serial.println("               Simpang Cabang (Belok Kanan) -> START   ");
      Serial.println("  Status: STANDBY (Misi 2 Arah Selesai Penuh)          ");
      Serial.println("*******************************************************\n");
      break;
    }
  }
}
