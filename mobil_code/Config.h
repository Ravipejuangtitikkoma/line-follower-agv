#pragma once
#include <Arduino.h>

// ====================================================================
// KONFIGURASI PIN & PARAMETER ROBOT AGV (MAPPI32 ESP32)
// File: Config.h
// ====================================================================

// --- 1. PIN PENGGERAK MOTOR (Driver BTS7960 / L9110 / TB6612) ---
const int pinMajuKiri    = 25; // IN1 Driver Kiri
const int pinMundurKiri  = 26; // IN2 Driver Kiri
const int pinMajuKanan   = 16; // IN3 Driver Kanan (Solder IO16)
const int pinMundurKanan = 17; // IN4 Driver Kanan (Solder IO17)

// --- 2. PIN ENCODER (Roda Kiri) ---
const int pinEncoderA    = 32; // Fase A
const int pinEncoderB    = 33; // Fase B

// --- 3. PIN LINE FOLLOWER 5-CHANNEL (TCRT5000) ---
// Logika Sensor (Active HIGH): 1 = Garis Hitam (Aktif), 0 = Lantai/Spanduk Putih
const int pinLine1 = 34; // S1
const int pinLine2 = 35; // S2
const int pinLine3 = 36; // S3 (Tengah)
const int pinLine4 = 39; // S4
const int pinLine5 = 23; // S5

// AKTIFKAN TRUE: Sesuai susunan fisik PCB (OUT5 kiri, OUT1 kanan)
const bool REVERSE_SENSOR_ORDER = true; 

// --- 4. PIN SENSOR JARAK ULTRASONIK (HC-SR04) ---
// DINONAKTIFKAN: Mobil tidak akan berhenti karena halangan & pin HC-SR04 dimatikan
const bool GUNAKAN_SENSOR_JARAK = false; 
const int pinTrig  = -1; // Nonaktif (sebelumnya IO19)
const int pinEcho  = -1; // Nonaktif (sebelumnya IO18)

// --- 5. PIN MODUL LORA INTERNAL MAPPI32 (RFM95 / SX1276) ---
const int pinLoRa_SCK  = 14;
const int pinLoRa_MISO = 12;
const int pinLoRa_MOSI = 13;
const int pinLoRa_SS   = 15;
const int pinLoRa_RST  = 0;
const int pinLoRa_DIO0 = 27;

// ================= PENGATURAN KECEPATAN & KINEMATIKA =================
// Kecepatan dasar mobil saat lurus (Bertenaga & Responsif di Atas Kardus)
const int KEC_BASE = 58; 

// Parameter PD Controller untuk Lurus
const float Kp = 30.0; // Ketajaman respon kurva
const float Kd = 20.0; // Peredam osilasi getaran

// Faktor Kehalusan Transisi (Exponential Moving Average Filter)
const float ALPHA_SMOOTH = 0.25; 

// Penyeimbang Kelurusan Roda (Trim untuk KEC_BASE = 58)
// Tuning lurus asli dari user yang sudah pas di lapangan (TIDAK DIUBAH):
const int TRIM_KIRI  = 22; // Tambah tenaga roda kiri (+22 PWM) mengatasi gesekan internal
const int TRIM_KANAN = -6; // Penyeimbang tenaga roda kanan (-6 PWM)

// ================= PARAMETER HENTAKAN RECOVERY KELUAR GARIS (ANTI-BABLAS) =================
// Hentakan instan saat ada faktor X yang membuat mobil melenceng keluar garis pada jalan lurus:
const int PWM_RECOVERY_MAJU          = 175; // Hentakan maju maksimal (175 PWM) saat lepas garis total
const int PWM_RECOVERY_MUNDUR        = 85;  // Torsi mundur aktif (-85 PWM) pembalik bodi
const int PWM_TEPI_MAJU              = 165; // Hentakan kuat (165 PWM) saat garis di sensor tepi (S1/S5)
const int PWM_TEPI_MUNDUR            = 60;  // Torsi penahan (-60 PWM) saat garis di sensor tepi (S1/S5)

// ================= PARAMETER PUTARAN 90 DERAJAT SEMPURNA =================
// 1. Offset Maju Saat Deteksi Simpang (Membawa As Roda ke Pusat Simpang)
// Simpang 1 (Kiri): Garis melintang penuh di depan moncong
const int OFFSET_MAJU_BELOK_MS         = 59;  // 59ms: Maju sedikit melewati garis sebelum belok kiri

// Simpang 2 (Kanan): Garis lurus terus ke depan, cabang 90° di samping kanan
// Maju singkat 59ms (identik dengan belok kiri) agar sensor melangkahi garis sebelum mulai putar poros
const int OFFSET_MAJU_SIMPANG_KANAN_MS = 59;  // 59ms: Maju sedikit identik Simpang 1
const int DURASI_MAJU_POROS_CABANG_MS  = OFFSET_MAJU_SIMPANG_KANAN_MS;
const int KEC_MAJU_OFFSET              = 58;  // Kecepatan maju presisi dasar
const int KEC_MAJU_OFFSET_KIRI         = 78;  // Kecepatan maju offset roda kiri (dengan trim kiri)
const int KEC_MAJU_OFFSET_KANAN        = 50;  // Kecepatan maju offset roda kanan (dengan trim kanan)

// 2. Jeda diam saat tiba di titik poros simpang (Mematikan inersia maju)
const int JEDA_DIEM_SIMPANG_MS = 80; 

// 3. Kecepatan belok terukur bertenaga tinggi (Mengatasi gesekan lateral ban karet high-grip)
// Belok Kanan:
const int KEC_BELOK_KANAN_MAJU          = 166; // Ditingkatkan (dari 148) untuk mengatasi grip tinggi ban karet putih
const int KEC_BELOK_KANAN_MUNDUR        = 166; // Ditingkatkan (dari 153) agar roda kanan aktif memutar penuh
const int KEC_BELOK_KANAN_CARI          = 142; // Ditingkatkan (dari 130) agar tidak stall saat mendekati garis target
const int WAKTU_ABAIKAN_SENSOR_KANAN_MS = 200; // Abaikan sensor 200ms: bodi wajib putar >35° dulu agar tidak mengunci garis lama!
const int WAKTU_LEPAS_SUDUT_MIN_MS      = 260; // Timeout pengaman Fase A: jika kardus tidak sepenuhnya putih, lepas sudut setelah 260ms (~40°) 

// Belok Kiri:
const int KEC_BELOK_KIRI_MAJU    = 158; // Ditingkatkan (dari 134) mengatasi hambatan putar ban karet baru
const int KEC_BELOK_KIRI_MUNDUR  = 168; // +10 ekstra torsi mundur roda kiri (mengatasi gesekan mekanik kiri lebih berat)

// 4. Parameter Breakaway Torque Kick (Mendobrak Gesekan Statis Ke-4 Roda Serentak)
const int PWM_BREAKAWAY_KICK           = 205; // Entakan torsi kuat agar roda kiri & belakang tidak mematung (belok kiri)
const int PWM_BREAKAWAY_KANAN_KICK     = 195; // Entakan awal belok kanan untuk ban karet high-grip
const int DURASI_BREAKAWAY_KICK_MS     = 40;  // Durasi sentakan 40ms mendobrak friksi statis awal ban karet
const int DURASI_RAMP_MS               = 40;  // Durasi transisi awal disamakan dengan breakaway kick
const int WAKTU_SWEEP_AWAL_MS          = 60;  // 60ms: Cepat buang bayangan garis lama agar siap tangkap garis baru (Belok Kiri)

// 5. Parameter Pelurusan Bodi Belakang Panjang 25 cm (Menarik Bodi Belakang 90° Penuh)
const int DURASI_TAMBAHAN_BELAKANG_MS       = 200; 
const unsigned long TIMEOUT_PENYESUAIAN_BELAKANG_MS = 160; 
const int KEC_SESUAI_KANAN_MAJU             = 148; // Ditingkatkan (dari 139) untuk penarikan bodi belakang anti-stall
const int KEC_SESUAI_KANAN_MUNDUR           = 152; // Ditingkatkan (dari 144)
const int KEC_SESUAI_KIRI_MAJU              = 132; // Ditingkatkan (dari 108) agar tidak macet saat S4 mulai kena garis
const int KEC_SESUAI_KIRI_MUNDUR            = 138; // Ditingkatkan (dari 108) dengan kompensasi gesekan kiri
const int DURASI_REM_BELOK_MS               = 35;  // Rem lawan arah 35ms menyerap inersia putar seketika (anti-overshoot) 
const int DURASI_REM_BELOK_KANAN_MS         = 25;  // Rem lawan arah belok kanan lebih singkat (25ms) agar tidak mental balik 3°-4°

// ================= PARAMETER KHUSUS BELOK KANAN KE-2 =================
// Mengatasi masalah bablas & memberikan torsi seimbang pada tikungan siku kanan kedua:
const int OFFSET_MAJU_KANAN_2_MS          = 0;   // Tidak maju saat deteksi (0 ms) agar roda tidak bablas keluar kardus
const int KEC_BELOK_KANAN_2_MAJU          = 144; // Diturunkan 10% (dari 160) agar tenaga putar pas dan tidak berlebihan
const int KEC_BELOK_KANAN_2_MUNDUR        = 147; // Diturunkan 10% (dari 163)
const int WAKTU_SWEEP_KANAN_2_MS          = 260; // Waktu putaran aktif minimal (260ms) agar roda belakang pasti berputar sebelum scan garis
const int DURASI_TAMBAHAN_BELAKANG_2_MS   = 350; // Durasi putaran penarikan bodi belakang
const int DURASI_EKSTRA_LURUSKAN_KANAN_MS = 70;  // Ekstra putar kanan (+10% sudut / ~10 derajat) agar bodi 25cm lurus 90° sempurna
const unsigned long TIMEOUT_BELOK_MS      = 1200;// Timeout proteksi belok diperpanjang ke 1200ms untuk ban karet high-grip

// Jeda cooldown setelah belok agar tidak memicu belokan ganda
const unsigned long COOLDOWN_BELOK_MS = 600; // Dikurangi ke 600ms agar siap deteksi tikungan kedua yang dekat

// ================= PARAMETER SENSOR & KESELAMATAN =================
// Parameter Keselamatan Ultrasonik HC-SR04
const float JARAK_STOP_CM  = 8.0;  // Berhenti jika jarak <= 8 cm
const float JARAK_LEPAS_CM = 35.0; // Jalan lagi jika rintangan sudah > 35 cm

// Parameter Deteksi Simpang
const unsigned long DEBOUNCE_NODE_MS = 500; // Jeda minimum antar deteksi simpang (ms)
const unsigned long EXIT_LATCH_MS    = 150; // Waktu minimum sebelum keluar dari simpang
const int AMBANG_SIMPANG_PENUH       = 4;   // Minimal 4 dari 5 sensor membaca gelap [1 1 1 1 1]

// Toleransi Miring Perempatan (ms): Jika S1 dan S5 aktif dalam selisih <= 35ms,
// tetap dianggap perempatan sah (tahan posisi masuk miring, kebal S-bend >150ms)
const unsigned long TOLERANSI_MIRING_SIMPANG_MS = 35;
const int PERSISTENSI_CABANG_KANAN              = 2;
const int PERSISTENSI_CABANG_KIRI               = 2;

// ================= PARAMETER RUTE PETA RIIL (START -> A -> B) =================
const unsigned long DURASI_MENGGELIR_KIRI_MS   = 0;    // 0ms: Putaran buta dihilangkan total sama seperti belok kanan agar tidak overshoot >90° ke kiri
const unsigned long DURASI_MENGGELIR_KANAN_MS  = 0;    // 0ms: Putaran buta dihilangkan total agar tidak overshoot >90°
const unsigned long DURASI_PENETRASI_TITIK_A_MS= 300;  // Maju sedikit memasuki zona parkir Titik A
const unsigned long DURASI_JEDA_TITIK_A_MS     = 3000; // Jeda bongkar muat 3 detik di Titik A sebelum ke Titik B
const unsigned long DURASI_PENETRASI_TITIK_B_MS= 495;  // Dikurangi 10% (dari 550ms ke 495ms) sesuai permintaan user agar tidak kejauhan

// ================= PARAMETER ENCODER RODA KIRI & JARAK CHECKPOINT =================
// Fisik robot (PANDUAN_LENGKAP_AGV.md & rumus.md): Diameter roda = 6.5 cm (Keliling ~20.42 cm)
// Estimasi encoder JGA25-370 / JGB37-520 (test_sensor_motor.ino): 330 ticks/putaran roda (11 PPR x 1:30)
// Peta arena 3m x 3m (3 ruas horizontal @ 100 cm):
//   N2 --(50cm)--> [A] --(50cm)--> N6 --(100cm)--> N10 --(50cm)--> [C] --(50cm)--> N14
const float DIAMETER_RODA_CM              = 6.5f;
const float KELILING_RODA_CM              = 3.1415926f * DIAMETER_RODA_CM; // ~20.42 cm per putaran roda
const float ENCODER_TICKS_PER_PUTARAN     = 330.0f; // Ubah sesuai hasil kalibrasi TEST_ENCODER (1 putaran penuh roda)

// Perkiraan jarak tempuh antar titik (dalam cm) — mudah diubah untuk kalibrasi lapangan:
const float JARAK_N2_KE_A_CM              = 50.0f;  // Dari selesai belok kanan N2 ke Checkpoint A (~0.5 m / ~808 ticks)
const float JARAK_N10_KE_C_CM             = 40.0f;  // Dari setelah melintasi N10 (~10 cm) menuju Checkpoint C (~40 cm / ~646 ticks)

// Jeda berhenti 3 detik di Checkpoint A dan Checkpoint C
const unsigned long DURASI_JEDA_TITIK_C_MS = 3000;  // Jeda bongkar muat 3 detik di Checkpoint C

// true  = Setelah jeda 3 detik di Checkpoint C, robot otomatis U-Turn 180° & pulang ke START
// false = Robot berhenti (STANDBY) di Checkpoint C menunggu perintah 'PULANG'
const bool OTOMATIS_PULANG_SETELAH_C      = true;

// Fallback & proteksi pengaman bila encoder tidak mengirim pulsa (0 ticks) atau PPR motor lebih kecil dari 330:
// Kecepatan linear robot pada KEC_BASE = 58 dari rumus.md adalah ~20.8 cm/s
const bool          GUNAKAN_FALLBACK_WAKTU_ENCODER = true;  // Otomatis pakai estimasi waktu jika pulsa encoder == 0
const float         KECEPATAN_ESTIMASI_CM_PER_S    = 20.8f; // Estimasi kecepatan maju (cm/detik) pada KEC_BASE = 58
const unsigned long WAKTU_DETEKSI_ENCODER_MATI_MS  = 600;   // Jika 600ms berjalan pulsa masih 0, aktifkan fallback waktu
const unsigned long TIMEOUT_ENCODER_A_MS           = 3500;  // Batas waktu maksimal ruas N2 -> A (~73 cm) agar tidak bablas
const unsigned long TIMEOUT_ENCODER_C_MS           = 3500;  // Batas waktu maksimal ruas N10 -> C (~40 cm) agar aman sebelum N14
const unsigned long LOCKOUT_PENGAMAN_N6_MS         = 600;   // Abaikan garis awal selama 600ms sebelum pengaman simpang aktif
const unsigned long LOCKOUT_PENGAMAN_SIMPANG_MS    = LOCKOUT_PENGAMAN_N6_MS; // Alias pengaman simpang N6 (Step 2) & N14 (Step 5)

// ================= PARAMETER MAPPING DINAMIS & FLASH ESP32 (NVS) =================
const int   MAX_PLAN_STEPS          = 28;    // Kapasitas maksimal langkah rute aktif (aman 1 halaman NVS ESP32)
const float OFFSET_CROSS_SIMPANG_CM = 9.0f;  // Kompensasi jarak maju ~450ms saat melintasi simpang lurus sebelum ukur 50cm ke stasiun
const bool  AUTO_START_SAAT_NYALA   = true;  // true = otomatis jalan 2s setelah boot (bisa dibatalkan via Serial)
 
// ================= PENGAMAN KALIBRASI AWAL & SIMPANG KANAN =================
const int WAKTU_AUTO_ALIGN_MAX_MS          = 250; // Waktu maksimal kalibrasi mikro posisi awal di tempat (ms)
const int DURASI_AUTO_ALIGN_KICK_MS        = 25;  // Sentakan mikro 25ms mendobrak gesekan statis 4 ban
const int PWM_AUTO_ALIGN_KICK              = 160; // Torsi sentakan awal (proporsional, tidak liar)
const int PWM_AUTO_ALIGN                   = 138; // Torsi pelurusan stabil (bertenaga, tidak stall di kardus)
const unsigned long WAKTU_MINIMAL_RUAS2_MS = 350; // 350ms: Aman dari osilasi pasca simpang 1 & sigap deteksi cabang kanan

// ================= PARAMETER PELURUSAN PASCA BELOK KIRI (SINYAL 3 TUNING) =================
// Roda kanan diam sebagai tumpuan di garis, roda kiri ditarik mundur halus terukur agar bodi pas 90° tanpa over-rotate ke kiri
const int PWM_ALIGN_KIRI_KICK                  = 125; // Diturunkan dari 160 ke 125 agar tidak menyentak terlalu ke kiri
const int PWM_ALIGN_KIRI_STABIL                = 110; // Diturunkan dari 138 ke 110 untuk pelurusan lembut presisi
const unsigned long DURASI_ALIGN_KIRI_KICK_MS  = 10;  // Durasi sentakan mikro dipersingkat ke 10ms (sebelumnya 20ms)
const unsigned long TIMEOUT_ALIGN_KIRI_MS      = 35;  // Timeout pengaman dipersingkat ke 35ms (sebelumnya 65ms)

// ================= PARAMETER RUTE PULANG (B -> START) =================
const unsigned long DURASI_JEDA_TITIK_B_MS      = 3000; // Jeda 3 detik di Titik B sebelum otomatis putar balik pulang
const int OFFSET_MAJU_PUTAR_180_MS             = 250;  // Diperkecil ke 250ms (~5 cm) agar aman dari ujung N14 saat U-Turn di C
const int WAKTU_ABAIKAN_180_MS                 = 350;  // Blind window 350ms: abaikan garis lama (0°-120°) saat putar 180°
const unsigned long DURASI_STOP_S4_PULANG_MS   = 1500; // Berhenti 1.5 detik di Simpang 4 sebelum melaju lurus
const unsigned long TIMEOUT_PUTAR_180_MS       = 1400; // Batas aman timeout putar balik 180°

// --- Parameter Khusus Putar Balik 180° Halus (State 13 Titik B) ---
const int JEDA_REDAM_180_MS                    = 90;   // Jeda redam 90ms saat selesai maju sebelum memutar (anti-tersentak)
const int KEC_PUTAR_180_MAJU                   = 142;  // Torsi putar 180° halus dan terkendali
const int KEC_PUTAR_180_MUNDUR                 = 146;  // Torsi putar mundur 180°
const int KEC_PUTAR_180_CARI                   = 118;  // Perlambatan halus saat mendekati garis (Fase Radar) agar pendaratan mulus
const int PWM_REM_180                          = 125;  // Rem lawan arah lembut (125 PWM, 25ms) anti-hentakan kasar

// ================= PARAMETER KHUSUS BELOK KIRI RUTE PULANG (STATE 21 SIMPANG T) =================
// Torsi khusus belok kiri saat rute pulang ditingkatkan agar kuat untuk ban karet high-grip
const int KEC_BELOK_KIRI_PULANG_MAJU           = 164;  // Torsi maju belok kiri pulang ditingkatkan (dari 156)
const int KEC_BELOK_KIRI_PULANG_MUNDUR         = 172;  // Torsi mundur belok kiri pulang ditingkatkan (dari 158)
const int PWM_BREAKAWAY_KIRI_PULANG            = 210;  // Sentakan awal bertenaga mendobrak gesekan statis ban karet
const int KEC_SESUAI_KIRI_PULANG_MAJU          = 135;  // Torsi pendekatan garis ditingkatkan (dari 125)
const int KEC_SESUAI_KIRI_PULANG_MUNDUR        = 140;  // Torsi mundur kiri pendekatan garis (dari 125)



