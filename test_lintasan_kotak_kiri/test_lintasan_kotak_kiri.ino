// ====================================================================
// PROGRAM UJI LINTASAN KOTAK BELOK KIRI (COUNTER-CLOCKWISE TEST FOR AGV)
// File  : test_lintasan_kotak_kiri.ino
// Board : Mappi32 (ESP32) by KMtek
// Driver: Dual BTS7960 Motor Driver
// Sensor: 5-Channel Line Follower TCRT5000
// 
// DESKRIPSI:
//   Program khusus untuk menguji mobil AGV pada lintasan kotak sempurna (loop).
//   Mobil melaju lurus menyusuri garis lakban, dan setiap kali menemui sudut kotak
//   (sensor kanan aktif ATAU semua sensor aktif), mobil berbelok 90° secara kontinu.
//   Arah belok (KANAN terus atau KIRI terus) dapat diubah HANYA DENGAN MENGGANTI
//   SATU VARIABEL: `ARAH_BELOK`.
//
// FITUR KHUSUS LINTASAN SPANDUK:
//   1. Torsi Maju & Belok Disesuaikan: Mengatasi masalah mobil loyo / kurang torsi di spanduk.
//   2. Anti-Lolos Garis (Enhanced Corner Detection): Deteksi sudut multi-kondisi agar tidak lolos.
//   3. Breakaway Torque Kick: Mendobrak gesekan statis ke-4 roda saat mulai memutar.
//   4. Mode Kalibrasi Sensor Interaktif: Menampilkan visualisasi pembacaan 5 sensor
//      di Serial Monitor secara real-time untuk memudahkan penyetelan trimpot di spanduk.
//   5. Kontrol On-The-Fly via Serial Monitor: Atur kecepatan (+ / -), arah (K / L), dan Stop (SPASI).
// ====================================================================

#include <Arduino.h>

// ====================================================================
// [1] PENGATURAN ARAH BELOK KOTAK (CUKUP GANTI SATU VARIABEL INI)
// ====================================================================
// Pilihan:
//   ARAH_BELOK_KANAN : Mobil berbelok KANAN terus di setiap sudut kotak
//   ARAH_BELOK_KIRI  : Mobil berbelok KIRI terus di setiap sudut kotak
enum ArahBelokKotak { 
  ARAH_BELOK_KANAN, 
  ARAH_BELOK_KIRI 
};

// >>>>>>>>>> GANTI ARAH BELOK DI VARIABEL INI <<<<<<<<<<
ArahBelokKotak ARAH_BELOK = ARAH_BELOK_KIRI; 
// ====================================================================


// ====================================================================
// [2] KONFIGURASI PIN HARDWARE (PERSIS SAMA DENGAN CONFIG.H PROYEK)
// ====================================================================
// --- PIN PENGGERAK MOTOR (Driver BTS7960 pada MAPPI32 ESP32) ---
const int pinMajuKiri    = 25; // IN1 Driver Kiri (RPWM)
const int pinMundurKiri  = 26; // IN2 Driver Kiri (LPWM)
const int pinMajuKanan   = 16; // IN3 Driver Kanan (RPWM - Solder IO16)
const int pinMundurKanan = 17; // IN4 Driver Kanan (LPWM - Solder IO17)

// --- PIN LINE FOLLOWER 5-CHANNEL (TCRT5000) ---
// Logika Sensor TCRT5000: 0 = Garis Hitam (Aktif), 1 = Lantai/Spanduk Terang
const int pinLine1 = 34; // S1 (OUT1 PCB)
const int pinLine2 = 35; // S2 (OUT2 PCB)
const int pinLine3 = 36; // S3 (OUT3 PCB - Tengah)
const int pinLine4 = 39; // S4 (OUT4 PCB)
const int pinLine5 = 23; // S5 (OUT5 PCB)

// Sesuai susunan fisik PCB mobil kita: OUT5 di kiri fisik, OUT1 di kanan fisik
const bool REVERSE_SENSOR_ORDER = true; 

// --- PIN ENCODER (Opsional / Proteksi Floating) ---
const int pinEncoderA    = 32; 
const int pinEncoderB    = 33; 


// ====================================================================
// [3] PARAMETER KECEPATAN & TORSI (OPTIMALISASI KHUSUS LINTASAN SPANDUK)
// ====================================================================
// Di kardus KEC_BASE = 58. Di atas spanduk, gesekan dan beban ban lebar butuh
// torsi lebih besar agar motor tidak stall / loyo.
int KEC_BASE              = 68;  // Kecepatan dasar melaju lurus di spanduk (Diturunkan agar stabil saat melaju lurus)

// Tuning Penyeimbang Roda Asli (dari Config.h) mengatasi gesekan mekanik roda kiri
const int TRIM_KIRI       = 22;  // Tambah tenaga roda kiri (+22 PWM)
const int TRIM_KANAN      = -6;  // Penyeimbang roda kanan (-6 PWM)

// Parameter PD Controller untuk Melacak Garis Lurus
const float Kp            = 28.0; // Respon koreksi tikungan lembut
const float Kd            = 18.0; // Peredam osilasi getaran zigzag
const float ALPHA_SMOOTH  = 0.30; // Filter kehalusan PWM

// Torsi Belok Poros 90° di Spanduk (Pivot Turn)
// --- Roda Sisi Kiri (Maju saat Belok Kanan) ---
const int KEC_BELOK_KIRI_MAJU          = 125; // Tenaga maju roda kiri disetel pas (tidak terlalu kencang membanting)
const int PWM_BREAKAWAY_KIRI_MAJU      = 155; // Sentakan awal maju roda kiri (30ms)

// --- Roda Sisi Kanan (Mundur saat Belok Kanan - Bertenaga Pas & Halus) ---
// Torsi disesuaikan agar roda kanan depan aktif berputar mundur tanpa membuat belokan terlalu liar/tajam
const int KEC_BELOK_KANAN_MUNDUR       = 158; // Torsi mundur roda kanan pas (158 PWM)
const int PWM_BREAKAWAY_KANAN_MUNDUR   = 185; // Sentakan instan 30ms (185 PWM) mendobrak friksi statis roda depan

// --- Parameter Simetris saat Belok Kiri (Cadangan jika ARAH_BELOK = ARAH_BELOK_KIRI) ---
const int KEC_BELOK_KANAN_MAJU         = 125; 
const int PWM_BREAKAWAY_KANAN_MAJU     = 155; 
const int KEC_BELOK_KIRI_MUNDUR        = 158; 
const int PWM_BREAKAWAY_KIRI_MUNDUR    = 185; 

const int DURASI_BREAKAWAY_MS          = 30;  // Durasi sentakan torsi awal (ms)

// Parameter Geometri & Sekuens Belok Kotak:
// 1. BERHENTI DI SIMPANG -> 2. MUNDUR DIKIT -> 3. BELOK NYARI -> 4. SAMPE LURUS -> 5. BARU MAJU
const int JEDA_BERHENTI_DI_SIMPANG_MS    = 150; // Langsung berhenti diam begitu deteksi simpang (ms)
const int DURASI_MUNDUR_SEBELUM_BELOK_MS = 70;  // MUNDUR DIKIT sebelum mulai putar belok (ms)
const int KEC_MUNDUR_SEBELUM_BELOK       = 85;  // Kecepatan mundur sebelum belok (PWM)
const int JEDA_DIAM_SEBELUM_BELOK_MS     = 120; // Jeda diam tenang setelah mundur sebelum memutar roda (ms)
const int WAKTU_BLIND_SWEEP_MS           = 80;  // 80ms: cukup lepas dari garis lama tanpa telat memindai garis baru // Wajib putar min 160ms agar sensor benar-benar lepas dari garis lama
const int PWM_REM_BELOK                  = 90;  // Torsi rem lawan arah penahan inersia putar (anti-overshoot)
const int DURASI_REM_BELOK_MS            = 30;  // Durasi rem lawan arah (ms)
const int JEDA_SETELAH_LURUS_MS          = 150; // Jeda diam stabil setelah bodi lurus sebelum melaju jalan (ms)
const unsigned long TIMEOUT_BELOK_MS     = 850; // Batas waktu maksimal putaran belok 90° (850ms: Tuntas & Anti Gagal Putar)

// Cooldown Pasca Belok (Mencegah Deteksi Sudut Ganda di Tikungan yang Sama)
const unsigned long COOLDOWN_TIKUNGAN_MS = 600;

// Validasi Deteksi Tikungan (Debounce Filter)
// Wajib membaca kondisi tikungan yang sama 2 KALI berturut-turut (~5ms) respon cepat anti-bablas
const int SYARAT_VALIDASI_TIKUNGAN = 2;


// ====================================================================
// [4] STRUKTUR DATA SENSOR & VARIABEL GLOBAL
// ====================================================================
struct LineData {
  int b1; // Kiri Luar   (S1 fisik)
  int b2; // Kiri Tengah (S2 fisik)
  int b3; // Tengah      (S3 fisik)
  int b4; // Kanan Tengah(S4 fisik)
  int b5; // Kanan Luar  (S5 fisik)
  int totalAktif;
  float error;
  bool isGarisHilang;
};

// State Kendali Mobil
static bool mobilAktif           = true;  // Status motor jalan / stop
static bool modeKalibrasi        = false; // Mode visualizer live sensor di serial
static float smoothSpeedKiri     = 0.0;
static float smoothSpeedKanan    = 0.0;
static float errSebelumnyaPD     = 0.0;
static int arahGarisTerakhir     = 0;     // -1 = Kiri, 0 = Tengah, +1 = Kanan
static unsigned long waktuTerakhirBelok = 0;
static int hitunganSudutKotak    = 0;
static int hitunganValidasiSudut = 0; // Counter konfirmasi tikungan (harus 2x berturut-turut)
static unsigned long lastTelemetriTime  = 0;


// ====================================================================
// [5] FUNGSI PENGGERAK MOTOR (LOW LEVEL DRIVER)
// ====================================================================
inline void initMotor() {
  pinMode(pinMajuKiri, OUTPUT);
  pinMode(pinMundurKiri, OUTPUT);
  pinMode(pinMajuKanan, OUTPUT);
  pinMode(pinMundurKanan, OUTPUT);

  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, 0);
}

inline void berhentiTotal() {
  smoothSpeedKiri  = 0;
  smoothSpeedKanan = 0;
  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, 0);
}

inline void setMotorLangsung(int vKiri, int vKanan) {
  smoothSpeedKiri  = vKiri;
  smoothSpeedKanan = vKanan;

  // Kontrol Roda Kiri
  if (vKiri >= 0) {
    analogWrite(pinMajuKiri, constrain(vKiri, 0, 255));
    analogWrite(pinMundurKiri, 0);
  } else {
    analogWrite(pinMajuKiri, 0);
    analogWrite(pinMundurKiri, constrain(-vKiri, 0, 255));
  }

  // Kontrol Roda Kanan
  if (vKanan >= 0) {
    analogWrite(pinMajuKanan, constrain(vKanan, 0, 255));
    analogWrite(pinMundurKanan, 0);
  } else {
    analogWrite(pinMajuKanan, 0);
    analogWrite(pinMundurKanan, constrain(-vKanan, 0, 255));
  }
}

inline void setMotorHalus(int targetKiri, int targetKanan) {
  smoothSpeedKiri  = (smoothSpeedKiri  * (1.0 - ALPHA_SMOOTH)) + (targetKiri  * ALPHA_SMOOTH);
  smoothSpeedKanan = (smoothSpeedKanan * (1.0 - ALPHA_SMOOTH)) + (targetKanan * ALPHA_SMOOTH);
  setMotorLangsung((int)smoothSpeedKiri, (int)smoothSpeedKanan);
}

inline void putarPorosKanan(int pwmMaju, int pwmMundur) {
  setMotorLangsung(pwmMaju, -pwmMundur);
}

inline void putarPorosKiri(int pwmMaju, int pwmMundur) {
  setMotorLangsung(-pwmMundur, pwmMaju);
}


// ====================================================================
// [6] PEMBACAAN & PENGOLAHAN SENSOR GARIS (TCRT5000)
// ====================================================================
inline void initSensors() {
  pinMode(pinLine1, INPUT);
  pinMode(pinLine2, INPUT);
  pinMode(pinLine3, INPUT);
  pinMode(pinLine4, INPUT);
  pinMode(pinLine5, INPUT);

  pinMode(pinEncoderA, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
}

inline LineData bacaSensorGaris() {
  LineData data;

  // Logika Sensor (Active HIGH): HIGH (1) saat di atas garis hitam, LOW (0) di spanduk putih
  int raw1 = (digitalRead(pinLine1) == 1) ? 1 : 0;
  int raw2 = (digitalRead(pinLine2) == 1) ? 1 : 0;
  int raw3 = (digitalRead(pinLine3) == 1) ? 1 : 0;
  int raw4 = (digitalRead(pinLine4) == 1) ? 1 : 0;
  int raw5 = (digitalRead(pinLine5) == 1) ? 1 : 0;

  // Pemetaan fisik (REVERSE_SENSOR_ORDER: OUT5 kiri, OUT1 kanan)
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
  data.isGarisHilang = (data.totalAktif == 0);

  // Perhitungan Bobot Posisi Garis (-2.0 s/d +2.0)
  if (data.totalAktif > 0) {
    float bobot = (data.b1 * -2.0) + (data.b2 * -1.0) + (data.b3 * 0.0) + (data.b4 * 1.0) + (data.b5 * 2.0);
    data.error = bobot / (float)data.totalAktif;

    if (data.error > 0.3) {
      arahGarisTerakhir = +1; // Cenderung di kanan
    } else if (data.error < -0.3) {
      arahGarisTerakhir = -1; // Cenderung di kiri
    } else {
      arahGarisTerakhir = 0;  // Pas di tengah
    }
  } else {
    data.error = 0.0;
  }

  return data;
}


// ====================================================================
// [7] DETEKSI SUDUT KOTAK & SIMPANG TIGA (CONTEK DARI PROGRAM UTAMA)
// ====================================================================
enum TipeTikungan {
  TIKUNGAN_NONE,
  TIKUNGAN_SIMPANG_3_KANAN, // Kegelapan HANYA di sebelah kanan [0 0 1 1 1] atau [0 1 1 1 1]
  TIKUNGAN_SIMPANG_3_KIRI,  // Kegelapan HANYA di sebelah kiri  [1 1 1 0 0] atau [1 1 1 1 0]
  TIKUNGAN_GELAP_SEMUA,     // Garis melintang penuh / simpang 4 [1 1 1 1 1] atau [1 ... 1]
  TIKUNGAN_SIKU_SUDUT       // Sudut siku ujung
};
static TipeTikungan tipeSudutTerdeteksi = TIKUNGAN_NONE;

inline bool cekSudutKotak(const LineData& line) {
  // Masih dalam masa cooldown pasca belok sebelumnya?
  if (millis() - waktuTerakhirBelok < COOLDOWN_TIKUNGAN_MS) {
    hitunganValidasiSudut = 0;
    tipeSudutTerdeteksi = TIKUNGAN_NONE;
    return false;
  }

  // 1. Simpang 3 Cabang Kanan (Contek dari Sensor.h & NavigationFSM.h):
  //    Karakteristik:
  //    - Sisi kanan membaca hitam tebal: S5=1 DAN S4=1 (WAJIB KEDUANYA AKTIF!)
  //    - Jalur lurus tetap ada di tengah: S3=1 atau totalAktif >= 3
  //    - Sisi kiri tetap spanduk putih: S1=0
  //    - Catatan: pola oleng garis tunggal [0 0 0 1 1] di mana S3=0 ditolak 100%!
  bool isSimpang3Kanan = (line.b5 == 1 && line.b4 == 1) && 
                         (line.b3 == 1 || line.totalAktif >= 3) && 
                         (line.b1 == 0);

  // 2. Simpang 3 Cabang Kiri (Simetris jika ARAH_BELOK == ARAH_BELOK_KIRI):
  bool isSimpang3Kiri  = (line.b1 == 1 && line.b2 == 1) && 
                         (line.b3 == 1 || line.totalAktif >= 3) && 
                         (line.b5 == 0);

  // 3. Garis Melintang Penuh / Sudut Gelap Semua / Simpang 4:
  //    Total sensor aktif minimal 4, ATAU kedua ujung luar kanan & kiri aktif bersamaan
  bool isGelapSemua    = (line.totalAktif >= 4 && line.b3 == 1);

  // 4. Sudut Siku Kanan Murni (Tikungan 90° di mana garis berbelok ke kanan):
  //    WAJIB kedua sensor kanan aktif: S5=1 DAN S4=1!
  //    Single sensor b5=1 saja TANPA b4 DILARANG KERAS memicu belok (hanya koreksi tepi PD biasa)!
  bool isSikuKanan     = false; // Digantikan oleh isSimpang3Kanan (Wajib S3=1 untuk mencegah false trigger di jalur lurus)

  // 5. Sudut Siku Kiri Murni:
  bool isSikuKiri      = false; // Digantikan oleh isSimpang3Kiri (Wajib S3=1 untuk mencegah false trigger di jalur lurus)

  bool terdeteksi = false;
  TipeTikungan tipeKandidat = TIKUNGAN_NONE;

  if (ARAH_BELOK == ARAH_BELOK_KANAN) {
    if (isSimpang3Kanan) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_SIMPANG_3_KANAN;
    } else if (isGelapSemua) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_GELAP_SEMUA;
    } else if (isSikuKanan) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_SIKU_SUDUT;
    }
  } else {
    if (isSimpang3Kiri) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_SIMPANG_3_KIRI;
    } else if (isGelapSemua) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_GELAP_SEMUA;
    } else if (isSikuKiri) {
      terdeteksi = true;
      tipeKandidat = TIKUNGAN_SIKU_SUDUT;
    }
  }

  // VALIDASI KETAT 3 KALI PEMBACAAN BERTURUT-TURUT (Triple-Confirmation Filter):
  // Mencegah trigger palsu akibat getaran atau bodi sedikit miring di garis lurus
  if (terdeteksi) {
    hitunganValidasiSudut++;
    tipeSudutTerdeteksi = tipeKandidat;
    if (hitunganValidasiSudut >= SYARAT_VALIDASI_TIKUNGAN) {
      hitunganValidasiSudut = 0; // Reset setelah valid
      return true;               // VALID TIKUNGAN KONSISTEN!
    }
  } else {
    hitunganValidasiSudut = 0;   // Reset seketika jika kondisi hilang
    tipeSudutTerdeteksi = TIKUNGAN_NONE;
  }

  return false;
}


// ====================================================================
// [8] EKSEKUSI BELOK SUDUT KOTAK:
//     1. KETEMU SIMPANG -> LANGSUNG BERHENTI
//     2. MUNDUR DIKIT SEBELUM BELOK
//     3. BELOK NYARI GARIS BARU 90° (Kiri Maju, Kanan Mundur)
//     4. SAMPE LURUS (Koreksi mikro bodi sejajar garis)
//     5. BARU MAJU (Meluncur kembali lurus menyusuri lintasan)



// ====================================================================
inline void eksekusiBelokSudutKotak() {
  hitunganSudutKotak++;
  hitunganValidasiSudut = 0;

  Serial.println(F("\n======================================================="));
  if (tipeSudutTerdeteksi == TIKUNGAN_SIMPANG_3_KANAN) {
    Serial.print(F(">>> [SUDUT KOTAK #"));
    Serial.print(hitunganSudutKotak);
    Serial.println(F("] DETEKSI SIMPANG TIGA KANAN (S5 & S4 Aktif [0 0 1 1 1] - 3x VALID)"));
  } else if (tipeSudutTerdeteksi == TIKUNGAN_SIMPANG_3_KIRI) {
    Serial.print(F(">>> [SUDUT KOTAK #"));
    Serial.print(hitunganSudutKotak);
    Serial.println(F("] DETEKSI SIMPANG TIGA KIRI (S1 & S2 Aktif [1 1 1 0 0] - 3x VALID)"));
  } else if (tipeSudutTerdeteksi == TIKUNGAN_GELAP_SEMUA) {
    Serial.print(F(">>> [SUDUT KOTAK #"));
    Serial.print(hitunganSudutKotak);
    Serial.println(F("] DETEKSI GARIS MELINTANG / PEREMPATAN [1 1 1 1 1] (3x VALID)"));
  } else {
    Serial.print(F(">>> [SUDUT KOTAK #"));
    Serial.print(hitunganSudutKotak);
    Serial.println(F("] DETEKSI SUDUT SIKU (S4 & S5 Aktif - 3x VALID)"));
  }
  Serial.println(F("======================================================="));


  // [LANGKAH 1] KETEMU SIMPANG -> LANGSUNG BERHENTI
  // Tidak ada overshoot / maju ke depan lagi

  Serial.println(F("  [1/5] KETEMU SIMPANG: Langsung BERHENTI Total..."));
  berhentiTotal();
  delay(JEDA_BERHENTI_DI_SIMPANG_MS);











  // [LANGKAH 2] MUNDUR DIKIT SEBELUM BELOK
  // Menarik bodi mundur sedikit secara presisi sebelum mulai berbelok

  Serial.print(F("  [2/5] MUNDUR DIKIT "));
  Serial.print(DURASI_MUNDUR_SEBELUM_BELOK_MS);
  Serial.println(F("ms Sebelum Belok..."));
  setMotorLangsung(-KEC_MUNDUR_SEBELUM_BELOK, -KEC_MUNDUR_SEBELUM_BELOK);
  delay(DURASI_MUNDUR_SEBELUM_BELOK_MS);

  // Berhenti total diam sejenak menyerap inersia sebelum memutar poros
  berhentiTotal();
  delay(JEDA_DIAM_SEBELUM_BELOK_MS); // Berhenti diam sejenak (120 ms)


  // [LANGKAH 3] BELOK NYARI GARIS BARU 90° (PUTAR POROS ANTI-BABLAS)
  //   Fase A: Putar sampai lepas dari garis lama
  //   Fase B: Deteksi garis baru 90°. Saat ujung sensor menyentuh garis, rotasi DIPERLAMBAT.
  //           Begitu sensor tengah (S3) menyambar garis, LANGSUNG REM & KUNCI SEKETIKA!

  if (ARAH_BELOK == ARAH_BELOK_KANAN) {
    Serial.println(F("  [3/5] BELOK NYARI (KANAN): Putar Poros Cari Garis Baru 90°..."));
    // Sentakan breakaway kick instan 30ms mendobrak gesekan statis ban di spanduk:
    setMotorLangsung(PWM_BREAKAWAY_KIRI_MAJU, -PWM_BREAKAWAY_KANAN_MUNDUR);
    delay(DURASI_BREAKAWAY_MS);

    unsigned long tMulaiScan = millis();
    bool sudahLepasGarisLama = false;
    bool berhasilKunciGaris = false;

    while (millis() - tMulaiScan < TIMEOUT_BELOK_MS) {
      LineData l = bacaSensorGaris();

      // FASE A: Pastikan sensor lepas dari garis lama
      if (!sudahLepasGarisLama) {
        putarPorosKanan(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KANAN_MUNDUR);
        if ((millis() - tMulaiScan >= WAKTU_BLIND_SWEEP_MS && (l.totalAktif <= 1 && l.b3 == 0)) ||
            (millis() - tMulaiScan >= 140)) {
          sudahLepasGarisLama = true;
          Serial.println(F("    -> Lepas dari Garis Lama. Memindai Garis Baru 90°..."));
        }
      } else {
        // FASE B: Deteksi Garis Baru 90°
        // Jika baru sensor kanan (S5 atau S4) yang kena: PERLAMBAT putaran agar tidak bablas!
        if (l.b5 == 1 || l.b4 == 1) {
          putarPorosKanan(90, 115); // Perlambat putaran mendekati S3
        } else {
          putarPorosKanan(KEC_BELOK_KIRI_MAJU, KEC_BELOK_KANAN_MUNDUR);
        }

        // BEGITU SENSOR TENGAH (S3) MENYAMBAR GARIS: SEGERA BREAK & REM AKTIF!
        if (l.b3 == 1) {
          Serial.println(F("    -> GARIS BARU 90° KENA SENSOR TENGAH (S3=1)!"));
          berhasilKunciGaris = true;
          break; // Garis baru terkunci di S3!
        }
      }
      delay(2);
    }

    // Rem aktif lawan arah sejenak untuk menahan laju putar poros (anti-overshoot)
    setMotorLangsung(-PWM_REM_BELOK, PWM_REM_BELOK);
    delay(DURASI_REM_BELOK_MS);
    berhentiTotal();
    if (!berhasilKunciGaris) {
      Serial.println(F("    -> [TIMEOUT PUTAR] Garis 90° akan dicari dan diluruskan di Langkah 4!"));
    }

  } else {
    Serial.println(F("  [3/5] BELOK NYARI (KIRI): Putar Poros Cari Garis Baru 90°..."));
    // Belok Kiri: sentakan breakaway kick
    setMotorLangsung(-PWM_BREAKAWAY_KIRI_MUNDUR, PWM_BREAKAWAY_KANAN_MAJU);
    delay(DURASI_BREAKAWAY_MS);

    unsigned long tMulaiScan = millis();
    bool sudahLepasGarisLama = false;
    bool berhasilKunciGaris = false;

    while (millis() - tMulaiScan < TIMEOUT_BELOK_MS) {
      LineData l = bacaSensorGaris();

      if (!sudahLepasGarisLama) {
        putarPorosKiri(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KIRI_MUNDUR);
        if ((millis() - tMulaiScan >= WAKTU_BLIND_SWEEP_MS && (l.totalAktif <= 1 && l.b3 == 0)) ||
            (millis() - tMulaiScan >= 140)) {
          sudahLepasGarisLama = true;
          Serial.println(F("    -> Lepas dari Garis Lama. Memindai Garis Baru 90°..."));
        }
      } else {
        if (l.b1 == 1 || l.b2 == 1) {
          putarPorosKiri(90, 115); // Perlambat mendekati S3 (Roda kanan maju 90, roda kiri mundur 115)
        } else {
          putarPorosKiri(KEC_BELOK_KANAN_MAJU, KEC_BELOK_KIRI_MUNDUR);
        }

        if (l.b3 == 1) {
          Serial.println(F("    -> GARIS BARU 90° KENA SENSOR TENGAH (S3=1)!"));
          berhasilKunciGaris = true;
          break;
        }
      }
      delay(2);
    }

    // Rem aktif lawan arah
    setMotorLangsung(PWM_REM_BELOK, -PWM_REM_BELOK);
    delay(DURASI_REM_BELOK_MS);
    berhentiTotal();
    if (!berhasilKunciGaris) {
      Serial.println(F("    -> [TIMEOUT PUTAR] Garis 90° akan dicari dan diluruskan di Langkah 4!"));
    }
  }

  berhentiTotal();
  delay(40);


  // ==================================================================
  // [LANGKAH 4] FITUR NGELURUSIN (PELURUSAN PRESISI WAJIB S3 DI TENGAH)
  //   1. Jika S3 belum kena (baru ujung kanan/kiri): Wajib putar sampai S3 kena!
  //   2. Setelah S3 kena: Luruskan sudut bodi mikro hingga simetris sejajar!
  //   3. Pengaman mutlak: DILARANG MAJU jika S3 belum kena garis hitam!
  // ==================================================================
  Serial.println(F("  [4/5] FITUR NGELURUSIN: Wajib S3 di Tengah & Bodi Simetris..."));

  unsigned long tLuruskan = millis();
  int counterLurusSempurna = 0;

  while (millis() - tLuruskan < 1500) { // Toleransi waktu cukup (1.5 detik) untuk pelurusan presisi
    LineData l = bacaSensorGaris();

    // 1. TARGET SEMPURNA: S3 aktif di tengah, ujung luar b1=0, b5=0, dan sayap b2==b4
    if (l.b3 == 1 && l.b1 == 0 && l.b5 == 0 && (l.b2 == l.b4)) {
      counterLurusSempurna++;
      if (counterLurusSempurna >= 2) {
        Serial.println(F("    -> [LURUS SEMPURNA] S3 terkunci tepat di tengah & bodi sejajar!"));
        break; // LURUS SEMPURNA TERCAPAI!
      }
      delay(20);
      continue;
    } else {
      counterLurusSempurna = 0;
    }

    // 2. KASUS A: SENSOR TENGAH (S3) BELUM KENA! (S3 == 0)
    //    Wajib cari dan putar sampai minimal S3 kena!
    if (l.b3 == 0) {
      if (ARAH_BELOK == ARAH_BELOK_KANAN) {
        if (l.b5 == 1 || l.b4 == 1) {
          // Putaran belum maksimal (garis masih di kanan) -> Putar kanan lagi cari S3!
          Serial.println(F("    -> [BELUM MAKSIMAL KANAN] Baru ujung kanan kena: Putar kanan lagi cari S3..."));
          putarPorosKanan(110, 140);
        } else if (l.b1 == 1 || l.b2 == 1) {
          // Putaran kebablasan (garis di kiri) -> Putar balik kiri cari S3!
          Serial.println(F("    -> [KEBABLASAN KANAN] Ujung kiri kena: Putar balik kiri cari S3..."));
          putarPorosKiri(110, 140);
        } else {
          putarPorosKanan(100, 125);
        }
      } else { // ARAH_BELOK == ARAH_BELOK_KIRI
        if (l.b1 == 1 || l.b2 == 1) {
          // Putaran belum maksimal (garis masih di kiri) -> Putar kiri lagi cari S3!
          Serial.println(F("    -> [BELUM MAKSIMAL KIRI] Baru ujung kiri kena: Putar kiri lagi cari S3..."));
          putarPorosKiri(110, 140);
        } else if (l.b5 == 1 || l.b4 == 1) {
          // Putaran kebablasan (garis di kanan) -> Putar balik kanan cari S3!
          Serial.println(F("    -> [KEBABLASAN KIRI] Ujung kanan kena: Putar balik kanan cari S3..."));
          putarPorosKanan(110, 140);
        } else {
          putarPorosKiri(100, 125);
        }
      }
      delay(25);
      berhentiTotal();
      delay(25);
    }
    // 3. KASUS B: SENSOR TENGAH (S3) SUDAH KENA! (S3 == 1)
    //    Luruskan sudut kemiringan bodi secara mikro (FITUR NGELURUSIN)
    else {
      if (l.b4 == 1 && l.b2 == 0) {
        // Garis agak ke kanan (bodi moncong ke kiri) -> putar kanan mikro
        putarPorosKanan(80, 80);
        delay(15);
        berhentiTotal();
        delay(25);
      } else if (l.b2 == 1 && l.b4 == 0) {
        // Garis agak ke kiri (bodi moncong ke kanan) -> putar kiri mikro
        putarPorosKiri(80, 80);
        delay(15);
        berhentiTotal();
        delay(25);
      } else {
        // b2 == b4, bodi sudah seimbang di S3!
        break;
      }
    }
  }

  berhentiTotal();

  // PENGAMAN MUTLAK ANTI-MAJU TANPA S3:
  // Jika karena kondisi ekstrem S3 masih belum kena, lakukan pulsa akhir sampai S3 kena!
  LineData lCek = bacaSensorGaris();
  int batasDarurat = 0;
  while (lCek.b3 == 0 && batasDarurat < 12) {
    Serial.println(F("    -> [PENGAMAN MUTLAK] S3 belum kena! Mencari S3 sebelum diizinkan maju..."));
    if (ARAH_BELOK == ARAH_BELOK_KANAN) {
      if (lCek.b5 == 1 || lCek.b4 == 1) {
        putarPorosKanan(110, 140);
      } else if (lCek.b1 == 1 || lCek.b2 == 1) {
        putarPorosKiri(110, 140);
      } else {
        putarPorosKanan(110, 140);
      }
    } else { // ARAH_BELOK_KIRI
      if (lCek.b1 == 1 || lCek.b2 == 1) {
        putarPorosKiri(110, 140);
      } else if (lCek.b5 == 1 || lCek.b4 == 1) {
        putarPorosKanan(110, 140);
      } else {
        putarPorosKiri(110, 140);
      }
    }
    delay(25);
    berhentiTotal();
    delay(25);
    lCek = bacaSensorGaris();
    batasDarurat++;
  }

  berhentiTotal();
  delay(JEDA_SETELAH_LURUS_MS); // Berhenti diam stabil setelah bodi benar-benar lurus di S3 (150 ms)



  // [LANGKAH 5] BARU MAJU

  Serial.println(F("  [5/5] BARU MAJU: Meluncur Kembali Menyusuri Lintasan!\n"));

  // Reset pewaktu cooldown dan filter kecepatan
  waktuTerakhirBelok = millis();
  errSebelumnyaPD    = 0.0;
  smoothSpeedKiri    = 0.0;
  smoothSpeedKanan   = 0.0;
}


// ====================================================================
// [9] TRACKING GARIS LURUS DENGAN FITUR NGELURUSIN WAJIB SENSOR TENGAH (S3)
// ====================================================================
inline void jalankanTrackingLurus(const LineData& line) {
  // ------------------------------------------------------------------
  // KASUS 1: SENSOR TENGAH KENA (line.b3 == 1) -> TARGET UTAMA TERCAPAI
  // ------------------------------------------------------------------
  if (line.b3 == 1) {
    // 1A. Lurus Sempurna di Tengah S3 (S2 == S4):
    if (line.b2 == line.b4) {
      setMotorHalus(KEC_BASE + TRIM_KIRI, KEC_BASE + TRIM_KANAN);
      errSebelumnyaPD = 0.0;
      arahGarisTerakhir = 0;
      return;
    }

    // 1B. Sedikit Condong ke Kiri (S3=1 dan S2=1): Garis ada di kiri tengah
    // FITUR NGELURUSIN: Tarik bodi sedikit ke kiri agar kembali ke S3 murni
    if (line.b2 == 1 && line.b4 == 0) {
      setMotorHalus(KEC_BASE - 20 + TRIM_KIRI, KEC_BASE + 25 + TRIM_KANAN);
      errSebelumnyaPD = -0.5;
      arahGarisTerakhir = -1;
      return;
    }

    // 1C. Sedikit Condong ke Kanan (S3=1 dan S4=1): Garis ada di kanan tengah
    // FITUR NGELURUSIN: Tarik bodi sedikit ke kanan agar kembali ke S3 murni
    if (line.b4 == 1 && line.b2 == 0) {
      setMotorHalus(KEC_BASE + 25 + TRIM_KIRI, KEC_BASE - 20 + TRIM_KANAN);
      errSebelumnyaPD = 0.5;
      arahGarisTerakhir = +1;
      return;
    }
  }

  // ------------------------------------------------------------------
  // KASUS 2: SENSOR TENGAH LEPAS (line.b3 == 0) -> WAJIB DIKEMBALIKAN KE S3 SEKETIKA!
  // Ban karet 4WD pada lantai spanduk butuh counter-torque agar hidung mobil berputar cepat
  // menyambar kembali sensor tengah (S3), BUKAN meluncur serong maju!
  // ------------------------------------------------------------------

  // 2A. Garis di S2 (Kiri Tengah): Mobil miring ke kanan, S3 lepas!
  // Tindakan: Roda kiri rem/mundur, roda kanan maju kuat -> hidung mobil terpelintir cepat ke kiri mencari S3!
  if (line.b2 == 1 && line.b4 == 0) {
    setMotorLangsung(-25, KEC_BASE + 35 + TRIM_KANAN);
    errSebelumnyaPD = -1.0;
    arahGarisTerakhir = -1;
    return;
  }

  // 2B. Garis di S4 (Kanan Tengah): Mobil miring ke kiri, S3 lepas!
  // Tindakan: Roda kanan rem/mundur, roda kiri maju kuat -> hidung mobil terpelintir cepat ke kanan mencari S3!
  if (line.b4 == 1 && line.b2 == 0) {
    setMotorLangsung(KEC_BASE + 35 + TRIM_KIRI, -25);
    errSebelumnyaPD = 1.0;
    arahGarisTerakhir = +1;
    return;
  }

  // 2C. Garis di S1 (Kiri Luar): Mobil sangat melenceng ke kanan!
  if (line.b1 == 1) {
    setMotorLangsung(-45, KEC_BASE + 50 + TRIM_KANAN);
    errSebelumnyaPD = -2.0;
    arahGarisTerakhir = -1;
    return;
  }

  // 2D. Garis di S5 (Kanan Luar): Mobil sangat melenceng ke kiri!
  if (line.b5 == 1) {
    setMotorLangsung(KEC_BASE + 50 + TRIM_KIRI, -45);
    errSebelumnyaPD = 2.0;
    arahGarisTerakhir = +1;
    return;
  }

  // ------------------------------------------------------------------
  // KASUS 3: GARIS LEPAS TOTAL (Semua Sensor Putih [0 0 0 0 0])
  // ------------------------------------------------------------------
  if (line.isGarisHilang) {
    if (arahGarisTerakhir == -1) {
      setMotorLangsung(-25, KEC_BASE + 25);
    } else if (arahGarisTerakhir == +1) {
      setMotorLangsung(KEC_BASE + 25, -25);
    } else {
      setMotorHalus(KEC_BASE, KEC_BASE);
    }
    return;
  }
}


// ====================================================================
// [10] DIAGNOSTIK & MONITOR PERINTAH SERIAL INTERAKTIF
// ====================================================================
inline void tampilkanBarSensor(const LineData& line) {
  Serial.print(F("[SENSOR] "));
  Serial.print(line.b1 ? F("[■]") : F("[ ]"));
  Serial.print(line.b2 ? F("[■]") : F("[ ]"));
  Serial.print(line.b3 ? F("[■]") : F("[ ]"));
  Serial.print(line.b4 ? F("[■]") : F("[ ]"));
  Serial.print(line.b5 ? F("[■]") : F("[ ]"));
  Serial.print(F(" | Err: "));
  Serial.print(line.error, 1);
  Serial.print(F(" | Tot: "));
  Serial.print(line.totalAktif);
  Serial.print(F(" | Torsi: "));
  Serial.print(KEC_BASE);
  Serial.print(F(" | Arah: "));
  Serial.print(ARAH_BELOK == ARAH_BELOK_KANAN ? F("KANAN") : F("KIRI"));
  Serial.print(F(" | Sudut: "));
  Serial.println(hitunganSudutKotak);
}

inline void prosesInputSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  // Abaikan newline / carriage return
  if (c == '\r' || c == '\n') return;

  switch (c) {
    case ' ': // Spasi: Toggle Jalan / Stop
      mobilAktif = !mobilAktif;
      if (!mobilAktif) {
        berhentiTotal();
        Serial.println(F("\n[STATUS] MOBIL DIHENTIKAN (STOPPED) via Serial."));
      } else {
        Serial.println(F("\n[STATUS] MOBIL MELANJUTKAN PERJALANAN (RUNNING)."));
      }
      break;

    case '+': // Tambah Kecepatan / Torsi
      KEC_BASE = constrain(KEC_BASE + 5, 40, 200);
      Serial.print(F("[PENGATURAN] KEC_BASE Dinaikkan -> "));
      Serial.println(KEC_BASE);
      break;

    case '-': // Kurangi Kecepatan
      KEC_BASE = constrain(KEC_BASE - 5, 40, 200);
      Serial.print(F("[PENGATURAN] KEC_BASE Diturunkan -> "));
      Serial.println(KEC_BASE);
      break;

    case 'k':
    case 'K': // Ganti ke Belok KANAN
      ARAH_BELOK = ARAH_BELOK_KANAN;
      Serial.println(F("\n[PENGATURAN] Arah Belok Diubah Menjadi: >>> KANAN <<<"));
      break;

    case 'l':
    case 'L': // Ganti ke Belok KIRI
      ARAH_BELOK = ARAH_BELOK_KIRI;
      Serial.println(F("\n[PENGATURAN] Arah Belok Diubah Menjadi: >>> KIRI <<<"));
      break;

    case 'c':
    case 'C': // Toggle Mode Kalibrasi Sensor
      modeKalibrasi = !modeKalibrasi;
      if (modeKalibrasi) {
        berhentiTotal();
        Serial.println(F("\n[MODE KALIBRASI] Motor Berhenti. Putar Trimpot LM393 Sambil Perhatikan Tampilan Sensor!"));
      } else {
        Serial.println(F("\n[MODE NORMAL] Keluar dari Kalibrasi."));
      }
      break;

    case 't':
    case 'T': // Uji Coba Belok di Tempat Sekarang Juga
      Serial.println(F("\n[TEST] Menguji Eksekusi Belok 90° Manual..."));
      eksekusiBelokSudutKotak();
      break;

    case 'r':
    case 'R': // Reset Hitungan Sudut
      hitunganSudutKotak    = 0;
      hitunganValidasiSudut = 0;
      Serial.println(F("\n[RESET] Hitungan Sudut Kotak Direset ke 0."));
      break;

    default:
      Serial.println(F("\n[MENU BANTUAN]:"));
      Serial.println(F("  ' ' (Spasi) : Start / Stop Mobil"));
      Serial.println(F("  '+' / '-'   : Naikkan / Turunkan Kecepatan Torsi (KEC_BASE)"));
      Serial.println(F("  'K'         : Set Arah Belok ke KANAN"));
      Serial.println(F("  'L'         : Set Arah Belok ke KIRI"));
      Serial.println(F("  'C'         : Toggle Mode Kalibrasi Sensor (Set Trimpot)"));
      Serial.println(F("  'T'         : Uji Coba Manuver Belok 90° di Tempat"));
      Serial.println(F("  'R'         : Reset Counter Sudut"));
      break;
  }
}


// ====================================================================
// [11] ARDUINO SETUP & LOOP
// ====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n======================================================="));
  Serial.println(F("   PROGRAM UJI LINTASAN KOTAK SEMPURNA (AGV ROBOT)     "));
  Serial.println(F("   Board : Mappi32 (ESP32)                             "));
  Serial.println(F("======================================================="));
  Serial.print(F(" [PENGATURAN AWAL]:\n"));
  Serial.print(F("   - Arah Belok Kotak : "));
  Serial.println(ARAH_BELOK == ARAH_BELOK_KANAN ? F("KANAN TERUS") : F("KIRI TERUS"));
  Serial.print(F("   - Kecepatan Dasar  : "));
  Serial.print(KEC_BASE);
  Serial.println(F(" PWM (Optimal Spanduk)"));
  Serial.print(F("   - Trim Motor       : Kiri +"));
  Serial.print(TRIM_KIRI);
  Serial.print(F(" | Kanan "));
  Serial.println(TRIM_KANAN);
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F(" [PANDUAN KONTROL KEYBOARD SERIAL MONITOR]:"));
  Serial.println(F("   Ketik ' ' (SPASI) -> Stop / Start Mobil"));
  Serial.println(F("   Ketik '+' / '-'   -> Naikkan / Turunkan Torsi"));
  Serial.println(F("   Ketik 'K'         -> Ganti ke Belok KANAN"));
  Serial.println(F("   Ketik 'L'         -> Ganti ke Belok KIRI"));
  Serial.println(F("   Ketik 'C'         -> Mode Kalibrasi Trimpot Sensor"));
  Serial.println(F("   Ketik 'T'         -> Tes Belok 90° di Tempat"));
  Serial.println(F("======================================================="));

  // Inisialisasi Perangkat Keras
  initMotor();
  initSensors();
  berhentiTotal();

  Serial.println(F("\n[INFO] Mobil otomatis meluncur dalam 3 detik...\n"));
  delay(3000);
  waktuTerakhirBelok = millis();
}

void loop() {
  // 1. Pantau Perintah Serial Monitor
  prosesInputSerial();

  // 2. Baca 5 Sensor Garis
  LineData line = bacaSensorGaris();

  // 3. Jika Dalam Mode Kalibrasi: Hanya Tampilkan Sensor & Matikan Motor
  if (modeKalibrasi) {
    berhentiTotal();
    if (millis() - lastTelemetriTime >= 100) {
      lastTelemetriTime = millis();
      tampilkanBarSensor(line);
    }
    delay(10);
    return;
  }

  // 4. Jika Mobil Sedang Di-Stop
  if (!mobilAktif) {
    berhentiTotal();
    delay(20);
    return;
  }

  // 5. Cek Apakah Menemui Sudut Kotak 90°
  if (cekSudutKotak(line)) {
    eksekusiBelokSudutKotak();
    return; // Langsung kembali ke loop berikutnya
  }

  // 6. Jika Tidak di Sudut: Lacak Garis Lurus Sisi Kotak
  jalankanTrackingLurus(line);

  // 7. Telemetri Ringkas Setiap 200ms
  if (millis() - lastTelemetriTime >= 200) {
    lastTelemetriTime = millis();
    tampilkanBarSensor(line);
  }

  delay(2); // Loop rate ~400-500 Hz (responsif tanpa membebani CPU)
}
