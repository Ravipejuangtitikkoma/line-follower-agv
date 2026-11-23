# Panduan Lengkap Proyek Robot AGV (Mappi32 ESP32)

> **Ringkasan 30 Detik:**  
> Proyek ini adalah **Robot AGV (*Automated Guided Vehicle*) Line Follower 4WD** berbasis **Mappi32 (ESP32)** untuk mengantar barang otonom di arena grid **3 m × 3 m**.  
> Saat dinyalakan, program utama (`mobil_code.ino`) otomatis menunggu 2 detik lalu menjalankan rute **Pulang-Pergi (PP) 24-State** dari `START`, berbelok di simpang `N0` dan `N2`, berhenti jeda di jalur atas, lalu putar balik 180° dan pulang parkir ke `START`.

---

## Daftar Isi

1. [Spesifikasi Hardware & Fisik Robot](#1-spesifikasi-hardware--fisik-robot)
2. [Peta Isi Semua File & Folder](#2-peta-isi-semua-file--folder)
3. [Konfigurasi Pin & Wiring](#3-konfigurasi-pin--wiring)
4. [Peta Jalur 3m × 3m & Alur Kerja Robot (24 State)](#4-peta-jalur-3m--3m--alur-kerja-robot-24-state)
5. [6 Hal Penting yang Wajib Diketahui (Gap Kode vs Foto)](#5-6-hal-penting-yang-wajib-diketahui-gap-kode-vs-foto)
6. [Panduan Menjalankan & Menguji (Step-by-Step)](#6-panduan-menjalankan--menguji-step-by-step)

---

## 1. Spesifikasi Hardware & Fisik Robot

| Komponen / Parameter | Spesifikasi | Catatan |
| :--- | :--- | :--- |
| **Mikrokontroler** | **Mappi32 (ESP32-WROOM-32E)** by KMtek | Punya modul radio LoRa internal (RFM95 / SX1276) |
| **Sistem Penggerak** | **4WD Skid-Steer** (4 roda, belok selip di tempat) | 2 motor kiri diparalel ke 1 driver; 2 motor kanan diparalel ke 1 driver |
| **Motor DC** | **4× Motor Gearbox** (tipe JGA25-370 / JGB37-520) | Roda kiri dilengkapi encoder Hall-Effect (6 kabel) |
| **Driver Motor** | **2× Driver BTS7960** (Kiri & Kanan) | Ditenagai baterai **12V LiPo** |
| **Sensor Garis** | **5-Channel TCRT5000** | Terpasang di moncong depan (±14 cm di depan as roda) |
| **Sensor Jarak** | **Ultrasonik HC-SR04** | Saat ini **DINONAKTIFKAN** di kode (`Config.h`) |
| **Dimensi & Bobot** | Massa `1,45 kg` \| Panjang `25 cm` \| Roda `Ø 6,5 cm` | Jarak sumbu roda (*wheelbase*) `18 cm` \| Lebar lintasan roda `18 cm` |

---

## 2. Peta Isi Semua File & Folder

Proyek ini terbagi menjadi **3 kelompok**: program utama AGV, dokumentasi perancangan, dan 9 folder skrip pengujian komponen.

### A. File Inti Program Utama (Folder Root)

| File | Fungsi Utama |
| :--- | :--- |
| `mobil_code.ino` | **Program utama** (`setup()` & `loop()`). Menjalankan sensor, kontrol FSM navigasi, perintah Serial, dan telemetri tiap `250 ms`. |
| `Config.h` | **Pusat semua pin & parameter tuning** (kecepatan PWM, PID `Kp`/`Kd`, trim kiri/kanan, offset belok, rem aktif, timeout). |
| `Motor.h` | Penggerak motor 2 arah untuk BTS7960, *EMA smoothing* (`ALPHA_SMOOTH`), putar poros, dan *Breakaway Torque Kick* (`30 ms`) agar ke-4 roda tidak selip/mematung. |
| `Sensor.h` | Pembacaan 5 sensor garis (`REVERSE_SENSOR_ORDER`), hitung error `-2.0` s/d `+2.0`, deteksi pola simpang, ISR encoder roda kiri, dan fungsi HC-SR04. |
| `NavigationFSM.h` | **Otak navigasi utama**: Finite State Machine **25 state (`0`–`25`)** untuk rute PP, `jalankanTrackingPD()`, *recovery* anti-bablas, dan parser perintah Serial. |
| `NodeDetector.h` | Modul belok 90° & deteksi node **versi lama** (blocking). Masih di-`#include`, namun fungsi beloknya **sudah tidak dipakai** (hanya `resetNodeDetector()` yang dipanggil). |

### B. File Dokumentasi Rancangan & Fisika

| File | Isi |
| :--- | :--- |
| `SKEMA_AGV_ROBOT.md` & `SKEMA_AGV_ROBOT.html` | Konsep awal sesuai permintaan klien (denah 3m×3m, 17 node, 5 stasiun, rencana protokol LoRa). File `.html` bisa dibuka di browser. |
| `plan.md` | Catatan rencana upgrade FSM versi 11-state (sebelum ditambah rute pulang menjadi 24-state). |
| `rumus.md` | Analisis fisika *skid-steer*, torsi PWM, *breakaway kick*, kontroler PD, dan *overhang* sensor. |

### C. 9 Folder Skrip Pengujian & Diagnostik (Standalone)

| Folder Uji | Target Board | Baudrate | Kegunaan |
| :--- | :---: | :---: | :--- |
| `test_sensor_garis_tunggal/` | Mappi32 / ESP32 | `115200` | Uji **1 kanal** sensor garis (digital + ADC 12-bit + voltase) untuk setel trimpot. |
| `test_sensor_garis/` | Mappi32 / ESP32 | `115200` | Uji **5 kanal** sensor garis sekaligus dengan visualisasi `[■]` / `[ ]` real-time. |
| `test_maju_mundur/` | Mappi32 / ESP32 | `115200` | Uji gerak dasar: maju 2 detik → stop → mundur 2 detik berulang. |
| `test_lintasan_kotak/` | Mappi32 / ESP32 | `115200` | Uji loop **lintasan kotak di atas spanduk** (`KEC_BASE = 68`, berhenti → mundur 70 ms → belok → wajib S3). |
| `test_lintasan_kotak_kiri/` | Mappi32 / ESP32 | `115200` | Uji lintasan kotak belok kiri *(catatan: isi `.ino` saat ini identik dengan `test_lintasan_kotak.ino`)*. |
| `test_lora/` (`sender` & `receiver`) | 2× Mappi32 | `115200` | Uji komunikasi radio **LoRa 915 MHz** dua arah (*ping-pong* + `CMD:START/STOP/PULANG`). Motor dikunci mati. |
| `test_motor_driver/` | Arduino Uno/Nano | `9600` | Cek driver BTS7960 & encoder tanpa Mappi32 (`D5, D6, D9, D10` & `D2, D3`). |
| `test_sensor_motor/` | Arduino Uno/Nano | `9600` | Cek pulsa & konslet encoder 6-kabel dengan memutar roda pakai tangan (`D2, D3`). |
| `test_sensor_jarak/` | Arduino Uno/Nano | `9600` | Cek kesehatan & konslet sensor HC-SR04 pada logika 5V (`TRIG=D9, ECHO=D10`). |

---

## 3. Konfigurasi Pin & Wiring

### A. Pin Resmi pada Robot Utama — Mappi32 ESP32 (`Config.h`)

#### 1. Driver Motor (2× BTS7960) & Baterai 12V

| Komponen | Pin Modul | Pin Mappi32 (GPIO) | Keterangan |
| :--- | :--- | :---: | :--- |
| **Driver Kiri** | `RPWM` (IN1 Maju) | **`GPIO 25`** | Output PWM maju 2 roda kiri |
| | `LPWM` (IN2 Mundur) | **`GPIO 26`** | Output PWM mundur 2 roda kiri |
| **Driver Kanan** | `RPWM` (IN3 Maju) | **`GPIO 16`** | Output PWM maju 2 roda kanan *(solder manual IO16)* |
| | `LPWM` (IN4 Mundur) | **`GPIO 17`** | Output PWM mundur 2 roda kanan *(solder manual IO17)* |
| **Enable & Daya** | `R_EN` & `L_EN` | **`5V`** | Jumper kedua pin ini ke 5V agar driver selalu aktif |
| | `VCC` / `GND` Driver | `5V` / `GND` | Suplai logika chip driver |
| | `B+` / `B-` Driver | `+` / `-` **Baterai 12V LiPo** | `B-` wajib tersambung ke `GND` Mappi32 (**Common Ground**) |

#### 2. Sensor Garis 5-Channel (TCRT5000)

> [!IMPORTANT]
> Di kode aktif sekarang (`Config.h` & `Sensor.h`):
> 1. **Logika Active HIGH:** `1` = **Garis Hitam**, `0` = **Lantai / Spanduk Putih**.
> 2. **Urutan Dibalik (`REVERSE_SENSOR_ORDER = true`):** Kabel fisik dari modul PCB dipetakan terbalik antara kiri dan kanan seperti tabel di bawah.

| Pin Modul PCB | Pin Mappi32 (GPIO) | Tipe GPIO | Pemetaan di Program (`REVERSE = true`) |
| :--- | :---: | :--- | :--- |
| `OUT1` | **`GPIO 34`** | Input-Only (GPI) | **`b5` — Kanan Luar** |
| `OUT2` | **`GPIO 35`** | Input-Only (GPI) | **`b4` — Kanan Tengah** |
| `OUT3` | **`GPIO 36`** (`VP`) | Input-Only (GPI) | **`b3` — Tengah (Wajib Kunci Garis)** |
| `OUT4` | **`GPIO 39`** (`VN`) | Input-Only (GPI) | **`b2` — Kiri Tengah** |
| `OUT5` | **`GPIO 23`** | GPIO Standar | **`b1` — Kiri Luar** |
| `VCC` / `GND` | `3.3V` atau `5V` / `GND` | Power | Gunakan common ground dengan Mappi32 |

#### 3. Encoder Roda Kiri (6 Kabel) & Sensor Jarak HC-SR04

| Komponen | Kabel / Pin | Pin Mappi32 (GPIO) | Keterangan |
| :--- | :--- | :---: | :--- |
| **Encoder Roda Kiri** | **Kuning** (Fase A) | **`GPIO 32`** | Interrupt `RISING` (`INPUT_PULLUP`) |
| | **Hijau** (Fase B) | **`GPIO 33`** | Penentu arah putaran (`INPUT_PULLUP`) |
| | **Biru** / **Hitam** | `5V` (atau `3.3V`) / `GND` | Daya IC Hall Effect |
| | **Merah** & **Putih** | `M+` / `M-` Driver Kiri | Kabel daya motor DC (**jangan colok ke pin GPIO!**) |
| **Ultrasonik HC-SR04** | `TRIG` / `ECHO` | **`-1` (NONAKTIF)** | Sebelumnya `GPIO 19` / `GPIO 18` — saat ini dimatikan |

#### 4. Pin Terlarang — LoRa Internal Mappi32 (RFM95 / SX1276)

| Fungsi Internal LoRa | Pin GPIO ESP32 | Status |
| :--- | :---: | :--- |
| `SCK` / `MISO` / `MOSI` | **`14` / `12` / `13`** | **DILARANG** dipakai untuk motor atau sensor eksternal! |
| `NSS (SS)` / `RST` / `DIO0` | **`15` / `0` / `27`** | **DILARANG** dipakai untuk motor atau sensor eksternal! |
| **LED Onboard Mappi32** | **`GPIO 5`** | Dipakai sebagai indikator TX/RX pada skrip uji LoRa |

### B. Ringkasan Pin untuk Skrip Diagnostik Arduino Uno / Nano (5V)

Gunakan tabel ini hanya saat memakai folder uji khusus Arduino Uno/Nano:

| Skrip Pengujian | Pin yang Dipakai di Arduino Uno / Nano | Baudrate |
| :--- | :--- | :---: |
| `test_motor_driver.ino` | Driver Kiri: `D5` (`RPWM`), `D6` (`LPWM`) \| Driver Kanan: `D9` (`RPWM`), `D10` (`LPWM`) \| Encoder: `D2`, `D3` | `9600` |
| `test_sensor_motor.ino` | Encoder: Kuning (`Fase A`) → `D2`, Hijau (`Fase B`) → `D3`, Biru → `5V`, Hitam → `GND` *(Merah & Putih lepas!)* | `9600` |
| `test_sensor_jarak.ino` | HC-SR04: `VCC` → `5V`, `GND` → `GND`, `TRIG` → `D9`, `ECHO` → `D10` | `9600` |

### C. Aturan Keselamatan Hardware (Wajib Dibaca!)

> [!CAUTION]
> 1. **Wajib Common Ground:** Kutub negatif baterai 12V (`B-`) harus tersambung dengan `GND` Mappi32. Jika putus saat motor berputar, arus balik (*back-EMF*) bisa merusak board.
> 2. **Bahaya Pin `ECHO` 5V ke ESP32 (3.3V):** Mappi32 sebelumnya sempat panas karena sinyal `5V` dari `ECHO` HC-SR04 langsung masuk ke pin GPIO ESP32 (batas aman `3.3V`). Jika mengaktifkan HC-SR04 lagi, **wajib pasang *voltage divider*** (`1 kΩ` + `2 kΩ`) di jalur `ECHO` dan cek dulu sensornya pakai Arduino Uno (`test_sensor_jarak.ino`) apakah IC belakangnya panas/konslet.
> 3. **Antena LoRa Wajib Terpasang:** Jangan menyalakan Mappi32 untuk uji LoRa tanpa antena terpasang agar modul RF tidak rusak.
> 4. **Ganjal Bodi Robot Saat Tes Motor:** Sebelum menguji motor, ganjal sasis agar ke-4 roda melayang di udara.

---

## 4. Peta Jalur 3m × 3m & Alur Kerja Robot (24 State)

### A. Denah Arena Sesuai Foto Jalur

Arena berukuran **3 m × 3 m** dengan **17 titik simpang (`N0`–`N16`)** dan **5 stasiun kotak merah (`[A]`–`[E]`)**:

```text
[N1] ────────────────── [N5] ────────────────── [N9] ────────────────── [N13]
 │                       │                       │                       │
 │       BLOK A          │                       │       BLOK C          │
 │                       │                       │                       │
[N2] ═════ [ A ] ══════ [N6] ══════════════════ [N10] ═════ [ C ] ═════ [N14]
 ║                       │                       │                       │
 ║                       │       BLOK B          │                       │
[N0] ◄════ (START)       │                       │                       │
 │                       │                       │                       │
[N3] ────────────────── [N7] ───── [ B ] ────── [N11] ────────────────── [N15]
 │                       │                       │                       │
 │       BLOK E          │                       │       BLOK D          │
 │                       │                       │                       │
[N4] ───── [ E ] ────── [N8] ────────────────── [N12] ───── [ D ] ───── [N16]
```
*(Garis tebal `═`/`║` menunjukkan koridor jalur kuning pada foto arena.)*

### B. Alur 24 State di `NavigationFSM.h`

```mermaid
flowchart LR
    S1["START -> N0 (State 1-3: Belok Kiri)"] --> S4["N0 -> N2 (State 4-6: Belok Kanan)"]
    S4 --> S7["N2 -> N6 (State 7-9: Stop 3s lalu Lurus)"]
    S7 --> S10["N6 -> Ruas C (State 10-12: Lewat N10 & Parkir 3s)"]
    S10 --> S13["U-Turn 180° (State 13)"]
    S13 --> S14["Pulang Lewati N10 & N6 (State 14-19: Stop 1,5s x2)"]
    S14 --> S20["N2 -> N0 -> START (State 20-25: Kiri, Kanan, Parkir)"]
```

#### 1. Perjalanan Berangkat (`State 1` → `State 12`)

| State | Lokasi di Foto | Aksi Robot | Pemicu Pindah State |
| :---: | :--- | :--- | :--- |
| **`1`** | `START` → `N0` | Auto-align di tempat (maks `250 ms`), lalu maju PD (`KEC_BASE = 58`) | Gelap semua `[11111]` atau `b1=b2=b3=1` → maju offset `59 ms` → rem |
| **`2–3`** | Simpang **`N0`** | *Breakaway kick* `185 PWM`, putar poros kiri (`134 PWM`) | `S3 & S4` mengunci garis vertikal `N0→N2` → rem lawan arah `35 ms` |
| **`4`** | `N0` → `N2` | Maju PD ke utara menyusuri kolom kiri | Deteksi cabang kanan (`S4=1 && S5=1` + `S3=1`/`tot>=3`) **2× berturut** → offset `59 ms` → rem |
| **`5–6`** | Simpang **`N2`** | Putar poros kanan (`148/153 PWM`), blind `200–260 ms`, lambat `130 PWM` | `isLurusBelokKanan()` (`[00110]`/`[01110]`/`[00100]`) → rem lawan arah |
| **`7–8`** | Ruas `N2`–`N6` (ke **`N6`**) | Maju PD → rem keras tepat sebelum `N6`, **BERHENTI 3 DETIK** *(di kode disebut "Titik A")* | Simpang 4 pertama (`N6`: gelap semua / perempatan), lalu tunggu `3000 ms` atau ketik `LANJUT` |
| **`9`** | Melewati **`N6`** | Dorong lurus tanpa belok `450 ms`, lalu lanjut PD | Setelah `450 ms` dan sensor keluar dari garis melintang `N6` |
| **`10–12`** | `N6` → `N10` → ruas `[C]` | Maju PD sampai `N10`, lalu maju penetrasi **`495 ms`** melewati `N10`, **PARKIR 3 DETIK** *(di kode disebut "Titik B")* | Deteksi `N10` (`> 500 ms` & gelap semua) → penetrasi `495 ms` → jeda `3000 ms` atau ketik `PULANG` |

#### 2. Perjalanan Pulang (`State 13` → `State 25`)

| State | Lokasi di Foto | Aksi Robot | Pemicu Pindah State |
| :---: | :--- | :--- | :--- |
| **`13`** | Ruas `N10–N14` | Maju `765 ms` → diam `90 ms` → **U-Turn 180°** ke kanan (`142/146 PWM`, blind `350 ms`) | `S3` mengunci garis kembali menghadap barat |
| **`14–16`** | Melintasi **`N10`** ke barat | Maju PD sampai `N10` → **berhenti `1,5 detik`** → maju lurus melangkahi `N10` | Simpang `N10` terdeteksi → tunggu `1500 ms` → lewati `N10` (`>= 450 ms`) |
| **`17–19`** | Melintasi **`N6`** ke barat | Maju PD sampai `N6` → **berhenti `1,5 detik`** → maju lurus melangkahi `N6` | Simpang `N6` terdeteksi → tunggu `1500 ms` → lewati `N6` (`>= 450 ms`) |
| **`20–21`** | Simpang **`N2`** (ke selatan) | Maju PD sampai simpang T melintang → offset `59 ms` → **belok kiri 90°** (torsi ekstra `156/158 PWM`, kick `195`) | `S3 & S4` mengunci garis vertikal `N2→N0` |
| **`22–23`** | Simpang **`N0`** (ke `START`) | Maju ke selatan → deteksi cabang kanan (validasi 2×) → offset `59 ms` → **belok kanan 90°** | `isLurusBelokKanan()` mengunci garis ke `START` |
| **`24–25`** | `N0` → **`START`** | Maju PD menyusuri garis keluar hingga menyentuh tanda `START` → **Parkir & STANDBY** | Gelap semua / perempatan di titik `START` |

### C. Cara Kerja Kontrol Garis Lurus & Anti-Selip

1. **Tracking Lurus PD (`jalankanTrackingPD`):**  
   Saat `S3 = 1`, robot dikendalikan rumus PD (`Kp = 30.0`, `Kd = 20.0`) + *EMA smoothing* (`0.25`) + penyeimbang roda (`KEC_BASE = 58`, `TRIM_KIRI = +22`, `TRIM_KANAN = -6`).
2. **Hentakan Tepi & Anti-Bablas:**  
   * Garis geser ke `S2`/`S4` tanpa `S3` → hentak `145` vs `15 PWM`.
   * Garis mepet `S1`/`S5` tanpa `S3` → *counter-torque* `165` vs `-60 PWM`.
   * Garis hilang `[00000]` → putar balik penuh `175` vs `-85 PWM` ke arah `arahGarisTerakhir`.
3. **Breakaway Torque Kick (`Motor.h`):**  
   Karena 2 motor per sisi diparalel dan beban belakang lebih berat, belokan selalu diawali hentakan PWM tinggi selama `30 ms` (`185` / `162` / `195`) agar ke-4 roda memecah gesekan statis serentak tanpa macet.

---

## 5. 6 Hal Penting yang Wajib Diketahui (Gap Kode vs Foto)

Karena ini proyek dari pemilik sebelumnya yang masih di tengah pengembangan, perhatikan **6 ketidaksesuaian** berikut:

1. **Penamaan "Titik A & B" di Kode ≠ Stasiun `[A]` & `[B]` di Foto:**  
   * Di kode (`NavigationFSM.h`), "Titik A" berhenti tepat di garis **`N6`**, sedangkan "Titik B" berhenti `495 ms` melewati **`N10`** (posisi **`[C]`** di foto).
   * Rute ke stasiun **`[A]`**, **`[B]`**, **`[D]`**, dan **`[E]`** belum dibuat — alur FSM saat ini masih *hardcoded* untuk 1 rute saja.
2. **LoRa Belum Terpasang di Program Utama:**  
   * `mobil_code.ino` **belum meng-include `<LoRa.h>`**. Semua perintah sementara hanya lewat kabel USB Serial Monitor, dan saat menyala robot langsung jalan sendiri setelah 2 detik.
   * Komunikasi LoRa baru tersedia sebagai skrip uji terpisah di folder `test_lora/`.
3. **Encoder Roda Kiri Belum Dipakai untuk Jarak:**  
   * Nilai `encoderTicks` dihitung oleh interrupt di `Sensor.h`, tetapi belum dipakai di `NavigationFSM.h`. Jarak berhenti masih memakai timer `delay()`/`millis()`.
4. **Sensor Jarak HC-SR04 Sedang Dimatikan:**  
   * `GUNAKAN_SENSOR_JARAK = false` di `Config.h`, jadi mobil tidak akan berhenti jika ada halangan di depannya.
5. **Beda Permukaan Lantai (Kardus vs Spanduk):**  
   * `mobil_code.ino` di-tuning untuk **lantai kardus** (`KEC_BASE = 58`).
   * `test_lintasan_kotak.ino` adalah eksperimen terbaru untuk **lantai spanduk** (`KEC_BASE = 68`, strategi belok berbeda: berhenti → mundur 70 ms → belok → wajib lurus S3). Jika arena 3m×3m Anda memakai **spanduk**, nilai torsi di `Config.h` perlu disesuaikan lagi.
6. **Aturan Nama Folder di Arduino IDE:**  
   * Arduino IDE mewajibkan nama folder sama dengan nama `.ino`. Saat ini foldernya bernama `2026-yusuf-mobil-agv-main`, sedangkan sketsanya `mobil_code.ino`.
   * **Solusi:** Rename folder proyek menjadi `mobil_code`, **atau** buat subfolder `mobil_code/` dan salin `mobil_code.ino` beserta 5 file `.h` (`Config.h`, `Motor.h`, `Sensor.h`, `NodeDetector.h`, `NavigationFSM.h`) ke dalamnya.

---

## 6. Panduan Menjalankan & Menguji (Step-by-Step)

### Persiapan Arduino IDE

* Install **ESP32 Board Package** (pilih Board: **ESP32 Dev Module** atau paket board **Mappi32** by KMtek).
* Untuk uji LoRa, install library **`LoRa by Sandeep Mistry`** via *Library Manager*.
* Atur **Baudrate Serial Monitor**:
  * Semua program Mappi32 / ESP32: **`115200 baud`**
  * Semua program uji Arduino Uno / Nano: **`9600 baud`**

---

### Urutan Pengujian Aman dari Awal

#### Langkah 1 — Tes Sensor Garis (Baterai Motor OFF)
1. Upload `test_sensor_garis/test_sensor_garis.ino` ke Mappi32, buka Serial Monitor **`115200`**.
2. Taruh robot di lantai putih/spanduk → pastikan muncul `[ ] [ ] [ ] [ ] [ ]`.
3. Geser ke atas lakban hitam dari kiri ke kanan → pastikan menyala `[■]` berurutan dari `b1` ke `b5`.
4. Jika kurang sensitif, putar trimpot biru di modul TCRT5000. Gunakan perintah Serial `R` (balik kiri↔kanan) atau `P` (balik polaritas hitam↔putih) untuk verifikasi.

#### Langkah 2 — Tes Motor 4 Arah (Ganjal Roda Melayang!)
1. Ganjal sasis robot sehingga ke-4 roda tidak menyentuh lantai.
2. Upload `test_maju_mundur/test_maju_mundur.ino` (atau pakai perintah `TEST_RODA` di program utama).
3. Pastikan arah maju & mundur sisi kiri/kanan benar dan heatsink BTS7960 tidak panas saat diam.

#### Langkah 3 — Tes Lintasan Kotak di Spanduk *(Opsional)*
1. Upload `test_lintasan_kotak/test_lintasan_kotak.ino`, buka Serial Monitor **`115200`**.
2. Tombol kontrol Serial yang tersedia:
   * `Spasi`: Start / Stop
   * `+` / `-`: Naikkan / Turunkan `KEC_BASE`
   * `K` / `L`: Ubah belok Kanan / Kiri
   * `C`: Mode kalibrasi trimpot sensor (motor diam)
   * `T`: Uji manuver belok 90° di tempat
   * `R`: Reset hitungan sudut

#### Langkah 4 — Tes Radio LoRa *(Jika Pakai 2 Mappi32 & Antena Terpasang)*
1. Upload `test_lora/test_lora_sender/test_lora_sender.ino` ke Mappi32 Laptop (Controller).
2. Upload `test_lora/test_lora_receiver/test_lora_receiver.ino` ke Mappi32 Robot.
3. Buka kedua Serial Monitor di **`115200`**, pastikan muncul `PING` dan balasan `ACK`. Anda bisa mengetik `START`, `STOP`, `PULANG`, atau `PING` di Serial Sender.

#### Langkah 5 — Menjalankan Program Utama (`mobil_code.ino`)
1. Pastikan `mobil_code.ino` dan ke-5 file `.h` berada di dalam folder bernama `mobil_code`.
2. Upload `mobil_code.ino` ke Mappi32 Robot.
3. Letakkan robot di titik **`START`** menghadap lurus ke garis menuju **`N0`**.
4. **Perhatian:** Begitu dinyalakan/reset, robot otomatis mulai berjalan setelah **2 detik**!

#### Daftar Perintah Serial Monitor (`mobil_code.ino` — `115200 baud`, Newline)

| Perintah | Fungsi |
| :--- | :--- |
| `START` *(atau `START_AB` / `1`)* | Mulai ulang misi penuh dari State 1 (`START → A → B → START`) |
| `LANJUT` *(atau `GO` / `NEXT`)* | Langsung lanjut dari Titik A tanpa menunggu jeda 3 detik |
| `PULANG` *(atau `BALIK` / `RETURN`)* | Langsung eksekusi U-Turn 180° untuk rute pulang ke `START` |
| `STOP` *(atau `ESTOP` / `0`)* | **Rem Darurat (*E-Stop*)** — hentikan motor seketika |
| `STANDBY` *(atau `PAUSE`)* | Jeda robot sementara |
| `RESET` | Reset state navigasi kembali ke State 1 |
| `STATUS` | Tampilkan status misi & nomor state aktif |
| `TEST_RODA` | Tes putar 4 arah motor berurutan (`1,5 detik` per arah) |
| `TEST_KANAN` | Tes putar poros kanan selama `2 detik` |
| `TEST_MUNDUR_KANAN` | Tes khusus roda kanan mundur (`GPIO 17`) selama `2 detik` |
