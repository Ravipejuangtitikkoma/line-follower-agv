# IMPLEMENTATION PLAN: UPGRADE NAVIGASI AGV SESUAI PETA RIIL
## Rute: START ➡️ Simpang 3 (Belok Kiri) ➡️ Simpang 3 Kanan ➡️ Titik A ➡️ Simpang 4 (Lurus) ➡️ Simpang 4 (Titik B)
**Lokasi File:** `plan.md` (Root Directory)  
**Tujuan Akhir:** Robot AGV otonom menyusuri grid peta riil gudang dengan manuver belok presisi, menggelir bodi belakang (panjang 25 cm, 4 roda lebar), berhenti di Titik A, melintasi Simpang 4 pertama lurus, dan berhenti di Titik B.

---

## 1. Ringkasan Eksekutif & Kebutuhan Lintasan Baru

Berdasarkan pembaruan peta riil lapangan, AGV harus menempuh sekuens jalur dengan karakteristik fisik dan topologi simpang sebagai berikut:

```
[START]
   │  (Maju Lurus PD Tracking)
   ▼
[SIMPANG 3 KIRI] ──► Deteksi Gelap Semua [1 1 1 1 1] ──► Belok Kiri 90° ──► Menggelir Bodi Belakang
   │
   ▼  (Maju Lurus Menyusuri Garis)
[SIMPANG 3 KANAN] ──► Sensor Kanan Deteksi Cabang [0 0 1 1 1] ──► Belok Kanan hingga [1 1 1 1 1]
   │                 ──► Luruskan Belakang Sebentar ──► Maju Sedikit
   ▼
[TITIK A] ──► Berhenti Sempurna (Jeda Bongkar/Muat / Ready Sinyal)
   │
   ▼  (Maju Lurus)
[SIMPANG 4 KE-1] ──► Deteksi Gelap Semua [1 1 1 1 1] ──► Jalan Terus Lurus (Cross & Debounce)
   │
   ▼  (Maju Lurus)
[SIMPANG 4 KE-2] ──► Deteksi Gelap Semua [1 1 1 1 1] ──► Melintas Lurus & Berhenti Total
   │
   ▼
[TITIK B] ──► Parkir & Misi Selesai!
```

---

## 2. Arsitektur State Machine yang Ringan & Modular

Agar navigasi ini **ringan diproses oleh ESP32**, **mudah dipahami dan didebug oleh manusia**, serta **mudah diuji dan dipelihara oleh AI Agent**, kita membaginya ke dalam **Finite State Machine yang Deterministik**:

Setiap state memiliki:
1. **Satu Aksi Tunggal (*Single Action*)**: Maju PD, Belok Poros, Pelurusan Bodi (*Menggelir*), atau Berhenti.
2. **Kondisi Keluar yang Jelas (*Deterministic Exit Trigger*)**: Berdasarkan pola bit sensor garis atau timer mikro.
3. **Timeout / Failsafe Proteksi**: Mencegah robot terjebak (*deadlock*) jika garis pudar atau selip.

---

### Tabel Definisi & Transisi State (11 State Terstruktur)

| ID State | Nama State | Aksi Motor | Pola Sensor / Kondisi Masuk | Kondisi Keluar (Transisi) | State Berikutnya |
| :---: | :--- | :--- | :--- | :--- | :--- |
| **0** | `STATE_STANDBY` | Berhenti (`0, 0`) | Menunggu perintah operator | Ketik `START` (Serial/LoRa) | `STATE_1_START_MAJU_S1` |
| **1** | `STATE_1_START_MAJU_S1` | Maju Lurus PD (`KEC_BASE=45`) | Bergerak dari Start | Gelap semua `[1 1 1 1 1]` (totalAktif $\ge 4$) | `STATE_2_BELOK_KIRI_SWEEP` |
| **2** | `STATE_2_BELOK_KIRI_SWEEP` | Putar Poros Kiri (`-PWM, +PWM`) | Simpang 3 kiri terdeteksi | Garis baru mulai tertangkap (`b3==1 && b4==1`) | `STATE_3_LURUSKAN_BELAKANG_KIRI` |
| **3** | `STATE_3_LURUSKAN_BELAKANG_KIRI` | Putar Kiri Pelan / Menggelir (`-110, +110`) | Menarik bodi 25cm & ban lebar | Sensor `b3==1` stabil DAN timer menggelir selesai ($\sim 180\text{ ms}$) | `STATE_4_MAJU_MENUJU_SIMPANG_KANAN` |
| **4** | `STATE_4_MAJU_MENUJU_SIMPANG_KANAN` | Maju Lurus PD (`KEC_BASE=45`) | Meluncur di ruas kedua | Sensor kanan mendeteksi cabang (`b5==1 && b4==1`) | `STATE_5_BELOK_KANAN_SIMPANG_3` |
| **5** | `STATE_5_BELOK_KANAN_SIMPANG_3` | Putar Poros Kanan (`+PWM, -PWM`) | Cabang kanan terdeteksi | Sensor membaca gelap penuh / garis cabang (`[1 1 1 1 1]` / `b3==1 && b2==1`) | `STATE_6_LURUSKAN_BELAKANG_KANAN` |
| **6** | `STATE_6_LURUSKAN_BELAKANG_KANAN` | Putar Kanan Pelan / Menggelir (`+110, -110`) | Bodi depan sudah sejajar | Timer menggelir bodi belakang selesai ($\sim 120\text{ ms}$) | `STATE_7_MAJU_PENETRASI_TITIK_A` |
| **7** | `STATE_7_MAJU_PENETRASI_TITIK_A` | Maju Lurus Pelan (`KEC_BASE=40`) | Bodi sudah lurus 90° | Timer penetrasi selesai ($\sim 300\text{ ms}$) | `STATE_8_STOP_TITIK_A` |
| **8** | `STATE_8_STOP_TITIK_A` | Berhenti Total (`0, 0`) | Tiba di Titik A | Timer jeda bongkar/muat (3.0 detik) atau perintah `LANJUT` | `STATE_9_MAJU_LEWATI_SIMPANG4_1` |
| **9** | `STATE_9_MAJU_LEWATI_SIMPANG4_1` | Maju Lurus PD (`KEC_BASE=45`) | Lepas dari Titik A | Mendeteksi Simpang 4 pertama `[1 1 1 1 1]` $\rightarrow$ Latch lewati tanpa belok | `STATE_10_MAJU_MENUJU_SIMPANG4_2` |
| **10** | `STATE_10_MAJU_MENUJU_SIMPANG4_2` | Maju Lurus PD (`KEC_BASE=45`) | Di ruas antara simpang 4 ke-1 & ke-2 | Mendeteksi Simpang 4 kedua `[1 1 1 1 1]` | `STATE_11_MAJU_LURUS_STOP_TITIK_B` |
| **11** | `STATE_11_MAJU_LURUS_STOP_TITIK_B` | Maju sedikit (150ms) lalu Rem Aktif | Tiba di garis Titik B | Mobil berhenti total di Titik B (Misi Sukses) | `STATE_SELESAI` |

---

## 3. Penanganan Dinamika Fisik: "Menggelir" Bodi Belakang

Faktor mekanik yang disebutkan:
- **Panjang bodi:** $L = 25\text{ cm}$
- **Lebar & 4 roda lebar:** Torsi gesek putar (*skid-steer turning resistance*) tinggi.
- **Bobot mobil:** $m \approx 1.45\text{ kg}$.

### Mengapa Tahap "Menggelir" (*Rear Straightening*) Sangat Penting?
Saat roda depan dan sensor sudah mencapai garis target (90° rotasi sensor depan), roda belakang belum tentu sejajar karena titik pivot diferensial berada di as penggerak. Jika mobil langsung melaju maju:
* Bodi belakang akan "menggeleng/menyerong" (diagonal).
* Hal ini dapat menyebabkan sensor terlempar keluar dari garis (*line loss*) atau berosilasi liar.

### Solusi Algoritma Menggelir:
1. **Tahap 1 (Sweep Depan):** Memutar poros cepat dengan torsi terukur (`PWM 138`) hingga sensor tengah menyentuh garis target.
2. **Tahap 2 (Menggelir Bodi Belakang):** Memberikan impuls torsi terkalibrasi (`PWM 110 - 120`) selama durasi terukur ($\Delta t = 120 - 180\text{ ms}$) untuk menarik sumbu belakang agar sejajar dengan lintasan.
3. **Tahap 3 (Active Reverse Pulse Brake):** Memberikan sinyal rem impuls singkat ($15 - 25\text{ ms}$) arah berlawanan untuk mematikan momentum putar sebelum roda berpindah ke mode maju lurus PD.

---

## 4. Rincian Teknis Implementasi per File

### A. `Config.h`
* Tambahkan definisi konstanta baru:
  - `DURASI_MENGGELIR_KIRI_MS = 180`
  - `DURASI_MENGGELIR_KANAN_MS = 120`
  - `DURASI_PENETRASI_TITIK_A_MS = 350`
  - `DURASI_JEDA_TITIK_A_MS = 3000` (Jeda di Titik A)
  - Ambang deteksi simpang gelap semua: `AMBANG_SIMPANG_PENUH = 4` (minimal 4 dari 5 sensor membaca gelap / `[1 1 1 1 1]`).

### B. `NavigationFSM.h`
* Perbarui `enum NavState` menjadi 11 state yang bersih.
* Modifikasi fungsi `gantiState()` dengan nama log yang deskriptif untuk setiap state.
* Implementasikan penanganan simpang 4 beruntun:
  - *Debounce Latch:* Menggunakan `waktuLewatSimpang` agar Simpang 4 pertama yang dilewati tidak memicu deteksi ulang ganda pada simpang yang sama.
* Buat telemetri yang ringkas dan informatif:
  `FSM: [9:LEWATI_S4_1] | Line: [1 1 1 1 1] | Dist: 180cm | Status: TRACKING`

### C. `mobil_code.ino`
* Menghubungkan Serial Command baru:
  - `START` / `START_AB`: Mulai misi penuh Start ➡️ A ➡️ B.
  - `LANJUT`: Melanjutkan dari Titik A ke Titik B secara manual (jika mode manual dipilih).

---

## 5. Rencana Tahapan Eksekusi & Validasi Lapangan

Kita akan menerapkan perubahan ini secara bertahap dan terukur:

```
[Langkah 1: Setup Enum & Parameter Config]
       │
       ▼
[Langkah 2: Implementasi State 1 s/d State 4 (Start -> Belok Kiri Simpang 3 & Menggelir)]
       │  └── Verifikasi: Robot belok kiri siku sempurna dan bodi belakang lurus rapi.
       ▼
[Langkah 3: Implementasi State 5 s/d State 8 (Simpang 3 Kanan -> Berhenti di Titik A)]
       │  └── Verifikasi: Robot mengunci cabang kanan, menggelir bodi, maju sedikit, dan berhenti di Titik A.
       ▼
[Langkah 4: Implementasi State 9 s/d State 11 (Titik A -> Simpang 4 Pertama -> Simpang 4 Kedua Titik B)]
       │  └── Verifikasi: Robot melintasi Simpang 4 ke-1 tanpa belok, lalu melaju hingga berhenti di Titik B.
       ▼
[Langkah 5: Uji Misi Penuh & Sinkronisasi LoRa/Serial]
```

---

## 6. Upgrade Torsi Maju & Belok serta Solusi Roda Mematung

### A. Diagnosa Fisika Roda Mematung:
* Motor kiri depan & belakang diparalel pada 1 driver.
* Gandar depan menopang 40% beban normal ($N_f$), gandar belakang menopang 60% ($N_r$).
* Saat belok di atas kardus ($\mu_s \approx 0{,}65$), roda depan kiri berputar lebih dulu (*free wheel slip*). Hal ini menyebabkan *current starvation* pada 3 roda lainnya sehingga mematung.

### B. Tindakan Solusi Rekayasa:
1. **Peningkatan Torsi Maju:** `KEC_BASE` dinaikkan dari `45` ke `58` PWM (+30% torsi dorong), `TRIM_KIRI = +22`, `TRIM_KANAN = -6`.
2. **Breakaway Torque Kick:** Sentakan instan PWM 185 selama 25 ms saat memulai belokan agar ke-4 roda memecahkan gesekan statis serempak.
3. **Peningkatan Torsi Belok:** `KEC_BELOK_*` dinaikkan dari `138` ke `160` PWM, dan `KEC_SESUAI_*` dinaikkan dari `120` ke `135` PWM.
4. **Penyesuaian Durasi Menggelir:** `DURASI_MENGGELIR_KIRI_MS = 150` dan `DURASI_MENGGELIR_KANAN_MS = 100` agar belokan 90° tetap presisi.

