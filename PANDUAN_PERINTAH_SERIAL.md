# 📘 Panduan Lengkap Perintah Serial Monitor & Remote LoRa AGV

Dokumentasi lengkap seluruh perintah untuk konfigurasi, kalibrasi, kontrol navigasi, penyimpanan Flash, dan pengujian perangkat keras mobil AGV (**Mappi32**) maupun Remote Controller (**ESP32 LoRa**).

> **Pengaturan Serial Monitor**: Baudrate **`115200`**, Line Ending **`Newline`** atau **`Both NL & CR`**. Huruf besar/kecil bebas (`start` = `START`).

---

## 📑 Daftar Isi (Akses Cepat)

- [⚡ Daftar Cepat Seluruh Perintah (Cheat Sheet)](#-daftar-cepat-seluruh-perintah)
  - [⚙️ Pengaturan & Kalibrasi](#1-pengaturan--kalibrasi-ringkas)
  - [🎮 Kontrol Jalan & Misi](#2-kontrol-jalan-ringkas)
  - [🗺️ Pilih & Ganti Rute](#3-pilih-rute-ringkas)
  - [💾 Tabel Mapping & Flash](#4-mapping--flash-ringkas)
  - [🔧 Tes Perangkat Keras](#5-tes-hardware-ringkas)
  - [📡 Khusus Remote LoRa (Sender)](#6-khusus-lora-ringkas)
- [🔀 Perbedaan Sender (LoRa) vs Receiver (USB Mappi32)](#-perbedaan-sender-vs-receiver)
- [📖 Panduan Detail, Contoh Penggunaan, & Output](#-panduan-detail-contoh--output)
  - [1. 🎮 Kontrol Jalan, Jeda, Pulang, & Rem Darurat](#detail-1-kontrol)
  - [2. 🗺️ Memilih & Mengganti Rute Stasiun (`A`–`E`)](#detail-2-rute)
  - [3. ⚙️ Kalibrasi Encoder & Pengaturan Runtime](#detail-3-kalibrasi)
  - [4. 💾 Lihat, Simpan, & Edit Tabel Mapping ke Flash](#detail-4-mapping)
  - [5. 🔧 Diagnostik Perangkat Keras (USB Mappi32)](#detail-5-diagnostik)
  - [6. 📡 Perintah Khusus Sender (Remote LoRa ESP32)](#detail-6-lora)

---

<a id="-daftar-cepat-seluruh-perintah"></a>
## ⚡ Daftar Cepat Seluruh Perintah (Cheat Sheet)

Gunakan daftar ringkas ini untuk langsung menyalin (*copy-paste*) perintah di Serial Monitor.

<a id="1-pengaturan--kalibrasi-ringkas"></a>
### ⚙️ Pengaturan & Kalibrasi

| Perintah (Siap Pakai) | Lokasi | Deskripsi | Catatan |
| :--- | :---: | :--- | :--- |
| `SET_JEDA 4000` | Sender & Receiver | Ubah jeda parkir stasiun menjadi 4 detik (`4000 ms`). | Lanjutkan `MAP_SAVE` agar permanen. |
| `SET_PPR 49.8` | Receiver / Sender | Setel manual nilai PPR encoder & simpan permanen ke NVS. | Rentang valid `10.0` – `5000.0`. |
| `SET_AUTOSTART 0` | Sender & Receiver | Saat nyala langsung diam dalam mode `STANDBY`. | Ketik `SET_AUTOSTART 1` untuk auto-jalan, lalu `MAP_SAVE`. |
| `KALIBRASI_50CM` | **Receiver USB** | Mulai proses kalibrasi dorong lurus sejauh `50.0 cm`. | Lanjutkan dorong 50 cm lalu `SIMPAN_50CM`. |
| `SIMPAN_50CM` | **Receiver USB** | Hitung & simpan otomatis hasil PPR setelah dorong 50 cm. | Tersimpan permanen ke NVS (`agv_cfg`). |

<a id="2-kontrol-jalan-ringkas"></a>
### 🎮 Kontrol Jalan (Sender & Receiver)

| Perintah | Alias | Lokasi | Deskripsi |
| :--- | :--- | :---: | :--- |
| `START` | `1`, `START_AB`, `START_AC` | Sender & Receiver | Mulai jalan dari titik **START** (`Step 0`). |
| `PAUSE` | `STANDBY` | Sender & Receiver | Berhenti **sementara** di tengah jalan atau stasiun. |
| `LANJUT` | `GO`, `NEXT` | Sender & Receiver | Lanjutkan perjalanan setelah `PAUSE` atau *skip* sisa jeda parkir stasiun. |
| `PULANG` | `BALIK`, `RETURN`, `CMD:HOME` | Sender & Receiver | Batalkan sisa tujuan & langsung pulang loop ke **START**. |
| `STOP` | `ESTOP`, `0`, `CMD:STOP` | Sender & Receiver | **Rem darurat (`E-STOP`)** — motor mati total seketika. |
| `RESET` | — | Sender & Receiver | Reset posisi step, arah, & encoder kembali ke awal (`Step 0`). |
| `STATUS` | — | Sender & Receiver | Cek status misi, step aktif, nilai PPR, & pembacaan encoder. |
| `HELP` | `?` | Sender & Receiver | Tampilkan daftar panduan perintah singkat. |

<a id="3-pilih-rute-ringkas"></a>
### 🗺️ Pilih & Ganti Rute (Sender & Receiver)

| Perintah (Siap Pakai) | Alur Rute | Lokasi | Perilaku |
| :--- | :--- | :---: | :--- |
| `RUTE 1` | `A → C → START` | Sender & Receiver | Simpan ke Flash, tunggu perintah `START`. |
| `RUTE 2` | `A → B → START` | Sender & Receiver | Simpan ke Flash, tunggu perintah `START`. |
| `RUTE 3` | `E → D → START` | Sender & Receiver | Simpan ke Flash, tunggu perintah `START`. |
| `RUTE 4` | `B → D → START` | Sender & Receiver | Simpan ke Flash, tunggu perintah `START`. |
| `RUTE A,B,C` | Rute PP bebas (`1–5` stasiun) | Sender & Receiver | Simpan ke Flash, tunggu perintah `START`. |
| `CMD:GO:A,B,C` | Rute PP bebas (`1–5` stasiun) | Sender & Receiver | **Langsung jalan seketika** tanpa menunggu `START`. |
| `RUTE_ONEWAY C` | Rute searah (`1–5` stasiun) | Sender & Receiver | Berhenti `STANDBY` di tujuan akhir (tunggu `START`). |

<a id="4-mapping--flash-ringkas"></a>
### 💾 Tabel Mapping & Memori Flash

| Perintah (Siap Pakai) | Lokasi Terbaik | Deskripsi |
| :--- | :---: | :--- |
| `MAP` | **Receiver USB** | Lihat tabel langkah rute aktif (tampil lengkap di Receiver USB). |
| `MAP_SAVE` | Sender & Receiver | Simpan permanen rute & pengaturan saat ini ke memori Flash. |
| `MAP_LOAD` | Sender & Receiver | Muat ulang rute terakhir dari Flash (batalkan editan di RAM). |
| `MAP_DEFAULT` | Sender & Receiver | Reset konfigurasi ke rute bawaan pabrik (`RUTE A,C`). |
| `MAP_CLEAR` | **Receiver USB** | Kosongkan semua baris pada tabel mapping (`0 / 40`). |
| `MAP_ADD ENC 50 4000 JEDA` | **Receiver USB** | Tambahkan 1 step baru di baris paling akhir. |
| `MAP_SET 2 ENC 45 4000 JEDA` | **Receiver USB** | Ubah parameter step pada baris nomor `2`. |
| `MAP_DEL 2` | **Receiver USB** | Hapus step pada baris nomor `2`. |

<a id="5-tes-hardware-ringkas"></a>
### 🔧 Tes Perangkat Keras *(Khusus Receiver USB Mappi32)*

| Perintah | Lokasi | Deskripsi |
| :--- | :---: | :--- |
| `TEST_ENCODER` | **Receiver USB** | Baca pulsa encoder secara *real-time* selama 5 detik (`20` sampel). |
| `RESET_ENC` | **Receiver USB** | Nolkan hitungan pulsa encoder roda kiri. |
| `TEST_RODA` | **Receiver USB** | Tes 4 arah motor bergantian *(angkat roda mobil dari lantai dulu!)*. |
| `TEST_KANAN` | **Receiver USB** | Tes putar poros kanan selama 2 detik. |
| `TEST_MUNDUR_KANAN` | **Receiver USB** | Tes roda kanan bergerak mundur (Pin 17) selama 2 detik. |

<a id="6-khusus-lora-ringkas"></a>
### 📡 Khusus Remote LoRa *(Hanya di Sender)*

| Perintah | Lokasi | Deskripsi |
| :--- | :---: | :--- |
| `PING` | **Khusus Sender** | Cek koneksi, status mobil, rute aktif, step, & sinyal (`RSSI` / `SNR`). |
| `AUTO` | **Khusus Sender** | Nyalakan/matikan fitur *Auto-PING* tiap 3 detik *(biarkan `OFF` saat mobil jalan)*. |

---

<a id="-perbedaan-sender-vs-receiver"></a>
## 🔀 Perbedaan Sender (Remote LoRa) vs Receiver (USB Mappi32)

| Aspek | **Sender** (Remote LoRa ESP32 — `test_lora_esp32_sender.ino`) | **Receiver** (Mobil AGV Mappi32 — `mobil_code.ino`) |
|---|---|---|
| **Cara koneksi** | Nirkabel lewat LoRa `915 MHz` ke mobil | Kabel USB langsung ke board Mappi32 |
| **Yang tampil di Serial Monitor setelah ketik perintah** | Hanya konfirmasi singkat **`ACK`** + status misi + posisi step + kekuatan sinyal (`RSSI` & `SNR`). | **Log lengkap & detail**: tabel `MAP`, status encoder/PPR, pesan error/info, serta telemetri `PLAN` real-time. |
| **Cocok dipakai untuk** | Kontrol jalan/jeda/pulang/stop, ganti rute, `CMD:GO:...`, `MAP_SAVE`, `SET_JEDA`, `PING`, `AUTO`. | **Semua perintah** (Bagian 1–5), terutama yang butuh tampilan tabel (`MAP`, `STATUS`), kalibrasi (`KALIBRASI_50CM`), dan tes roda/encoder (`TEST_*`). |

> **Aturan Penting**:
> - Jika kamu mengetik perintah (misal `map`, `status`, atau `RUTE 1`) lewat **Sender**, mobil **tetap menjalankan perintah tersebut**, tetapi tabel/detail lengkapnya tercetak di **Serial Monitor Receiver (USB Mappi32)** — di **Sender** hanya tampil balasan `ACK`.
> - Jangan jalankan `TEST_ENCODER`, `TEST_RODA`, `TEST_KANAN`, `TEST_MUNDUR_KANAN`, atau `KALIBRASI_50CM` lewat **Sender** (pakailah **Receiver USB**), karena tes tersebut memblokir sistem beberapa detik sehingga di Sender akan muncul `[GAGAL] Tidak ada ACK`.

---

<a id="-panduan-detail-contoh--output"></a>
## 📖 Panduan Detail, Contoh Penggunaan, & Output

<a id="detail-1-kontrol"></a>
### 1. 🎮 Kontrol Jalan, Jeda, Pulang, & Rem Darurat
*(Bisa dijalankan di **Sender** & **Receiver**)*

#### Contoh 1: Ketik `START`
- **Output di Sender (Remote LoRa)**:
  ```text
  >>> [TX LORA] "#1|START" ...
  <<< [RX ACK] "ACK:#1|START|JALAN|STEP:0/12" | RSSI: -42 dBm | SNR: 10.00 dB
  ```
- **Output di Receiver (USB Mappi32)**:
  ```text
  [LORA RX] "#1|START" | RSSI: -42 dBm | SNR: 10.00 dB
  [COMMAND DITERIMA] -> START

  >>> [PLAN STEP 0/12 : N0_BELOK_KIRI | FOLLOW] Alasan: Perintah Operator START
  ```

#### Contoh 2: Ketik `PAUSE`, lalu `LANJUT` (atau `PULANG`)
- **Ketik `PAUSE` di Sender**:
  ```text
  >>> [TX LORA] "#2|PAUSE" ...
  <<< [RX ACK] "ACK:#2|PAUSE|PAUSE|STEP:2/12" | RSSI: -45 dBm | SNR: 9.75 dB
  ```
  *(Arti `|PAUSE|STEP:2/12`: mobil sedang jeda sementara di Step 2).*
- **Output `PAUSE` di Receiver**:
  ```text
  [COMMAND DITERIMA] -> PAUSE
  >>> [PAUSE/STANDBY] Robot berhenti sementara di Step 2. Ketik 'LANJUT' untuk meneruskan atau 'PULANG' untuk kembali ke START.
  ```
- **Jika lalu ketik `LANJUT`**:
  - **Sender**: `<<< [RX ACK] "ACK:#3|LANJUT|JALAN|STEP:2/12" | RSSI: -44 dBm | SNR: 10.00 dB`
  - **Receiver**: `>>> [LANJUT] Melanjutkan rute pada Step 2 (Fase FOLLOW).` *(atau langsung melanjutkan bila sedang jeda parkir 3 detik di stasiun)*.
- **Jika berhenti di Stasiun A pada `RUTE A,B`, lalu ketik `PULANG`**:
  - **Sender**: `<<< [RX ACK] "ACK:#3|PULANG|JALAN|STEP:0/4" | RSSI: -44 dBm | SNR: 10.00 dB` *(rute berubah menjadi rute pulang 5 step: `0/4`)*.
  - **Receiver**: menyusun & mencetak tabel baru `PULANG_DARI_A` (`N6_KANAN → N7_KANAN → N3_KANAN → N0_KIRI → FINISH_START`) lalu langsung berjalan pulang ke `START`.

#### Contoh 3: Ketik `STATUS`
- **Output di Sender**:
  ```text
  >>> [TX LORA] "#4|STATUS" ...
  <<< [RX ACK] "ACK:#4|STATUS|PAUSE|STEP:2/12" | RSSI: -43 dBm | SNR: 9.75 dB
  ```
- **Output di Receiver**:
  ```text
  --- [STATUS AGV DIAGNOSTIK] ---
  Rute Aktif: RUTE_A_C_LOOP
  Misi: PAUSE (Menunggu LANJUT / PULANG)
  Step: 2/12 (CHECKPOINT_A) | Fase: FOLLOW
  Kalibrasi PPR: 78.8 (Tersimpan di NVS)
  Encoder Total Q:815 A:815 B:812 | Fase Efektif: 312 (dQ:312 dA:312 dB:310) -> 25.7 cm / Target: 50.0 cm
  --------------------------------
  ```

---

<a id="detail-2-rute"></a>
### 2. 🗺️ Memilih & Mengganti Rute Stasiun (`A`, `B`, `C`, `D`, `E`)
*(Bisa dijalankan di **Sender** & **Receiver**)*

> **Batas Tujuan**: Maksimal **`5` stasiun** per misi (contoh `CMD:GO:A,C,D,E,B`) dan total maksimal **`40` langkah simpang** (`MAX_PLAN_STEPS = 40`). Jika melebihi batas, di Receiver muncul `[ERROR RUTE] Maksimal 5 stasiun per misi.` atau `melebihi kapasitas MAX_PLAN_STEPS`.

#### Contoh Output `RUTE 1` *(Simpan Flash, tunggu `START`)*
- **Di Sender**:
  ```text
  >>> [TX LORA] "#2|RUTE 1" ...
  <<< [RX ACK] "ACK:#2|RUTE 1|STANDBY|STEP:0/12" | RSSI: -42 dBm | SNR: 10.00 dB
  ```
- **Di Receiver**:
  ```text
  [COMMAND DITERIMA] -> RUTE 1
  >>> [FLASH NVS] Mapping 'RUTE_A_C_LOOP' (13 step) berhasil DISIMPAN permanen ke Flash ESP32!
  ... (tabel mapping 13 step tampil) ...
  >>> [SUKSES TERSIMPAN] RUTE A ke C aktif & tersimpan permanen di Flash! Kabel USB sudah boleh dicabut.
  ```

#### Contoh Output `CMD:GO:A,B,C` *(Susun rute & langsung jalan seketika)*
- **Di Sender**:
  ```text
  >>> [TX LORA] "#6|CMD:GO:A,B,C" ...
  <<< [RX ACK] "ACK:#6|CMD:GO:A,B,C|JALAN|STEP:0/11" | RSSI: -45 dBm | SNR: 9.50 dB
  ```
- **Di Receiver**: mencetak tabel rute `RUTE_A_B_C_LOOP`, lalu langsung masuk `PHASE_FOLLOW` Step 0 dan mobil berangkat.

---

<a id="detail-3-kalibrasi"></a>
### 3. ⚙️ Kalibrasi Encoder & Pengaturan Runtime (Tanpa Upload Ulang)

#### Contoh Output `KALIBRASI_50CM` → `SIMPAN_50CM` *(Di Receiver USB)*
```text
[COMMAND DITERIMA] -> KALIBRASI_50CM
>>> [MODE KALIBRASI JARAK 50 CM]
1. Pastikan posisi start ditandai dengan penggaris/lakban.
2. Dorong robot LURUS ke depan sejauh tepat 50.0 cm.
3. Setelah sampai 50 cm, ketik 'SIMPAN_50CM' di Serial Monitor.

... (dorong robot lurus tepat 50,0 cm, lalu ketik SIMPAN_50CM) ...

[COMMAND DITERIMA] -> SIMPAN_50CM
>>> [KALIBRASI 50CM] Pulsa terbaca: 193 | PPR terhitung: 78.8
>>> [NVS TERSIMPAN] ENCODER_TICKS_PER_PUTARAN = 78.8
```

> **Catatan**: Nilai PPR yang disimpan lewat `SIMPAN_50CM` atau `SET_PPR` berada di partisi NVS `agv_cfg/enc_ppr` sehingga tetap aman dan tidak tertimpa ketika memuat mapping rute.

---

<a id="detail-4-mapping"></a>
### 4. 💾 Lihat, Simpan, & Edit Tabel Mapping ke Flash

Secara sederhana, rute di dalam mobil tersimpan di **2 tempat**:
1. **RAM (Sementara)**: Rute yang sedang aktif dipakai sekarang. Perubahan dari `SET_JEDA`, `SET_AUTOSTART`, `RUTE_ONEWAY`, maupun `MAP_ADD/SET/DEL` **masih di RAM** dan hilang jika mobil dimatikan.
2. **Flash ESP32 / NVS (Permanen)**: Tetap diingat walau USB/baterai dicabut.

#### 4A. Melihat & Mengelola Flash (`MAP`, `MAP_SAVE`, `MAP_LOAD`, `MAP_DEFAULT`)

| Perintah | Output di **Sender** | Output di **Receiver (USB Mappi32)** |
|---|---|---|
| `MAP` | `ACK:#seq\|MAP\|STANDBY\|STEP:0/12` *(tabel tidak dikirim ke Sender)* | Mencetak tabel `TABEL MAPPING RUTE AKTIF` lengkap baris per baris (`Idx 0..12`). |
| `MAP_SAVE` | `ACK:#seq\|MAP_SAVE\|STANDBY\|STEP:0/12` | `>>> [FLASH NVS] Mapping '...' (13 step) berhasil DISIMPAN permanen ke Flash ESP32!` |
| `MAP_LOAD` | `ACK:#seq\|MAP_LOAD\|STANDBY\|STEP:0/12` | `>>> [FLASH NVS] Berhasil memuat mapping '...'` + mencetak ulang tabel. |
| `MAP_DEFAULT` | `ACK:#seq\|MAP_DEFAULT\|STANDBY\|STEP:0/12` | `>>> [FLASH NVS] Mapping custom dihapus. Kembali ke Rute Default (A -> C PP).` + tabel. |

**Contoh output `map` di Sender vs Receiver**:
- **Di Sender**:
  ```text
  >>> [TX LORA] "#3|map" ...
  <<< [RX ACK] "ACK:#3|MAP|STANDBY|STEP:0/12" | RSSI: -44 dBm | SNR: 10.25 dB
  ```
- **Di Receiver (USB Mappi32)**:
  ```text
  ================ [TABEL MAPPING RUTE AKTIF] ================
  Nama Rute   : RUTE_A_C_LOOP
  Jumlah Step : 13 / 40
  Kalibrasi   : PPR=78.8 | Jeda=3000ms | Pulang=AUTO | AutoStart=OFF
  ------------------------------------------------------------
  Idx | Label               | Trig     | JarakCm | PauseMs | Aksi
  ----+---------------------+----------+---------+---------+-------
   0  | N0_BELOK_KIRI       | TKIRI    | -       | 80      | KIRI
   1  | N2_BELOK_KANAN      | CKANAN   | -       | 80      | KANAN
   2  | CHECKPOINT_A        | ENC      | 50.0    | 3000    | JEDA
   3  | LEWAT_N6            | S4       | -       | 0       | LURUS
   4  | LEWAT_N10           | S4       | -       | 0       | LURUS
   5  | CHECKPOINT_C        | ENC      | 40.0    | 3000    | JEDA
  ...
  12  | FINISH_START        | S4       | -       | 50      | STOP
  ============================================================
  ```
  - **`Idx`**: Nomor urut baris (`0`, `1`, `2`, ...) untuk target `MAP_SET` / `MAP_DEL`.
  - **`Trig`**: Pemicu (`TKIRI`, `CKANAN`, `CKIRI`, `S4`, `ENC`, `LANGSUNG`).
  - **`JarakCm`**: Target jarak cm bila `Trig = ENC` (selain itu `-`).
  - **`PauseMs`**: Lama diam dalam ms (`3000` = 3 detik, `80` = jeda sebelum belok, `0` = melintas terus).
  - **`Aksi`**: Gerakan (`KIRI`, `KIRI_P`, `KANAN`, `LURUS`, `JEDA`, `UTURN`, `STOP`).

#### 4B. Edit Langkah Manual (`MAP_CLEAR`, `MAP_ADD`, `MAP_SET`, `MAP_DEL`)
*(Disarankan di **Receiver USB** agar tabel & nomor `Idx` terlihat langsung)*

> **Penting**: Perintah edit manual wajib diisi format lengkap (misal `MAP_ADD` butuh **4 parameter** `<TRIG> <CM> <PAUSE_MS> <AKSI>`). Jika hanya mengetik `MAP_ADD 4000 JEDA` akan muncul `[ERROR] Format: MAP_ADD <TRIG> <CM> <PAUSE_MS> <AKSI>`.

- **Daftar `<TRIG>`**: `TKIRI`, `CKANAN`, `CKIRI`, `S4` *(perempatan)*, `ENC` *(jarak encoder cm)*, `LANGSUNG` *(langsung eksekusi)*.
- **Nilai `<CM>`**: Isi angka jarak cm kalau `<TRIG>` = `ENC`, selain itu isi `0`.
- **Daftar `<AKSI>`**: `KIRI`, `KIRI_P` *(kiri pulang)*, `KANAN`, `LURUS`, `JEDA`, `UTURN`, `STOP`.

**Contoh A — Koreksi jarak Stasiun A (`Idx 2`) dari `50 cm` ke `45 cm` & jeda `4000 ms`:**
```text
MAP
MAP_SET 2 ENC 45 4000 JEDA
MAP_SAVE
```
Output di **Receiver**:
```text
>>> [MAP_SET] Step 2 diperbarui.
================ [TABEL MAPPING RUTE AKTIF] ================
Nama Rute   : MANUAL_CUSTOM
...
 2  | CHECKPOINT_A        | ENC      | 45.0    | 4000    | JEDA
...
>>> [FLASH NVS] Mapping 'MANUAL_CUSTOM' (13 step) berhasil DISIMPAN permanen ke Flash ESP32!
```

**Contoh B — Ubah lama jeda di SEMUA stasiun sekaligus jadi `4000 ms`:**
```text
SET_JEDA 4000
MAP_SAVE
```

**Contoh C — Buat rute pendek dari nol lalu hapus baris yang salah:**
```text
MAP_CLEAR
MAP_ADD TKIRI 0 80 KIRI
MAP_ADD CKANAN 0 80 KANAN
MAP_ADD ENC 40 0 STOP
MAP_DEL 1
```
Output di **Receiver**:
```text
>>> [MAP_CLEAR] Tabel mapping dikosongkan. Gunakan MAP_ADD untuk menambah langkah.
>>> [MAP_ADD] Step 0 ditambahkan.
>>> [MAP_ADD] Step 1 ditambahkan.
>>> [MAP_ADD] Step 2 ditambahkan.
>>> [MAP_DEL] Step 1 dihapus.
================ [TABEL MAPPING RUTE AKTIF] ================
Nama Rute   : MANUAL_CUSTOM
Jumlah Step : 2 / 40
------------------------------------------------------------
Idx | Label               | Trig     | JarakCm | PauseMs | Aksi
----+---------------------+----------+---------+---------+-------
 0  | MANUAL_0            | TKIRI    | -       | 80      | KIRI
 1  | MANUAL_2            | ENC      | 40.0    | 0       | STOP
============================================================
```
*(Jika salah edit dan belum `MAP_SAVE`, ketik `MAP_LOAD` untuk batal. Ketik `MAP_DEFAULT` untuk reset ke `RUTE A,C`.)*

---

<a id="detail-5-diagnostik"></a>
### 5. 🔧 Diagnostik Perangkat Keras *(Khusus Receiver USB Mappi32)*

| Perintah | Output & Perilaku di Receiver USB |
|---|---|
| `TEST_ENCODER` | Mencetak `PinA`, `PinB`, `Quad`, `A`, `B`, dan `cm` setiap 250 ms selama 5 detik (`20` baris). |
| `RESET_ENC` | `>>> [RESET_ENC] Pulsa encoder roda kiri di-nolkan.` |
| `TEST_RODA` | Menjalankan 4 arah motor berurutan masing-masing 1,5 detik (Pin 25 → 26 → 16 → 17). **Angkat mobil dulu.** |
| `TEST_KANAN` | Memutar poros kanan selama 2 detik (kiri maju, kanan mundur). |
| `TEST_MUNDUR_KANAN` | Menggerakkan roda kanan mundur (Pin 17) selama 2 detik. |

---

<a id="detail-6-lora"></a>
### 6. 📡 Perintah Khusus Sender (Remote LoRa ESP32)

| Perintah | Output di **Sender** | Output di **Receiver** |
|---|---|---|
| `PING` | `>>> [TX LORA] "#1\|PING" ...`<br>`<<< [RX ACK] "ACK:#1\|PONG\|STANDBY\|RUTE_A_C_LOOP\|STEP:0/12" \| RSSI: -42 dBm \| SNR: 10.00 dB` | `[LORA RX] "#1\|PING" \| RSSI: -42 dBm \| SNR: 10.00 dB` |
| `AUTO` | `[INFO] Auto-PING sekarang: ON` *(ketik `AUTO` lagi untuk `OFF`)* | Tidak ada pesan saat toggle; Receiver menerima `PING` tiap 3 detik bila `ON`. |
| *(Gagal terkirim)* | `[GAGAL] Tidak ada ACK dari Mappi32 setelah 3x percobaan (cek jarak/daya/antena).` | — |
