# Panduan Uji & Kalibrasi Sensor Garis 5-Channel (TCRT5000) Menggunakan ESP32

Dokumen ini memuat panduan lengkap pengujian sensor garis 5-channel baru menggunakan **ESP32 (Mappi32 / NodeMCU ESP32)** untuk memeriksa apakah masing-masing dari ke-5 sensor berfungsi normal, sensitivitasnya pas di atas spanduk, atau ada pin yang mati/stuck.

---

## 1. Skema Pengkabelan (Wiring) ke ESP32

Sesuai dengan konfigurasi resmi pin proyek AGV kita (`Config.h`):

| Pin Modul Sensor | Hubungkan ke Pin ESP32 | Tipe Pin GPIO ESP32 | Keterangan |
| :---: | :---: | :---: | :--- |
| **VCC** | **3.3V atau 5V** | Power | Tegangan modul sensor |
| **GND** | **GND** | Ground | Ground bersama (*common ground*) |
| **OUT1** | **GPIO 34** | GPI (Input-only) | Sensor 1 (Kanan luar fisik jika reversed) |
| **OUT2** | **GPIO 35** | GPI (Input-only) | Sensor 2 |
| **OUT3** | **GPIO 36 (VP)** | GPI (Input-only) | Sensor 3 (Tengah) |
| **OUT4** | **GPIO 39 (VN)** | GPI (Input-only) | Sensor 4 |
| **OUT5** | **GPIO 23** | GPIO Standar | Sensor 5 (Kiri luar fisik jika reversed) |

> **Catatan Penting GPIO ESP32:**  
> Pin GPIO 34, 35, 36, dan 39 pada ESP32 adalah pin *Input Only* (GPI) tanpa internal pull-up hardware. Modul TCRT5000 sudah memiliki resistor komparator sendiri, sehingga cocok dihubungkan langsung ke pin-pin ini.

---

## 2. Cara Menjalankan Uji Coba di ESP32

1. Buka berkas [`test_sensor_garis.ino`](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_sensor_garis/test_sensor_garis.ino) di Arduino IDE.
2. Pada menu **Tools**:
   - Pilih Board: **ESP32 Dev Module** (atau board Mappi32 Anda).
   - Pilih Port: Sesuai port USB ESP32 Anda.
3. Klik tombol **Upload**.
4. Buka **Serial Monitor** pada kecepatan **`115200 baud`**.
5. Letakkan sensor di atas lintasan spanduk dan geser melintasi garis lakban hitam.

---

## 3. Tampilan Hasil di Serial Monitor (115200 Baud)

Serial Monitor akan menampilkan status tegangan mentah (RAW GPIO) dan visualisasi grafis secara *real-time*:

```text
GPIO [34:1 35:0 36:0 39:0 23:1] | FISIK: | [ ] [■] [■] [■] [ ] | Tot:3 | Err:+0.0 | --> LURUS SEMPURNA [0 1 1 1 0] (Pas Tengah)
GPIO [34:0 35:1 36:1 39:1 23:1] | FISIK: | [ ] [ ] [ ] [ ] [■] | Tot:1 | Err:+2.0 | --> SIKU / BELOK KANAN (Garis di Kanan Luar)
GPIO [34:1 35:1 36:1 39:1 23:1] | FISIK: | [ ] [ ] [ ] [ ] [ ] | Tot:0 | Err:+0.0 | --> LEPAS GARIS (Semua Putih)
GPIO [34:0 35:0 36:0 39:0 23:0] | FISIK: | [■] [■] [■] [■] [■] | Tot:5 | Err:+0.0 | --> PERSIMPANGAN / SIKU (Semua Hitam)
```

> **Simbol:**
> * `[■]` = Sensor mendeteksi **GARIS HITAM** (Aktif).
> * `[ ]` = Sensor mendeteksi **LANTAI SPANDUK** (Putih / Terang).

---

## 4. Tombol Kontrol Serial Monitor (Interaktif)

Anda dapat mengetik huruf berikut di Serial Monitor lalu tekan Enter / Send:

- **`R`** : Membalik urutan sensor Kiri $\leftrightarrow$ Kanan (*Reverse Sensor Order*).
- **`P`** : Mengubah polaritas logika (*Active LOW $\leftrightarrow$ Active HIGH*).
- **`A`** : Menampilkan / menyembunyikan nilai analog ADC (Pin 34, 35, 36, 39).
- **`H`** : Menampilkan menu bantuan (*Help*).

---

## 5. Cara Kalibrasi Sensor Baru di Lintasan Spanduk

1. Letakkan modul sensor di atas **lantai spanduk putih** pada jarak kerja robot ($\sim 6 - 8\text{ mm}$ dari permukaan):
   - Perhatikan Serial Monitor: kelima bar harus menampilkan kosong: `| [ ] [ ] [ ] [ ] [ ] |`.
   - Jika ada sensor yang menyala `[■]`, putar trimpot/potensiometer biru kecil pada modul sensor tersebut perlahan menggunakan obeng minus kecil hingga LED indikatornya padam dan di Serial Monitor menjadi `[ ]`.
2. Geser sensor ke atas **lakban hitam**:
   - Sensor yang tepat berada di atas lakban hitam harus langsung menyala solid: `[■]`.
3. Uji tiap sensor satu per satu dari sensor 1 sampai sensor 5 untuk memastikan semua kanal optik responsif dan tidak ada yang macet/rusak.
