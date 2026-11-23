# Panduan Uji 1 Sensor Garis (Single Line Sensor Test) di ESP32 / Mappi32

Program ini dibuat khusus untuk menguji **SATU sensor garis saja** pada board ESP32 atau Mappi32. Tujuannya adalah mempermudah diagnostik modul sensor secara mendalam, mengecek respon mata sensor, mengukur voltase/ADC, dan memutar trimpot kalibrasi tanpa bingung membaca 5 sensor sekaligus.

---

## 1. Skema Pengkabelan (Wiring 1 Sensor ke ESP32)

Hanya perlu menghubungkan 3 kabel:

| Pin Sensor Garis | Hubungkan ke Pin ESP32 / Mappi32 | Keterangan |
| :---: | :---: | :--- |
| **VCC** | **3.3V atau 5V** | Sumber tegangan sensor |
| **GND** | **GND** | Ground bersama (*common ground*) |
| **OUT / D0** | **Pin GPIO Pilihan Anda** (misal: **GPIO 34**) | Pin sinyal sensor |

### Pilihan Pin pada Board Mappi32:
Jika Anda sedang menguji satu per satu kanal dari modul 5-channel mobil AGV kita, gunakan pin berikut:
- **Kanal 1 (S1):** `GPIO 34`
- **Kanal 2 (S2):** `GPIO 35`
- **Kanal 3 (S3 Tengah):** `GPIO 36` (VP)
- **Kanal 4 (S4):** `GPIO 39` (VN)
- **Kanal 5 (S5):** `GPIO 23`

---

## 2. Cara Mengganti Pin yang Diuji

Buka berkas [`test_sensor_garis_tunggal.ino`](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_sensor_garis_tunggal/test_sensor_garis_tunggal.ino), cukup ubah angka pada baris ke-35:

```cpp
const int PIN_SENSOR_GARIS = 34; // Ganti ke 35, 36, 39, atau 23 sesuai kabel yang sedang dicolok
```

---

## 3. Cara Menjalankan Uji Coba

1. Buka berkas [`test_sensor_garis_tunggal.ino`](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_sensor_garis_tunggal/test_sensor_garis_tunggal.ino) di Arduino IDE.
2. Pilih Board: **ESP32 Dev Module** (atau Mappi32).
3. Upload program ke board ESP32.
4. Buka **Serial Monitor** pada kecepatan **`115200 baud`**.

---

## 4. Tampilan Hasil di Serial Monitor (115200 Baud)

Setiap 100 ms, program akan menampilkan status live:

```text
Pin 34 | DIG: 0 (LOW ) | ADC:  450 (0.36V) | STATUS: [■■ GARIS HITAM ■■] | BAR: [##........] | Total Deteksi: 5
Pin 34 | DIG: 1 (HIGH) | ADC: 3950 (3.18V) | STATUS: [  LANTAI PUTIH  ] | BAR: [##########] | Total Deteksi: 5
```

- **DIG:** Nilai logika digital (`0 / LOW` atau `1 / HIGH`).
- **ADC & Volt:** Nilai analog 12-bit (0 - 4095) dan estimasi tegangan nyata (Volt).
- **STATUS:** Menunjukkan langsung apakah sedang membaca `[■■ GARIS HITAM ■■]` atau `[  LANTAI PUTIH  ]`.
- **Total Deteksi:** Menghitung berapa kali garis hitam melintas di depan sensor.

---

## 5. Tombol Kontrol Langsung via Serial Monitor

Anda dapat mengetik huruf berikut di Serial Monitor lalu tekan Enter:

- **`P`** : Membalik polaritas deteksi (*Active LOW $\leftrightarrow$ Active HIGH*).
- **`C`** : Mereset counter total deteksi kembali ke 0.
- **`H`** : Menampilkan menu bantuan (*Help*).

---

## 6. Cara Kalibrasi Trimpot di Lapangan

1. Letakkan sensor di atas **lantai spanduk / kardus putih** dengan jarak kerja robot ($\sim 6 - 8\text{ mm}$):
   - Perhatikan Serial Monitor: Status harus terbaca `[  LANTAI PUTIH  ]`.
   - Jika masih terbaca `[■■ GARIS HITAM ■■]`, putar trimpot biru kecil pada modul sensor menggunakan obeng minus perlahan sampai status berubah menjadi `[  LANTAI PUTIH  ]`.
2. Geser sensor ke atas **lakban hitam**:
   - Status harus langsung beralih ke `[■■ GARIS HITAM ■■]` dan `Total Deteksi` bertambah 1.
3. Geser bolak-balik beberapa kali untuk memastikan respon sensor cepat dan tidak macet (*stuck*).
