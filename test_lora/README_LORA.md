# PANDUAN PENGUJIAN KOMUNIKASI LORA (DUA MAPPI32 ESP32)

Folder ini berisi sketsa pengujian komunikasi nirkabel **LoRa (Long Range)** dua arah (*Two-Way / Ping-Pong*) antara dua buah board **Mappi32 (ESP32)** buatan KMtek.

---

## 1. Arsitektur Pengujian

```
┌─────────────────────────────────┐                 ┌─────────────────────────────────┐
│       MAPPI32 - NODE 1          │                 │       MAPPI32 - NODE 2          │
│   (Controller / Base Station)   │                 │     (AGV Robot / Receiver)      │
│  File: test_lora_sender.ino     │                 │  File: test_lora_receiver.ino   │
├─────────────────────────────────┤      LoRa       ├─────────────────────────────────┤
│ • Kirim: PING #1 / CMD:START    │ ──────────────► │ • Terima: Baca data & RSSI/SNR  │
│ • Terima: ACK Konfirmasi Balasan│ ◄────────────── │ • Balas: ACK:START [AGV READY]  │
│ • Input Manual via Serial Mon   │    (915 MHz)    │ • Kedipkan LED Indikator (IO5)  │
└─────────────────────────────────┘                 └─────────────────────────────────┘
```

---

## 2. Pinout Internal Modul LoRa Mappi32 (RFM95 / SX1276)

Karena modul LoRa sudah terintegrasi (*built-in*) di dalam board Mappi32, koneksi pin SPI ke ESP32 adalah sebagai berikut:

| Jalur Sinyal | Pin GPIO ESP32 | Keterangan |
| :--- | :---: | :--- |
| **SCK** | **`14`** | SPI Clock |
| **MISO** | **`12`** | Master In Slave Out |
| **MOSI** | **`13`** | Master Out Slave In |
| **NSS / SS** | **`15`** | Chip Select LoRa |
| **RST** | **`0`** | Reset LoRa |
| **DIO0** | **`27`** | Interrupt Pin (RX Done / TX Done) |
| **Onboard LED** | **`5`** | Indikator Kedip Transmit / Receive |

> **Catatan Antena:** Pastikan antena LoRa terpasang pada kedua board Mappi32 sebelum dinyalakan agar modul radio RFM95 tidak rusak.

---

## 3. Prasyarat Library Arduino IDE

Pastikan library **LoRa** sudah terpasang di Arduino IDE Anda:
1. Buka Arduino IDE $\to$ Menu **Tools** $\to$ **Manage Libraries...** (atau `Ctrl + Shift + I`).
2. Ketik di kolom pencarian: **`LoRa`**.
3. Pilih library **`LoRa by Sandeep Mistry`** dan klik **Install**.

---

## 4. Langkah-Langkah Pengujian

### Langkah 1: Upload ke Mappi32 Pertama (Sender / Controller)
1. Buka file: [`test_lora_sender/test_lora_sender.ino`](test_lora_sender/test_lora_sender.ino).
2. Sambungkan Mappi32 (Node 1) ke port USB komputer.
3. Pilih board: **ESP32 Dev Module** (atau board Mappi32 jika sudah install board package KMtek).
4. Klik **Upload**.
5. Buka **Serial Monitor** pada baudrate **`115200`**.

### Langkah 2: Upload ke Mappi32 Kedua (Receiver / AGV)
1. Buka jendela Arduino IDE baru.
2. Buka file: [`test_lora_receiver/test_lora_receiver.ino`](test_lora_receiver/test_lora_receiver.ino).
3. Sambungkan Mappi32 (Node 2) ke port USB kedua (atau komputer lain/laptop/powerbank).
4. Klik **Upload**.
5. Buka **Serial Monitor** pada baudrate **`115200`**.

---

## 5. Cara Interaksi & Verifikasi di Serial Monitor

### A. Pengujian Otomatis (*Ping-Pong Heartbeat*)
* Node Sender akan otomatis mengirim pesan `PING #1`, `PING #2`, dst. setiap 3 detik.
* Node Receiver akan menangkap paket, menyalakan LED (IO5), dan langsung mengirim balasan `ACK:PING #1 [RSSI: -45dBm]`.
* Di Serial Monitor Sender akan tampil:
  ```text
  [TX LORA] Mengirim: "PING #1" ...
  >>> [RX BALASAN] Diterima: "ACK:PING #1 [RSSI:-42dBm]" | RSSI: -44 dBm | SNR: 9.75 dB
  ```

### B. Pengujian Perintah Misi AGV (Ketik Manual di Serial Sender)
Anda dapat mengetik perintah berikut di kolom atas Serial Monitor Sender:

| Perintah yang Diketik | Fungsi | Respon di Receiver (AGV) |
| :--- | :--- | :--- |
| **`START`** | Memulai Misi A $\to$ C | `PERINTAH START DITERIMA -> AGV MEMULAI MISI!` |
| **`STOP`** | Rem Darurat (*E-Stop*) | `PERINTAH STOP DITERIMA -> AGV EMERGENCY STOP!` |
| **`PULANG`** | Perintah Balik ke Awal | `PERINTAH PULANG DITERIMA -> AGV RETURN TO BASE!` |
| **`PING`** | Cek Koneksi Radio | Mengirim respon tes koneksi instan |

---

## 6. Parameter Frekuensi (Opsional)

Frekuensi bawaan diatur ke **`915E6`** (915 MHz). Jika modul Mappi32 Anda menggunakan pita frekuensi lain (misal 920 MHz atau 433 MHz), cukup ubah baris berikut di kedua file `.ino`:

```cpp
#define LORA_BAND  915E6  // Ganti ke 920E6 atau 433E6 jika perlu
```
*(Pastikan frekuensi di Sender dan Receiver selalu bernilai sama).*
