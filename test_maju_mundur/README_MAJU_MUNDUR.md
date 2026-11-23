# Program Uji Sederhana: Maju & Mundur Saja

Program ini dibuat khusus dan terpisah di file [test_maju_mundur.ino](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_maju_mundur/test_maju_mundur.ino) untuk menguji gerakan fisik dasar robot: **hanya maju dan mundur secara otomatis bergantian**.

---

## 1. Pin yang Digunakan (Mappi32 ESP32):

| Pin ESP32 / Mappi | Terhubung ke Driver BTS7960 | Fungsi |
| :---: | :---: | :--- |
| **`GPIO 25`** | **RPWM Kiri** | Maju Roda Kiri |
| **`GPIO 26`** | **LPWM Kiri** | Mundur Roda Kiri |
| **`GPIO 16`** | **RPWM Kanan** | Maju Roda Kanan |
| **`GPIO 17`** | **LPWM Kanan** | Mundur Roda Kanan |

---

## 2. Alur Pengujian:
1. Nyalakan board / upload sketch $\to$ Jeda aman **3 detik**.
2. Mobil **MAJU** dengan kecepatan PWM 110 selama **2 detik**.
3. Mobil **STOP** (rem) selama **1.5 detik**.
4. Mobil **MUNDUR** selama **2 detik**.
5. Mobil **STOP** selama **3 detik**.
6. Siklus diulang terus-menerus.

---

## 3. Catatan Penggunaan:
* Baud rate Serial Monitor: **`115200 baud`**.
* Jika ingin mengubah kecepatan, ubah nilai `const int KECEPATAN = 110;` di baris 19 (rentang 0 - 255).
* Ganjal sasis robot terlebih dahulu saat pertama kali uji coba agar roda melayang di udara.
