# Panduan Pengujian & Diagnostik Driver Motor BTS7960 Menggunakan Arduino

Dokumen ini memuat panduan lengkap untuk menguji **Driver Motor (BTS7960)** dan **Sensor Encoder Motor** menggunakan **Arduino Uno / Nano** sebelum memasang mikrokontroler utama yang baru.

---

## 1. Skema Pengkabelan (Wiring) ke Arduino Uno / Nano

### A. Sisi Sinyal Logika Driver BTS7960 $\to$ Pin Arduino:

| Pin Driver BTS7960 | Hubungkan ke Pin Arduino | Fungsi / Keterangan |
| :--- | :---: | :--- |
| **VCC** | **5V** Arduino | Daya logika chip driver (5V) |
| **GND** | **GND** Arduino | Ground logika |
| **R_EN** & **L_EN** | **5V** Arduino | Jumper kedua pin ini ke 5V agar driver selalu aktif (*Enable*) |
| **RPWM (Driver Kiri)** | **Pin D5** Arduino | PWM Maju Motor Kiri |
| **LPWM (Driver Kiri)** | **Pin D6** Arduino | PWM Mundur Motor Kiri |
| **RPWM (Driver Kanan)**| **Pin D9** Arduino | PWM Maju Motor Kanan |
| **LPWM (Driver Kanan)**| **Pin D10** Arduino | PWM Mundur Motor Kanan |

---

### B. Sisi Daya Motor & Baterai (Arus Besar):

| Terminal Driver | Hubungkan ke | Keterangan |
| :--- | :--- | :--- |
| **B+** | **Kutub Positif (+) Baterai 12V LiPo** | Daya tegangan tinggi untuk motor |
| **B-** | **Kutub Negatif (-) Baterai 12V LiPo** | Ground baterai (**Wajib tersambung ke GND Arduino!**) |
| **M+** | **Kabel Merah Motor** | Output daya motor DC |
| **M-** | **Kabel Putih Motor** | Output daya motor DC |

---

### C. Sisi Sensor Encoder (Roda Kiri):

| Kabel Encoder Motor | Hubungkan ke Pin Arduino | Keterangan |
| :--- | :---: | :--- |
| **Biru (Blue)** | **5V** Arduino | Daya IC Hall Effect |
| **Hitam (Black)** | **GND** Arduino | Ground sensor |
| **Kuning (Yellow)** | **Pin D2** Arduino | Pulsa Fase A (Interrupt INT0) |
| **Hijau (Green)** | **Pin D3** Arduino | Pulsa Fase B (Arah putaran) |

---

## 2. Peringatan Keselamatan Penting (Safety First)

> [!CAUTION]
> 1. **Ganjal Bodi Robot!**  
>    Letakkan robot di atas balok/kotak sehingga **keempat rodanya melayang (tidak menyentuh meja)** saat diuji agar robot tidak melompat atau menabrak.
> 2. **Wajib Common Ground:**  
>    Kutub negatif baterai (B-) **wajib tersambung dengan GND Arduino**. Jika kabel GND terlepas saat motor berputar kencang, arus balik (*back-EMF*) dapat merusak mikrokontroler!
> 3. **Raba Suhu Driver:**  
>    Saat baterai pertama kali dicolokkan (kondisi motor masih diam), raba heatsink aluminium BTS7960. Jika terasa langsung panas membara padahal belum diberi perintah putar, berarti **MOSFET driver telah rusak / short circuit**.

---

## 3. Alur Kerja Pengujian Otomatis

Program ini **100% otomatis** (tidak perlu mengetik menu apapun di Serial Monitor). Begitu sketch di-upload, Arduino akan langsung menguji motor secara mandiri dengan siklus berulang:

1. **Jeda Awal 3 Detik** (Waktu aman untuk menjauhkan tangan).
2. **Motor KIRI Maju** selama 2 detik $\to$ Rem/Stop 1 detik.
3. **Motor KIRI Mundur** selama 2 detik $\to$ Rem/Stop 2 detik.
4. **Motor KANAN Maju** selama 2 detik $\to$ Rem/Stop 1 detik.
5. **Motor KANAN Mundur** selama 2 detik $\to$ Rem/Stop 3 detik.
6. **Siklus diulang terus-menerus secara otomatis**.

---

## 4. Analisis Diagnostik

* **Driver Normal:**
  - Sisi Kiri berputar maju 2 detik, lalu mundur 2 detik.
  - Sisi Kanan berputar maju 2 detik, lalu mundur 2 detik.
  - Nilai *Encoder Ticks* pada roda kiri bertambah dan tercatat di Serial Monitor saat roda kiri berputar.
* **Driver Rusak / Jebol:**
  - Motor langsung berputar liar begitu baterai dicolok tanpa menunggu aba-aba program (MOSFET jebol tembus).
  - Motor hanya mendengung tapi tidak mau berputar, atau heatsink driver mendadak sangat panas.
