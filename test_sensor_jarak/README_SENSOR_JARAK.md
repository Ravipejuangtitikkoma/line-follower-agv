# Panduan Uji & Diagnostik Sensor Ultrasonik HC-SR04 Menggunakan Arduino

Dokumen ini memuat panduan pengujian sensor jarak HC-SR04 menggunakan board **Arduino Uno / Nano** untuk memeriksa apakah sensor ini masih berfungsi normal atau mengalami kerusakan konslet (*short-circuit*) yang berpotensi menjadi penyebab kerusakan pada Mappi32 sebelumnya.

---

## 1. Skema Pengkabelan (Wiring) ke Arduino

| Pin HC-SR04 | Hubungkan ke Pin Arduino | Keterangan |
| :---: | :---: | :--- |
| **VCC** | **5V** | Suplai tegangan utama sensor |
| **GND** | **GND** | Jalur ground bersama (*common ground*) |
| **TRIG** | **D9** | Pin pemicu pulsa ultrasonik (*Trigger*) |
| **ECHO** | **D10** | Pin penerima pantulan pulsa (*Echo*) |

> [!NOTE]
> Arduino Uno / Nano menggunakan **logika 5V**, sehingga pin ECHO dari HC-SR04 (yang mengeluarkan sinyal 5V) **sangat aman** dihubungkan langsung ke pin Arduino tanpa perlu resistor tambahan.

---

## 2. Cara Menjalankan Uji Coba

1. Buka berkas [test_sensor_jarak.ino](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_sensor_jarak/test_sensor_jarak.ino) di Arduino IDE.
2. Pada menu **Tools**:
   - Pilih Board: **Arduino Uno** (atau **Arduino Nano** sesuai board yang Anda pakai).
   - Pilih Port: Sesuai port USB Arduino Anda.
3. Klik tombol **Upload** (tanda panah ke kanan).
4. Setelah upload selesai, buka **Serial Monitor** (Ctrl + Shift + M):
   - Atur baud rate ke **`9600 baud`**.
5. Dekatkan telapak tangan Anda ke depan silinder sensor (maju-mundur antara 5 cm hingga 50 cm).

---

## 3. Cara Memeriksa Apakah Sensor Ini yang Merusak Mappi32 Kemarin

### A. Tes Suhu Fisik (Sangat Penting!)
Saat sensor tercolok ke Arduino dan menyala:
1. Raba bagian **belakang PCB sensor HC-SR04** (terutama 3 buah IC chip hitam kecil di baliknya, yaitu IC LM324, MAX3232, dan MCU mikro).
2. **Kondisi Normal:** Seluruh chip di belakang sensor harus **DINGIN** atau suhu ruangan biasa.
3. **Kondisi Rusak / Konslet:** Jika ada chip di belakang HC-SR04 yang terasa **HANGAT atau MENDIDIH PANAS**, berarti sensor tersebut mengalami **internal short circuit**. Jika sensor konslet ini kemarin dicolokkan ke Mappi32, sensor inilah yang menarik arus berlebih atau membakar regulator 3.3V Mappi32! **Jangan gunakan sensor itu lagi.**

### B. Analisis Log Serial Monitor
- **NORMAL OK (Jarak Terbaca Presisi):**
  Sensor sehat dan berfungsi normal.
- **ECHO STUCK HIGH / SHORTED:**
  Pin ECHO mengeluarkan tegangan 5V secara terus-menerus tanpa berhenti. Ini adalah indikasi kuat IC sensor rusak dan bisa merusak pin GPIO mikrokontroler.
- **TIMEOUT TERUS-MENERUS (Durasi 0 us):**
  Transduser pemancar/penerima putus atau kristal 4MHz di modul sensor mati.

---

### C. Mengapa HC-SR04 Berbahaya jika Langsung ke ESP32 / Mappi32?
- HC-SR04 bekerja pada **5V**. Pin ECHO mengeluarkan sinyal pulsa sebesar **5V**.
- Sedangkan pin GPIO ESP32 / Mappi32 hanya dirancang untuk tegangan maksimal **3.3V (maks. 3.6V)**.
- Jika dihubungkan langsung ke ESP32 tanpa rangkaian pembagi tegangan (*voltage divider*) menggunakan 2 resistor (misal 1kΩ dan 2kΩ) untuk menurunkan sinyal ECHO dari 5V ke 3.3V, tegangan 5V tersebut lama-kelamaan akan merusak gerbang I/O internal ESP32.
