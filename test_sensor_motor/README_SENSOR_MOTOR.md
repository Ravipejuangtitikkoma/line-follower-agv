# Panduan Uji & Diagnostik Sensor Encoder Motor Menggunakan Arduino

Dokumen ini memuat panduan lengkap pengujian **Sensor Encoder Motor (Roda Kiri)** menggunakan **Arduino Uno / Nano** untuk memeriksa apakah sensor hall-effect / optocoupler pada motor masih menghasilkan pulsa (*ticks*) atau mengalami kerusakan konslet (*short-circuit*).

---

## 1. Skema Pengkabelan (Wiring) 6 Warna Kabel Motor Encoder ke Arduino

Motor DC dengan encoder magnetik (Hall Effect) umumnya memiliki **6 kabel** dengan urutan warna:
**Red (Merah), Black (Hitam), Yellow (Kuning), Green (Hijau), Blue (Biru), White (Putih)**.

### Tipe Standar JGB37-520 / JGA25-370 (Paling Umum di Indonesia):

| No | Warna Kabel | Fungsi Asli | Sambungkan ke Pin Arduino | Catatan Penting |
| :-: | :--- | :--- | :---: | :--- |
| 1 | **Merah (Red)** | Motor Power (+) | **JANGAN DIHUBUNGKAN** | Kabel daya motor DC (biarkan mengambang) |
| 2 | **Hitam (Black)** | Encoder GND | **GND** Arduino | Ground sensor |
| 3 | **Kuning (Yellow)**| Sinyal Fase A | **Pin D2** Arduino | Input pulsa Interrupt INT0 |
| 4 | **Hijau (Green)** | Sinyal Fase B | **Pin D3** Arduino | Input quadrature arah putaran |
| 5 | **Biru (Blue)** | Encoder VCC | **5V** (atau 3.3V) Arduino | Daya IC Hall Effect (3.3V - 5V) |
| 6 | **Putih (White)** | Motor Power (-) | **JANGAN DIHUBUNGKAN** | Kabel daya motor DC (biarkan mengambang) |

> [!WARNING]
> **Kabel Motor (Merah & Putih) JANGAN dicolok ke pin Arduino manapun!**  
> Daya motor membutuhkan arus besar dari driver/baterai. Untuk pengujian sensor ini, **kabel Merah & Putih biarkan lepas (tidak terhubung)**, dan Anda cukup **memutar roda menggunakan tangan secara manual**.

> [!TIP]
> **Cara Memastikan Tipe Motor Anda:**
> Periksa tulisan sablon (silkscreen) pada PCB bundar di pantat motor:
> - Jika tertulis `M+`, `GND`, `C1/A`, `C2/B`, `VCC`, `M-` $\to$ **Tipe Standar di atas (Biru=VCC, Hitam=GND)**.
> - Jika tertulis `M1`, `M2`, `GND`, `VCC`, `A`, `B` (Tipe Pololu) $\to$ **Biru=VCC, Hijau=GND, Kuning=A, Putih=B (Merah & Hitam = Motor)**.

---

## 2. Cara Menjalankan Uji Coba

1. Buka berkas [test_sensor_motor.ino](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_sensor_motor/test_sensor_motor.ino) di Arduino IDE.
2. Pada menu **Tools**:
   - Pilih Board: **Arduino Uno** (atau **Arduino Nano**).
   - Pilih Port: Sesuai port USB Arduino Anda di Device Manager.
3. Klik tombol **Upload**.
4. Buka **Serial Monitor** pada kecepatan **`9600 baud`**.
5. Putar roda kiri yang memiliki encoder perlahan dengan tangan Anda:
   - Putar ke depan (maju).
   - Putar ke belakang (mundur).

---

## 3. Tampilan Hasil di Serial Monitor

```text
Pin A: 1 | Pin B: 0 | Ticks: 45  | Arah: MAJU (CW)   | Kecepatan: 12.3 cm/s (36 RPM) | Total Jarak: 2.8 cm
Pin A: 0 | Pin B: 1 | Ticks: 120 | Arah: MAJU (CW)   | Kecepatan: 18.5 cm/s (54 RPM) | Total Jarak: 7.4 cm
Pin A: 1 | Pin B: 1 | Ticks: 85  | Arah: MUNDUR (CCW)| Kecepatan: -10.2 cm/s (30 RPM)| Total Jarak: 5.2 cm
Pin A: 1 | Pin B: 1 | Ticks: 85  | Arah: DIAM        | Kecepatan: 0.0 cm/s (0 RPM)   | Total Jarak: 5.2 cm
```

---

## 4. Analisis Diagnostik: Apakah Sensor Motor Rusak / Konslet?

### A. Sensor Normal (Sehat):
- Saat roda diputar maju $\to$ Nilai **`Ticks`** bertambah secara konsisten (+).
- Saat roda diputar mundur $\to$ Nilai **`Ticks`** berkurang (-).
- Nilai `Pin A` dan `Pin B` bergantian berubah antara `0` dan `1` saat diputar perlahan.

### B. Sensor Rusak / Mati:
- Roda sudah diputar-putar dengan tangan, tetapi nilai **`Ticks` tetap 0** dan tidak pernah berubah.
- Nilai `Pin A` atau `Pin B` diam terus-menerus di `0` atau `1` (tidak ada gelombang pulsa).

### C. Cek Potensi Korsleting (Penyebab Mappi32 Panas Kemarin):
- Raba bagian belakang PCB encoder di pantat motor DC saat kabel VCC/GND dicolok ke 5V Arduino:
  - Jika PCB encoder di motor terasa **hangat atau panas**, atau jika Arduino langsung mati saat kabel encoder dicolokkan, berarti **IC Hall Effect di encoder motor mengalami konslet internal**.
