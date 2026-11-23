# PANDUAN PENGUJIAN LINTASAN KOTAK BELOK KIRI (COUNTER-CLOCKWISE AGV)
**File Sketch:** [`test_lintasan_kotak_kiri.ino`](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_lintasan_kotak_kiri/test_lintasan_kotak_kiri.ino)  
**Board:** Mappi32 (ESP32 by KMtek)  
**Driver Motor:** Dual BTS7960  
**Sensor Garis:** 5-Channel TCRT5000  

Program ini adalah versi **pencerminan simetris (inversi logika penuh)** dari uji lintasan kotak belok kanan yang telah terbukti sempurna. Dioptimalkan khusus untuk mengitari lintasan kotak berlawanan arah jarum jam (*Counter-Clockwise*).

---

## 1. Logika Inversi Deteksi Tikungan Kiri 90° (`cekSudutKotak`)
- **Simpang Tiga Cabang Kiri / Sudut Siku Kiri:**
  - Wajib kedua sensor kiri aktif: `S1 == 1 && S2 == 1`
  - Wajib sensor tengah aktif atau jalur tebal: `S3 == 1` ATAU `totalAktif >= 3`
  - Sisi kanan tetap spanduk putih: `S5 == 0`
- **Pola Oleng Ditolak:**
  Pola garis lurus melenceng `[1 1 0 0 0]` di mana `S3 == 0` ditolak 100% dari pemicu belok, melainkan langsung diluruskan oleh counter-torque tracking lurus.
- **Validasi Cepat (~5 ms):**
  Tikungan dikonfirmasi valid setelah **2 kali pembacaan berturut-turut** (`SYARAT_VALIDASI_TIKUNGAN = 2`) agar mobil langsung mengerem di simpang tanpa bablas.

---

## 2. Sekuens Belok Kiri 90° Presisi (Inversi Penuh)

1. **[Langkah 1] Ketemu Simpang Kiri -> Langsung Berhenti:** Pengereman penuh seketika (`berhentiTotal()`, jeda 150 ms).
2. **[Langkah 2] Mundur Dikit:** Mundur perlahan 70 ms (`KEC_MUNDUR_SEBELUM_BELOK = 85`), lalu jeda 120 ms untuk menyerap inersia dan memberi ruang putar.
3. **[Langkah 3] Belok Nyari Garis Baru 90° (Putar Poros ke Kiri):**
   - **Sentakan Awal (Breakaway Kick 30 ms):** Roda kiri mundur `-185`, roda kanan maju `+155` untuk mendobrak friksi statis ban karet di spanduk.
   - **Putaran Poros:** Roda kiri mundur `-158`, roda kanan maju `+125`.
   - **Fase A:** Sapu lepas dari garis lama (80 ms atau sensor menyentuh putih).
   - **Fase B (Deteksi Masuk Garis Baru):** Karena belok ke kiri, garis baru datang dari sisi kiri:
     - Saat sensor tepi kiri (`S1 == 1` atau `S2 == 1`) menyentuh garis, **kecepatan putar otomatis diperlambat** (`putarPorosKiri(90, 115)`) agar tidak kebablasan.
     - Begitu sensor tengah menyambar garis (`S3 == 1`), **seketika rem aktif lawan arah** (roda kiri `+90`, roda kanan `-90` selama 30 ms) dan kunci posisi!
4. **[Langkah 4] FITUR NGELURUSIN (Dedicated Alignment Loop Belok Kiri):**
   - **Jika S3 belum kena (`S3 == 0`):**
     - **Baru ujung kiri kena (`S1 == 1 || S2 == 1`):** Putaran kiri belum maksimal! Robot **putar kiri lagi** (`putarPorosKiri(110, 140)`) sampai `S3` kena!
     - **Kebablasan ke kanan (`S5 == 1 || S4 == 1`):** Putaran kelewat! Robot **putar balik ke kanan** (`putarPorosKanan(110, 140)`) sampai `S3` kena!
   - **Setelah S3 kena (`S3 == 1`):**
     - Jika garis condong di kiri (`b2 == 1 && b4 == 0`): putar kiri mikro (`putarPorosKiri(80, 80)`).
     - Jika garis condong di kanan (`b4 == 1 && b2 == 0`): putar kanan mikro (`putarPorosKanan(80, 80)`).
     - Jika seimbang (`b2 == b4`): Bodi sudah 100% lurus simetris di tengah!
   - **Pengaman Mutlak:** DILARANG MAJU jika S3 belum kena! Jika ada kondisi ekstrem, ada pulsa darurat mencari S3 sebelum lanjut.
   - Jeda diam stabil 150 ms setelah bodi benar-benar lurus di S3.
5. **[Langkah 5] BARU MAJU:**
   Meluncur kembali menyusuri garis lurus berikutnya.

---

## 3. Fitur Ngelurusin Wajib Sensor Tengah (S3) di Jalur Lurus

- **Kasus 1: S3 Kena (`line.b3 == 1`):**
  - Tengah murni: Lurus seimbang (`vL = KEC_BASE + TRIM_KIRI`, `vR = KEC_BASE + TRIM_KANAN`).
  - Geser kiri (`S2 == 1`): Koreksi lembut ke kiri (`vL - 20`, `vR + 25`).
  - Geser kanan (`S4 == 1`): Koreksi lembut ke kanan (`vL + 25`, `vR - 20`).
- **Kasus 2: S3 Lepas (`line.b3 == 0`) -> Counter-Torque Seketika:**
  - `S2 == 1`: Roda kiri mundur `-25`, roda kanan maju `KEC_BASE + 35` (putar cepat ke kiri menyambar S3).
  - `S4 == 1`: Roda kanan mundur `-25`, roda kiri maju `KEC_BASE + 35` (putar cepat ke kanan menyambar S3).
  - `S1 == 1`: Putar tajam kiri (`vL = -45`, `vR = KEC_BASE + 50`).
  - `S5 == 1`: Putar tajam kanan (`vL = KEC_BASE + 50`, `vR = -45`).

---

## 4. Tombol Kontrol Serial Monitor (115200 Baud)
- `' '` (Spasi) : Start / Stop Mobil
- `'+'` / `'-'` : Tambah / Kurangi Kecepatan Maju (`KEC_BASE`)
- `'L'`         : Set Arah Belok ke KIRI
- `'K'`         : Set Arah Belok ke KANAN
- `'k'`         : Buka Mode Visualizer Sensor

