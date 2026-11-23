# PANDUAN PENGUJIAN LINTASAN KOTAK SEMPURNA (AGV ROBOT)
**File Sketch:** [	est_lintasan_kotak.ino](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_lintasan_kotak/test_lintasan_kotak.ino)  
**Board:** Mappi32 (ESP32 by KMtek)  
**Driver Motor:** Dual BTS7960  
**Sensor Garis:** 5-Channel TCRT5000  

---

## 1. Fitur Ngelurusin Wajib Sensor Tengah (S3) di Jalur Lurus

Ketika melaju di jalur lurus, **target mutlak mobil adalah menjaga sensor tengah (S3) selalu berada di atas lakban hitam**:

1. **Kasus 1: Sensor Tengah Kena (S3 == 1)**
   - **Tengah Sempurna (S2 == S4):** Melaju lurus seimbang penuh (L = KEC_BASE + TRIM_KIRI, R = KEC_BASE + TRIM_KANAN).
   - **Sedikit Condong ke Kiri (S3 == 1 && S2 == 1):** Fitur ngelurusin menarik bodi ke kiri (L - 20, R + 25) agar kembali ke S3 murni.
   - **Sedikit Condong ke Kanan (S3 == 1 && S4 == 1):** Fitur ngelurusin menarik bodi ke kanan (L + 25, R - 20) agar kembali ke S3 murni.

2. **Kasus 2: Sensor Tengah Lepas (S3 == 0) -> Counter-Torque Seketika!**
   *Pada chassis 4WD ban karet lebar di lantai spanduk, perbedaan kecepatan maju saja tidak cukup memutar hidung mobil. Diterapkan counter-torque (roda satu sisi mundur) agar mobil berputar cepat menyambar kembali S3:*
   - **Garis di Tengah-Kiri (S2 == 1):** Roda kiri mundur -25, roda kanan maju KEC_BASE + 35. Hidung mobil seketika berputar ke kiri menyambar S3.
   - **Garis di Tengah-Kanan (S4 == 1):** Roda kanan mundur -25, roda kiri maju KEC_BASE + 35. Hidung mobil seketika berputar ke kanan menyambar S3.
   - **Garis di Ujung Kiri (S1 == 1):** Putar tajam kiri (L = -45, R = KEC_BASE + 50).
   - **Garis di Ujung Kanan (S5 == 1):** Putar tajam kanan (L = KEC_BASE + 50, R = -45).

3. **Kasus 3: Garis Lepas Total (isGarisHilang [0 0 0 0 0])**
   - Putar poros lembut mencari kembali ke arah terakhir garis terlihat sampai S3 menyambar garis.

---

## 2. Deteksi Simpang / Sudut Kotak Anti-Bablas (SYARAT_VALIDASI_TIKUNGAN = 2)

- **Validasi Cepat (~5 ms):**
  Tikungan kotak dikonfirmasi valid setelah **2 kali pembacaan berturut-turut** agar mobil langsung berhenti tanpa bablas.
- **Syarat Mutlak Anti-False-Trigger:**
  - Wajib kedua sensor kanan aktif: S5 == 1 && S4 == 1.
  - Wajib ada garis tengah atau jalur tebal: S3 == 1 ATAU 	otalAktif >= 3.
  - Sisi kiri tetap bersih: S1 == 0.
  - Pola oleng garis lurus [0 0 0 1 1] di mana S3 == 0 dan 	otalAktif == 2 **ditolak 100%**.

---

## 3. Sekuens Belok 90° Presisi (Anti-Belum-Dapat-Tengah-Udah-Maju)

1. **[Langkah 1] Ketemu Simpang -> Langsung Berhenti:** Pengereman penuh (erhentiTotal(), jeda 150 ms).
2. **[Langkah 2] Mundur Dikit:** Mundur perlahan 70 ms (KEC_MUNDUR_SEBELUM_BELOK = 85), lalu jeda 120 ms untuk menyerap inersia dan memberi ruang putar.
3. **[Langkah 3] Belok Nyari Garis Baru 90°:**
   - Putar poros ke kanan (kiri maju 125, kanan mundur 158).
   - Saat sensor tepi kanan (S5 atau S4) mulai menyentuh garis, **kecepatan putar otomatis diperlambat** mendekati S3.
   - Begitu sensor tengah (S3 == 1) menyambar garis, **langsung rem aktif seketika dan kunci!**
4. **[Langkah 4] FITUR NGELURUSIN (Dedicated Alignment Loop):**
   - **Jika S3 belum kena (S3 == 0):** Wajib putar pulsa bertenaga sampai S3 kena (kanan jika baru ujung kanan kena, kiri jika kebablasan).
   - **Setelah S3 kena (S3 == 1):** Luruskan sudut bodi mikro hingga simetris (2 == b4).
   - **Pengaman Mutlak:** DILARANG MAJU jika S3 belum kena garis hitam! Jika belum dapat, ada pulsa darurat pencari S3.
   - Jeda diam stabil 150 ms setelah bodi benar-benar lurus di S3.
5. **[Langkah 5] BARU MAJU:**
   - Mobil meluncur maju kembali menyusuri sisi lintasan lurus berikutnya.

---

## 4. Cara Mengubah Arah Belok (Cukup 1 Variabel)

Buka file [	est_lintasan_kotak.ino](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/test_lintasan_kotak/test_lintasan_kotak.ino#L26-L35). Pada bagian atas:

`cpp
// Pilihan:
//   ARAH_BELOK_KANAN : Mobil berbelok KANAN terus di setiap sudut kotak (Clockwise)
//   ARAH_BELOK_KIRI  : Mobil berbelok KIRI terus di setiap sudut kotak (Counter-Clockwise)
const ArahBelok ARAH_BELOK = ARAH_BELOK_KANAN;
`

---

## 5. Tombol Serial Interaktif (115200 Baud)

- ' ' (Spasi) : Start / Stop Mobil
- '+' / '-' : Naikkan / Turunkan Kecepatan Maju (KEC_BASE)
- 'K'         : Set Arah Belok ke KANAN
- 'L'         : Set Arah Belok ke KIRI
- 'k'         : Buka Mode Visualizer Sensor
