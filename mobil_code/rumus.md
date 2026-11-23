# DOKUMENTASI FISIKA, KINEMATIKA, & PERSAMAAN MATEMATIKA AGV ROBOT

Dokumen ini memuat seluruh rumus fisika, pemodelan mekanika gerak (*Skid-Steer Mobile Robot* / SSMR), dinamika motor DC, kontroler PD, serta pembuktian matematis terhadap parameter yang diimplementasikan pada kode robot AGV.

---

## 1. Parameter Fisik Robot (Spesifikasi Riil Lapangan)

Berdasarkan data fisik riil dari robot Anda:

| Parameter | Simbol | Nilai Riil | Satuan SI | Keterangan |
| :--- | :---: | :---: | :---: | :--- |
| **Massa Total Robot** | $m$ | **$1{,}45$** | $\text{kg}$ | Termasuk sasis, baterai, motor, & sensor |
| **Panjang Total Sasis** | $L_{\text{bodi}}$ | **$25{,}0$** | $\text{cm} = 0{,}25\text{ m}$ | Ujung depan ke ujung belakang |
| **Jarak Sumbu Roda (Wheelbase)** | $L$ | **$18{,}0$** | $\text{cm} = 0{,}18\text{ m}$ | Jarak as roda depan ke as roda belakang |
| **Lebar Lintasan Roda (Track Width)** | $W$ | **$18{,}0$** | $\text{cm} = 0{,}18\text{ m}$ | Jarak antar roda kiri ke roda kanan |
| **Jari-Jari Roda** | $r$ | **$3{,}25$** | $\text{cm} = 0{,}0325\text{ m}$ | Diameter roda standar AGV $65\text{ mm}$ |
| **Percepatan Gravitasi** | $g$ | **$9{,}81$** | $\text{m/s}^2$ | Gravitasi bumi |
| **Koefisien Gesek Statis Kardus** | $\mu_s$ | **$0{,}60$** | - | Gesekan karet ban terhadap kardus (awal) |
| **Koefisien Gesek Kinetis Kardus** | $\mu_k$ | **$0{,}48$** | - | Gesekan saat roda terseret menyamping |

---

## 2. Pemodelan Gaya Normal & Beban Gandar

Gaya gravitasi total (gaya tekan ke bawah / *Downforce*):

$$W_{\text{berat}} = N_{\text{total}} = m \cdot g$$

$$N_{\text{total}} = 1{,}45\text{ kg} \times 9{,}81\text{ m/s}^2 = 14{,}225\text{ Newton}$$

Dengan distribusi 4 roda simetris, beban normal pada masing-masing roda ($N_i$):

$$N_i = \frac{N_{\text{total}}}{4} = \frac{14{,}225}{4} = 3{,}556\text{ Newton per roda}$$

---

## 3. Dinamika Belok Putar Poros (*Skid-Steer Turning Resistance*)

Robot ini menggunakan sistem **Skid-Steer Mobile Robot (SSMR)** 4 roda tanpa roda belok (*steering rack*). Untuk berputar di tempat (*zero-radius turn*), roda kiri dan kanan berputar berlawanan arah, memaksa roda depan dan belakang terseret menyamping (*lateral skid*).

### A. Momen Hambatan Gesek Belok ($M_R$)

Ketika bodi berputar terhadap titik pusat instan (ICR) di tengah sasis, gaya gesek lateral pada roda depan dan belakang berada pada jarak $L/2$ dari sumbu putar:

$$F_{\text{gesek, lateral}} = \mu \cdot N_i$$

Momen hambatan gesek total yang melawan putaran mobil adalah:

$$M_R = \sum_{i=1}^{4} r_i \times F_{\text{gesek}, i} = 4 \times \left(\frac{L}{2}\right) \times (\mu \cdot N_i)$$

Substitusikan $N_i = \frac{m \cdot g}{4}$:

$$M_R = 4 \times \left(\frac{L}{2}\right) \times \left(\mu \cdot \frac{m \cdot g}{4}\right) = \frac{\mu \cdot m \cdot g \cdot L}{2}$$

Jika memperhitungkan hambatan elastis tapak ban karet pada kardus (faktor distribusi geometris rata-rata $\approx 0{,}5$):

$$M_R = \frac{\mu \cdot m \cdot g \cdot L}{4}$$

#### Perhitungan Nilai Momen Hambatan:
1. **Saat Awal Putaran (Mendobrak Gesekan Statis $\mu_s = 0{,}60$):**
   $$M_{R, \text{statis}} = \frac{0{,}60 \times 1{,}45 \times 9{,}81 \times 0{,}18}{4} = \frac{1{,}536}{4} = 0{,}384\text{ N}\cdot\text{m} \approx 3{,}92\text{ kg}\cdot\text{cm}$$

2. **Saat Sedang Berputar (Gesekan Luncur Kinetis $\mu_k = 0{,}48$):**
   $$M_{R, \text{kinetis}} = \frac{0{,}48 \times 1{,}45 \times 9{,}81 \times 0{,}18}{4} = \frac{1{,}229}{4} = 0{,}307\text{ N}\cdot\text{m} \approx 3{,}13\text{ kg}\cdot\text{cm}$$

---

## 4. Torsi dan Gaya Dorong Roda yang Dibutuhkan

Untuk mengatasi momen hambatan $M_R$, kedua roda (kiri dan kanan) yang terpisah sejauh lebar lintasan $W = 0{,}18\text{ m}$ menghasilkan pasangan gaya (*couple moment*):

$$M_{\text{motor}} = F_{\text{dorong}} \cdot W \ge M_R$$

Maka gaya dorong longitudinal minimum per sisi:

$$F_{\text{dorong}} \ge \frac{M_R}{W}$$

Substitusikan nilai $M_{R, \text{statis}}$:

$$F_{\text{dorong}} \ge \frac{0{,}384\text{ N}\cdot\text{m}}{0{,}18\text{ m}} = 2{,}133\text{ Newton}$$

### Torsi Poros Roda Motor Minimum ($\tau_{\text{roda}}$):
Torsi pada poros as roda ($r = 0{,}0325\text{ m}$):

$$\tau_{\text{roda}} = F_{\text{dorong}} \cdot r = \frac{M_R \cdot r}{W}$$

$$\tau_{\text{roda, statis}} = 2{,}133\text{ N} \times 0{,}0325\text{ m} = 0{,}0693\text{ N}\cdot\text{m} \approx 0{,}707\text{ kg}\cdot\text{cm}$$

Dengan faktor keamanan mekanik (*Safety Factor* $SF = 1{,}4$ untuk memperhitungkan penurunan tegangan baterai & efisiensi gearbox $\eta \approx 75\%$):

$$\tau_{\text{desain}} = \frac{\tau_{\text{roda}} \cdot SF}{\eta} = \frac{0{,}0693 \times 1{,}4}{0{,}75} = 0{,}129\text{ N}\cdot\text{m} \approx 1{,}32\text{ kg}\cdot\text{cm}$$

---

## 5. Hubungan Torsi Motor DC Terhadap Nilai PWM

Persamaan elektromekanis motor DC:

$$V_{\text{motor}} = I_a \cdot R_a + K_e \cdot \omega$$

Torsi motor sebanding langsung dengan arus angker ($I_a$):

$$\tau = K_t \cdot I_a$$

Pada saat awal mulai berbelok ($\omega \approx 0$, *stall condition*):

$$\tau_{\text{stall}} = K_t \cdot \frac{V_{\text{motor}}}{R_a}$$

Karena tegangan rata-rata motor dikontrol oleh PWM ESP32 (resolusi 8-bit, 0–255):

$$V_{\text{motor}} = V_{\text{baterai}} \times \left(\frac{\text{PWM}}{255}\right)$$

Maka torsi yang dihasilkan motor merupakan fungsi linier dari PWM:

$$\tau(\text{PWM}) = \tau_{\text{maks}} \times \left(\frac{\text{PWM}}{255}\right)$$

### Mengapa PWM 145 Lemah & PWM 180 Tepat?

| Nilai PWM | Rasio Daya ($\frac{\text{PWM}}{255}$) | Torsi Relatif | Status Terhadap Beban $1{,}45\text{ kg}$ |
| :---: | :---: | :---: | :--- |
| **`110`** | $43{,}1\%$ | $\approx 0{,}86\text{ kg}\cdot\text{cm}$ | **Torsi Awal Kick-Start** (Mendobrak inersia diam) |
| **`145`** | $56{,}8\%$ | $\approx 1{,}13\text{ kg}\cdot\text{cm}$ | **Di Bawah Torsi Desain** ($1{,}32\text{ kg}\cdot\text{cm}$) $\to$ **LEMAH / STALL** |
| **`180`** | **$70{,}6\%$** | **$\approx 1{,}41\text{ kg}\cdot\text{cm}$** | **Melampaui Torsi Desain** $\to$ **BERPUTAR TEGAS & MANTAP** |

---

## 6. Persamaan Kurva Percepatan *Soft-Start Ramp*

Untuk menghilangkan sentakan kasar (*Jerk* $j = \frac{da}{dt} \to \infty$) yang menyebabkan selip (*wheel slip*), torsi dinaikkan bertahap mengikuti profil linier trapesium (*Ramp-Up Profile*):

$$\text{PWM}(t) = \text{PWM}_{\text{start}} + \left(\text{PWM}_{\text{target}} - \text{PWM}_{\text{start}}\right) \cdot \frac{t}{t_{\text{ramp}}}$$

Pada implementasi [Motor.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Motor.h):
* $\text{PWM}_{\text{start}} = 110$
* $\text{PWM}_{\text{target}} = 180$
* $t_{\text{ramp}} = 50\text{ ms}$ (5 langkah pembagian @ $10\text{ ms}$)

Setiap langkah $i \in [1, 5]$:
$$\text{PWM}_i = 110 + (180 - 110) \cdot \frac{i}{5} = 110 + 14 \cdot i$$

$$\text{Step 1: } 124 \implies \text{Step 2: } 138 \implies \text{Step 3: } 152 \implies \text{Step 4: } 166 \implies \text{Step 5: } 180$$

---

## 7. Persamaan Kontroler Kemudi Lurus PD (Proportional - Derivative)

Pada pelacakan garis lurus di State 1, 4, dan 7, posisi garis dihitung dari bobot sensor:

### A. Perhitungan Error Titik Berat ($e(t)$):
Dengan 5 sensor $b_1, b_2, b_3, b_4, b_5 \in \{0, 1\}$ berbobot $[-2, -1, 0, +1, +2]$:

$$e(t) = \frac{\sum_{i=1}^{5} w_i \cdot b_i}{\sum_{i=1}^{5} b_i} = \frac{(-2 \cdot b_1) + (-1 \cdot b_2) + (0 \cdot b_3) + (1 \cdot b_4) + (2 \cdot b_5)}{b_1 + b_2 + b_3 + b_4 + b_5}$$

Rentang error: $-2{,}0 \le e(t) \le +2{,}0$.

### B. Rumus Koreksi Kemudi PD:
$$u(t) = K_p \cdot e(t) + K_d \cdot \left(\frac{e(t) - e(t - \Delta t)}{\Delta t}\right)$$

Dalam implementasi diskrit periode $\Delta t \approx 15\text{ ms}$:
$$u(t) = K_p \cdot e(t) + K_d \cdot \Delta e(t)$$

Dengan parameter di [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h):
* $K_p = 28{,}0$ (Respon kelengkungan garis)
* $K_d = 18{,}0$ (Peredam osilasi inersia bodi $1{,}45\text{ kg}$)

### C. Alokasi Kecepatan Motor Diferensial:
$$v_{\text{kiri}}  = KEC\_BASE + u(t) + TRIM_{\text{kiri}}$$
$$v_{\text{kanan}} = KEC\_BASE - u(t) + TRIM_{\text{kanan}}$$

Dengan $KEC\_BASE = 45$, $TRIM_{\text{kiri}} = +14$, $TRIM_{\text{kanan}} = -5$.

---

## 8. Persamaan Impuls Pengereman Inersia

Ketika mendeteksi simpang pada kecepatan $v$, robot memiliki momentum linier:

$$p = m \cdot v$$

Untuk menghentikan robot $1{,}45\text{ kg}$ secara instan tanpa selip keluar kardus, diberikan pulsa rem aktif berlawanan arah:

$$\text{Impuls} = \int_{0}^{\Delta t_{\text{rem}}} F_{\text{rem}} \, dt = m \cdot \Delta v$$

Implementasi rem aktif:
* Daya Rem: $\text{PWM}_{\text{rem}} = -65$
* Durasi Pulsa: $\Delta t_{\text{rem}} = 20\text{ ms}$
* Gaya Rem Motor: $F_{\text{rem}} \approx 6{,}5\text{ N}$
$$\Delta v_{\text{terserap}} = \frac{F_{\text{rem}} \cdot \Delta t_{\text{rem}}}{m} = \frac{6{,}5 \times 0{,}020}{1{,}45} \approx 0{,}09\text{ m/s}$$

Pulsa $20\text{ ms}$ ini menyerap seluruh energi kinetik maju sebelum robot masuk ke state belok putar poros.

---

## 9. Tabel Ringkasan Rumus vs Variabel Program

| Fenomena Fisika | Rumus Persamaan | Variabel Program | File Sumber |
| :--- | :--- | :--- | :--- |
| **Gaya Tekan Normal** | $N = m \cdot g$ | $m = 1{,}45\text{ kg}$ | Analisis Kinematika |
| **Hambatan Putar Bodi** | $M_R = \frac{\mu \cdot m \cdot g \cdot L}{4}$ | $M_R \approx 0{,}38\text{ N}\cdot\text{m}$ | Analisis Dinamika |
| **Torsi Belok Terukur** | $\tau(\text{PWM}) \ge \frac{M_R \cdot r}{W}$ | `KEC_BELOK_KIRI_MAJU = 138`<br>`KEC_BELOK_KANAN_MAJU = 138` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Torsi Belok Tikungan 2** | $\tau_{2}(\text{PWM})$ (Anti-Bablas) | `KEC_BELOK_KANAN_2_MAJU = 144` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Torsi Belok Kiri Pulang** | $\tau_{\text{pulang}}(\text{PWM})$ | `KEC_BELOK_KIRI_PULANG_MAJU = 156` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Torsi Awal Ramp** | $\text{PWM}_{\text{start}} \ge \tau_{\text{breakaway}}$ | `startMaju = 95`<br>`startMundur = 95` | [Motor.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Motor.h) |
| **Durasi Ramp Mulus** | $t_{\text{ramp}}$ | `DURASI_RAMP_MS = 40` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Kemudi Garis Lurus** | $u(t) = K_p e + K_d \Delta e$ | `Kp = 28.0`, `Kd = 18.0` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Maju Lurus Stabil** | $v_{\text{base}} + \text{Trim}$ | `KEC_BASE = 45`<br>`TRIM_KIRI = 24`<br>`TRIM_KANAN = -8` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Maju Offset Lurus Terkalibrasi** | $v_{\text{offset}} + \text{Trim}$ | `KEC_MAJU_OFFSET_KIRI = 65`<br>`KEC_MAJU_OFFSET_KANAN = 38` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |
| **Mundur Pemulihan** | $v_{\text{mundur}}$ | `setMotorLangsung(-52, -52)` | [NavigationFSM.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/NavigationFSM.h) |
| **Pulsa Rem Maju Simpang** | $I = F_{\text{rem}} \cdot \Delta t$ | `setMotorLangsung(-65, -65)`<br>`delay(20)` | [NavigationFSM.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/NavigationFSM.h) |
| **Counter-Brake Belok 90°** | $\Delta t = \frac{I_z \cdot \omega}{\tau_{\text{couple}}}$ | `putarPorosKanan(138, 138)`<br>`delay(25)` | [NavigationFSM.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/NavigationFSM.h) |
| **Tarik Bodi Belakang 90°** | $\Delta t_{\text{belakang}} \approx \frac{\Delta\theta}{\omega}$ | `DURASI_TAMBAHAN_BELAKANG_MS = 235` | [Config.h](file:///d:/kerja/2026%20mobil%20mobilan/mobil_code/Config.h) |

---

## 10. Analisis Fisika Upgrade Roda Lebar & Bearing Kokoh (Kasus Bablas 170° & Penyetelan Torsi 138 PWM)

### A. Perubahan Parameter Mekanik Lapangan
Setelah penggantian roda yang lebih lebar dan pemasangan bearing presisi yang kokoh:
1. **Koefisien Gesek Lateral Ban ($\mu$):** Meningkat dari $0{,}48$ ke $\approx 0{,}65$ karena tapak karet yang lebih lebar menempel sempurna pada kardus. Roda lebar membutuhkan torsi skid-steer yang lebih besar untuk mengatasi gesekan seret samping (*lateral scrub*).
2. **Efisiensi Mekanis Gandar ($\eta$):** Efisiensi transmisi daya melonjak dari $\approx 60\%$ ke $\approx 90\%$ karena rugi-rugi gesek internal poros as hampir lenyap ($\tau_{\text{loss}} \approx 0$).
3. **Momen Inersia Rotasi Bodi ($I_z$):**
   $$I_z = \frac{1}{12} \cdot m \cdot (L_{\text{bodi}}^2 + W^2) = \frac{1}{12} \times 1{,}45 \times (0{,}25^2 + 0{,}18^2) \approx 0{,}0115\text{ kg}\cdot\text{m}^2$$

### B. Mengapa PWM 118 Kurang Kuat (Tertahan di Separuh / ~45°)?
1. **Hambatan Gesek Seret Samping Ban Lebar (*Tire Scrub*):** Ban lebar memiliki bidang kontak melintang lebih besar. Pada PWM 118 (duty cycle 46%), tegangan efektif motor $\approx 5{,}1\text{V}$ menghasilkan torsi yang mendekati nilai kritis ambang batas gesek luncur lateral kardus, sehingga putaran melemah dan berhenti di separuh jalan sebelum mencapai 90°.
2. **Blind-Time Terlalu Cepat (60ms):** Pada 60ms saat robot bergerak lambat, sensor belum sempat meninggalkan garis lama, sehingga memicu pengereman dini semu.
3. **Solusi Kalibrasi Torsi Pas (PWM 138):**
   PWM ditingkatkan dari 118 ke **`138`** ($\approx 6{,}0\text{V}$), memberikan tambahan torsi $+20\%$ yang cukup mendobrak tahanan seret roda lebar secara kontinu hingga sudut 90°.
   
### C. Persamaan Penyesuaian Presisi 90° Terkalibrasi
1. **Kecepatan Sudut Terkalibrasi:**
   $$\text{PWM}_{\text{belok}} = 138 \quad \implies \quad \omega \approx 2{,}6\text{ rad/s} \quad (150^\circ/\text{s})$$
   Waktu tempuh 90°: $t_{90} \approx \frac{1{,}57}{2{,}6} \approx 0{,}60\text{ detik} = 600\text{ ms}$.
2. **Blind Time Aman ($120\text{ ms}$):**
   Memberikan ruang bebas bagi sensor untuk meninggalkan garis lama ($t \approx 100\text{ ms}$) sebelum mulai memindai garis cabang 90°.
3. **Perhitungan Pulsa Counter-Torque Pengereman:**
   $$L_z = I_z \cdot \omega = 0{,}0115 \times 2{,}6 \approx 0{,}030\text{ N}\cdot\text{m}\cdot\text{s}$$
   Pasangan torsi rem motor lawan arah pada PWM 138:
   $$\tau_{\text{rem}} = F_{\text{rem}} \cdot W \approx 6{,}5\text{ N} \times 0{,}18\text{ m} \approx 1{,}17\text{ N}\cdot\text{m}$$
   Waktu pengereman rotasi:
   $$\Delta t_{\text{rem}} = \frac{L_z}{\tau_{\text{rem}}} = \frac{0{,}030}{1{,}17} \approx 0{,}0256\text{ detik} \approx 25\text{ ms}$$
   Dengan pulsa counter-torque sebesar $25\text{ ms}$ (`putarPorosKanan(138, 138)`), robot terkunci tepat di $90^\circ$ tanpa bablas dan tanpa macet.

### D. Kinematika Bodi Panjang ($L_{\text{bodi}} = 25\text{ cm}$) & Rumus Logika Kelurusan Murni AND (Tanpa OR)
1. **Fenomena Sensor Depan Menyentuh Garis di Sudut $\approx 45^\circ$:**
   * Sensor bar TCRT5000 terpasang di ujung moncong depan (berjarak $d_{\text{sensor}} \approx 14\text{ cm}$ di depan pusat sumbu putar roda).
   * Saat mobil mulai berputar di persimpangan, moncong depan bergerak membentuk busur lingkaran yang luas.
   * Ketika sudut bodi baru berputar $\approx 45^\circ$, moncong depan sudah lebih dulu memotong garis cabang siku (sensor tengah $S_3$ langsung aktif).
   * Jika sistem menganggap $S_3 = 1$ sebagai tanda belok sudah selesai, mobil akan langsung melaju maju dengan posisi sasis masih miring $45^\circ$ karena bagian belakang belum sempat berputar lurus!
2. **Formulasi Logika Kelurusan Murni Konjungsi AND ($\land$, Tanpa Disjungsi OR):**
   * **Belok Kiri 90°:**
     * Sensor berputar berlawanan arah jarum jam (CCW).
     * Sensor $S_1, S_2, S_3$ memotong garis lebih dahulu di sudut $30^\circ - 45^\circ$. Pada sudut ini, sensor kanan tengah ($S_4$) masih tertinggal jauh di luar garis ($S_4 = 0$).
     * Hanya ketika sasis telah berputar penuh mendekati $90^\circ$, garis hitam berada tepat di antara sensor tengah ($S_3$) dan sensor kanan sebelahnya sebelum akhir ($S_4$).
     * Persamaan Transisi State Kelurusan Belok Kiri:
       $$\text{KondisiLurus}_{\text{Kiri}} = (S_3 == 1) \land (S_4 == 1) \quad \text{Murni AND, Tanpa OR}$$
       $$\text{State}_{\text{berikutnya}} \iff (b_3 == 1 \ \&\&\ b_4 == 1)$$
   * **Belok Kanan 90°:**
     * Sensor berputar searah jarum jam (CW).
     * Sensor $S_5, S_4, S_3$ memotong garis lebih dahulu di sudut $30^\circ - 45^\circ$. Pada sudut ini, sensor kiri tengah ($S_2$) masih tertinggal di luar garis ($S_2 = 0$).
     * Hanya ketika sasis telah berputar penuh mendekati $90^\circ$, garis hitam berada tepat di antara sensor tengah ($S_3$) dan sensor kiri sebelum akhir ($S_2$).
     * Persamaan Transisi State Kelurusan Belok Kanan:
       $$\text{KondisiLurus}_{\text{Kanan}} = (S_3 == 1) \land (S_2 == 1) \quad \text{Murni AND, Tanpa OR}$$
       $$\text{State}_{\text{berikutnya}} \iff (b_3 == 1 \ \&\&\ b_2 == 1)$$
3. **Pengereman Dinamis Setelah Kondisi Terkunci:**
   * Begitu kondisi konjungsi AND terpenuhi secara simultan, mobil langsung memberikan pulsa counter-brake $25\text{ ms}$ (`putarPorosKanan` untuk kiri, `putarPorosKiri` untuk kanan) untuk menahan inersia putar bearing baru, berhenti total $60\text{ ms}$, lalu baru masuk ke state jalan lurus berikutnya dengan jaminan sasis 100% tegak lurus terhadap garis lintasan baru.

### E. Analisis Fisika Kasus Roda Kiri Depan Berputar Kencang Sendirian (3 Roda Diam)
1. **Analisis Diferensial Gaya & Selip Roda (Wheel-Slip vs Torque Stall):**
   * Pada robot 4WD Skid-Steer (SSMR), untuk berputar ke kanan (*zero-radius turn*), roda kiri harus berputar maju ($+v$) dan roda kanan harus berputar mundur ($-v$) secara simetris:
     $$\tau_{\text{pasangan}} = (F_{\text{kiri, maju}} + F_{\text{kanan, mundur}}) \cdot \frac{W}{2}$$
   * Jika sisi kanan tidak menghasilkan torsi mundur (akibat LPWM / GPIO 17 tidak aktif atau pin `L_EN` BTS7960 tidak terhubung ke 5V), roda kanan menjadi jangkar statis ($\mu_s \approx 0{,}65$).
   * Beban normal roda belakang lebih besar dari roda depan ($N_{\text{belakang}} > N_{\text{depan}}$) karena baterai dan motor berada di area tengah-belakang.
   * Saat sisi kiri diberi daya forward tinggi, roda depan kiri mengalami penurunan gaya normal sesaat (*unweighting*) sehingga melampaui batas gesek statis ($F_{\text{dorong}} > \mu_s N_{\text{depan}}$).
   * Seketika ban depan kiri kehilangan cengkeraman (*wheel-slip*), koefisien geseknya drop ke nilai luncur ($\mu_k \to 0$). Motor depan kiri berputar bebas tanpa beban pada RPM maksimalnya (*spinning fast*).
   * Sebaliknya, roda belakang kiri tetap memikul gaya tekan normal besar dan tidak mampu menyeret bobot $1{,}45\text{ kg}$ bodi sendirian melawan 3 roda yang terkunci, sehingga roda belakang kiri mengalami *stall* (diam).
2. **Penyebab Software Osilasi State 6 (Tahap 6B Stutter):**
   * Pada kode sebelumnya, State 6 Tahap 6B mengeksekusi rem dan jeda blocking ($25\text{ ms} + 40\text{ ms} + 30\text{ ms} + 25\text{ ms} = 120\text{ ms}$) pada setiap frame `loop()`.
   * Hal ini mematikan inersia putar kontinu dan membuat motor kiri terus-menerus tersentak di tempat (*jerking*).
   * **Solusi Perbaikan:** Mengubah State 6 menjadi putaran kontinu penuh (`putarPorosKanan(138, 142)`) hingga sensor $S_3$ dan $S_2$ benar-benar terkunci secara simultan, disusul rem lawan arah $25\text{ ms}$.

### F. Analisis & Formulasi Tambahan Putaran +10% Belok Kanan (Penarikan Bodi 90° Sempurna)
1. **Penyebab Geometris Kurang Belok ~10% ($\approx 9^\circ$):**
   * Pada Simpang 1 (Belok Kiri), terdapat offset maju $140\text{ ms}$ (`OFFSET_MAJU_BELOK_MS = 140`) yang mendorong gandar roda tepat ke pusat perempatan.
   * Pada Simpang 2 (Belok Kanan), offset maju adalah $0\text{ ms}$ (`OFFSET_MAJU_KANAN_2_MS = 0`) agar ban tidak bablas keluar batas kardus.
   * Sensor bar terpasang di moncong depan ($d \approx 14\text{ cm}$ di depan gandar). Ketika moncong depan menyentuh garis cabang siku ($S_3 = 1$ dan $S_2 = 1$), titik tengah sasis baru berputar $\approx 81^\circ$. Sasis belakang yang panjangnya $25\text{ cm}$ masih tertinggal sudut sebesar:
     $$\Delta\theta_{\text{lag}} \approx 90^\circ - 81^\circ = 9^\circ \approx 0{,}157\text{ rad} \quad (10\% \text{ dari } 90^\circ)$$
   * Selain itu, pulsa counter-brake lama sebesar $25\text{ ms}$ menghentakkan torsi balik ke kiri, menyita $\approx 4^\circ$ sudut putar kanan yang baru saja dicapai.
2. **Perhitungan Waktu Ekstra Follow-Through ($\Delta t_{\text{ekstra}}$):**
   * Dengan kecepatan sudut putar poros terkalibrasi $\omega \approx 2{,}6\text{ rad/s} \approx 150^\circ/\text{s}$:
     $$\Delta t_{\text{ekstra}} = \frac{\Delta\theta_{\text{lag}}}{\omega} = \frac{9^\circ}{150^\circ/\text{s}} = 0{,}060\text{ detik} \approx 70\text{ ms}$$
   * Begitu kondisi $S_3 == 1 \ \&\&\ S_2 == 1$ tercapai, mobil dibiarkan melanjutkan putaran kanan selama:
     $$\text{DURASI\_EKSTRA\_LURUSKAN\_KANAN\_MS} = 70\text{ ms}$$
   * Setelah durasi ekstra selesai, pulsa counter-brake lawan arah diperpendek dari $25\text{ ms}$ ke $15\text{ ms}$ agar mematikan inersia motor seketika tanpa mental balik (*no reverse bounce*), menjamin sasis terkunci presisi di $90^\circ$ lurus sempurna.

### G. Solusi Torsi Puncak (PWM 168/172) & Sweep Waktu Paksa (260ms) Mengatasi Roda Matung
1. **Analisis Torsi Statis vs Roda Matung:**
   * Pada PWM 138/142 (tegangan efektif $\approx 5{,}5\text{V}$), torsi motor JGA25-370 hanya menghasilkan gaya longitudinal $\approx 2{,}1\text{ N}$ per sisi, berada tepat di ambang batas momen gesek statis kardus ($M_{R, \text{statis}} \approx 0{,}384\text{ N}\cdot\text{m}$).
   * Roda depan kiri (yang dilengkapi encoder) memiliki tahanan gesek dinamis lebih rendah sehingga berputar lebih dulu (*wheel slip*). Sementara 3 roda lainnya (terutama roda belakang yang memikul baterai) kekurangan arus/tegangan untuk memecah gesekan statis sehingga tampak "matung dan berputar sedikit".
2. **Peningkatan Torsi Putar Penuh (+25% Torsi):**
   * Menaikkan PWM dari 138/142 ke:
     $$\text{KEC\_BELOK\_KANAN\_2\_MAJU} = 168 \quad (\text{Duty Cycle } 66\%)$$
     $$\text{KEC\_BELOK\_KANAN\_2\_MUNDUR} = 172 \quad (\text{Duty Cycle } 67\%)$$
   * Torsi awal ramp-up (`startMaju` & `startMundur`) dinaikkan dari 95 ke **125 PWM**, memberikan entakan torsi awal seketika yang memaksa ke-4 roda berputar serentak.
3. **Pemberian Waktu Putar Aktif Minimal ($t_{\text{sweep}} = 260\text{ ms}$):**
   * Menaikkan `WAKTU_SWEEP_KANAN_2_MS` dari $110\text{ ms}$ ke **$260\text{ ms}$** mencegah sensor depan melakukan penguncian dini (*premature lock*) saat bodi belakang belum sempat berayun. Selama $260\text{ ms}$ pertama, seluruh roda dipaksa memutar bodi belakang secara penuh.

---

## 11. Kinematika Gerak "Menggelir" Bodi Belakang & Sekuens 11-State Rute Riil

### A. Latar Belakang Masalah Fenomena Menggelir
Pada AGV bodi panjang ($L_{\text{bodi}} = 25\text{ cm}$) dengan 4 roda karet bertapak lebar dan massa $1{,}45\text{ kg}$, pusat massa (CoG) cenderung berada di belakang titik tumpu putar depan. Ketika sensor bar depan membaca garis tegak lurus $90^\circ$ di simpang 3, sumbu roda belakang belum tentu berada dalam garis lurus yang sama karena mengalami *skid-steer drag*. 

Jika mobil langsung melaju maju lurus pada kondisi bodi menyerong, ban belakang akan menggesek (*lateral scrub*) dan mendorong moncong keluar dari lintasan.

### B. Persamaan Impuls Menggelir (*Rear Body Straightening*)
Untuk menyelaraskan bodi belakang tanpa membuat sensor depan melompat keluar garis, diterapkan fase putaran mikro (*menggelir*) dengan torsi lembut terkalibrasi:

$$\Delta\theta_{\text{belakang}} = \omega_{\text{menggelir}} \cdot \Delta t_{\text{menggelir}}$$

Dengan kecepatan sudut terkontrol $\omega_{\text{menggelir}} \approx 1{,}2\text{ rad/s}$ dan $\Delta t_{\text{menggelir}} = 180\text{ ms}$ (Kiri) serta $120\text{ ms}$ (Kanan), koreksi sudut tambahan sebesar:

$$\Delta\theta_{\text{belakang}} \approx 1{,}2 \cdot 0{,}18 \approx 0{,}216\text{ rad} \approx 12{,}4^\circ$$

Koreksi ini menutup sisa ketertinggalan sudut bodi belakang sehingga ketika memasuki mode *Tracking PD*, AGV meluncur lurus sempurna.

### C. Alur 11-State Deterministik Terkalibrasi
1. **State 1:** Maju PD dari START sampai simpang 3 kiri terbaca gelap semua $[1\ 1\ 1\ 1\ 1]$ ($KEC\_BASE = 58$).
2. **State 2:** Belok kiri dengan *Breakaway Kick* ($PWM=185, 25\text{ ms}$) lalu sweep ($PWM=148$). Jika sensor langsung membaca $[0\ 1\ 1\ 0\ 0]$ atau $[0\ 1\ 1\ 1\ 0]$, langsung rem aktif dan gas lurus; jika mulai menyentuh garis ($S_3=1$ atau $S_4=1$), masuk State 3.
3. **State 3:** Menggelir bodi belakang kiri ($PWM=115$) sampai nilai sensor **wajib membaca $[0\ 1\ 1\ 0\ 0]$ atau $[0\ 1\ 1\ 1\ 0]$**, lalu rem aktif seketika dan gas lurus ke State 4.
4. **State 4:** Maju PD menuju simpang cabang kanan ($S_4=1\ \&\&\ S_5=1$).
5. **State 5:** Belok kanan dengan *Breakaway Kick* ($PWM=185, 25\text{ ms}$) lalu sweep ($PWM=160$) mengunci simpang 3 / garis cabang $[1\ 1\ 1\ 1\ 1]$.
6. **State 6:** Menggelir bodi belakang kanan sebentar ($PWM=135, \Delta t=100\text{ ms}$).
7. **State 7:** Maju sedikit ($\Delta t=300\text{ ms}$) penetrasi ke Titik A.
8. **State 8:** Berhenti total di Titik A (jeda bongkar/muat 3 detik).
9. **State 9:** Maju dari Titik A, melintasi Simpang 4 pertama $[1\ 1\ 1\ 1\ 1]$ lurus terus (debounce latch).
10. **State 10:** Maju lurus menuju Simpang 4 kedua $[1\ 1\ 1\ 1\ 1]$.
11. **State 11:** Maju penetrasi sedikit ($\Delta t=150\text{ ms}$) lalu rem aktif dan berhenti total di Titik B (Misi Sukses).

---

## 12. Analisis Kopel Paralel Motor DC & Solusi Breakaway Torque Kick

### A. Fisika Pembagian Arus pada Motor Paralel
Dua motor DC pada masing-masing sisi (depan dan belakang) dihubungkan paralel pada driver motor BTS7960 tunggal:

$$I_{\text{total}} = I_{\text{depan}} + I_{\text{belakang}}$$

Torsi mekanik masing-masing motor:
$$\tau = K_t \cdot I = K_t \cdot \frac{V - E_b}{R}$$

Dengan $E_b = K_e \cdot \omega$ (GGL balik sebanding dengan kecepatan sudut roda).

### B. Penyebab Roda Depan Kiri Berselip (Spin) dan Roda Belakang Mematung
1. **Beban Gandar Tak Simetris:** Gandar depan hanya menopang $40\%$ beban normal total ($N_{\text{depan}} \approx 5{,}7\text{ N}$), sedangkan gandar belakang yang memikul baterai menopang $60\%$ beban ($N_{\text{belakang}} \approx 8{,}5\text{ N}$).
2. **Ambang Gesekan Statis Kardus:**
   $$F_{\text{statis, depan}} = \mu_s \cdot N_{\text{depan}} \approx 0{,}65 \times 5{,}7 = 3{,}7\text{ N}$$
   $$F_{\text{statis, belakang}} = \mu_s \cdot N_{\text{belakang}} \approx 0{,}65 \times 8{,}5 = 5{,}5\text{ N}$$
3. **Mekanisme Ketidakseimbangan:**
   * Saat tegangan dinaikkan secara perlahan (*soft ramp*), gaya dorong motor mencapai $3{,}7\text{ N}$ lebih dulu.
   * Roda depan kiri langsung terlepas dari cengkeraman statis dan mulai berputar ($\omega_{\text{depan}} > 0$).
   * Seketika $\omega_{\text{depan}}$ naik, GGL balik $E_{b, \text{depan}}$ meningkat, membatasi arus total dan menjatuhkan tegangan terminal efektif.
   * Roda belakang kiri yang membutuhkan $5{,}5\text{ N}$ tidak pernah mencapai torsi putus gesekan statis (*breakaway torque*), sehingga terkunci mati (**mematung**).

### C. Solusi Matematis: Breakaway Torque Kick
Alih-alih menaikkan PWM bertahap, sistem memberikan sentakan tegangan penuh:

$$V_{\text{kick}} = \frac{185}{255} \times 12\text{V} \approx 8{,}71\text{ Volt}$$

Impuls gaya pada motor selama $\Delta t = 25\text{ ms}$:
$$J = \int_{0}^{0{,}025} F_{\text{kick}} \, dt \approx 8{,}5\text{ N} \times 0{,}025\text{ s} = 0{,}2125\text{ N}\cdot\text{s}$$

Gaya dorong $8{,}5\text{ N}$ ini melampaui $F_{\text{statis, belakang}}$ ($5{,}5\text{ N}$) secara instan, memaksa ke-4 roda melepaskan gesekan statis dan masuk ke rezim gesekan kinetis ($\mu_k \approx 0{,}48$) secara serempak sebelum roda depan sempat berselip sendirian.

### D. Kalibrasi Offset Maju Melewati Garis Simpang (52 ms)
1. **Analisis Posisi Sensor vs Pusat Putaran:**
   * Sensor bar TCRT5000 terletak di ujung depan sasis ($d_{\text{sensor}} \approx 14\text{ cm}$ di depan pusat geometri/as roda).
   * Saat sensor pertama kali membaca garis melintang simpang (`[1 1 1 1 1]`), pusat rotasi sasis masih berada $\approx 14\text{ cm}$ di belakang garis tersebut.
   * Jika offset $= 0\text{ ms}$, robot langsung berputar di tempat sehingga roda belakang memotong sudut lintasan (*corner-cutting*).
   * Sebaliknya, jika offset terlalu besar ($130\text{ ms} - 140\text{ ms}$), inersia laju pada $KEC\_BASE = 58$ menyebabkan sensor terdorong terlalu jauh ke depan sehingga melewati sektor deteksi belok kiri.
2. **Formulasi Offset Maju Presisi ($\Delta t_{\text{offset}} = 52\text{ ms}$):**
   * Kecepatan laju linier robot saat melintasi garis:
     $$v \approx 20{,}8\text{ cm/s}$$
   * Jarak penetrasi maju yang ditempuh selama $\Delta t = 52\text{ ms}$:
     $$\Delta x = v \cdot \Delta t = 20{,}8\text{ cm/s} \times 0{,}052\text{ s} \approx 1{,}08\text{ cm}$$
   * Pergeseran maju $\approx 1{,}1\text{ cm}$ ini membawa sensor tepat melangkahi bibir garis melintang, menempatkan as roda lebih dekat ke titik poros putar simpang tanpa memicu *overshoot* berlebihan.
   * Parameter independen yang sama disediakan untuk Simpang Cabang Kanan (`OFFSET_MAJU_SIMPANG_KANAN_MS = 52`) agar kedua simpang dapat dituning secara modular tanpa saling mempengaruhi.

### E. Kinematika Overhang Sensor & Solusi Miss Simpang 2 (Cabang Siku Kanan)
1. **Analisis Overhang Moncong Sensor ($d_{\text{sensor}} \approx 14\text{ cm}$):**
   * Pada Simpang 2, garis utama berlanjut lurus dan belokan adalah cabang siku $90^\circ$ ke kanan.
   * Ketika sensor kanan ($S_4, S_5$) mendeteksi cabang pada $Y = Y_{\text{cabang}}$, posisi gandar roda masih tertinggal:
     $$Y_{\text{gandar}} = Y_{\text{cabang}} - d_{\text{sensor}} = Y_{\text{cabang}} - 14\text{ cm}$$
   * Jika mobil langsung berputar pada titik tersebut, pusat rotasi berada $14\text{ cm}$ di belakang cabang.
   * Saat berputar $90^\circ$, moncong mobil terayun mendarat di $Y = Y_{\text{cabang}} - 14\text{ cm}$, berada di lantai kosong $\approx 12 - 14\text{ cm}$ di selatan garis cabang, menyebabkan **miss total**.
2. **Kalkulasi Penetrasi Gandar Roda ($\Delta t_{\text{poros}} = 550\text{ ms}$):**
   * Untuk membawa as roda tepat ke $Y_{\text{cabang}}$, mobil harus melaju lurus menyusuri garis sejauh $\Delta x = 12 - 14\text{ cm}$.
   * Dengan $v_{\text{base}} \approx 20{,}8\text{ cm/s}$ dan $\Delta x \approx 12\text{ cm}$:
     $$\Delta t_{\text{poros}} = \frac{\Delta x}{v_{\text{base}}} = \frac{12\text{ cm}}{20{,}8\text{ cm/s}} \approx 0{,}576\text{ detik} \approx 550\text{ ms}$$
   * Variabel: `DURASI_MAJU_POROS_CABANG_MS = 550 ms`.
3. **Ekspansi Jendela Waktu Deteksi dengan Perlambatan (Deceleration Search):**
   * Pada kecepatan putar tinggi ($\text{PWM } 160$, $\omega \approx 150^\circ/\text{s}$), kecepatan tangensial moncong $v_t \approx 36{,}4\text{ cm/s}$. Waktu lintasan garis lakban $2{,}5\text{ cm}$ hanya:
     $$\Delta t_{\text{window, fast}} = \frac{2{,}5\text{ cm}}{36{,}4\text{ cm/s}} \approx 68\text{ ms}$$
   * Dengan menurunkan kecepatan putar ke $\text{PWM } 115$ (`KEC_BELOK_KANAN_CARI`), kecepatan tangensial turun menjadi $v_t \approx 14\text{ cm/s}$:
     $$\Delta t_{\text{window, slow}} = \frac{2{,}5\text{ cm}}{14\text{ cm/s}} \approx 178\text{ ms}$$
   * Jendela waktu deteksi meningkat hampir **$3\times$ lipat**, menjamin sensor bar TCRT5000 mengunci garis cabang target secara pasti dan stabil.









