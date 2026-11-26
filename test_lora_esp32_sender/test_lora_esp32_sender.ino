// ====================================================================
// REMOTE CONTROLLER LORA AGV (ESP32 + MODUL MRV-RFM95W)
// File  : test_lora_esp32_sender.ino
// Board : ESP32 Dev Module + Modul Eksternal MRV-RFM95W
// Fungsi:
//   - Mengirim perintah AGV ke Mappi32 dengan format #<seq>|<cmd>
//   - Auto-retry hingga 3x (timeout 500ms/percobaan) sampai ACK diterima
//   - Auto-PING off secara default (ketik 'AUTO' untuk nyalakan/matikan)
//
// LIBRARY WAJIB (Arduino IDE -> Library Manager):
//   "LoRa" by Sandeep Mistry
//
// WIRING MRV-RFM95W -> ESP32 (VSPI + PIN AMAN NON-STRAPPING):
//   VIN  -> 5V / VIN ESP32 (WAJIB 5V agar regulator onboard modul tidak drop saat TX)
//   GND  -> GND
//   D0   -> GPIO 26  (DIO0 - aman saat boot/upload)
//   SCK  -> GPIO 18  (VSPI SCK)
//   MISO -> GPIO 19  (VSPI MISO)
//   MOSI -> GPIO 23  (VSPI MOSI)
//   CS   -> GPIO 5   (VSPI SS, default HIGH aman)
//   RST  -> GPIO 27  (Reset - aman saat boot/upload)
//
// CATATAN ANTENA:
//   Pastikan antena terpasang di konektor ANT1 sebelum ESP32 dinyalakan!
// ====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- 1. PINOUT ESP32 AMAN (VSPI) KE MRV-RFM95W ---
#define LORA_SCK        18
#define LORA_MISO       19
#define LORA_MOSI       23
#define LORA_SS          5
#define LORA_RST        27
#define LORA_DIO0       26

// --- 2. PARAMETER RADIO LORA (WAJIB SAMA DENGAN MAPPI32 AGV) ---
#define LORA_BAND       915E6
#define LORA_SYNC_WORD  0x34
#define LORA_SF         7
#define LORA_BW         125E3
#define LORA_TX_POWER   14      // dBm (stabil terhadap lonjakan arus TX & jarak dekat)

#define MAX_LORA_RETRY         3
#define TIMEOUT_ACK_LORA_MS  500UL

static bool          autoPingAktif   = false;
static unsigned long waktuPingAkhir  = 0;
static unsigned long seqCounter      = 0;

void kirimPaketLoRaRaw(const String& payload) {
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  LoRa.receive();
}

// Kirim perintah dengan framing #<seq>|<cmd> dan tunggu ACK yang cocok (retry maks 3x)
bool kirimPerintahAndal(const String& cmd) {
  seqCounter++;
  unsigned long seq = seqCounter;
  String payload        = "#" + String(seq) + "|" + cmd;
  String expectedPrefix = "ACK:#" + String(seq) + "|";

  for (int attempt = 1; attempt <= MAX_LORA_RETRY; attempt++) {
    if (attempt == 1) {
      Serial.print(">>> [TX LORA] \"");
    } else {
      Serial.print(">>> [RETRY ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.print(MAX_LORA_RETRY);
      Serial.print("] \"");
    }
    Serial.print(payload);
    Serial.println("\" ...");

    kirimPaketLoRaRaw(payload);

    unsigned long tStart = millis();
    while (millis() - tStart < TIMEOUT_ACK_LORA_MS) {
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        String rx = "";
        while (LoRa.available()) {
          rx += (char)LoRa.read();
        }
        rx.trim();
        int   rssi = LoRa.packetRssi();
        float snr  = LoRa.packetSnr();

        if (rx.startsWith(expectedPrefix)) {
          Serial.print("<<< [RX ACK] \"");
          Serial.print(rx);
          Serial.print("\" | RSSI: ");
          Serial.print(rssi);
          Serial.print(" dBm | SNR: ");
          Serial.print(snr, 2);
          Serial.println(" dB");
          return true;
        } else if (rx.length() > 0) {
          Serial.print("<<< [RX LAIN] \"");
          Serial.print(rx);
          Serial.print("\" | RSSI: ");
          Serial.print(rssi);
          Serial.println(" dBm");
        }
      }
      delay(5);
    }
  }

  Serial.println("[GAGAL] Tidak ada ACK dari Mappi32 setelah 3x percobaan (cek jarak/daya/antena).");
  return false;
}

void cekPaketMasukPasif() {
  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return;

  String pesan = "";
  while (LoRa.available()) {
    pesan += (char)LoRa.read();
  }
  pesan.trim();
  if (pesan.length() == 0) return;

  Serial.print("<<< [RX LORA] \"");
  Serial.print(pesan);
  Serial.print("\" | RSSI: ");
  Serial.print(LoRa.packetRssi());
  Serial.print(" dBm | SNR: ");
  Serial.print(LoRa.packetSnr(), 2);
  Serial.println(" dB");
}

void cetakPanduan() {
  Serial.println("\n================ [REMOTE LORA AGV - PERINTAH SERIAL (115200)] ================");
  Serial.println("  START        -> Mulai misi AGV dari step 0");
  Serial.println("  STOP         -> Rem darurat (E-Stop)");
  Serial.println("  STANDBY      -> Pause/jeda mobil sementara (tunggu LANJUT atau PULANG)");
  Serial.println("  LANJUT       -> Lanjutkan rute dari PAUSE/STANDBY atau bypass jeda stasiun");
  Serial.println("  PULANG       -> Mulai rute pulang ke START");
  Serial.println("  STATUS       -> Cek status misi & step aktif Mappi32");
  Serial.println("  RUTE A,C     -> Ganti rute ke A,C (loop) | RUTE A,B | RUTE E,D | RUTE B,D");
  Serial.println("  CMD:GO:A,C   -> Susun rute PP/loop & langsung jalan seketika");
  Serial.println("  MAP_SAVE     -> Simpan permanen rute aktif ke Flash Mappi32");
  Serial.println("  PING         -> Uji koneksi instan ke Mappi32");
  Serial.println("  AUTO         -> Nyalakan/matikan Auto-PING tiap 3 detik (default OFF)");
  Serial.println("  HELP         -> Tampilkan panduan ini");
  Serial.println("==============================================================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=======================================================");
  Serial.println("   REMOTE LORA CONTROLLER: ESP32 + MRV-RFM95W          ");
  Serial.println("=======================================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6); // SPI 1 MHz stabil di kabel jumper + level shifter

  Serial.print("[INIT] Memulai LoRa pada ");
  Serial.print(LORA_BAND / 1E6, 1);
  Serial.println(" MHz...");

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[ERROR] Gagal inisialisasi MRV-RFM95W!");
    Serial.println("Cek wiring: VIN->5V, GND->GND, D0->26, SCK->18, MISO->19, MOSI->23, CS->5, RST->27.");
    while (1) {
      delay(500);
    }
  }

  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println("[OK] Remote LoRa siap! (SyncWord=0x34, SF=7, BW=125kHz, Tx=14dBm, SPI=1MHz, CRC=ON)");
  cetakPanduan();
  waktuPingAkhir = millis();
}

void loop() {
  // 1. Pantau paket pasif yang mungkin datang di luar window ACK
  cekPaketMasukPasif();

  // 2. Baca perintah dari Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      String cmdUpper = cmd;
      cmdUpper.toUpperCase();

      if (cmdUpper == "HELP" || cmdUpper == "?") {
        cetakPanduan();
      } else if (cmdUpper == "AUTO") {
        autoPingAktif = !autoPingAktif;
        Serial.print("[INFO] Auto-PING sekarang: ");
        Serial.println(autoPingAktif ? "ON" : "OFF");
      } else {
        kirimPerintahAndal(cmd);
      }
      waktuPingAkhir = millis();
    }
  }

  // 3. Auto-PING berkala tiap 3 detik jika diaktifkan manual
  if (autoPingAktif && (millis() - waktuPingAkhir >= 3000)) {
    waktuPingAkhir = millis();
    kirimPerintahAndal("PING");
  }

  delay(10);
}
