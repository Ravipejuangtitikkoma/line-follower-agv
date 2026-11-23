// ====================================================================
// PROGRAM TEST LORA - SENDER / CONTROLLER (MAPPI32 ESP32)
// Board : Mappi32 by KMtek
// Modul : Built-in LoRa RFM95 / SX1276
// Fungsi: Mengirim Perintah (START, STOP, PULANG, PING) & Menerima ACK Telemetri
// ====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- 1. PINOUT MODUL LORA INTERNAL MAPPI32 ---
#define LORA_SCK   14
#define LORA_MISO  12
#define LORA_MOSI  13
#define LORA_SS    15
#define LORA_RST   0
#define LORA_DIO0  27
#define LED_PIN    5   // LED bawaan di board Mappi32

// --- 2. FREKUENSI LORA ---
// Standar Indonesia: 915E6 / 920E6 (atau 433E6 jika modul Anda versi 433MHz)
#define LORA_BAND  915E6 

// Variabel Pewaktu
static unsigned long lastSendTime = 0;
static unsigned int packetCounter = 0;
const unsigned long INTERVAL_PING = 3000; // Kirim PING otomatis tiap 3 detik

// Fungsi Kirim Paket LoRa
void kirimPesanLoRa(String pesan) {
  digitalWrite(LED_PIN, HIGH); // Nyalakan LED tanda transmit

  Serial.print("\n[TX LORA] Mengirim: \"");
  Serial.print(pesan);
  Serial.println("\" ...");

  LoRa.beginPacket();
  LoRa.print(pesan);
  LoRa.endPacket();

  digitalWrite(LED_PIN, LOW); // Matikan LED
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("   MAPPI32 LORA TESTER: SENDER / CONTROLLER (TX)      ");
  Serial.println("=======================================================");

  // Inisialisasi SPI Mappi32 LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  Serial.print("[INIT] Memulai LoRa pada frekuensi ");
  Serial.print(LORA_BAND / 1E6, 1);
  Serial.println(" MHz...");

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("\n[ERROR] Gagal menginisialisasi modul LoRa!");
    Serial.println("Periksa koneksi hardware Mappi32 atau frekuensi modul.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  // Konfigurasi Link RF Kuat & Rendah Latensi
  LoRa.setTxPower(20);          // Daya transmisi maksimal 20 dBm (100 mW)
  LoRa.setSpreadingFactor(7);   // SF7: Transmisi cepat dan responsif
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xF3);       // Sync Word khusus jaringan private AGV

  Serial.println("[OK] Modul LoRa Berhasil Terhubung!");
  Serial.println("\n[PANDUAN SERIAL MONITOR]:");
  Serial.println("  Ketik 'START'   -> Perintah AGV Jalan (A ke C)");
  Serial.println("  Ketik 'STOP'    -> Perintah Rem Darurat (E-Stop)");
  Serial.println("  Ketik 'PULANG'  -> Perintah Pulang ke Awal");
  Serial.println("  Ketik 'PING'    -> Cek Koneksi / Link Radio");
  Serial.println("  Atau biarkan, sistem akan otomatis PING tiap 3 detik.\n");
}

void loop() {
  // 1. CEK INPUT DARI SERIAL MONITOR (Kirim Perintah Manual)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    if (input.length() > 0) {
      kirimPesanLoRa("CMD:" + input);
    }
  }

  // 2. TRANSMISI PING OTOMATIS TIAP 3 DETIK (JIKA IDLE)
  if (millis() - lastSendTime >= INTERVAL_PING) {
    lastSendTime = millis();
    packetCounter++;
    String pingMsg = "PING #" + String(packetCounter);
    kirimPesanLoRa(pingMsg);
  }

  // 3. TERIMA BALASAN / ACK DARI RECEIVER (AGV)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    digitalWrite(LED_PIN, HIGH);

    String receivedText = "";
    while (LoRa.available()) {
      receivedText += (char)LoRa.read();
    }

    Serial.print(">>> [RX BALASAN] Diterima: \"");
    Serial.print(receivedText);
    Serial.print("\" | RSSI: ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR: ");
    Serial.print(LoRa.packetSnr());
    Serial.println(" dB");

    digitalWrite(LED_PIN, LOW);
  }

  delay(10);
}
