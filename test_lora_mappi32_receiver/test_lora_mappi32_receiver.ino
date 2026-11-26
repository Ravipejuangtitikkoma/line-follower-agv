// ====================================================================
// PROGRAM TEST LORA - RECEIVER / AGV ROBOT (MAPPI32 ESP32)
// File  : test_lora_mappi32_receiver.ino
// Board : Mappi32 by KMtek (LoRa Internal RFM95 / SX1276)
// Fungsi:
//   - Mengunci seluruh pin motor AGV ke LOW agar aman saat uji radio
//   - Menerima paket dari ESP32 + MRV-RFM95W (PING / perintah AGV)
//   - Mengirimkan balasan ACK + RSSI ke pengirim dan mengedipkan LED GPIO 5
//
// LIBRARY WAJIB (Arduino IDE -> Library Manager):
//   "LoRa" by Sandeep Mistry
//
// CATATAN ANTENA:
//   Pastikan antena LoRa terpasang di board Mappi32 sebelum dinyalakan!
// ====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- 1. PINOUT LORA INTERNAL MAPPI32 (RFM95 / SX1276) ---
#define LORA_SCK        14
#define LORA_MISO       12
#define LORA_MOSI       13
#define LORA_SS         15
#define LORA_RST         0
#define LORA_DIO0       27
#define LED_PIN          5   // LED onboard Mappi32

// --- 2. PARAMETER RADIO LORA (WAJIB SAMA DENGAN SENDER) ---
#define LORA_BAND       915E6   // Ubah ke 920E6 / 868E6 / 433E6 jika modul beda pita
#define LORA_SYNC_WORD  0x34
#define LORA_SF         7
#define LORA_BW         125E3
#define LORA_TX_POWER   14      // dBm (stabil terhadap saturasi jarak dekat)

// --- 3. PIN MOTOR AGV (DIKUNCI MATI DEMI KEAMANAN SAAT UJI RADIO) ---
const int pinMajuKiri    = 25;
const int pinMundurKiri  = 26;
const int pinMajuKanan   = 16;
const int pinMundurKanan = 17;

void kunciMotorAman() {
  pinMode(pinMajuKiri, OUTPUT);
  pinMode(pinMundurKiri, OUTPUT);
  pinMode(pinMajuKanan, OUTPUT);
  pinMode(pinMundurKanan, OUTPUT);

  digitalWrite(pinMajuKiri, LOW);
  digitalWrite(pinMundurKiri, LOW);
  digitalWrite(pinMajuKanan, LOW);
  digitalWrite(pinMundurKanan, LOW);
}

void kirimBalasan(const String& pesanAck) {
  delay(40);
  digitalWrite(LED_PIN, HIGH);

  Serial.print(">>> [TX ACK BALASAN] \"");
  Serial.print(pesanAck);
  Serial.println("\"");

  LoRa.beginPacket();
  LoRa.print(pesanAck);
  LoRa.endPacket();
  LoRa.receive();

  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  kunciMotorAman();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("   TEST LORA RECEIVER: MAPPI32 (INTERNAL SX1276)       ");
  Serial.println("   [AMAN] Seluruh pin motor AGV dikunci OUTPUT LOW     ");
  Serial.println("=======================================================");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6); // SPI 1 MHz stabil

  Serial.print("[INIT] Memulai LoRa internal Mappi32 pada ");
  Serial.print(LORA_BAND / 1E6, 1);
  Serial.println(" MHz...");

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[ERROR] Gagal inisialisasi LoRa internal Mappi32!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println("[OK] LoRa Mappi32 siap menerima paket! (SyncWord=0x34, SF=7, BW=125kHz, Tx=14dBm, SPI=1MHz, CRC=ON)");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String pesan = "";
    while (LoRa.available()) {
      pesan += (char)LoRa.read();
    }
    pesan.trim();

    int   rssi = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();

    digitalWrite(LED_PIN, HIGH);
    Serial.print("\n<<< [RX DITERIMA] \"");
    Serial.print(pesan);
    Serial.print("\" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.print(snr, 2);
    Serial.println(" dB");
    digitalWrite(LED_PIN, LOW);

    String balasan = "ACK:" + pesan + " [RSSI:" + String(rssi) + "dBm]";
    kirimBalasan(balasan);
  }

  delay(5);
}
