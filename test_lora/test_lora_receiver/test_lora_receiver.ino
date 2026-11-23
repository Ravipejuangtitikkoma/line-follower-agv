// ====================================================================
// PROGRAM TEST LORA - RECEIVER / AGV ROBOT (MAPPI32 ESP32)
// Board : Mappi32 by KMtek
// Modul : Built-in LoRa RFM95 / SX1276
// Fungsi: Menerima Perintah dari Controller & Mengirimkan Balasan (ACK Telemetri)
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
// Wajib SAMA PERSIS dengan Sender (915E6, 920E6, atau 433E6)
#define LORA_BAND  915E6 

// --- 3. PIN MOTOR ROBOT (DIKUNCI MATI/BRAKE DEMI KEAMANAN UJI COBA) ---
const int pinMajuKiri    = 25; // IN1 Driver Kiri
const int pinMundurKiri  = 26; // IN2 Driver Kiri
const int pinMajuKanan   = 16; // IN3 Driver Kanan (Solder)
const int pinMundurKanan = 17; // IN4 Driver Kanan (Solder)

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

// Fungsi Kirim Balasan / Telemetri Balik ke Sender
void kirimBalasan(String pesanAck) {
  delay(50); // Jeda singkat sebelum transmit agar radio stabil
  digitalWrite(LED_PIN, HIGH);

  Serial.print("<<< [TX ACK BALASAN] Mengirim: \"");
  Serial.print(pesanAck);
  Serial.println("\" ...");

  LoRa.beginPacket();
  LoRa.print(pesanAck);
  LoRa.endPacket();

  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Kunci semua pin motor robot agar tidak berputar liar saat uji coba radio
  kunciMotorAman();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("   MAPPI32 LORA TESTER: RECEIVER / AGV ROBOT (RX)     ");
  Serial.println("   [STATUS] Motor Dikunci Aman (Brake/Off)            ");
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

  // Konfigurasi Link RF (Harus Sama dengan Sender)
  LoRa.setTxPower(20);          // Daya transmisi maksimal 20 dBm
  LoRa.setSpreadingFactor(7);   // SF7: Transmisi cepat dan responsif
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xF3);       // Sync Word khusus jaringan private AGV

  Serial.println("[OK] Modul LoRa Berhasil Terhubung & Siap Menerima!");
  Serial.println("[LISTENING] Menunggu paket data dari Controller Mappi32...\n");
}

void loop() {
  // CEK APAKAH ADA PAKET LORA YANG MASUK
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Kedipkan LED tanda menerima paket
    digitalWrite(LED_PIN, HIGH);

    String receivedText = "";
    while (LoRa.available()) {
      receivedText += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    Serial.print(">>> [RX MASUK] Data: \"");
    Serial.print(receivedText);
    Serial.print("\" | Panjang: ");
    Serial.print(packetSize);
    Serial.print(" bytes | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.print(snr);
    Serial.println(" dB");

    // PARSING PERINTAH YANG DITERIMA:
    if (receivedText.startsWith("CMD:")) {
      String perintah = receivedText.substring(4);
      perintah.trim();

      Serial.print("    [EKSEKUSI PERINTAH] -> ");
      if (perintah == "START") {
        Serial.println("PERINTAH START DITERIMA -> AGV MEMULAI MISI!");
        kirimBalasan("ACK:START [AGV EXECUTING MISSION A->C]");
      } 
      else if (perintah == "STOP") {
        Serial.println("PERINTAH STOP DITERIMA -> AGV EMERGENCY STOP!");
        kirimBalasan("ACK:STOP [AGV STOPPED SAFELY]");
      } 
      else if (perintah == "PULANG") {
        Serial.println("PERINTAH PULANG DITERIMA -> AGV RETURN TO BASE!");
        kirimBalasan("ACK:PULANG [AGV RETURNING]");
      }
      else {
        Serial.print("PERINTAH KHUSUS: ");
        Serial.println(perintah);
        kirimBalasan("ACK:" + perintah + " [OK]");
      }
    } 
    else if (receivedText.startsWith("PING")) {
      // Balas Ping dengan ACK dan kualitas sinyal
      String ackMsg = "ACK:" + receivedText + " [RSSI:" + String(rssi) + "dBm]";
      kirimBalasan(ackMsg);
    }
    else {
      // Paket umum
      kirimBalasan("ACK:OK [TERIMA]");
    }

    digitalWrite(LED_PIN, LOW);
  }

  delay(10);
}
