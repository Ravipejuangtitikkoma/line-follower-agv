#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "Config.h"
#include "RoutePlans.h"
#include "NavigationFSM.h"

// ====================================================================
// MODUL KOMUNIKASI LORA NIRKABEL (INTERNAL MAPPI32 SX1276 / RFM95)
// File : LoRaComm.h
// Deskripsi:
//   - Menginisialisasi radio LoRa internal Mappi32 (915 MHz, SyncWord 0x34,
//     SF7, BW 125kHz, TxPower 14 dBm, SPI 1 MHz, CRC ON)
//   - Menerima perintah operator nirkabel (format "#seq|CMD" atau "CMD" polos)
//   - Deduplikasi perintah retry selama 5 detik & membalas ACK + status AGV
// ====================================================================

#define AGV_LORA_BAND       915E6
#define AGV_LORA_SYNC_WORD  0x34
#define AGV_LORA_SF         7
#define AGV_LORA_BW         125E3
#define AGV_LORA_TX_POWER   14

static bool          loraAktif     = false;
static long          lastLoRaSeq   = -1;
static unsigned long lastLoRaSeqMs = 0;
static String        lastLoRaCmd   = "";
static String        lastLoRaAck   = "";

// Helper nama singkat status misi untuk balasan ACK LoRa
inline const char* loraNamaStatusMisi() {
  switch (statusMisiSaatIni) {
    case MISI_STANDBY:         return sedangDijedaMisi ? "PAUSE" : "STANDBY";
    case MISI_JALAN_RUTE_REAL: return "JALAN";
    case MISI_EMERGENCY_STOP:  return "E-STOP";
    default:                   return "?";
  }
}

// Inisialisasi LoRa internal Mappi32 secara aman (non-blocking bila gagal)
inline void initLoRaAGV() {
  SPI.begin(pinLoRa_SCK, pinLoRa_MISO, pinLoRa_MOSI, pinLoRa_SS);
  LoRa.setPins(pinLoRa_SS, pinLoRa_RST, pinLoRa_DIO0);
  LoRa.setSPIFrequency(1E6);

  if (!LoRa.begin(AGV_LORA_BAND)) {
    loraAktif = false;
    Serial.println(F("[LORA] GAGAL inisialisasi SX1276/RFM95! Mode Serial/Otonom tetap aktif."));
    return;
  }

  LoRa.setTxPower(AGV_LORA_TX_POWER);
  LoRa.setSpreadingFactor(AGV_LORA_SF);
  LoRa.setSignalBandwidth(AGV_LORA_BW);
  LoRa.setSyncWord(AGV_LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();

  loraAktif = true;
  Serial.println(F("[LORA] Radio LoRa AGV AKTIF (915.0 MHz, Sync=0x34, SF7, BW=125kHz, Tx=14dBm, SPI=1MHz, CRC=ON)."));
}

// Kirim paket balasan ACK lewat LoRa lalu segera kembali ke mode RX
inline void kirimBalasanLoRa(const String& payload) {
  if (!loraAktif) return;
  delay(20);
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  LoRa.receive();
}

// Cek paket LoRa masuk; kembalikan true jika ada perintah valid yang diproses
inline bool cekPerintahLoRa() {
  if (!loraAktif) return false;

  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return false;

  String raw = "";
  while (LoRa.available()) {
    raw += (char)LoRa.read();
  }
  raw.trim();
  if (raw.length() == 0) return false;

  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();

  long   seqNum    = -1;
  String tagPrefix = "";
  String cmd       = raw;

  int sep = raw.indexOf('|');
  if (raw.startsWith("#") && sep > 1) {
    String seqStr = raw.substring(1, sep);
    seqStr.trim();
    if (seqStr.length() > 0) {
      seqNum    = seqStr.toInt();
      tagPrefix = "#" + String(seqNum) + "|";
      cmd       = raw.substring(sep + 1);
      cmd.trim();
    }
  }

  if (cmd.length() == 0) return false;

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  // Deduplikasi bila controller mengirim ulang seq yang sama karena ACK sempat hilang
  if (seqNum >= 0 && seqNum == lastLoRaSeq && cmdUpper == lastLoRaCmd && (millis() - lastLoRaSeqMs < 5000UL) && lastLoRaAck.length() > 0) {
    Serial.print("[LORA RX RETRY] \"");
    Serial.print(raw);
    Serial.print("\" (RSSI:"); Serial.print(rssi);
    Serial.println("dBm) -> Kirim ulang ACK cache.");
    kirimBalasanLoRa(lastLoRaAck);
    return true;
  }

  Serial.print("\n[LORA RX] \"");
  Serial.print(raw);
  Serial.print("\" | RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm | SNR: ");
  Serial.print(snr, 2);
  Serial.println(" dB");

  String ack = "";

  if (cmdUpper == "PING") {
    int maxIdx = (jumlahStepAktif > 0) ? (jumlahStepAktif - 1) : 0;
    ack = "ACK:" + tagPrefix + "PONG|" + String(loraNamaStatusMisi()) +
          "|" + String(namaRuteAktif) + "|STEP:" + String(stepIdxSaatIni) + "/" + String(maxIdx);
  } else {
    prosesPerintah(cmd);
    int maxIdx = (jumlahStepAktif > 0) ? (jumlahStepAktif - 1) : 0;
    ack = "ACK:" + tagPrefix + cmdUpper + "|" + String(loraNamaStatusMisi()) +
          "|STEP:" + String(stepIdxSaatIni) + "/" + String(maxIdx);
  }

  lastLoRaSeq   = seqNum;
  lastLoRaSeqMs = millis();
  lastLoRaCmd   = cmdUpper;
  lastLoRaAck   = ack;

  kirimBalasanLoRa(ack);
  return true;
}
