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
//     SF7, BW 125kHz, CR 4/6, Preamble 12, TxPower 12 dBm, SPI 1 MHz, CRC ON)
//   - Menjaga mode RX_CONTINUOUS sejati lewat pin DIO0 (GPIO 27) + auto re-arm
//   - Menerima perintah operator nirkabel (format "#seq|CMD" atau "CMD" polos)
//   - Pre-ACK instan untuk perintah uji blocking (TEST_*) & deduplikasi retry
// ====================================================================

#define AGV_LORA_BAND       915E6
#define AGV_LORA_SYNC_WORD  0x34
#define AGV_LORA_SF         7
#define AGV_LORA_BW         125E3
#define AGV_LORA_TX_POWER   12

static bool          loraAktif       = false;
static long          lastLoRaSeq     = -1;
static unsigned long lastLoRaSeqMs   = 0;
static String        lastLoRaCmd     = "";
static String        lastLoRaAck     = "";
static unsigned long lastLoRaRearmMs = 0;

// Helper nama singkat status misi untuk balasan ACK LoRa
inline const char* loraNamaStatusMisi() {
  switch (statusMisiSaatIni) {
    case MISI_STANDBY:         return sedangDijedaMisi ? "PAUSE" : "STANDBY";
    case MISI_JALAN_RUTE_REAL: return "JALAN";
    case MISI_EMERGENCY_STOP:  return "E-STOP";
    default:                   return "?";
  }
}

// Terapkan seluruh parameter radio LoRa agar sinkron 100% dengan Remote Sender
inline void konfigurasiParameterLoRaAGV() {
  LoRa.setTxPower(AGV_LORA_TX_POWER);
  LoRa.setSpreadingFactor(AGV_LORA_SF);
  LoRa.setSignalBandwidth(AGV_LORA_BW);
  LoRa.setCodingRate4(6);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(AGV_LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();
  lastLoRaRearmMs = millis();
}

// Inisialisasi LoRa internal Mappi32 secara aman (dengan retry saat boot, non-blocking bila gagal)
inline void initLoRaAGV() {
  pinMode(pinLoRa_DIO0, INPUT);
  SPI.begin(pinLoRa_SCK, pinLoRa_MISO, pinLoRa_MOSI, pinLoRa_SS);
  LoRa.setPins(pinLoRa_SS, pinLoRa_RST, pinLoRa_DIO0);
  LoRa.setSPIFrequency(1E6);

  bool ok = false;
  for (int i = 0; i < 5; i++) {
    if (LoRa.begin(AGV_LORA_BAND)) {
      ok = true;
      break;
    }
    delay(150);
  }

  if (!ok) {
    loraAktif = false;
    Serial.println(F("[LORA] GAGAL inisialisasi SX1276/RFM95! Mode Serial/Otonom tetap aktif."));
    return;
  }

  konfigurasiParameterLoRaAGV();
  loraAktif = true;
  Serial.println(F("[LORA] Radio LoRa AGV AKTIF (915.0 MHz, Sync=0x34, SF7, BW=125kHz, CR4/6, Tx=12dBm, RX_CONTINUOUS DIO0)."));
}

// Kirim paket balasan ACK lewat LoRa lalu segera kembali ke mode RX_CONTINUOUS
inline void kirimBalasanLoRa(const String& payload) {
  if (!loraAktif) return;
  delay(12);
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  LoRa.idle();
  delay(2);
  LoRa.receive();
  lastLoRaRearmMs = millis();
}

// Cek paket LoRa masuk via DIO0 (menjaga RX_CONTINUOUS); kembalikan true jika ada perintah valid
inline bool cekPerintahLoRa() {
  if (!loraAktif) return false;

  // Jangan memanggil parsePacket() saat DIO0 LOW agar radio tidak berpindah ke RX_SINGLE
  if (digitalRead(pinLoRa_DIO0) == LOW) {
    if ((statusMisiSaatIni != MISI_JALAN_RUTE_REAL || faseSaatIni == PHASE_FOLLOW) &&
        (millis() - lastLoRaRearmMs >= 5000UL)) {
      LoRa.receive();
      lastLoRaRearmMs = millis();
    }
    return false;
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) {
    LoRa.receive();
    lastLoRaRearmMs = millis();
    return false;
  }

  String raw = "";
  while (LoRa.available()) {
    raw += (char)LoRa.read();
  }
  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();
  LoRa.receive();
  lastLoRaRearmMs = millis();

  raw.trim();
  if (raw.length() == 0) return false;

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

  // Deduplikasi bila controller mengirim ulang seq + cmd yang sama karena ACK sempat hilang
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
    lastLoRaSeq   = seqNum;
    lastLoRaSeqMs = millis();
    lastLoRaCmd   = cmdUpper;
    lastLoRaAck   = ack;
    kirimBalasanLoRa(ack);
  } else if (cmdUpper.startsWith("TEST_")) {
    // Pre-ACK terlebih dahulu karena TEST_* menahan eksekusi 2-8 detik (mencegah timeout/retry palsu di Sender)
    int maxIdx = (jumlahStepAktif > 0) ? (jumlahStepAktif - 1) : 0;
    ack = "ACK:" + tagPrefix + cmdUpper + "|" + String(loraNamaStatusMisi()) +
          "|STEP:" + String(stepIdxSaatIni) + "/" + String(maxIdx);
    lastLoRaSeq   = seqNum;
    lastLoRaSeqMs = millis();
    lastLoRaCmd   = cmdUpper;
    lastLoRaAck   = ack;
    kirimBalasanLoRa(ack);
    prosesPerintah(cmd);
  } else {
    prosesPerintah(cmd);
    int maxIdx = (jumlahStepAktif > 0) ? (jumlahStepAktif - 1) : 0;
    ack = "ACK:" + tagPrefix + cmdUpper + "|" + String(loraNamaStatusMisi()) +
          "|STEP:" + String(stepIdxSaatIni) + "/" + String(maxIdx);
    lastLoRaSeq   = seqNum;
    lastLoRaSeqMs = millis();
    lastLoRaCmd   = cmdUpper;
    lastLoRaAck   = ack;
    kirimBalasanLoRa(ack);
  }

  return true;
}
