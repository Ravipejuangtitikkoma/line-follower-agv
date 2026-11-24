#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
#include "Config.h"
#include "Sensor.h"
#include "RoutePlans.h"

// ====================================================================
// MODUL PENYIMPANAN PERMANEN FLASH ESP32 (NVS PREFERENCES)
// File: RouteStorage.h
// Deskripsi: Menyimpan & memuat ruteAktif[] + parameter kalibrasi
//            ke memori non-volatile ESP32 agar mapping baru tetap
//            bertahan saat kabel USB dicabut / robot di-restart.
// ====================================================================

const uint32_t ROUTE_STORAGE_MAGIC   = 0x41475632UL; // "AGV2"
const uint16_t ROUTE_STORAGE_VERSION = 4;

struct RouteStorageBlob {
  uint32_t      magic;
  uint16_t      version;
  uint16_t      jumlahStep;
  char          namaRute[32];
  float         ticksPerPutaran;
  unsigned long jedaStasiunMs;
  bool          pulangOtomatis;
  bool          autoStart;
  PlanStep      steps[MAX_PLAN_STEPS];
};

// Simpan mapping & kalibrasi aktif ke Flash ESP32 (NVS)
inline bool simpanMappingKeFlash() {
  RouteStorageBlob blob;
  memset(&blob, 0, sizeof(blob));

  blob.magic           = ROUTE_STORAGE_MAGIC;
  blob.version         = ROUTE_STORAGE_VERSION;
  blob.jumlahStep      = (uint16_t)constrain(jumlahStepAktif, 0, MAX_PLAN_STEPS);
  blob.ticksPerPutaran = runtimeTicksPerPutaran;
  blob.jedaStasiunMs   = runtimeJedaStasiunMs;
  blob.pulangOtomatis  = runtimePulangOtomatis;
  blob.autoStart       = runtimeAutoStart;
  strncpy(blob.namaRute, namaRuteAktif, sizeof(blob.namaRute) - 1);

  for (int i = 0; i < (int)blob.jumlahStep; i++) {
    blob.steps[i] = ruteAktif[i];
  }

  Preferences prefs;
  if (!prefs.begin("agv_map", false)) {
    Serial.println("[FLASH NVS] GAGAL membuka partisi penyimpanan NVS!");
    return false;
  }

  size_t written = prefs.putBytes("blob", &blob, sizeof(blob));
  prefs.end();

  if (written != sizeof(blob)) {
    Serial.println("[FLASH NVS] GAGAL menulis data mapping ke Flash ESP32!");
    return false;
  }

  Serial.print(">>> [FLASH NVS] Mapping '");
  Serial.print(blob.namaRute);
  Serial.print("' (");
  Serial.print(blob.jumlahStep);
  Serial.println(" step) berhasil DISIMPAN permanen ke Flash ESP32!");
  return true;
}

// Muat mapping & kalibrasi dari Flash ESP32 saat boot (fallback ke default jika kosong)
inline bool muatMappingDariFlash() {
  Preferences prefs;
  if (!prefs.begin("agv_map", true)) {
    Serial.println("[FLASH NVS] Partisi NVS belum ada -> memuat Rute Default (A -> C PP).");
    muatRuteDefaultAC(true);
    return false;
  }

  size_t len = prefs.getBytesLength("blob");
  if (len != sizeof(RouteStorageBlob)) {
    prefs.end();
    Serial.println("[FLASH NVS] Belum ada mapping tersimpan -> memuat Rute Default (A -> C PP).");
    muatRuteDefaultAC(true);
    return false;
  }

  RouteStorageBlob blob;
  memset(&blob, 0, sizeof(blob));
  prefs.getBytes("blob", &blob, sizeof(blob));
  prefs.end();

  if (blob.magic != ROUTE_STORAGE_MAGIC ||
      blob.version != ROUTE_STORAGE_VERSION ||
      blob.jumlahStep == 0 ||
      blob.jumlahStep > MAX_PLAN_STEPS ||
      blob.ticksPerPutaran <= 1.0f) {
    Serial.println("[FLASH NVS] Data NVS tidak valid/versi lama -> memuat Rute Default (A -> C PP).");
    muatRuteDefaultAC(true);
    return false;
  }

  jumlahStepAktif        = (int)blob.jumlahStep;
  runtimeTicksPerPutaran = blob.ticksPerPutaran;
  runtimeJedaStasiunMs   = blob.jedaStasiunMs;
  runtimePulangOtomatis  = blob.pulangOtomatis;
  runtimeAutoStart       = blob.autoStart;

  strncpy(namaRuteAktif, blob.namaRute, sizeof(namaRuteAktif) - 1);
  namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';

  for (int i = 0; i < jumlahStepAktif; i++) {
    ruteAktif[i] = blob.steps[i];
    ruteAktif[i].label[sizeof(ruteAktif[i].label) - 1] = '\0';
  }

  Serial.print(">>> [FLASH NVS] Berhasil memuat mapping '");
  Serial.print(namaRuteAktif);
  Serial.print("' (");
  Serial.print(jumlahStepAktif);
  Serial.println(" step) dari Flash ESP32.");
  return true;
}

// Hapus mapping tersimpan di Flash & kembalikan ke preset bawaan
inline void hapusMappingFlash() {
  Preferences prefs;
  if (prefs.begin("agv_map", false)) {
    prefs.clear();
    prefs.end();
  }
  muatRuteDefaultAC(true);
  Serial.println(">>> [FLASH NVS] Mapping custom dihapus. Kembali ke Rute Default (A -> C PP).");
}
