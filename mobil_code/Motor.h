#pragma once
#include <Arduino.h>
#include "Config.h"

// ====================================================================
// MODUL KONTROL PENGGERAK MOTOR (BI-DIRECTIONAL SMOOTH & HIGH TORQUE PIVOT)
// File: Motor.h
// ====================================================================

static float smoothSpeedKiri  = 0.0;
static float smoothSpeedKanan = 0.0;

// Inisialisasi Pin Motor
inline void initMotor() {
  pinMode(pinMajuKiri, OUTPUT);
  pinMode(pinMundurKiri, OUTPUT);
  pinMode(pinMajuKanan, OUTPUT);
  pinMode(pinMundurKanan, OUTPUT);

  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, 0);
}

// Berhenti Total Seketika (Brake)
inline void berhentiTotal() {
  smoothSpeedKiri  = 0;
  smoothSpeedKanan = 0;
  analogWrite(pinMajuKiri, 0);
  analogWrite(pinMundurKiri, 0);
  analogWrite(pinMajuKanan, 0);
  analogWrite(pinMundurKanan, 0);
}

// Set Kecepatan Motor Dua Roda dengan Filter Eksponensial Halus (Untuk Line Tracking)
inline void setKecepatanMotor(int targetKiri, int targetKanan) {
  smoothSpeedKiri  = (smoothSpeedKiri  * (1.0 - ALPHA_SMOOTH)) + (targetKiri  * ALPHA_SMOOTH);
  smoothSpeedKanan = (smoothSpeedKanan * (1.0 - ALPHA_SMOOTH)) + (targetKanan * ALPHA_SMOOTH);

  int vKiri  = (int)smoothSpeedKiri;
  int vKanan = (int)smoothSpeedKanan;

  // Kontrol Roda Kiri
  if (vKiri >= 0) {
    analogWrite(pinMajuKiri, constrain(vKiri, 0, 255));
    analogWrite(pinMundurKiri, 0);
  } else {
    analogWrite(pinMajuKiri, 0);
    analogWrite(pinMundurKiri, constrain(-vKiri, 0, 255));
  }

  // Kontrol Roda Kanan
  if (vKanan >= 0) {
    analogWrite(pinMajuKanan, constrain(vKanan, 0, 255));
    analogWrite(pinMundurKanan, 0);
  } else {
    analogWrite(pinMajuKanan, 0);
    analogWrite(pinMundurKanan, constrain(-vKanan, 0, 255));
  }
}

// Kontrol Motor Langsung Tanpa Filter (Respon Cepat & Torsi Penuh Manuver)
inline void setMotorLangsung(int vKiri, int vKanan) {
  smoothSpeedKiri  = vKiri;
  smoothSpeedKanan = vKanan;

  if (vKiri >= 0) {
    analogWrite(pinMajuKiri, constrain(vKiri, 0, 255));
    analogWrite(pinMundurKiri, 0);
  } else {
    analogWrite(pinMajuKiri, 0);
    analogWrite(pinMundurKiri, constrain(-vKiri, 0, 255));
  }

  if (vKanan >= 0) {
    analogWrite(pinMajuKanan, constrain(vKanan, 0, 255));
    analogWrite(pinMundurKanan, 0);
  } else {
    analogWrite(pinMajuKanan, 0);
    analogWrite(pinMundurKanan, constrain(-vKanan, 0, 255));
  }
}

// Putar Poros ke Kanan Bertenaga Penuh (Torsi Seimbang Sisi Kiri & Kanan)
inline void putarPorosKanan(int pwmMaju, int pwmMundur) {
  setMotorLangsung(pwmMaju, -pwmMundur);
}

// Putar Poros ke Kiri Bertenaga Penuh (Torsi Seimbang Sisi Kiri & Kanan)
inline void putarPorosKiri(int pwmMaju, int pwmMundur) {
  setMotorLangsung(-pwmMundur, pwmMaju);
}

// ====================================================================
// ALGORITMA BREAKAWAY TORQUE KICK (INSTANT OVERCOME STATIC FRICTION)
// Mengatasi Masalah Roda Depan Kiri Spin & 3 Roda Lain Mematung:
// Motor depan & belakang diparalel pada driver 2-channel. Memberikan sentakan
// tegangan instan PWM_BREAKAWAY_KICK (185 PWM) selama 25ms agar ke-4 roda
// memecahkan gesekan statis kardus (mu_s = 0.65) secara serempak.
// ====================================================================
inline void putarPorosKananHalus(int targetMaju, int targetMundur, int durasiKickMs = DURASI_BREAKAWAY_KICK_MS, int pwmKick = PWM_BREAKAWAY_KANAN_KICK) {
  // 1. Entakan Torsi Breakaway Instan ke-4 roda bergerak serentak
  setMotorLangsung(pwmKick, -pwmKick);
  delay(durasiKickMs);

  // 2. Langsung stabil ke torsi belok operasional
  setMotorLangsung(targetMaju, -targetMundur);
}

inline void putarPorosKiriHalus(int targetMaju, int targetMundur, int durasiKickMs = DURASI_BREAKAWAY_KICK_MS, int pwmKick = PWM_BREAKAWAY_KICK) {
  // 1. Entakan Torsi Breakaway Instan ke-4 roda bergerak serentak
  setMotorLangsung(-pwmKick, pwmKick);
  delay(durasiKickMs);

  // 2. Langsung stabil ke torsi belok operasional
  setMotorLangsung(-targetMundur, targetMaju);
}
