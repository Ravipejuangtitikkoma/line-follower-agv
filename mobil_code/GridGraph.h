#pragma once
#include <Arduino.h>
#include <string.h>
#include "Config.h"
#include "RoutePlans.h"

// ====================================================================
// MODUL GRAPH ROUTE PLANNER ARENA 3m x 3m (OTOMATIS STASIUN A - E)
// File: GridGraph.h
// Deskripsi: Memodelkan 23 node arena (START, N0..N16, ST_A..ST_E)
//            dan menyusun tabel PlanStep ruteAktif[] secara otomatis
//            menggunakan algoritma Dijkstra 4-arah hadap.
// ====================================================================

enum GridDir : int8_t {
  DIR_UTARA   = 0,
  DIR_TIMUR   = 1,
  DIR_SELATAN = 2,
  DIR_BARAT   = 3
};

enum GridNodeId : uint8_t {
  ND_START = 0,
  ND_N0    = 1,
  ND_N1    = 2,
  ND_N2    = 3,
  ND_N3    = 4,
  ND_N4    = 5,
  ND_N5    = 6,
  ND_N6    = 7,
  ND_N7    = 8,
  ND_N8    = 9,
  ND_N9    = 10,
  ND_N10   = 11,
  ND_N11   = 12,
  ND_N12   = 13,
  ND_N13   = 14,
  ND_N14   = 15,
  ND_N15   = 16,
  ND_N16   = 17,
  ND_ST_A  = 18,
  ND_ST_B  = 19,
  ND_ST_C  = 20,
  ND_ST_D  = 21,
  ND_ST_E  = 22
};

const int GRID_NUM_NODES  = 23;
const int GRID_NUM_DIRS   = 4;
const int GRID_NUM_STATES = GRID_NUM_NODES * GRID_NUM_DIRS; // 92 state (node x arah)

struct GridNodeInfo {
  const char* name;
  int8_t      nbr[4];   // Tetangga di arah: [UTARA, TIMUR, SELATAN, BARAT] (-1 jika tidak ada)
  uint16_t    dist[4];  // Jarak ke tetangga dalam cm
};

// Peta Topologi Persis Arena 3 m x 3 m:
//   [N1] ---100--- [N5] ---100--- [N9] ---100--- [N13]
//    |100           |100           |100           |100
//   [N2] -50-[A]-50-[N6] ---100--- [N10]-50-[C]-50-[N14]
//    |50            |100           |100           |100
//   [N0] <--30-- (START)           |              |
//    |50            |              |              |
//   [N3] ---100--- [N7] -50-[B]-50-[N11]---100--- [N15]
//    |100           |100           |100           |100
//   [N4] -50-[E]-50-[N8] ---100--- [N12]-50-[D]-50-[N16]
static const GridNodeInfo GRID_NODES[GRID_NUM_NODES] = {
  /* 0  ND_START */ { "START", { -1,      ND_N0,   -1,      -1      }, { 0,   30,  0,   0   } },
  /* 1  ND_N0    */ { "N0",    { ND_N2,   -1,      ND_N3,   ND_START}, { 50,  0,   50,  30  } },
  /* 2  ND_N1    */ { "N1",    { -1,      ND_N5,   ND_N2,   -1      }, { 0,   100, 100, 0   } },
  /* 3  ND_N2    */ { "N2",    { ND_N1,   ND_ST_A, ND_N0,   -1      }, { 100, 50,  50,  0   } },
  /* 4  ND_N3    */ { "N3",    { ND_N0,   ND_N7,   ND_N4,   -1      }, { 50,  100, 100, 0   } },
  /* 5  ND_N4    */ { "N4",    { ND_N3,   ND_ST_E, -1,      -1      }, { 100, 50,  0,   0   } },
  /* 6  ND_N5    */ { "N5",    { -1,      ND_N9,   ND_N6,   ND_N1   }, { 0,   100, 100, 100 } },
  /* 7  ND_N6    */ { "N6",    { ND_N5,   ND_N10,  ND_N7,   ND_ST_A }, { 100, 100, 100, 50  } },
  /* 8  ND_N7    */ { "N7",    { ND_N6,   ND_ST_B, ND_N8,   ND_N3   }, { 100, 50,  100, 100 } },
  /* 9  ND_N8    */ { "N8",    { ND_N7,   ND_N12,  -1,      ND_ST_E }, { 100, 100, 0,   50  } },
  /* 10 ND_N9    */ { "N9",    { -1,      ND_N13,  ND_N10,  ND_N5   }, { 0,   100, 100, 100 } },
  /* 11 ND_N10   */ { "N10",   { ND_N9,   ND_ST_C, ND_N11,  ND_N6   }, { 100, 50,  100, 100 } },
  /* 12 ND_N11   */ { "N11",   { ND_N10,  ND_N15,  ND_N12,  ND_ST_B }, { 100, 100, 100, 50  } },
  /* 13 ND_N12   */ { "N12",   { ND_N11,  ND_ST_D, -1,      ND_N8   }, { 100, 50,  0,   100 } },
  /* 14 ND_N13   */ { "N13",   { -1,      -1,      ND_N14,  ND_N9   }, { 0,   0,   100, 100 } },
  /* 15 ND_N14   */ { "N14",   { ND_N13,  -1,      ND_N15,  ND_ST_C }, { 100, 0,   100, 50  } },
  /* 16 ND_N15   */ { "N15",   { ND_N14,  -1,      ND_N16,  ND_N11  }, { 100, 0,   100, 100 } },
  /* 17 ND_N16   */ { "N16",   { ND_N15,  -1,      -1,      ND_ST_D }, { 100, 0,   0,   50  } },
  /* 18 ND_ST_A  */ { "A",     { -1,      ND_N6,   -1,      ND_N2   }, { 0,   50,  0,   50  } },
  /* 19 ND_ST_B  */ { "B",     { -1,      ND_N11,  -1,      ND_N7   }, { 0,   50,  0,   50  } },
  /* 20 ND_ST_C  */ { "C",     { -1,      ND_N14,  -1,      ND_N10  }, { 0,   50,  0,   50  } },
  /* 21 ND_ST_D  */ { "D",     { -1,      ND_N16,  -1,      ND_N12  }, { 0,   50,  0,   50  } },
  /* 22 ND_ST_E  */ { "E",     { -1,      ND_N8,   -1,      ND_N4   }, { 0,   50,  0,   50  } }
};

inline bool isNodeStasiun(int nodeId) {
  return (nodeId >= ND_ST_A && nodeId <= ND_ST_E);
}

inline int hurufKeNodeStasiun(char c) {
  switch (toupper(c)) {
    case 'A': return ND_ST_A;
    case 'B': return ND_ST_B;
    case 'C': return ND_ST_C;
    case 'D': return ND_ST_D;
    case 'E': return ND_ST_E;
    default:  return -1;
  }
}

// Dijkstra ringan atas (node, arahHadap) mencari jalur terpendek antara dua waypoint
inline bool cariJalurTerpendekGrid(int startNode,
                                   int startDir,
                                   int goalNode,
                                   int outNodes[],
                                   int outDirs[],
                                   int& outLen,
                                   int maxLen) {
  uint32_t distState[GRID_NUM_STATES];
  int16_t  prevState[GRID_NUM_STATES];
  bool     visited[GRID_NUM_STATES];

  for (int i = 0; i < GRID_NUM_STATES; i++) {
    distState[i] = 0x3FFFFFFF;
    prevState[i] = -1;
    visited[i]   = false;
  }

  int startState = startNode * GRID_NUM_DIRS + startDir;
  distState[startState] = 0;

  for (int iter = 0; iter < GRID_NUM_STATES; iter++) {
    int bestS = -1;
    uint32_t bestD = 0x3FFFFFFF;
    for (int s = 0; s < GRID_NUM_STATES; s++) {
      if (!visited[s] && distState[s] < bestD) {
        bestD = distState[s];
        bestS = s;
      }
    }
    if (bestS < 0) break;
    visited[bestS] = true;

    int u   = bestS / GRID_NUM_DIRS;
    int dIn = bestS % GRID_NUM_DIRS;
    if (u == goalNode) break;

    for (int dOut = 0; dOut < GRID_NUM_DIRS; dOut++) {
      int v = GRID_NODES[u].nbr[dOut];
      if (v < 0) continue;
      if (v == ND_START && v != goalNode) continue; // START hanya boleh jadi tujuan akhir

      uint32_t turnCost = 0;
      if (dOut == dIn) {
        turnCost = 0;
      } else if (((dIn + 2) % 4) == dOut) {
        // Putar balik 180° hanya diizinkan pada ruas lurus titik stasiun
        if (!isNodeStasiun(u)) continue;
        turnCost = 60;
      } else {
        turnCost = 25;
      }

      uint32_t newCost = distState[bestS] + GRID_NODES[u].dist[dOut] + turnCost;
      int nextState = v * GRID_NUM_DIRS + dOut;
      if (newCost < distState[nextState]) {
        distState[nextState] = newCost;
        prevState[nextState] = bestS;
      }
    }
  }

  // Cari arah kedatangan terbaik di goalNode
  int bestGoalState = -1;
  uint32_t bestGoalDist = 0x3FFFFFFF;
  for (int d = 0; d < GRID_NUM_DIRS; d++) {
    int s = goalNode * GRID_NUM_DIRS + d;
    if (distState[s] < bestGoalDist) {
      bestGoalDist  = distState[s];
      bestGoalState = s;
    }
  }
  if (bestGoalState < 0) return false;

  // Rekonstruksi mundur jalur state
  int revStates[GRID_NUM_STATES];
  int cnt = 0;
  int cur = bestGoalState;
  while (cur >= 0 && cnt < GRID_NUM_STATES) {
    revStates[cnt++] = cur;
    if (cur == startState) break;
    cur = prevState[cur];
  }
  if (cnt <= 0 || revStates[cnt - 1] != startState || cnt > maxLen) {
    return false;
  }

  outLen = 0;
  for (int i = cnt - 1; i >= 0; i--) {
    outNodes[outLen] = revStates[i] / GRID_NUM_DIRS;
    outDirs[outLen]  = revStates[i] % GRID_NUM_DIRS;
    outLen++;
  }
  return true;
}

// Susun otomatis tabel PlanStep ke ruteAktif[] dari daftar huruf stasiun (mis. "A,C", "B", "E,D")
inline bool bangunRuteStasiun(const String& daftarRaw, bool pulangKeStart) {
  const int MAX_WAYPOINTS = 8;
  int  waypoints[MAX_WAYPOINTS];
  char hurufTargets[6];
  int  numTargets = 0;

  waypoints[0] = ND_START;
  int numWp = 1;

  for (unsigned int i = 0; i < daftarRaw.length(); i++) {
    char c = daftarRaw.charAt(i);
    if (c == ' ' || c == ',' || c == '-' || c == '>' || c == ';' || c == ':') continue;
    int stNode = hurufKeNodeStasiun(c);
    if (stNode < 0) {
      Serial.print("[ERROR RUTE] Stasiun tidak dikenal '");
      Serial.print(c);
      Serial.println("'. Gunakan huruf A, B, C, D, atau E (contoh: RUTE A,C).");
      return false;
    }
    if (numTargets >= 5) {
      Serial.println("[ERROR RUTE] Maksimal 5 stasiun per misi.");
      return false;
    }
    hurufTargets[numTargets++] = (char)toupper(c);
    waypoints[numWp++] = stNode;
  }

  if (numTargets == 0) {
    Serial.println("[ERROR RUTE] Daftar stasiun kosong! Contoh: RUTE A,C atau RUTE B");
    return false;
  }

  if (pulangKeStart) {
    waypoints[numWp++] = ND_START;
  }

  PlanStep tempRute[MAX_PLAN_STEPS];
  int tempCount = 0;

  auto tambahStep = [&](const PlanStep& s) -> bool {
    if (tempCount >= MAX_PLAN_STEPS) return false;
    tempRute[tempCount++] = s;
    return true;
  };

  int curNode = ND_START;
  int curDir  = DIR_TIMUR;

  for (int w = 1; w < numWp; w++) {
    int goalNode = waypoints[w];
    bool isPulangLeg = (pulangKeStart && w == numWp - 1);

    if (goalNode == curNode) continue; // Lewati duplikat stasiun berurutan yang sama

    int pathNodes[GRID_NUM_STATES];
    int pathDirs[GRID_NUM_STATES];
    int pathLen = 0;

    if (!cariJalurTerpendekGrid(curNode, curDir, goalNode, pathNodes, pathDirs, pathLen, GRID_NUM_STATES)) {
      Serial.println("[ERROR RUTE] Gagal menghitung jalur antar stasiun.");
      return false;
    }

    // Jika segmen diawali putar balik 180° dari titik stasiun sebelumnya
    if (pathLen >= 2 && ((curDir + 2) % 4) == pathDirs[1]) {
      char lbl[20];
      snprintf(lbl, sizeof(lbl), "UTURN_%s", GRID_NODES[curNode].name);
      if (!tambahStep(buatPlanStep(lbl, TRIG_IMMEDIATE, 0, 0.0f, 0, false,
                                   0, false, 0, 0, 0, ACT_UTURN_180))) {
        Serial.println("[ERROR RUTE] Jumlah langkah melebihi kapasitas MAX_PLAN_STEPS.");
        return false;
      }
    }

    // Proses tiap simpang perantara (1 .. pathLen - 2)
    for (int k = 1; k < pathLen - 1; k++) {
      int u      = pathNodes[k];
      int dirIn  = pathDirs[k];
      int dirOut = pathDirs[k + 1];

      if (isNodeStasiun(u)) {
        continue; // Titik tengah stasiun yang sekadar dilewati bukan target berhenti
      }

      int rel = (dirOut - dirIn + 4) % 4;
      int leftDir  = (dirIn + 3) % 4;
      int rightDir = (dirIn + 1) % 4;
      bool hasLeft  = (GRID_NODES[u].nbr[leftDir]  >= 0);
      bool hasRight = (GRID_NODES[u].nbr[rightDir] >= 0);

      StepTrigger trig = TRIG_PEREMPATAN;
      if (hasLeft && hasRight) {
        trig = (rel == 3) ? TRIG_T_KIRI : TRIG_PEREMPATAN;
      } else if (hasRight && !hasLeft) {
        trig = TRIG_CABANG_KANAN;
      } else if (hasLeft && !hasRight) {
        trig = TRIG_CABANG_KIRI;
      }

      PlanStep st;
      char lbl[20];
      if (rel == 0) {
        uint16_t pauseMs = (isPulangLeg && (u == ND_N10 || u == ND_N6)) ? DURASI_STOP_S4_PULANG_MS : 0;
        uint8_t  brkPwm  = (pauseMs > 0) ? 85 : 0;
        uint16_t brkMs   = (pauseMs > 0) ? 35 : 0;
        snprintf(lbl, sizeof(lbl), "%s_%s_LURUS", isPulangLeg ? "PLG" : "LTS", GRID_NODES[u].name);
        st = buatPlanStep(lbl, trig, 350, 0.0f, 0, false, 0, false, brkPwm, brkMs, pauseMs, ACT_CROSS_STRAIGHT);
      } else if (rel == 1) {
        uint16_t lock = (trig == TRIG_CABANG_KANAN) ? WAKTU_MINIMAL_RUAS2_MS : 250;
        snprintf(lbl, sizeof(lbl), "%s_%s_KANAN", isPulangLeg ? "PLG" : "GO", GRID_NODES[u].name);
        st = buatPlanStep(lbl, trig, lock, 0.0f, 0, false,
                          OFFSET_MAJU_SIMPANG_KANAN_MS, false, 85, 35, JEDA_DIEM_SIMPANG_MS, ACT_TURN_RIGHT);
      } else { // rel == 3 (Kiri)
        StepAction act = isPulangLeg ? ACT_TURN_LEFT_PULANG : ACT_TURN_LEFT;
        uint16_t lock  = isPulangLeg ? 350 : 250;
        snprintf(lbl, sizeof(lbl), "%s_%s_KIRI", isPulangLeg ? "PLG" : "GO", GRID_NODES[u].name);
        st = buatPlanStep(lbl, trig, lock, 0.0f, 0, false,
                          OFFSET_MAJU_BELOK_MS, false, 85, 35, JEDA_DIEM_SIMPANG_MS, act);
      }

      if (!tambahStep(st)) {
        Serial.println("[ERROR RUTE] Jumlah langkah melebihi kapasitas MAX_PLAN_STEPS.");
        return false;
      }
    }

    // Proses node tujuan segmen ini (Stasiun A..E atau FINISH START)
    if (isNodeStasiun(goalNode)) {
      float targetCm = 50.0f;
      if (tempCount > 0 && tempRute[tempCount - 1].action == ACT_CROSS_STRAIGHT &&
          tempRute[tempCount - 1].pauseMs == 0) {
        // Setelah ACT_CROSS_STRAIGHT tanpa berhenti, robot sudah meluncur ~9 cm melewati simpang
        targetCm = 50.0f - OFFSET_CROSS_SIMPANG_CM;
        if (targetCm < 20.0f) targetCm = 20.0f;
      }

      char lbl[20];
      snprintf(lbl, sizeof(lbl), "CHECKPOINT_%s", GRID_NODES[goalNode].name);
      if (!tambahStep(buatPlanStep(lbl, TRIG_ENCODER_JARAK, 100, targetCm, TIMEOUT_ENCODER_A_MS, true,
                                   0, false, 85, 35,
                                   (uint16_t)constrain((long)runtimeJedaStasiunMs, 0L, 60000L),
                                   ACT_PAUSE_ONLY))) {
        Serial.println("[ERROR RUTE] Jumlah langkah melebihi kapasitas MAX_PLAN_STEPS.");
        return false;
      }
    } else if (goalNode == ND_START) {
      if (!tambahStep(buatPlanStep("FINISH_START", TRIG_PEREMPATAN, 350, 0.0f, 0, false,
                                   0, false, 85, 35, 50, ACT_STOP_MISSION))) {
        Serial.println("[ERROR RUTE] Jumlah langkah melebihi kapasitas MAX_PLAN_STEPS.");
        return false;
      }
    }

    curNode = goalNode;
    curDir  = pathDirs[pathLen - 1];
  }

  if (!pulangKeStart) {
    if (!tambahStep(buatPlanStep("STOP_TUJUAN", TRIG_IMMEDIATE, 0, 0.0f, 0, false,
                                 0, false, 0, 0, 0, ACT_STOP_MISSION))) {
      Serial.println("[ERROR RUTE] Jumlah langkah melebihi kapasitas MAX_PLAN_STEPS.");
      return false;
    }
  }

  // Salin ke buffer aktif hanya setelah seluruh segmen sukses dibangun
  for (int i = 0; i < tempCount; i++) {
    ruteAktif[i] = tempRute[i];
  }
  jumlahStepAktif       = tempCount;
  runtimePulangOtomatis = pulangKeStart;

  // Format nama rute (contoh: "RUTE_A_C_PP" atau "RUTE_B_1WAY")
  String nama = "RUTE_";
  for (int i = 0; i < numTargets; i++) {
    if (i > 0) nama += "_";
    nama += hurufTargets[i];
  }
  nama += pulangKeStart ? "_PP" : "_1WAY";
  strncpy(namaRuteAktif, nama.c_str(), sizeof(namaRuteAktif) - 1);
  namaRuteAktif[sizeof(namaRuteAktif) - 1] = '\0';

  return true;
}
