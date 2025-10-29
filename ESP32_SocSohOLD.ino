// ESP32 Master - Daisy Chain Configuration + SOC/SOH
// TX (GPIO 17) → Slave1, RX (GPIO 16) ← Slave2
//
// Incoming line (from end of chain after "READ_VOLTAGES"):
//   v1,v2,v3,v4,v5,v6,v7,v8,temp1,temp2,balance1,balance2\n
//
// Adds:
//  - ACS712 current sensing on CURRENT_SENSOR_PIN
//  - SOC (coulomb counting + OCV correction at rest) per pack (A: cells 1-4, B: cells 5-8)
//  - SOH learned from discharged mAh between near-full and near-empty resting windows
//
// Tune in "USER CONFIG" as needed.

#include <Arduino.h>
#include <math.h>

// ========================== USER CONFIG ===========================

// UART to slaves (unchanged from your code)
static const int UART_RX_PIN = 16;               // RX from Slave2
static const int UART_TX_PIN = 17;               // TX to Slave1
static const uint32_t UART_BAUD = 9600;

// Current sensor (ACS712)
static const int   CURRENT_SENSOR_PIN = 4;       // <-- SET: ADC-capable pin (GPIO4 default)
static const float ACS712_SENSITIVITY = 0.066f;  // V/A (0.066=30A, 0.100=20A, 0.185=5A)
static const float CURRENT_VREF       = 3.30f;   // ESP32 ADC reference (approx)
static const float CURRENT_ZERO_VOLT  = 1.65f;   // zero-current midpoint voltage (calibrate)
static const int   CURRENT_SAMPLES    = 16;      // averaging for noise

// Pack model (two 4S packs in SERIES, same current)
static const float RATED_CAPACITY_mAh_A = 2000.0f; // Pack A rated capacity (mAh)
static const float RATED_CAPACITY_mAh_B = 2000.0f; // Pack B rated capacity (mAh)

// OCV->SOC table (typical 18650 NMC/NCA-like). Adjust if needed.
static const int OCV_N = 13;
static const float OCV_V[OCV_N]   = {3.00,3.20,3.30,3.45,3.55,3.65,3.70,3.75,3.80,3.85,3.95,4.10,4.20};
static const float OCV_SOC[OCV_N] = {  0,   3,   6,  12,  20,  30,  40,  50,  60,  70,  80,   90, 100};

// Rest/limits (per cell) for corrections/learning
static const float REST_CURRENT_A = 0.2f;    // |I| below this = "resting" (for OCV correction)
static const float OCV_BLEND      = 0.10f;   // blend factor toward OCV-SOC when resting (0..1)
static const float FULL_V_CELL    = 4.18f;   // near-full (resting window start)
static const float EMPTY_V_CELL   = 3.30f;   // near-empty (resting window end)

// Imbalance alert threshold (your original logic)
static const float IMBALANCE_V_THRESHOLD = 0.10f; // 100 mV spread

// Print cadence
static const uint32_t REPORT_MS = 2000;

// ========================== STATE ================================

struct FuelPack {
  float soc_pct = 100.0f;    // State of Charge (%)
  float soh_pct = 100.0f;    // State of Health (% capacity)
  float measure_mAh = 0.0f;  // discharge capacity accumulated in current learning window
  bool  in_measure = false;  // currently measuring discharge window
};

static FuelPack packA, packB;

static uint32_t lastTickMs = 0;
static uint32_t lastReport = 0;

// ========================= UTILITIES =============================

float readADC_Avg(int pin, int samples, float vref) {
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) acc += analogRead(pin);
  float avg = float(acc) / samples;
  // ESP32 ADC ~12-bit (0..4095)
  return (avg / 4095.0f) * vref;
}

float readCurrentACS712_A() {
  // Positive current means CHARGING into packs; negative = discharging
  float v = readADC_Avg(CURRENT_SENSOR_PIN, CURRENT_SAMPLES, CURRENT_VREF);
  return (v - CURRENT_ZERO_VOLT) / ACS712_SENSITIVITY;
}

float lerp(float a, float b, float t) { return a + (b - a) * t; }

float socFromOCV(float v_cell) {
  if (v_cell <= OCV_V[0]) return OCV_SOC[0];
  if (v_cell >= OCV_V[OCV_N-1]) return OCV_SOC[OCV_N-1];
  for (int i = 0; i < OCV_N - 1; ++i) {
    if (v_cell >= OCV_V[i] && v_cell <= OCV_V[i + 1]) {
      float t = (v_cell - OCV_V[i]) / (OCV_V[i + 1] - OCV_V[i]);
      return lerp(OCV_SOC[i], OCV_SOC[i + 1], t);
    }
  }
  return 0.0f;
}

float avg4(const float v[4]) {
  return 0.25f * (v[0] + v[1] + v[2] + v[3]);
}

// Coulomb counting + OCV correction + SOH learning
void updateFuel(FuelPack &fp, float dt_s, float packCurrentA, const float cells[4], float rated_mAh) {
  // Discharge current is NEGATIVE on bus (out of battery). Count only discharge.
  float dischargeA = (packCurrentA < 0) ? (-packCurrentA) : 0.0f;
  float d_mAh = (dischargeA * 3600.0f * dt_s) / 1000.0f;  // A*s to mAh

  // SOC from coulomb counting (decrease on discharge)
  float dSOC = (rated_mAh > 0) ? (100.0f * d_mAh / rated_mAh) : 0.0f;
  fp.soc_pct = constrain(fp.soc_pct - dSOC, 0.0f, 100.0f);

  // OCV correction at rest
  float v_avg = avg4(cells);
  float soc_ocv = socFromOCV(v_avg);
  if (fabs(packCurrentA) < REST_CURRENT_A) {
    fp.soc_pct = (1.0f - OCV_BLEND) * fp.soc_pct + OCV_BLEND * soc_ocv;
  }

  // SOH learning window (near-full -> near-empty, both at rest)
  bool nearFull  = (v_avg >= FULL_V_CELL);
  bool nearEmpty = (v_avg <= EMPTY_V_CELL);
  bool resting   = (fabs(packCurrentA) < REST_CURRENT_A);

  if (!fp.in_measure && nearFull && resting) {
    fp.in_measure = true;
    fp.measure_mAh = 0.0f;  // start new window
  }

  if (fp.in_measure) {
    fp.measure_mAh += d_mAh;        // accumulate discharge only
    if (nearEmpty && resting) {
      if (rated_mAh > 0) {
        fp.soh_pct = constrain(100.0f * fp.measure_mAh / rated_mAh, 0.0f, 150.0f);
      }
      fp.in_measure = false;        // end window; wait for next near-full
    }
  }
}

// ====================== YOUR ORIGINAL HELPERS ====================

bool parseAllData(String data, float cells[], float temps[], int balance[]) {
  int idx[11]; // 11 commas for 12 values
  idx[0] = data.indexOf(',');
  if (idx[0] == -1) return false;

  for (int i = 1; i < 11; i++) {
    idx[i] = data.indexOf(',', idx[i-1] + 1);
    if (idx[i] == -1) return false;
  }

  // Parse 8 cell voltages
  cells[0] = data.substring(0, idx[0]).toFloat();
  for (int i = 1; i < 8; i++) {
    cells[i] = data.substring(idx[i-1] + 1, idx[i]).toFloat();
  }

  // Parse temperatures
  temps[0] = data.substring(idx[7] + 1, idx[8]).toFloat();
  temps[1] = data.substring(idx[8] + 1, idx[9]).toFloat();

  // Parse balance status
  balance[0] = data.substring(idx[9] + 1, idx[10]).toInt();
  balance[1] = data.substring(idx[10] + 1).toInt();

  return true;
}

void checkOverallBalance(float cells[]) {
  float maxV = cells[0];
  float minV = cells[0];

  for (int i = 1; i < 8; i++) {
    if (cells[i] > maxV) maxV = cells[i];
    if (cells[i] < minV) minV = cells[i];
  }

  float diff = maxV - minV;
  Serial.print("Overall Voltage Difference: ");
  Serial.print(diff, 3); Serial.println(" V");

  if (diff > IMBALANCE_V_THRESHOLD) {
    Serial.println("⚠ WARNING: Pack imbalance detected!");
  } else {
    Serial.println("✓ Pack balanced");
  }
}

// ============================= SETUP =============================

void setup() {
  Serial.begin(115200);

  // ADC setup
  analogReadResolution(12);
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  // UART to chain (your mapping)
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  lastTickMs = millis();
  lastReport = millis();

  Serial.println("ESP32 Master - BMS 8-Cell Daisy Chain Monitor (SOC/SOH)");
  Serial.println("========================================================");
  Serial.printf("Current sensor pin = GPIO%d\n", CURRENT_SENSOR_PIN);
}

// ============================== LOOP =============================

void loop() {
  const uint32_t now = millis();
  float dt_s = (now - lastTickMs) / 1000.0f;
  if (dt_s < 0) dt_s = 0;
  lastTickMs = now;

  // Read bus current (positive = charging, negative = discharging)
  float busCurrentA = readCurrentACS712_A();

  // Send request down the chain (unchanged)
  Serial2.println("READ_VOLTAGES");
  Serial.println("Request sent to chain...");

  // Wait for response from Slave 2 (end of chain)
  unsigned long timeout = millis();
  while (!Serial2.available() && (millis() - timeout < 2000)) {
    delay(10);
  }

  if (Serial2.available()) {
    String voltageData = Serial2.readStringUntil('\n');

    // Parse: v1,v2,v3,v4,v5,v6,v7,v8,temp1,temp2,balance1,balance2
    float cells[8];
    float temps[2];
    int balanceStatus[2];

    if (parseAllData(voltageData, cells, temps, balanceStatus)) {
      // --- Split into pack A (cells 0..3) and pack B (cells 4..7)
      float aCells[4] = {cells[0], cells[1], cells[2], cells[3]};
      float bCells[4] = {cells[4], cells[5], cells[6], cells[7]};

      // --- Update SOC/SOH for both packs (series topology: same current)
      updateFuel(packA, dt_s, busCurrentA, aCells, RATED_CAPACITY_mAh_A);
      updateFuel(packB, dt_s, busCurrentA, bCells, RATED_CAPACITY_mAh_B);

      // --- Print like your original, plus SOC/SOH
      Serial.println("\n=== Slave 1 (Cells 1-4) ===");
      for (int i = 0; i < 4; i++) {
        Serial.print("Cell "); Serial.print(i+1);
        Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
      }
      Serial.print("Temperature 1: "); Serial.print(temps[0], 2); Serial.println(" °C");
      Serial.print("Balance Status 1: ");
      Serial.println(balanceStatus[0] ? "✓ Balanced" : "⚠ Imbalanced");

      Serial.printf("Pack A: SOC=%.1f %%  SOH=%.1f %%  %s\n",
                    packA.soc_pct, packA.soh_pct,
                    packA.in_measure ? "(measuring discharge window…)" : "");

      Serial.println("\n=== Slave 2 (Cells 5-8) ===");
      for (int i = 4; i < 8; i++) {
        Serial.print("Cell "); Serial.print(i+1);
        Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
      }
      Serial.print("Temperature 2: "); Serial.print(temps[1], 2); Serial.println(" °C");
      Serial.print("Balance Status 2: ");
      Serial.println(balanceStatus[1] ? "✓ Balanced" : "⚠ Imbalanced");

      Serial.printf("Pack B: SOC=%.1f %%  SOH=%.1f %%  %s\n",
                    packB.soc_pct, packB.soh_pct,
                    packB.in_measure ? "(measuring discharge window…)" : "");

      // Total pack voltage
      float total = 0;
      for (int i = 0; i < 8; i++) total += cells[i];
      Serial.print("\nTotal Pack Voltage: "); Serial.print(total, 3); Serial.println(" V");

      // Overall balance check (your existing function)
      checkOverallBalance(cells);

      // Periodic general readout
      if (now - lastReport >= REPORT_MS) {
        lastReport = now;
        Serial.printf("\nBus Current: %.3f A  (+: charging, -: discharging)\n", busCurrentA);
      }
    } else {
      Serial.println("Error: Failed to parse data");
      Serial.print("Raw data: "); Serial.println(voltageData);
    }
  } else {
    Serial.println("Error: No response from chain");
  }

  delay(2000);
}
