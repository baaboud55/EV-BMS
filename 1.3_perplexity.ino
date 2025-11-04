/*
  ESP32-S3 BMS + PID Charger Controller with ACS712 Current Sensing
  
  Features:
  - BMS monitoring via daisy-chain (GPIO 16 RX, GPIO 17 TX)
  - ACS712 current sensing (GPIO 4 - CHANGED from 17 to avoid conflict)
  - Two-stage charging:
    * Constant Current (CC): 1A until pack reaches 32.5V
    * Constant Voltage (CV): 33.2V (4.15V per cell) until current drops
  - PID control for both CC and CV modes
  - Automatic calibration of current sensor
  - Safety interlocks (temperature, voltage, balance)
*/

// ==================== ACS712 CURRENT SENSOR CONFIGURATION ====================
// IMPORTANT: Changed from GPIO 17 to GPIO 4 to avoid conflict with Serial2 TX
const int CURRENT_SENSOR_PIN = 4;  // CHANGED FROM 17 to avoid Serial2 TX conflict

// ACS712 sensor parameters (your calibrated values)
const float ACS_SENSITIVITY = 0.06568;  // V/A (DO NOT CHANGE - your calibrated value)
const int ACS_VCC_VOLTAGE = 3284;       // mV (DO NOT CHANGE - your measured value)

// Current sensor calibration
float currentOffsetVoltage = 0.0;  // Will be auto-calibrated
bool currentSensorCalibrated = false;

// ==================== PID CONTROLLER STRUCTURES ====================
struct PIDController {
  float Kp;           // Proportional gain
  float Ki;           // Integral gain
  float Kd;           // Derivative gain
  float setpoint;     // Target value
  float integral;     // Integral accumulator
  float prev_error;   // Previous error for derivative
  float output_min;   // Minimum output limit
  float output_max;   // Maximum output limit
};

// Two PID controllers: one for current (CC mode), one for voltage (CV mode)
PIDController currentPID;  // For constant current control
PIDController voltagePID;  // For constant voltage control

// ==================== TIMING VARIABLES ====================
unsigned long lastBMSReadTime = 0;
unsigned long lastStartupCheckTime = 0;
unsigned long lastCurrentReadTime = 0;
const unsigned long BMS_READ_INTERVAL = 2000;      // 2 seconds
const unsigned long STARTUP_CHECK_INTERVAL = 5000;  // 5 seconds
const unsigned long CURRENT_READ_INTERVAL = 100;   // 100ms for current sensing

// ==================== PIN DEFINITIONS ====================
const int CHARGE_ENABLE_PIN = 5;   // GPIO to enable/disable charger
const int PWM_PIN = 1;             // GPIO for PWM charge current control

// PWM settings
const int PWM_FREQ = 62500;        // 62.5 kHz
const int PWM_CHANNEL = 0;
const int PWM_RES = 9;             // 9-bit resolution (0-511)

// ==================== BATTERY PARAMETERS ====================
// Battery state variables
float cells[8];
float temps[2];
int balanceStatus[2];
float totalPackVoltage = 0;
float maxCellVoltage = 0;
float minCellVoltage = 0;
float avgTemp = 0;

// Current sensing
float measuredCurrent = 0.0;  // Measured current in Amps

// ==================== CHARGING PARAMETERS ====================
// Two-stage charging parameters
const float CC_TARGET_CURRENT = 1.0;      // Constant Current: 1A
const float CC_TO_CV_VOLTAGE = 32.5;      // Switch to CV at 32.5V pack voltage
const float CV_TARGET_VOLTAGE = 33.2;     // Constant Voltage: 33.2V (4.15V per cell)
const float CHARGE_COMPLETE_VOLTAGE = 33.2; // Stop charging at 33.2V
const float CHARGE_COMPLETE_CURRENT = 0.1;  // Stop when current drops to 0.1A

const float MIN_CHARGE_VOLTAGE = 24.0;    // Minimum voltage to start (3.0V per cell)
const float MAX_TEMP = 45.0;              // Maximum temperature in Celsius
const float MIN_TEMP = 0.0;               // Minimum temperature for charging

// ==================== CHARGING STATES ====================
enum ChargeState {
  IDLE,
  CALIBRATING_CURRENT_SENSOR,
  WAITING_FOR_CONDITIONS,
  CHARGING_CC,          // Constant Current mode
  CHARGING_CV,          // Constant Voltage mode
  BALANCING,
  COMPLETE,
  ERROR
};

ChargeState currentState = IDLE;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 17, 16);  // BMS communication
  
  // Initialize charge control pins
  pinMode(CHARGE_ENABLE_PIN, OUTPUT);
  digitalWrite(CHARGE_ENABLE_PIN, LOW);  // Charger disabled initially
  
  // Configure PWM for charge current control
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 511 - 0);  // Start with 0 duty cycle (inverted PWM)
  
  // Initialize analog pin for current sensor
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  
  // Initialize PID controllers
  // Current PID: For CC mode (target = 1A)
  initPID(&currentPID, 50.0, 10.0, 2.0, 0.0, 511.0);
  currentPID.setpoint = CC_TARGET_CURRENT;
  
  // Voltage PID: For CV mode (target = 33.2V)
  initPID(&voltagePID, 30.0, 5.0, 1.5, 0.0, 511.0);
  voltagePID.setpoint = CV_TARGET_VOLTAGE;
  
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║  ESP32-S3 BMS + Two-Stage PID Charge Controller       ║");
  Serial.println("║  with ACS712 Current Sensing                           ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Charging Profile:");
  Serial.println("  Stage 1: Constant Current (CC) @ 1.0A until 32.5V");
  Serial.println("  Stage 2: Constant Voltage (CV) @ 33.2V until 0.1A");
  Serial.println();
  
  delay(1000);
  
  // Start current sensor calibration
  currentState = CALIBRATING_CURRENT_SENSOR;
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Handle current sensor calibration state
  if (currentState == CALIBRATING_CURRENT_SENSOR) {
    calibrateCurrentSensor();
    currentState = IDLE;
    return;
  }
  
  // Read BMS data every 2 seconds
  if (currentTime - lastBMSReadTime >= BMS_READ_INTERVAL) {
    lastBMSReadTime = currentTime;
    readBMSData();
  }
  
  // Read current sensor every 100ms
  if (currentTime - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
    lastCurrentReadTime = currentTime;
    if (currentSensorCalibrated) {
      readCurrent();
    }
  }
  
  // Check startup conditions every 5 seconds
  if (currentTime - lastStartupCheckTime >= STARTUP_CHECK_INTERVAL) {
    lastStartupCheckTime = currentTime;
    waitForStartupCondition();
  }
  
  // Execute charging state machine
  executeChargeControl();
}

// ==================== CURRENT SENSOR CALIBRATION ====================
void calibrateCurrentSensor() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║           ACS712 AUTO-CALIBRATION                      ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("⚠ IMPORTANT: Ensure NO current is flowing through sensor!");
  Serial.println("Starting calibration in 5 seconds...");
  
  delay(5000);
  Serial.println("Calibrating...");
  
  long totalRawValue = 0;
  for (int i = 0; i < 1000; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  
  // Calculate offset voltage
  currentOffsetVoltage = ((float)totalRawValue / 1000.0 / 4096.0) * ACS_VCC_VOLTAGE;
  currentSensorCalibrated = true;
  
  Serial.print("✓ Calibration complete! Offset voltage: ");
  Serial.print(currentOffsetVoltage, 2);
  Serial.println(" mV");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();
  delay(1000);
}

// ==================== CURRENT READING ====================
void readCurrent() {
  // Average multiple samples for stability
  const int numSamples = 50;  // Reduced from 100 for faster response
  long totalRawValue = 0;
  
  for (int i = 0; i < numSamples; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
  }
  
  float avgRawValue = (float)totalRawValue / numSamples;
  
  // Convert to voltage
  float voltage_mV = (avgRawValue / 4096.0) * ACS_VCC_VOLTAGE;
  
  // Calculate current (using your calibrated formula)
  measuredCurrent = abs((currentOffsetVoltage - voltage_mV) / (ACS_SENSITIVITY * 1000));
}

// ==================== BMS DATA READING ====================
void readBMSData() {
  // Send request down the chain
  Serial2.println("READ_VOLTAGES");
  
  // Wait for response from Slave 2 (end of chain)
  unsigned long timeout = millis();
  while (!Serial2.available() && (millis() - timeout < 2000)) {
    delay(10);
  }
  
  if (Serial2.available()) {
    String voltageData = Serial2.readStringUntil('\n');
    
    if (parseAllData(voltageData, cells, temps, balanceStatus)) {
      displayBMSData();
      calculatePackStatistics();
    } else {
      Serial.println("⚠ Error: Failed to parse BMS data");
      Serial.print("Raw data: "); Serial.println(voltageData);
      currentState = ERROR;
    }
  } else {
    Serial.println("⚠ Error: No response from BMS chain");
    currentState = ERROR;
  }
}

// ==================== DATA PARSING ====================
bool parseAllData(String data, float cells[], float temps[], int balance[]) {
  int idx[11];
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

// ==================== BMS DATA DISPLAY ====================
void displayBMSData() {
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║                    BMS DATA                            ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");

  Serial.println("\n┌─── Slave 1 (Cells 1-4) ───┐");
  for (int i = 0; i < 4; i++) {
    Serial.print("│ Cell "); Serial.print(i+1); 
    Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
  }
  Serial.print("│ Temperature: "); Serial.print(temps[0], 2); Serial.println(" °C");
  Serial.print("│ Balance: "); 
  Serial.println(balanceStatus[0] ? "✓ OK" : "⚠ Imbalanced");
  Serial.println("└───────────────────────────┘");
  
  Serial.println("\n┌─── Slave 2 (Cells 5-8) ───┐");
  for (int i = 4; i < 8; i++) {
    Serial.print("│ Cell "); Serial.print(i+1); 
    Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
  }
  Serial.print("│ Temperature: "); Serial.print(temps[1], 2); Serial.println(" °C");
  Serial.print("│ Balance: "); 
  Serial.println(balanceStatus[1] ? "✓ OK" : "⚠ Imbalanced");
  Serial.println("└───────────────────────────┘");
  
  Serial.print("\n📊 Total Pack Voltage: "); 
  Serial.print(totalPackVoltage, 3); Serial.println(" V");
  
  checkOverallBalance(cells);
}

// ==================== PACK STATISTICS ====================
void calculatePackStatistics() {
  totalPackVoltage = 0;
  maxCellVoltage = cells[0];
  minCellVoltage = cells[0];
  
  for (int i = 0; i < 8; i++) {
    totalPackVoltage += cells[i];
    if (cells[i] > maxCellVoltage) maxCellVoltage = cells[i];
    if (cells[i] < minCellVoltage) minCellVoltage = cells[i];
  }
  
  avgTemp = (temps[0] + temps[1]) / 2.0;
}

// ==================== BALANCE CHECK ====================
void checkOverallBalance(float cells[]) {
  float diff = maxCellVoltage - minCellVoltage;
  Serial.print("📏 Voltage Difference: "); 
  Serial.print(diff, 3); Serial.print(" V ");
  
  if (diff > 0.1) {
    Serial.println("⚠ IMBALANCED!");
  } else {
    Serial.println("✓ Balanced");
  }
}

// ==================== STARTUP CONDITION CHECK ====================
void waitForStartupCondition() {
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║            STARTUP CONDITIONS CHECK                    ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  
  bool tempOK = (avgTemp >= MIN_TEMP && avgTemp <= MAX_TEMP);
  bool voltageOK = (totalPackVoltage >= MIN_CHARGE_VOLTAGE && totalPackVoltage < CHARGE_COMPLETE_VOLTAGE);
  bool balanceOK = ((maxCellVoltage - minCellVoltage) <= 0.1);
  bool sensorOK = currentSensorCalibrated;
  
  Serial.print("🌡️  Temperature: "); 
  Serial.print(avgTemp, 2); Serial.print(" °C ");
  Serial.println(tempOK ? "✓ OK" : "✗ FAIL");
  
  Serial.print("⚡ Pack Voltage: "); 
  Serial.print(totalPackVoltage, 2); Serial.print(" V ");
  Serial.println(voltageOK ? "✓ OK" : "✗ FAIL");
  
  Serial.print("⚖️  Balance: ");
  Serial.print(maxCellVoltage - minCellVoltage, 3); Serial.print(" V ");
  Serial.println(balanceOK ? "✓ OK" : "✗ FAIL");
  
  Serial.print("📡 Current Sensor: ");
  Serial.print(measuredCurrent);Serial.print(" A ");

  Serial.println(sensorOK ? "✓ Calibrated" : "✗ Not Ready");
  
  if (tempOK && voltageOK && balanceOK && sensorOK) {
    if (currentState == IDLE || currentState == WAITING_FOR_CONDITIONS) {
      Serial.println("\n✓✓✓ ALL CONDITIONS MET - READY TO CHARGE! ✓✓✓");
      currentState = CHARGING_CC;  // Start with Constant Current mode
    }
  } else {
    if (currentState == CHARGING_CC || currentState == CHARGING_CV) {
      Serial.println("\n⚠⚠⚠ CONDITIONS NOT MET - STOPPING CHARGE ⚠⚠⚠");
      currentState = WAITING_FOR_CONDITIONS;
      stopCharging();
    } else if (currentState == IDLE) {
      currentState = WAITING_FOR_CONDITIONS;
    }
  }
  Serial.println("════════════════════════════════════════════════════════");
}

// ==================== CHARGE CONTROL STATE MACHINE ====================
void executeChargeControl() {
  switch (currentState) {
    case IDLE:
      // Waiting for first BMS read
      break;
      
    case WAITING_FOR_CONDITIONS:
      stopCharging();
      break;
      
    case CHARGING_CC:
      performCCCharging();  // Constant Current mode
      break;
      
    case CHARGING_CV:
      performCVCharging();  // Constant Voltage mode
      break;
      
    case BALANCING:
      if (balanceStatus[0] && balanceStatus[1]) {
        Serial.println("✓ Balancing complete");
        currentState = COMPLETE;
      }
      stopCharging();
      break;
      
    case COMPLETE:
      stopCharging();
      break;
      
    case ERROR:
      stopCharging();
      break;
  }
}

// ==================== CONSTANT CURRENT CHARGING ====================
void performCCCharging() {
  // Check if we should transition to CV mode
  if (totalPackVoltage >= CC_TO_CV_VOLTAGE) {
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║   SWITCHING TO CONSTANT VOLTAGE (CV) MODE              ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    currentState = CHARGING_CV;
    // Reset PID integral for smooth transition
    voltagePID.integral = 0;
    voltagePID.prev_error = 0;
    return;
  }
  
  // Check temperature safety
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    Serial.println("\n⚠ Temperature out of range!");
    currentState = WAITING_FOR_CONDITIONS;
    return;
  }
  
  // Compute PID output based on current error
  float pidOutput = computePID(&currentPID, measuredCurrent, 0.1);
  
  // Apply PID output to PWM (inverted: 511 - pidOutput)
  ledcWrite(PWM_PIN, 511 - (int)pidOutput);
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  
  // Display charging status (only occasionally to avoid flooding serial)
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 1000) {  // Every 1 second
    lastDisplay = millis();
    
    Serial.println("\n┌─────────────────────────────────────────────────┐");
    Serial.println("│      CONSTANT CURRENT (CC) MODE                 │");
    Serial.println("└─────────────────────────────────────────────────┘");
    Serial.print("🔋 Pack Voltage: "); Serial.print(totalPackVoltage, 3); Serial.println(" V");
    Serial.print("⚡ Target Current: "); Serial.print(CC_TARGET_CURRENT, 2); Serial.println(" A");
    Serial.print("📊 Measured Current: "); Serial.print(measuredCurrent, 3); Serial.println(" A");
    Serial.print("🎛️  PID Output: "); Serial.print(pidOutput, 1); Serial.print("/511");
    Serial.print(" ("); Serial.print((pidOutput/511.0)*100, 1); Serial.println("%)");
    Serial.print("🌡️  Temperature: "); Serial.print(avgTemp, 2); Serial.println(" °C");
    Serial.print("📈 Progress: ");
    float progress = ((totalPackVoltage - MIN_CHARGE_VOLTAGE) / (CC_TO_CV_VOLTAGE - MIN_CHARGE_VOLTAGE)) * 100;
    Serial.print(progress, 1); Serial.println("% to CV mode");
  }
}

// ==================== CONSTANT VOLTAGE CHARGING ====================
void performCVCharging() {
  // Check if charging is complete
  if (totalPackVoltage >= CHARGE_COMPLETE_VOLTAGE && measuredCurrent <= CHARGE_COMPLETE_CURRENT) {
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║           ✓✓✓ CHARGING COMPLETE ✓✓✓                   ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    
    if (!balanceStatus[0] || !balanceStatus[1]) {
      currentState = BALANCING;
      Serial.println("→ Entering balancing state");
    } else {
      currentState = COMPLETE;
      Serial.println("→ Battery fully charged and balanced!");
    }
    return;
  }
  
  // Check temperature safety
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    Serial.println("\n⚠ Temperature out of range!");
    currentState = WAITING_FOR_CONDITIONS;
    return;
  }
  
  // Compute PID output based on voltage error
  float pidOutput = computePID(&voltagePID, totalPackVoltage, 0.1);
  
  // Apply PID output to PWM (inverted: 511 - pidOutput)
  ledcWrite(PWM_PIN, 511 - (int)pidOutput);
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  
  // Display charging status
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 1000) {  // Every 1 second
    lastDisplay = millis();
    
    Serial.println("\n┌─────────────────────────────────────────────────┐");
    Serial.println("│      CONSTANT VOLTAGE (CV) MODE                 │");
    Serial.println("└─────────────────────────────────────────────────┘");
    Serial.print("🔋 Pack Voltage: "); Serial.print(totalPackVoltage, 3); Serial.println(" V");
    Serial.print("🎯 Target Voltage: "); Serial.print(CV_TARGET_VOLTAGE, 2); Serial.println(" V");
    Serial.print("⚡ Measured Current: "); Serial.print(measuredCurrent, 3); Serial.println(" A");
    Serial.print("🎛️  PID Output: "); Serial.print(pidOutput, 1); Serial.print("/511");
    Serial.print(" ("); Serial.print((pidOutput/511.0)*100, 1); Serial.println("%)");
    Serial.print("🌡️  Temperature: "); Serial.print(avgTemp, 2); Serial.println(" °C");
    Serial.print("📉 Tapering: ");
    float taper = (measuredCurrent / CC_TARGET_CURRENT) * 100;
    Serial.print(taper, 1); Serial.println("% of CC current");
  }
}

// ==================== STOP CHARGING ====================
void stopCharging() {
  digitalWrite(CHARGE_ENABLE_PIN, LOW);
  ledcWrite(PWM_PIN, 511 - 0);  // Inverted PWM: full off
}

// ==================== PID FUNCTIONS ====================
void initPID(PIDController* pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->integral = 0;
  pid->prev_error = 0;
  pid->output_min = min_out;
  pid->output_max = max_out;
}

float computePID(PIDController* pid, float input, float dt) {
  float error = pid->setpoint - input;
  
  // Proportional term
  float P = pid->Kp * error;
  
  // Integral term with anti-windup
  pid->integral += error * dt;
  float I = pid->Ki * pid->integral;
  
  // Derivative term
  float derivative = (error - pid->prev_error) / dt;
  float D = pid->Kd * derivative;
  
  // Compute total output
  float output = P + I + D;
  
  // Clamp output
  if (output > pid->output_max) {
    output = pid->output_max;
    pid->integral -= error * dt;  // Anti-windup
  }
  if (output < pid->output_min) {
    output = pid->output_min;
    pid->integral -= error * dt;  // Anti-windup
  }
  
  pid->prev_error = error;
  
  return output;
}
