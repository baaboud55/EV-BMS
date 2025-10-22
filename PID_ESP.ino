
// --- Boost Converter PID Voltage Control (ESP32-S3 Version) ---
// Mohammad Baaboud – Adapted for ESP32-S3 (LEDC PWM + 12-bit ADC)

// --- Pin Definitions ---
#define FEEDBACK_PIN       1  // ADC input for PID feedback
#define STARTUP_SENSE_PIN  2  // Digital input for startup voltage detection
#define PWM_PIN            45  // PWM output to MOSFET gate
#define NMOS_PIN           47  // NMOS control
#define RELAY_PIN          48  // Relay control

// --- Voltage Divider (for up to ~33.6V scaled to ~3.3V ADC) ---
const float R1 = 100000.0;  
const float R2 = 10100.0;  

// --- ADC Reference Voltage ---
const float VREF = 3.3;    

// --- Target Voltage ---
float targetVoltage = 32.0;  

// --- PID Coefficients ---
float Kp = 0.25;
float Ki = 0.01;
float Kd = 0.0;

// --- PID Variables ---
float error = 0, lastError = 0, integral = 0, derivative = 0;
float outputPWM = 0;  
unsigned long lastPIDTime = 0, lastPrintTime = 0;

// --- Control Interval (ms) ---
const unsigned long PID_INTERVAL = 10; // 2 ms

// --- PWM Configuration ---
const int PWM_FREQ = 62500;  // 62.5 kHz
const int PWM_RES = 9;      // 10-bit resolution

// --- PWM Duty Clamp (35–60%) ---
const int PWM_MIN = int(0.35 * 511); // 358
const int PWM_MAX = int(0.70 * 511); // 614

// --- Helper Functions ---
float readVoltage_FB() {
  const int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(FEEDBACK_PIN);
    delayMicroseconds(100);
  }
  int adcValue = sum / samples;
  return (adcValue * VREF / 4095.0) * ((R1 + R2) / R2);
}


float readVoltage_IN() {
  int adcValue = analogRead(STARTUP_SENSE_PIN);
  return (adcValue * VREF / 4095.0) * ((R1 + R2) / R2);
}

void setInitialState() {
  outputPWM = 0;                    
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 511 - (int)outputPWM); // inverted PWM
  digitalWrite(NMOS_PIN, LOW);      
  digitalWrite(RELAY_PIN, LOW);     
  Serial.println("System in initial state - waiting for startup voltage...");
}

void setupPWM() {
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
}

void setup() {
  Serial.begin(115200);
  pinMode(NMOS_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FEEDBACK_PIN, INPUT);
  pinMode(STARTUP_SENSE_PIN, INPUT);

  setupPWM();
  setInitialState();
  lastPIDTime = millis();
}

// --- Startup sequence using digital sense pin ---
void waitForStartupCondition() {
  // Wait until STARTUP_SENSE_PIN goes HIGH
  while (float IN_measuredVoltage = readVoltage_IN() < 1.0) {
    Serial.println("Waiting for startup voltage...");
    delay(1000);
  }

  Serial.println("Startup voltage detected HIGH. Starting sequence...");
  delay(6000); // Precharge delay

  digitalWrite(NMOS_PIN, HIGH);
  Serial.println("NMOS ON.");

  delay(100);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Relay ON. Precharge disabled.");

  delay(100);
  Serial.println("Starting PID control...");
}

void loop() {
  static bool systemReady = false;

  float FB_measuredVoltage = readVoltage_FB();
  float IN_measuredVoltage = readVoltage_IN();

  // --- Safety: reset if feedback voltage is too low ---
  if (IN_measuredVoltage < 1.0) {
    systemReady = false;
    setInitialState();
  }

  // --- Wait for startup sequence ---
  if (!systemReady) {
    waitForStartupCondition();
    systemReady = true;
    integral = 0;
    lastError = 0;
    outputPWM = 0;  // Start from 0
    lastPIDTime = millis();
  }

  // --- PID Loop ---
  unsigned long now = millis();
  if (now - lastPIDTime >= PID_INTERVAL) {
    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    FB_measuredVoltage = readVoltage_FB();

    error = targetVoltage - FB_measuredVoltage;
    integral += error * dt;
    derivative = (error - lastError) / dt;
    lastError = error;

    float control = (Kp * error) + (Ki * integral) + (Kd * derivative);
    outputPWM += control;  // PID adjusted for inverted PWM

    // Clamp PWM to 35–60% duty cycle
    outputPWM = constrain(outputPWM, PWM_MIN, PWM_MAX);
    ledcWrite(PWM_PIN, 511 - (int)outputPWM);
  }

  // --- Serial Print every 1 second ---
  if (now - lastPrintTime >= 300) {
    lastPrintTime = now;
    Serial.print("Target: ");   Serial.print(targetVoltage);
    Serial.print(" V  | Measured: "); Serial.print(FB_measuredVoltage);
    Serial.print(" V  | PWM: "); Serial.print((outputPWM / 511.0) * 100);
    Serial.print("% | NMOS: "); Serial.print(digitalRead(NMOS_PIN));
    Serial.print(" | Relay: "); Serial.println(digitalRead(RELAY_PIN));
  }
}