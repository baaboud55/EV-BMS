
// Arduino Nano Slave 1 - Daisy Chain Node
// RX (D0) ← Master TX, TX (D1) → Slave 2 RX
// Reads 4 cells, temp1, balance1 and forwards to Slave 2

const int cell1Pin = A1;
const int cell2Pin = A2;
const int cell3Pin = A3;
const int cell4Pin = A6;
const int temp1Pin = A7;
const int BALANCE_PIN = 4;
const float referenceVoltage = 4.95;
const int adcResolution = 1023;
// --- Voltage Divider Resistor Values (Ohms) ---
const float a1 = 0.0;  const float b1 = 1.0;
const float a2 = 8081.0;  const float b2 = 10020.0;
const float a3 = 19720.0; const float b3 = 9990.0;
const float a6 = 26580.0; const float b6 = 10010.0;
// --- Thermistor Config ---
const float SERIES_RESISTOR = 9900.0;        // Fixed resistor
const float THERMISTOR_NOMINAL = 10000.0;     // 10k at 25°C
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT = 3950.0;

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);
}

void loop() {
  // Read Slave 1 data
  float v1s = readCellVoltage(cell1Pin, a1, b1);
  float v2s = readCellVoltage(cell2Pin, a2, b2);
  float v3s = readCellVoltage(cell3Pin, a3, b3);
  float v4s = readCellVoltage(cell4Pin, a6, b6);
  float v1 = v1s ;
  float v2 = v2s - v1s;
  float v3 = v3s - v2s;
  float v4 = v4s - v3s;
  float temp1 = readTemperature(temp1Pin);
  int balanceStatus1 = getBalanceStatus(v1, v2, v3, v4);

  // Wait for command from Master
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');

    // Forward command AND Slave 1 data to Slave 2
    // Format: READ_VOLTAGES,v1,v2,v3,v4,temp1,balance1
    Serial.print("READ_VOLTAGES"); Serial.print(",");
    Serial.print(v1, 3); Serial.print(",");
    Serial.print(v2, 3); Serial.print(",");
    Serial.print(v3, 3); Serial.print(",");
    Serial.print(v4, 3); Serial.print(",");
    Serial.print(temp1, 2); Serial.print(",");
    Serial.println(balanceStatus1);
    }
 delay(50);
}

float readCellVoltage(int pin, float R1, float R2) {
  long sum = 0;
  const int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  int averageReading = sum / numReadings;
  float voltage = (averageReading * referenceVoltage / adcResolution) * (R1 + R2) / R2;

  return voltage;
}

float readTemperature(int pin) {
  long sum = 0;
  const int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  
  float averageReading = sum / numReadings;
  float voltage = (averageReading * referenceVoltage) / adcResolution;
  float thermistorResistance = SERIES_RESISTOR * (referenceVoltage / voltage - 1.0);

  float steinhart;
  steinhart = thermistorResistance / THERMISTOR_NOMINAL; 
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  float temperature = steinhart - 273.15;
  
  return temperature;
}

int getBalanceStatus(float v1, float v2, float v3, float v4) {
  float maxV = max(max(v1, v2), max(v3, v4));
  float minV = min(min(v1, v2), min(v3, v4));
  
  if ((maxV - minV) < 0.1) {
    return 1; digitalWrite(BALANCE_PIN,HIGH);  // Balanced
  } else {
    return 0; digitalWrite(BALANCE_PIN,LOW); // Not balanced
  }
}
