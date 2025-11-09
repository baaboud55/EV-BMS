// Arduino Nano Slave 2 
const int cell4Pin = A0;
const int cell5Pin = A1;
const int cell6Pin = A2;
const int cell7Pin = A3;
const int cell8Pin = A4;
const int temp2Pin = A7;
const int BALANCE_PIN = 4;
const float referenceVoltage = 5.0;
const int adcResolution = 1023;
// --- Voltage Divider Resistor Values (Ohms) ---
const float a0 = 26510.0; const float b0 = 9970.0;
const float a1 = 46040.0; const float b1 = 9990.0;
const float a2 = 48070.0; const float b2 = 9980.0;
const float a3 = 55930.0; const float b3 = 9890.0;
const float a4 = 69200.0; const float b4 = 9950.0;
// --- Thermistor Config ---
const float SERIES_RESISTOR = 9970.0;        // Fixed resistor
const float THERMISTOR_NOMINAL = 10000.0;     // 10k at 25°C
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT = 3950.0;

float v1, v2, v3, v4, temp1;
int balanceStatus1;

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);
}

void loop() {
  // Read Slave 2's own 4 cell voltages

  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    // Parse Slave 1 data
    if (parseSlave1Data(receivedData)) {
      float v4s = readCellVoltage(cell4Pin, a0, b0);
      float v5s = readCellVoltage(cell5Pin, a1, b1);
      float v6s = readCellVoltage(cell6Pin, a2, b2);
      float v7s = readCellVoltage(cell7Pin, a3, b3);
      float v8s = readCellVoltage(cell8Pin, a4, b4);
      float v5 = v5s - v4s;
      float v6 = v6s - v5s;
      float v7 = v7s - v6s;
      float v8 = v8s - v7s;
      float temp2 = readTemperature(temp2Pin);
      int balanceStatus2 = getBalanceStatus(v5, v6, v7, v8);
      // Send combined data to Master
      Serial.print(v1, 3); Serial.print(",");
      Serial.print(v2, 3); Serial.print(",");
      Serial.print(v3, 3); Serial.print(",");
      Serial.print(v4, 3); Serial.print(",");
      Serial.print(v5, 3); Serial.print(",");
      Serial.print(v6, 3); Serial.print(",");
      Serial.print(v7, 3); Serial.print(",");
      Serial.print(v8, 3); Serial.print(",");
      Serial.print(temp1, 2); Serial.print(",");
      Serial.print(temp2, 2); Serial.print(",");
      Serial.print(balanceStatus1); Serial.print(",");
      Serial.println(balanceStatus2);
      
      Serial.flush();
    }
  }
  
  delay(50);
}

//PARSING FUNCTION
bool parseSlave1Data(String data) {
  // Remove any whitespace
  data.trim();
  
  // Check if starts with READ_VOLTAGES,
  int commandEnd = data.indexOf(',');
  if (commandEnd == -1) return false;
  
  String command = data.substring(0, commandEnd);
  if (command != "READ_VOLTAGES") return false;
  
  // Get the values part (after "READ_VOLTAGES,")
  String values = data.substring(commandEnd + 1);
  
  // Parse the 6 comma-separated values: v1,v2,v3,v4,temp1,balance1
  int commaPositions[5];
  int commaCount = 0;
  
  // Find all comma positions
  for (int i = 0; i < values.length() && commaCount < 5; i++) {
    if (values.charAt(i) == ',') {
      commaPositions[commaCount] = i;
      commaCount++;
    }
  }
  
  // We need exactly 5 commas for 6 values
  if (commaCount != 5) return false;
  
  // Extract each value
  v1 = values.substring(0, commaPositions[0]).toFloat();
  v2 = values.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
  v3 = values.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
  v4 = values.substring(commaPositions[2] + 1, commaPositions[3]).toFloat();
  temp1 = values.substring(commaPositions[3] + 1, commaPositions[4]).toFloat();
  balanceStatus1 = values.substring(commaPositions[4] + 1).toInt();
  
  return true;
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
  
  int averageReading = sum / numReadings;
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

int getBalanceStatus(float v5, float v6, float v7, float v8) {
  float maxV = max(max(v5, v6), max(v7, v8));
  float minV = min(min(v5, v6), min(v7, v8));
  
  if ((maxV - minV) < 0.1) {
    return 1; digitalWrite(BALANCE_PIN,HIGH);  // Balanced
  } else {
    return 0; digitalWrite(BALANCE_PIN,LOW); // Not balanced
  }
}
