// ESP32 Master - Daisy Chain Configuration
// TX (GPIO 17) → Slave1, RX (GPIO 16) ← Slave2

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.println("ESP32 Master - BMS 8-Cell Daisy Chain Monitor");
  Serial.println("==============================================");
}

void loop() {
  // Send request down the chain
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
      Serial.println("\n=== Slave 1 (Cells 1-4) ===");
      for (int i = 0; i < 4; i++) {
        Serial.print("Cell "); Serial.print(i+1); 
        Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
      }
      Serial.print("Temperature 1: "); Serial.print(temps[0], 2); Serial.println(" °C");
      Serial.print("Balance Status 1: "); 
      Serial.println(balanceStatus[0] ? "✓ Balanced" : "⚠ Imbalanced");
      
      Serial.println("\n=== Slave 2 (Cells 5-8) ===");
      for (int i = 4; i < 8; i++) {
        Serial.print("Cell "); Serial.print(i+1); 
        Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
      }
      Serial.print("Temperature 2: "); Serial.print(temps[1], 2); Serial.println(" °C");
      Serial.print("Balance Status 2: "); 
      Serial.println(balanceStatus[1] ? "✓ Balanced" : "⚠ Imbalanced");
      
      // Total pack voltage
      float total = 0;
      for (int i = 0; i < 8; i++) total += cells[i];
      Serial.print("\nTotal Pack Voltage: "); Serial.print(total, 3); Serial.println(" V");
      
      // Overall balance check
      checkOverallBalance(cells);
    } else {
      Serial.println("Error: Failed to parse data");
      Serial.print("Raw data: "); Serial.println(voltageData);
    }
  } else {
    Serial.println("Error: No response from chain");
  }
  
  delay(2000);
}

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
  
  if (diff > 0.1) {
    Serial.println("⚠ WARNING: Pack imbalance detected!");
  } else {
    Serial.println("✓ Pack balanced");
  }
}
