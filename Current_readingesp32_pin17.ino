/*
  ACS712 Current Sensor - Improved Accuracy & Stability Code
  - Automatically calibrates the zero-current offset on startup.
  - Averages multiple readings in the loop to reduce noise.
  - Accounts for user's measured VCC and sensor sensitivity.
*/

// --- Adjustable Parameters ---

// The analog pin your ACS712's VOUT is connected to.
const int SENSOR_PIN = 17;

// The sensitivity of your ACS712 module in V/A.
const float SENSITIVITY = 0.06568; // Your value: 0.125 V/A

// Your measured VCC voltage in millivolts.
const int VCC_VOLTAGE = 3284; // Your measured value

// --- Global Variables ---

// This will be determined automatically by the calibration.
float offsetVoltage = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  // --- Automatic Calibration ---
  Serial.println("--- ACS712 Auto-Calibration ---");
  Serial.println("Ensure NO current is flowing through the sensor.");
  Serial.println("Starting in 5 seconds...");
  delay(5000);
  Serial.println("Calibrating...");

  long totalRawValue = 0;
  for (int i = 0; i < 1000; i++) {
    totalRawValue += analogRead(SENSOR_PIN);
    delay(1);
  }

  // Calculate the precise offset voltage based on your VCC measurement
  offsetVoltage = ((float)totalRawValue / 1000.0 / 4096.0) * VCC_VOLTAGE;

  Serial.print("Calibration complete. Offset (mV): ");
  Serial.println(offsetVoltage, 2);
  Serial.println("---------------------------------");
  delay(1000);
}

void loop() {
  // --- Averaging Readings for Stability ---
  // Take multiple samples to smooth out noise.
  const int numSamples = 100;
  long totalRawValue = 0;
  for (int i = 0; i < numSamples; i++) {
    totalRawValue += analogRead(SENSOR_PIN);
  }
  float avgRawValue = (float)totalRawValue / numSamples;

  // 1. Convert the average raw ADC value to millivolts (mV).
  float voltage_mV = (avgRawValue / 4096.0) * VCC_VOLTAGE;

  // 2. Calculate the current in Amperes.
  // **IMPORTANT FIX:** We subtract the measured voltage from the offset.
  // This corrects the inverted reading (e.g., -0.50A -> 0.50A).
  // If your current ever reads negative when it should be positive,
  // simply swap the two variables back to: (voltage_mV - offsetVoltage)
  float current_A = (offsetVoltage - voltage_mV) / (SENSITIVITY * 1000);

  // 3. Print the results to the Serial Monitor.
  Serial.print("Voltage (mV): ");
  Serial.print(voltage_mV, 2);
  Serial.print("\t | Current (A): ");
  // Using the abs() function to show the absolute current,
  // which is often more useful for magnitude. Remove if you need to see the direction.
  Serial.println(abs(current_A),3); // Print with 3 decimal places for more precision

  delay(500);
}