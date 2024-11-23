// Define the analog pin connected to the WCS1800 sensor
const int currentSensorPin = A0;

// Sensor specifications
const float sensitivity = 72.0; // Sensitivity in mV/A
float zeroCurrentOutput = 2500; // Default zero-current voltage in mV (calibrated later)

void setup() {
  Serial.begin(9600); // Initialize serial communication
  delay(1000); // Allow time for the sensor to stabilize

  // Calibrate the zero-current voltage
  zeroCurrentOutput = calibrateZeroCurrentOutput();
  Serial.print("Calibrated Zero-Current Voltage: ");
  Serial.print(zeroCurrentOutput);
  Serial.println(" mV");
}

void loop() {
  // Get the averaged sensor value to reduce noise
  float sensorVoltage = getAverageSensorVoltage(currentSensorPin, 10);

  // Calculate current: (Vout - Vzero) / Sensitivity
  float current = (sensorVoltage - zeroCurrentOutput) / sensitivity;

  // Display the current in amperes
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" A");

  delay(500); // Update reading every 500 ms
}

// Function to read the averaged sensor voltage
float getAverageSensorVoltage(int pin, int numSamples) {
  long total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += analogRead(pin);
    delay(10); // Small delay for stability
  }
  float averageADCValue = total / numSamples;
  return averageADCValue * (5.0 / 1023.0) * 1000; // Convert to mV
}

// Function to calibrate the zero-current voltage
float calibrateZeroCurrentOutput() {
  Serial.println("Calibrating... Ensure no current is flowing through the sensor.");
  delay(3000); // Wait 3 seconds for the user to prepare
  return getAverageSensorVoltage(currentSensorPin, 50); // Take multiple samples for accuracy
}
