#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants for the voltage divider
const float R1 = 55000.0; // Resistor R1 value in ohms
const float R2 = 5000.0;  // Resistor R2 value in ohms

// Constants for Arduino ADC
const float ADC_REF_VOLTAGE = 5.0;  // Reference voltage of Arduino (5V)
const int ADC_RESOLUTION = 1023;   // 10-bit ADC resolution (0 to 1023)

// Initialize the LCD (address 0x27 for a 20x4 LCD)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define the analog input pin connected to the HST016L sensor
const int voltagePin = A1; // Voltage divider pin
const int currentPin = A3; // WCS1800 current sensor pin

// Sensor parameters for the WCS1800
const float sensitivity = 72.0; // Sensitivity in mV/A for the WCS1800
float zeroCurrentOutput = 2500; // Default zero-current voltage in mV (to be calibrated)

// Variables
int analogValue = 0;               // Raw ADC value for voltage
float vOut = 0.0;                  // Voltage at the divider output
float vIn = 0.0;                   // Calculated input voltage

// Variables for current sensing
float current = 0.0;               // Current in amperes

void setup() {
  // Initialize serial communication and LCD
  Serial.begin(9600);
  pinMode(voltagePin, INPUT);      // Voltage divider input
  pinMode(currentPin, INPUT);      // Current sensor input

  lcd.init();
  lcd.backlight();

  // Display initial message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  delay(2000);

  // Measure the zero-current voltage
  zeroCurrentOutput = calibrateOffset();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Offset Calibrated:");
  lcd.setCursor(0, 1);
  lcd.print(zeroCurrentOutput);
  lcd.print(" mV");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Read the analog value from the voltage divider
  analogValue = analogRead(voltagePin);

  // Convert ADC value to output voltage (vOut)
  vOut = (analogValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE;

  // Calculate input voltage (vIn) based on the voltage divider formula
  vIn = vOut * ((R1 + R2) / R2);

  // Read the analog value from the current sensor
  int rawValue = analogRead(currentPin);
  float sensorVoltage = (rawValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE * 1000; // Convert to mV

  // Calculate current: (Vout - Vzero) / Sensitivity
  current = -1 * (sensorVoltage - zeroCurrentOutput) / sensitivity;

  // Display results on the LCD
  displayResults();

  // Print results to the Serial Monitor
  printResults();

  delay(1000); // Update every 1 second
}

// Function to calibrate the zero-current voltage
float calibrateOffset() {
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    int rawValue = analogRead(currentPin);
    float voltage = (rawValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE * 1000; // Convert to mV
    sum += voltage;
    delay(10);
  }
  return sum / 50; // Return the average offset voltage
}

void displayResults() {
  // Clear the LCD and display the current and voltage readings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage In: ");
  lcd.print(vIn, 2);
  lcd.print(" V");

  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.print(current, 2);
  lcd.print(" A");
}

void printResults() {
  // Print the results to the Serial Monitor
  Serial.print("Voltage In: ");
  Serial.print(vIn, 2);
  Serial.print(" V\t");

  Serial.print("Current: ");
  Serial.print(current, 2);
  Serial.println(" A");
}
