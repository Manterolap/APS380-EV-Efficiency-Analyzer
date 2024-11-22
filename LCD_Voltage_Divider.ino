#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants for the voltage divider
const float R1 = 55000.0; // Resistor R1 value in ohms
const float R2 = 5000.0;  // Resistor R2 value in ohms

// Constants for Arduino ADC
const float ADC_REF_VOLTAGE = 5.0;  // Reference voltage of Arduino (5V)
const int ADC_RESOLUTION = 1023;   // 10-bit ADC resolution (0 to 1023)

// Variables
int analogValue = 0;               // Raw ADC value
float vOut = 0.0;                  // Voltage at the divider output
float vIn = 0.0;                   // Calculated input voltage

// Initialize the LCD (address 0x27 for a 20x4 LCD)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define the analog input pin connected to the HST016L sensor
const int currentPin = 36; // GPIO 36 (A0) on Arduino Nano ESP32
// Variables to store sensor readings and calculated current

// Sensor parameters
const float VREF = 3.3;           // Reference voltage of the ESP32 Nano (3.3V)
const float SENSITIVITY = 0.066;  // Sensitivity in V/A (e.g., 66 mV/A for HST016L)

// Number of samples for averaging
const int numSamples = 10;
int sampleIndex = 0;
float voltageSamples[numSamples] = {0};
float averageVoltage = 0;

void setup() {
  // Initialize serial communication and LCD
  Serial.begin(9600);
  pinMode(A1, INPUT);              // Set A1 as input
  //Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22 (Comment O
  lcd.init();
  lcd.backlight();

  // Display initial message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  delay(2000);

  // Measure the offset voltage (when no current is flowing)
  lcd.clear();
  lcd.setCursor(0, 0);
  delay(2000);
}

void loop() {
  // Read the analog value from A1
  analogValue = analogRead(A1);

  // Convert ADC value to output voltage (V_out)
  vOut = (analogValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE;
  
  // Calculate input voltage (V_in) based on the voltage divider formula
  vIn = vOut * ((R1 + R2) / R2);

  displayResults();
  printResults();
  delay(1000); // Update every 1 second
}

float calibrateOffset() {
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    int rawValue = analogRead(currentPin);
    float voltage = (rawValue * VREF) / ADC_RESOLUTION;
    sum += voltage;
    delay(10);
  }
  return sum / 50; // Return the average offset voltage
}

void displayResults() {
  // Clear the LCD and display the current reading
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage Out: ");
  lcd.print(vOut);
  lcd.print(" V");

  lcd.setCursor(0, 1);
  lcd.print("Voltage In: ");
  lcd.print(vIn, 3);
  lcd.print(" V");

/*
  lcd.setCursor(0, 2);
  lcd.print("Current: ");
  lcd.print(0.000, 3);
  lcd.print(" A");
*/

}

void printResults() {
  // Print the results to the Serial Monitor
  Serial.print("Voltage Out: ");
  Serial.print(vOut);
  Serial.print("\t Voltage In : ");
  Serial.print(vIn, 3);

 /*
  Serial.print(" \t Current: ");
  Serial.print(0.000, 3);
  Serial.println(" A");
*/
}
