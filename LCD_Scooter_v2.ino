#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <avr/interrupt.h>

// Constants for the voltage divider
const float R1 = 55000.0; // Resistor R1 value in ohms
const float R2 = 5000.0;  // Resistor R2 value in ohms

// Constants for Arduino ADC
const float ADC_REF_VOLTAGE = 4.1;  // Reference voltage of Arduino (3.3V for ESP32)
const int ADC_RESOLUTION = 1024;   // 12-bit ADC resolution (0 to 4095)

// Initialize the LCD (address 0x27 for a 20x4 LCD)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define the analog input pin connected to the HST016L sensor
const int voltagePin    = A1;  // Voltage divider pin (use any available analog pin)
const int hallSensorPin = A2;  // Hall-effect sensor pin
const int currentPin    = A3;  // WCS1800 current sensor pin

// Sensor parameters for the WCS1800
const float sensitivity = 72.0; // Sensitivity in mV/A for the WCS1800
float zeroCurrentOutput = 2500; // Default zero-current voltage in mV (to be calibrated)

// Variables
int analogValue = 0;               // Raw ADC value for voltage
float vOut = 0.0;                  // Voltage at the divider output
float vIn = 0.0;                   // Calculated input voltage
float current = 0.0;               // Current in amperes
float instPower = 0.0;             // Instantaneous power in W
float avgCurrent = 0.0;            // Averaged current to reduce noise
float instPowerAvg = 0.0;          // Averaged power for smooth output

// Moving average parameters (for smoothing sensor data)
const int filterSize = 3;
float currentReadings[filterSize];
int currentIndex = 0;

// Variables for speed and distance
volatile int state = 0;        // State of the Hall effect sensor
volatile int passing = 0;      // Tracks whether the magnet is passing
volatile float wheelDiam = 6.5;  // Wheel diameter in cm (adjust if needed)
volatile float wheelCircumference = 0.0; // Calculated circumference in cm
volatile float distance = 0.0;  // Total distance traveled in cm
volatile int revolutionCount = 0; // Revolution count

// Timer variables
volatile unsigned long lastPulseTime = 0; // Last time the Hall effect sensor triggered (micros)
volatile unsigned long currentPulseTime = 0; // Current time the Hall effect sensor triggered (micros)
volatile unsigned long noPulseTime = 0; // Time in between pulses (micros)

volatile float speed = 0.0; // Speed in cm/s

void setup() {
  // Initialize serial communication and LCD
  Serial.begin(9600);
  pinMode(voltagePin, INPUT);      // Voltage divider input
  pinMode(currentPin, INPUT);      // Current sensor input
  pinMode(hallSensorPin, INPUT);   // Hall effect sensor input

  lcd.init();
  lcd.backlight();

  // Calculate wheel circumference
  wheelCircumference = wheelDiam * M_PI;
  // Timer setup (Timer1 for 1 ms interval)
  noInterrupts();             // Disable interrupts during setup
  TCCR1A = 0;                 // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1 = 0;                  // Initialize counter to 0
  OCR1A = 99;              // Compare match register for 1 ms (16 MHz clock / 1024 prescaler)
  TCCR1B |= (1 << WGM12);     // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);    // Enable Timer1 compare interrupt
  interrupts();               // Enable interrupts

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
  // Read and average the analog value from the voltage divider
  analogValue = analogRead(voltagePin);
  vOut = (analogValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE;
  vIn = vOut * ((R1 + R2) / R2);

  // Read and smooth current sensor values
  float rawCurrent = readAndFilterCurrent();

  // Calculate instantaneous power
  instPower = rawCurrent * vIn;

  delay(500);  // Adjust for the desired display update rate

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

// Function to read and filter current sensor readings
float readAndFilterCurrent() {
  int rawValue = analogRead(currentPin);
  float sensorVoltage = (rawValue / float(ADC_RESOLUTION)) * ADC_REF_VOLTAGE * 1000; // Convert to mV

  // Calculate current (accounting for the offset)
  float currentReading = -1 * (sensorVoltage - zeroCurrentOutput) / sensitivity;

  // Apply moving average filter
  currentReadings[currentIndex] = currentReading;
  currentIndex = (currentIndex + 1) % filterSize;

  // Calculate the average current from the filter
  float sum = 0;
  for (int i = 0; i < filterSize; i++) {
    sum += currentReadings[i];
  }
  avgCurrent = abs(sum / filterSize);

  return avgCurrent;
}

void detectRevolution() {
  state = analogRead(hallSensorPin);

  // Trigger revolution detection on low sensor state
  if (state < 650) {
    noPulseTime = 0;
    if (passing == 0) { // Ensure only one pulse per magnet pass
      currentPulseTime = micros(); // Record current time
      unsigned long timeDiff = 0;
      if (currentPulseTime >= lastPulseTime){
        timeDiff = currentPulseTime - lastPulseTime;
      }
      else {
        timeDiff = 999999999 - lastPulseTime - currentPulseTime;
      }

      if (timeDiff > 0) {
        speed = wheelCircumference / (timeDiff / 1000000.0); // Speed in cm/s
      }

      lastPulseTime = currentPulseTime; // Update last pulse time
      distance += wheelCircumference;  // Increment total distance
      revolutionCount++;               // Increment revolution count
      passing = 1;
    }
  } else {
    passing = 0;              // Reset passing state

    noPulseTime += micros();
    if (noPulseTime >= 500000000)
    {
      speed = 0;
    }
  }
}

// Function to display results on the LCD
void displayResults() {
  // Clear the LCD and display the current and voltage readings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Power: ");
  lcd.print(instPower, 4);
  lcd.print(" W");

  lcd.setCursor(0, 1);
  lcd.print("Eff: ");
  if(speed != 0){
    lcd.print(instPower/(speed * 0.036), 4); //convert Ws/cm to Wh/km
  } else {
    lcd.print("0");
  }
  lcd.print(" Wh/km");

  lcd.setCursor(0, 2);
  lcd.print(vIn, 2);
  lcd.print(" V,   ");
  lcd.print(avgCurrent, 2);
  lcd.print(" A");

    // Main loop displays speed and distance at regular intervals
  lcd.setCursor(0, 3);
  //lcd.print("Speed: ");
  lcd.print(state);
  lcd.print("  ");
  lcd.print(speed * 0.036);  // Convert cm/s to km/h
  lcd.print(" km/h");
  // lcd.print("Distance: ");
  // lcd.print(distance / 100000.0);  // Convert cm to km
  // lcd.print(" m");
  // lcd.setCursor(0, 4);
  // lcd.print(currentPulseTime);
}

// Function to print results to the Serial Monitor
void printResults() {
  // Print the results to the Serial Monitor
  // Serial.print("Voltage In: ");
  // Serial.print(vIn, 2);
  // Serial.print(" V\t");

  // Serial.print("Current: ");
  // Serial.print(avgCurrent, 2);
  // Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(instPower, 4);
  Serial.println(" W");

  Serial.print("Hall: ");
  Serial.print(state);
  Serial.println(" ");

  // Main loop displays speed and distance at regular intervals
  Serial.print("Speed: ");
  Serial.print(speed / 100.0);  // Convert cm/s to m/s
  Serial.print(" m/s, Distance: ");
  Serial.print(distance / 100.0);  // Convert cm to meters
  Serial.println(" m");
}


// Timer1 interrupt service routine (1 ms interval)
ISR(TIMER1_COMPA_vect) {
  detectRevolution(); // Check Hall effect sensor state in the interrupt
}
