#include <math.h>
#include <avr/interrupt.h>

// Pin definitions
const int hallSensorPin = A1;  // Analog pin for Hall effect sensor
const int ledPin = 2;          // LED pin for visual indicator

// Bicycle and calculation variables
volatile int state = 0;        // State of the Hall effect sensor
volatile int passing = 0;      // Tracks whether the magnet is passing
volatile float wheelDiam = 40.0;  // Wheel diameter in cm (adjust if needed)
volatile float wheelCircumference = 0.0; // Calculated circumference in cm
volatile float distance = 0.0;  // Total distance traveled in cm
volatile int revolutionCount = 0; // Revolution count

// Timer variables
volatile unsigned long lastPulseTime = 0; // Last time the Hall effect sensor triggered (micros)
volatile unsigned long currentPulseTime = 0; // Current time the Hall effect sensor triggered (micros)
volatile float speed = 0.0; // Speed in cm/s

void setup() {
  Serial.begin(9600);

  // Pin setup
  pinMode(ledPin, OUTPUT);
  pinMode(hallSensorPin, INPUT);

  // Calculate wheel circumference
  wheelCircumference = wheelDiam * M_PI;

  // Timer setup (Timer1 for 1 ms interval)
  noInterrupts();             // Disable interrupts during setup
  TCCR1A = 0;                 // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1 = 0;                  // Initialize counter to 0
  OCR1A = 15999;              // Compare match register for 1 ms (16 MHz clock / 1024 prescaler)
  TCCR1B |= (1 << WGM12);     // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);    // Enable Timer1 compare interrupt
  interrupts();               // Enable interrupts
}

void loop() {
  // Main loop displays speed and distance at regular intervals
  Serial.print("Speed: ");
  Serial.print(speed / 100.0);  // Convert cm/s to m/s
  Serial.print(" m/s, Distance: ");
  Serial.print(distance / 100.0);  // Convert cm to meters
  Serial.println(" m");

  delay(500);  // Adjust for the desired display update rate
}

// Hall effect sensor pulse detection function
void detectRevolution() {
  state = analogRead(hallSensorPin);

  // Trigger revolution detection on low sensor state
  if (state < 400) {
    if (passing == 0) { // Ensure only one pulse per magnet pass
      digitalWrite(ledPin, HIGH); // Turn on LED
      currentPulseTime = micros(); // Record current time
      unsigned long timeDiff = currentPulseTime - lastPulseTime;

      if (timeDiff > 0) {
        speed = wheelCircumference / (timeDiff / 1000000.0); // Speed in cm/s
      }

      lastPulseTime = currentPulseTime; // Update last pulse time
      distance += wheelCircumference;  // Increment total distance
      revolutionCount++;               // Increment revolution count
      passing = 1;
    }
  } else {
    digitalWrite(ledPin, LOW); // Turn off LED when magnet is not passing
    passing = 0;              // Reset passing state
  }
}

// Timer1 interrupt service routine (1 ms interval)
ISR(TIMER1_COMPA_vect) {
  detectRevolution(); // Check Hall effect sensor state in the interrupt
}
