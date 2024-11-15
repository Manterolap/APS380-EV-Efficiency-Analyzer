#include <math.h>

int hallSensorPin = A1;     
int ledPin =  2;    
int state = 0;
int count = 0;
int wheel_diam = 40; // in cm
int dist = 0;
int passing = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);      
  pinMode(hallSensorPin, INPUT);     
}

void loop(){
  
  state = analogRead(hallSensorPin);
  Serial.print("Hall Sensor: ");
  Serial.println(state);

  if (state < 400) {        
    Serial.println("One Revolution!");
    if (passing == 0) { 
      digitalWrite(ledPin, HIGH);
      int rev = wheel_diam*M_PI;
      dist = dist + rev;
      passing = 1;
    }
  } 
  else {
    digitalWrite(ledPin, LOW); 
    if (passing == 1){
      count++;
      passing = 0;
    }
  }
  Serial.print("Total distance travelled: ");
  Serial.println(dist);
  delay(500);
}