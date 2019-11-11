#include <Servo.h>

// turn off print for better performance when not debugging
bool debug = false;

// Gain values
uint8_t Kp = 1;
uint8_t Kd = 1;
uint8_t Ki = 1;

// Reference distance cm
uint8_t reference = 10;

// timing related
unsigned long lastMs;
unsigned long Ms;
unsigned long updateMs = 10;

// SR04 pin and variable setup
#define trigPin 11
#define echoPin 12
float distance; // cm
float lastDistance;
float distanceError;
float distanceRate; // cm/ms

// Servo pin and variable setup
Servo servo;
int servoAngle;



void setup() {
  Serial.begin(9600);
  // pin 9 to servo PWM input
  servo.attach(9);
  // Define SR04 pin mode
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

}


float distanceRead() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.5 * 29.1;
}

void loop() {
  Ms = millis(); // get current time
  if (Ms - lastMs < updateMs) {
    return; // do nothing if iterates too fast
  }
  lastMs = Ms;
  // read distance
  distance = distanceRead();
  distanceError = distance - reference;
  distanceRate = (distance - lastDistance) * 0.1; // rate of change in distance cm/ms
  lastDistance = distance;
  // pd control
  servoAngle = (int) Kp * (distanceError + distanceRate * Kd);
  // deal with saturation
  if (servoAngle < 0) {
    servoAngle = 0;
  } else if (servoAngle > 165) {
    servoAngle = 165;
  }
  servo.write(servoAngle);
  // debug print info
  if (debug) {
    Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
    Serial.print("servoAngle: "); Serial.println(servoAngle);
  }
}
