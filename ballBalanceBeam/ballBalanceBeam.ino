#include <Servo.h>

// 85 degree is zero; 150 max; 30 min 
// turn off print for better performance when not debugging
bool debug = true;

// Gain values
uint8_t Kp = 10;
uint8_t Kd = 10;
uint8_t Ki = 1;

// Reference distance cm
uint8_t reference = 30;

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
uint8_t servoAngle;
uint8_t servoOffset = 85;




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
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  //delay(1000);
  return duration * 0.5 * 0.000343;
}

uint8_t pControl() {
  return (int) Kp * distanceError + servoOffset;
}

uint8_t pdControl() {
  return (int) Kp * (distanceError + distanceRate * Kd) + servoOffset;
}

uint8_t piControl() {
  return 1;
}

uint8_t pidControl() {
  return 1;
}

void loop() {
  Ms = millis(); // get current time
  if (Ms - lastMs < updateMs) {
    return; // do nothing if iterates too fast
  }
  lastMs = Ms;
  // read distance
  distance = distanceRead();
  // check sensor reading is within reasonable range
  while (distance > 90 || distance < 0) {
    distance = distanceRead();
  }
  distanceError = reference - distance;
  distanceRate = (lastDistance - distance) * 100; // rate of change in distance m/s
  lastDistance = distance;
  servoAngle = pdControl();
  // deal with saturation
  if (servoAngle < 30) {
    servoAngle = 30;
  } else if (servoAngle > 150) {
    servoAngle = 150;
  }
  servo.write(servoAngle);
  // debug print info
  if (debug) {
    Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
    Serial.print("distance error:"); Serial.println(distanceError); 
    Serial.print("servoAngle: "); Serial.println(servoAngle);
  }
}
