#include <Servo.h>
#include "Adafruit_VL53L0X.h"

// 85 degree is zero; 150 max; 30 min 
// turn off print for better performance when not debugging
bool debug = false;

// Gain values
float Kp = 4;
float Kd = 8;
// For PI
float Kp2 = 1;
int Ki = 10;

// Reference distance cm
int reference = 37;

// timing related
unsigned long lastMs;
unsigned long Ms;
unsigned long updateMs = 10;


// Servo pin and variable setup
Servo servo;
int servoAngle;
int servoOffset = 85;

// VL5310
// Vin - 5v
// GND - GND
// SCL - A5
// SDA - A4
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float distanceError;
float distance;
float distanceRate;
float lastDistance;
float cumDistErr;



void setup() {
  if (debug) {
    Serial.begin(115200);    
  }
  // pin 9 to servo PWM input
  servo.attach(9);
  // Vl5310 setup
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}


float distanceRead() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  //delay(1000);
  if (measure.RangeStatus != 4) {  // returns zero if out of range
    return measure.RangeMilliMeter * 0.1;
  } else {
    return 0;
  }
}

int pControl() {
  return (int) Kp * distanceError + servoOffset;
}

int pdControl() {
  return (int) Kp * (distanceError + distanceRate * Kd) + servoOffset;
}

int piControl() {
  return servoAngle + Kp2 * (distanceRate + Ki * distanceError);
}

int pidControl() {
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
  distanceError = (reference - distance);
  distanceRate = (lastDistance - distance); // rate of change in distance cm/s
  lastDistance = distance;
  cumDistErr = cumDistErr + distanceError;
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
    Serial.print("distanceRate:"); Serial.println(distanceRate); 
    Serial.print("servoAngle: "); Serial.println(servoAngle);
  }
}
