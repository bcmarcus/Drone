#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <DroneMotors.hpp>

#include <Arduino.h>
#include <Servo.h>

// UNPLUG, THEN WAIT FOR PROMPT, AND PLUG IN

const int motorPin = 15;
const int minThrottle = 1000; // PWM minimum value
const int maxThrottle = 2000; // PWM maximum value
bool calibrationMode = false;

Servo motor;

void calibrateESC() {
  Serial.println("Calibration started. Now power the motor.");

  motor.writeMicroseconds(maxThrottle);
  delay(10000);
  motor.writeMicroseconds(minThrottle);
  delay(2000);

  Serial.println("Calibration complete. You can now control the motor.");
}

void runMotor(int throttle) {
  motor.writeMicroseconds(throttle);
  delay(3000);
  motor.writeMicroseconds(minThrottle);
}

void setup() {
  motor.attach(motorPin);

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial Monitor to open
  }

  calibrateESC();

  Serial.println("Enter desired throttle value between 0 and 100:");
}

void loop() {
  if (Serial.available() > 0) {
    int throttlePercent = Serial.parseInt();
    throttlePercent = constrain(throttlePercent, 0, 100);
    int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
    runMotor(throttleValue);
    Serial.println("Enter desired throttle value between 0 and 100:");
  }
}