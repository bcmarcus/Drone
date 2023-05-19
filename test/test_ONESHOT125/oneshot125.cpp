#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <DroneMotors.hpp>

#include <Arduino.h>
#include <Servo.h>



const int motorPin = 2;
const int minThrottle = 125; // OneShot125 minimum value
const int maxThrottle = 250; // OneShot125 maximum value
bool calibrationMode = false;

Servo motor;

void calibrateESC() {
  Serial.println("Calibration started. Ensure the propeller is removed.");

  motor.writeMicroseconds(maxThrottle);
  delay(5000);
  motor.writeMicroseconds(minThrottle);
  delay(7000);

  Serial.println("Calibration complete. You can now control the motor.");
}

void runMotor(double throttle) {
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

  // Serial.println("Enter 'c' to start calibration or any other character to skip:");
  // while (Serial.available() == 0) {}
  // char inputChar = Serial.read();
  // if (inputChar == 'c') {
  //   calibrationMode = true;
  // }

  // if (calibrationMode) {
    calibrateESC();
  // }

  Serial.println("Enter desired throttle value between 0 and 10000 (ex: 987 is 9.87%):");
}

void loop() {
  if (Serial.available() > 0) {
    double throttlePercent = Serial.parseInt() / 100.0;
    throttlePercent = constrain(throttlePercent, 0, 100);
    // double throttleValue = throttlePercent / (100 / (maxThrottle - minThrottle))
    Serial.print ("Running at ");
    Serial.print (throttlePercent);
    Serial.println (" percent.");
    double throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
    runMotor(throttleValue);
    Serial.println("Enter desired throttle value between 0 and 10000 (ex: 987 is 9.87%):");
  }
}