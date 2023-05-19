// bluetooth version


#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>

// Motor pins
const int motorPins[4] = {15, 33, 36, 37};
const int numMotors = 4;
const int minThrottle = 1000;
const int maxThrottle = 2000;

const int baseThrottle = 1050; // Replace 1500 with the appropriate value for your drone

Servo motors[numMotors];

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire1;

// Define the RX and TX pins for the Bluetooth module
const int rxPin = 7;
const int txPin = 8;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

// timeout to kill
unsigned long lastReceivedTime = 0;
const unsigned long TIMEOUT = 10000; // 10 seconds


float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;
uint16_t calibrationCount = 100;

void calibrateSensors()
{
  sensors_event_t a, g, temp, event;
  for (int i = 0; i < calibrationCount; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    mag.getEvent(&event);
    ax += a.acceleration.x;
    ay += a.acceleration.y;
    az += a.acceleration.z;
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;
    mx += event.magnetic.x;
    my += event.magnetic.y;
    mz += event.magnetic.z;
    t += bmp.readTemperature();
    pres += bmp.readPressure();
    alt += bmp.readAltitude();
  }

  ax /= calibrationCount;
  ay /= calibrationCount;
  az /= calibrationCount;
  az -= 9.81;
  gx /= calibrationCount;
  gy /= calibrationCount;
  gz /= calibrationCount;
  mx /= calibrationCount;
  my /= calibrationCount;
  mz /= calibrationCount;
  t /= calibrationCount;
  pres /= calibrationCount;
  alt /= calibrationCount;

}

void calibrateESC() {
  Serial.println("Calibration started. Now power the motors.");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  for (int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(maxThrottle);
  }
  delay(10000);

  for (int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }
  delay(2000);

  Serial.println("Calibration complete. You can now control the motors.");
}

void runMotors(int throttle[]) {
  for (int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(throttle[i]);
  }
}

void killMotors() {
  for(int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }
  Serial.println("Kill command received. Motors shut down.");
  bluetooth.print("Killing motors...");
}

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  
  // while (!Serial && !bluetooth) {
    // delay(10); // Wait for Serial Monitor to open
  // }

  Serial.print("Setup Beginning on DroneV2!");
  bluetooth.print("Setup Beginning on DroneV2!");

  calibrateESC();

  wire.begin();

  // Initialize sensors
  bmp.begin(BMP085_HIGHRES, &wire);
  mpu.begin(MPU6050_I2CADDR_DEFAULT, &wire, 0);
  mag.begin(&wire);

  // Calibrate sensors
  calibrateSensors();
  delay(100);
  Serial.print("Setup Complete!");
}

void loop () {
  // Forward data from the Bluetooth module to the serial monitor
  if (bluetooth.available()) {
    lastReceivedTime = millis();
    String s = bluetooth.readStringUntil('\n');
    if(s.equalsIgnoreCase("KILL")) {
      killMotors();
    }
    else if(s.equalsIgnoreCase("RECALIBRATE")) {
      calibrateESC();
    }
    else {
      int motorNumber = s.substring(0, s.indexOf(' ')).toInt();
      if(motorNumber < 0 || motorNumber >= numMotors) {
        killMotors();
      }
      else {
        float throttlePercent = s.substring(s.indexOf(' ') + 1).toFloat();
        throttlePercent = constrain(throttlePercent, 0, 100);
        int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
        motors[motorNumber].writeMicroseconds(throttleValue);
        Serial.println("Motor " + String(motorNumber) + " set to throttle " + String(throttleValue) + "%");
      }
    }
  }

  if (millis() - lastReceivedTime > TIMEOUT) {
    killMotors();
  }
  delay (100);
}