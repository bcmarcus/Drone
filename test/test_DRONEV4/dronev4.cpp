// first pid version

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>



// BLUETOOTH // 
// Define the RX and TX pins for the Bluetooth module
const int rxPin = 7;
const int txPin = 8;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

// timeout to kill
unsigned long lastReceivedTime = 0;
const unsigned long TIMEOUT = 10000; // 10 seconds
// BLUETOOTH // 



// MOTORS //
// Motor pins
const int motorPins[4] = {15, 13, 36, 37};
const int numMotors = 4;
const int minThrottle = 1000;
const int maxThrottle = 2000;

const int baseThrottle = 1050; // Replace 1500 with the appropriate value for your drone

Servo motors[numMotors];

// PID variables
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.1;
float error_x, error_y, error_z, error_sum_x, error_sum_y, error_sum_z, error_prev_x, error_prev_y, error_prev_z;

float error_gyro_x, error_gyro_y, error_gyro_z;
float error_prev_gyro_x, error_prev_gyro_y, error_prev_gyro_z;

float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;
uint16_t calibrationCount = 100;

static boolean ready = false;
// MOTORS //



// SENSORS // 
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire1;
// SENSORS // 


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
  ready = false;
}

void restartSequence() {
  bluetooth.print("Preparing motors...");
  ready = true;
  bluetooth.print("Motors starting!");
}

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  delay(10000);
  // while (!Serial) {
  //   delay(10); // Wait for Serial Monitor to open
  // }
  bluetooth.print ("Setup Beginning!");
  Serial.print("Setup Beginning!");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  calibrateESC();

  wire.begin();

  // Initialize sensors
  bmp.begin(BMP085_HIGHRES, &wire);
  mpu.begin(MPU6050_I2CADDR_DEFAULT, &wire, 0);
  mag.begin(&wire);

  // Calibrate sensors
  calibrateSensors();
  delay(100);
  bluetooth.print ("Setup Complete! Preparing to launch!");
  Serial.print("Setup Complete! Preparing to launch!");
  delay(1000);
  bluetooth.print("5");
  Serial.print("5");
  delay(1000);
  bluetooth.print("4");
  Serial.print("4");
  delay(1000);
  bluetooth.print("3");
  Serial.print("3");
  delay(1000);
  bluetooth.print("2");
  Serial.print("2");
  delay(1000);
  bluetooth.print("1");
  Serial.print("1");
  delay(1000);
  bluetooth.print("Launching...");
  Serial.print("Launching...");
  delay(1000);
}

void loop() {

    // Replace the existing Bluetooth command parsing code
  if (bluetooth.available()) {
    lastReceivedTime = millis();
    String s = bluetooth.readStringUntil('\n');
    if (s.equalsIgnoreCase("KILL")) {
      killMotors();
    }
    else if (s.equalsIgnoreCase("RESTART")) {
      restartSequence();
    }
    else if (s.equalsIgnoreCase("ISREADY")) {
      bluetooth.print(ready ? "Ready!" : "Not Ready");
    }
    else {
      String payload = "Invalid Command: " + s;
      bluetooth.print (payload);
    }
  }
  
  if (ready) {
    // Get sensor data
    sensors_event_t a, g, temp, event;
    mpu.getEvent(&a, &g, &temp);
    mag.getEvent(&event);

    // Calculate errors
    error_x = a.acceleration.x - ax;
    error_y = a.acceleration.y - ay;
    error_z = a.acceleration.z - az - 9.81;

    // Calculate gyro errors
    error_gyro_x = g.gyro.x;
    error_gyro_y = g.gyro.y;
    error_gyro_z = g.gyro.z;

    // Calculate PID terms
    float P_x = Kp * error_x;
    float P_y = Kp * error_y;
    float P_z = Kp * error_z;

    error_sum_x += error_x;
    error_sum_y += error_y;
    error_sum_z += error_z;

    float I_x = Ki * error_sum_x;
    float I_y = Ki * error_sum_y;
    float I_z = Ki * error_sum_z;

    float D_x = Kd * (error_x - error_prev_x);
    float D_y = Kd * (error_y - error_prev_y);
    float D_z = Kd * (error_z - error_prev_z);

    error_prev_x = error_x;
    error_prev_y = error_y;
    error_prev_z = error_z;

    // Add the gyro errors to the D (derivative) terms
    float D_x = Kd * (error_x - error_prev_x) + error_gyro_x;
    float D_y = Kd * (error_y - error_prev_y) + error_gyro_y;
    float D_z = Kd * (error_z - error_prev_z) + error_gyro_z;

    // Calculate motor throttles
    int throttle[numMotors];
    throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);

    // Run motors with calculated throttles
    runMotors(throttle);

    String payload = 
      "Motor 0: " + String(throttle[0]) + " \n" + 
      "Motor 1: " + String(throttle[1]) + " \n" + 
      "Motor 2: " + String(throttle[2]) + " \n" + 
      "Motor 3: " + String(throttle[3]) + " \n";

    bluetooth.print(payload);
  }
}