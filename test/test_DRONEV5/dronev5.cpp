// first pid version

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

// Motor pins
const int motorPins[4] = {15, 13, 36, 37};
const int numMotors = 4;
const int minThrottle = 1000;
const int maxThrottle = 2000;

const int baseThrottle = 1050; // Replace 1500 with the appropriate value for your drone

Servo motors[numMotors];

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire1;

// PID variables
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.1;
float error_x, error_y, error_z, error_sum_x, error_sum_y, error_sum_z, error_prev_x, error_prev_y, error_prev_z;

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

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {
    delay(10); // Wait for Serial Monitor to open
  }

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
  Serial.print("Setup Complete!");
}

void loop() {
  // Get sensor data
  sensors_event_t a, g, temp, event;
  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&event);

  // Calculate errors
  error_x = a.acceleration.x - ax;
  error_y = a.acceleration.y - ay;
  error_z = a.acceleration.z - az - 9.81;

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

  // Calculate motor throttles
  int throttle[numMotors];
  throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
  throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
  throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
  throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);

  // Run motors with calculated throttles
  runMotors(throttle);

  // Print throttle values if PRINT flag is true
  bool PRINT = true;
  if (PRINT) {
    for (int i = 0; i < numMotors; i++) {
      Serial.print("Throttle ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(throttle[i]);
    }
  }

  // Add a delay for easier reading of printed values
  if (PRINT) {
    delay(500);
  }
}