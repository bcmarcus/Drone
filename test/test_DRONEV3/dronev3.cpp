// add feed-forward control


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

// {36, 33, 37, 15}
const int motorPins[4] = {36, 33, 37, 15};
const int numMotors = 4;
const int minThrottle = 1000;
const int maxThrottle = 2000;

// run at 15 percent power because it cant fly with that little thrust.
// const int baseThrottle = 1150;

//testing number
static int baseThrottles[4] = {1050, 1050, 1050, 1050}; // Replace 1500 with the appropriate value for your drone

Servo motors[numMotors];



// PID variables
// float multiplier = ;

float Kp = 5;
float Ki = 0.1;
float Kd = 0.1;
float Kb = 0.1;
float error_x, error_y, error_z, error_sum_x, error_sum_y, error_sum_z, error_prev_x, error_prev_y, error_prev_z;

float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;

float maxI = 100;
float minI = -100;

uint16_t calibrationCount = 500;

static boolean ready = false;
static boolean dry = true;
// MOTORS //



// SENSORS // 
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire;

#define RETRIES 3
static boolean hasBMP = false;
static boolean hasMPU = false;
static boolean hasMAG = false;
// SENSORS // 


void calibrateSensors()
{
  wire.begin();

  for (int i = 0; i < RETRIES; i++) {
    if (bmp.begin(BMP085_HIGHRES, &wire))
    {
      Serial.println("BMP180 Found!");
      hasBMP = true;
      break;
    }
    delay (1000);
  }

  for (int i = 0; i < RETRIES; i++) {
    if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &wire, 0))
    {
      Serial.println("MPU6050 Found!");
      hasMPU = true;
      break;
    }
    delay (1000);
  }

  mpu.setI2CBypass(true);

  // setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_2_5_HZ);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  for (int i = 0; i < RETRIES; i++) {
    if (mag.begin(&wire)) {
      Serial.println("HMC5883L Found!");
      hasMAG = true;
      break;
    }
    delay(1000);
  }

  sensors_event_t a, g, temp, event;
  for (int i = 0; i < calibrationCount; i++)
  {
    if (hasMPU) {
      mpu.getEvent(&a, &g, &temp);
      ax += a.acceleration.x;
      ay += a.acceleration.y;
      az += a.acceleration.z;
      gx += g.gyro.x;
      gy += g.gyro.y;
      gz += g.gyro.z;
    }
    if (hasMAG) {
      mag.getEvent(&event);
      mx += event.magnetic.x;
      my += event.magnetic.y;
      mz += event.magnetic.z;
    }
    if (hasBMP) {
      t += bmp.readTemperature();
      pres += bmp.readPressure();
      alt += bmp.readAltitude();
    }
  }

  ax /= calibrationCount;
  ay /= calibrationCount;
  az /= calibrationCount;
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

String formatFloat(float val) {
  char out[7];
  dtostrf(val, 7, 2, out);
  return String(out);
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

void resetPID () {
  error_x = 0;
  error_y = 0;
  error_z = 0;
  error_sum_x = 0;
  error_sum_y = 0;
  error_sum_z = 0;
  error_prev_x = 0;
  error_prev_y = 0;
  error_prev_z = 0;
  Serial.println("PID Reset!");
  bluetooth.println("PID Reset!");
}

void killMotors() {
  for(int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }
  resetPID();

  Serial.println("Kill command received. Motors shut down.");
  bluetooth.println("Killing motors...");
  ready = false;
}


void restartSequence() {
  bluetooth.println("Preparing motors...");
  ready = true;
  dry = false;
  bluetooth.println("Motors starting!");
}

void drySequence() {
  bluetooth.println("Preparing dry run...");
  ready = true;
  dry = true;
  bluetooth.println("Dry run beginning!");
}

void status () {
  String payload = "";
  payload = payload + "Kp: " + formatFloat(Kp) + " // ";
  payload = payload + "Ki: " + formatFloat(Ki) + " // ";
  payload = payload + "Kd: " + formatFloat(Kd) + " // ";
  payload = payload + "Kb: " + formatFloat(Kb) + " // ";
  payload = payload + "baseThrottle: " + String(baseThrottle) + " // ";
  payload = payload + "MPU: " + (hasMPU ? "Yes" : "No") + " // ";
  payload = payload + "BMP: " + (hasBMP ? "Yes" : "No") + " // ";
  payload = payload + "MAG: " + (hasMAG ? "Yes" : "No") + " // ";
  payload = payload + "Ready: " + (ready ? "Yes" : "No") + " // ";
  payload = payload + "Dry: " + (dry ? "Yes" : "No") + " // ";
  // add battery status when available
  Serial.println(payload);
  bluetooth.println(payload);
}

void takeoff() {
  Serial.println("Attempting takeoff...");
  bluetooth.println("Attempting takeoff...");
  baseThrottle = 1400;
}

void land() {
  Serial.println("Attempting landing...");
  bluetooth.println("Attempting landing...");
  baseThrottle = 1050;
}

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  // while (!Serial) {
  //   delay(10); // Wait for Serial Monitor to open
  // }
  bluetooth.println("Setup Beginning!");
  Serial.println("Setup Beginning!");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  calibrateESC();
  calibrateSensors();

  if (!hasMPU) {
    Serial.println("No Mpu found");
    bluetooth.println("No Mpu found");
    while (1){
      Serial.println("No Mpu found");
      bluetooth.println("No Mpu found");
      delay (1000);
    }
  }

  delay(100);
  bluetooth.println("Setup Complete! Preparing to launch!");
  Serial.println("Setup Complete! On Standby");
  delay(1000);
}

void loop() {

    // Replace the existing Bluetooth command parsing code
  if (bluetooth.available()) {
    lastReceivedTime = millis();
    String s = bluetooth.readStringUntil('\n').trim();
    if (s.equalsIgnoreCase("KILL")) {
      killMotors();
    }
    else if (s.equalsIgnoreCase("STATUS")) {
      status();
    }
    else if (s.equalsIgnoreCase("RESETPID")) {
      resetPID();
    }
    else if (s.equalsIgnoreCase("RESTART")) {
      restartSequence();
    }
    else if (s.equalsIgnoreCase("ISREADY")) {
      bluetooth.print(ready ? "Ready!" : "Not Ready");
    }
    else if (s.equalsIgnoreCase("TAKEOFF")) {
      takeoff();
    }
    else if (s.equalsIgnoreCase("LAND")) {
      land();
    }
    else if (s.equalsIgnoreCase("RECALIBRATE")) {
      calibrateESC();
      calibrateSensors();
    }
    else if (s.equalsIgnoreCase("DRY")) {
      drySequence();
    }
    else {
      // check pid
      String identifier = s.substring(0, 2);
      identifier.toUpperCase();

      int motorNumber = s.substring(0, s.indexOf(' ')).toInt();

      if (identifier == "KP" || identifier == "KI" || identifier == "KD" || identifier == "KB" || identifier == "BT") {
        String valueStr = s.substring(2);
        float value = valueStr.toFloat();

        if (identifier == "KP") {
          Kp = value;
          Serial.print("Kp set to: ");
          bluetooth.println("Kp set.");
        } else if (identifier == "KI") {
          Ki = value;
          Serial.print("Ki set to: ");
          bluetooth.println("Ki set.");
        } else if (identifier == "KD") {
          Kd = value;
          Serial.print("Kd set to: ");
          bluetooth.println("Kd set.");
        } else if (identifier == "KB") {
          Kb = value;
          Serial.print("Kb set to: ");
          bluetooth.println("Kb set.");
        } else if (identifier == "BT") {
          baseThrottle = value;
          Serial.print("BaseThrottle set.");
          bluetooth.println("BaseThrottle set.");
        }
        Serial.println(value, 2);
      }
      else if(motorNumber >= 1 && motorNumber <= numMotors) {
        float throttlePercent = s.substring(s.indexOf(' ') + 1).toFloat();
        throttlePercent = constrain(throttlePercent, 0, 100);
        int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
        motors[motorNumber - 1].writeMicroseconds(throttleValue);
        Serial.println("Motor " + String(motorNumber) + " set to throttle " + String(throttleValue) + "%");
        bluetooth.println("Motor " + String(motorNumber) + " set to throttle " + String(throttleValue) + "%");
      }
      else {
        killMotors();
      }
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
    error_z = a.acceleration.z - az;

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

    I_x = constrain(I_x, minI, maxI);
    I_y = constrain(I_y, minI, maxI);
    I_z = constrain(I_z, minI, maxI);

    float D_x = Kd * (error_x - error_prev_x);
    float D_y = Kd * (error_y - error_prev_y);
    float D_z = Kd * (error_z - error_prev_z);

    error_prev_x = error_x;
    error_prev_y = error_y;
    error_prev_z = error_z;

    // Calculate unsaturated output for each motor
    int unsat_throttle[numMotors];
    unsat_throttle[0] = baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z;
    unsat_throttle[1] = baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z;
    unsat_throttle[2] = baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z;
    unsat_throttle[3] = baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z;

    // Apply saturation constraints to get the saturated output
    int throttle[numMotors];
    throttle[0] = constrain(unsat_throttle[0], minThrottle, maxThrottle);
    throttle[1] = constrain(unsat_throttle[1], minThrottle, maxThrottle);
    throttle[2] = constrain(unsat_throttle[2], minThrottle, maxThrottle);
    throttle[3] = constrain(unsat_throttle[3], minThrottle, maxThrottle);



    // // Calculate motor throttles
    // int throttle[numMotors];
    // throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);




    // back calculation for integral
    float back_calc[4];
    back_calc[0] = Kb * (throttle[0] - unsat_throttle[0]);
    back_calc[1] = Kb * (throttle[1] - unsat_throttle[1]);
    back_calc[2] = Kb * (throttle[2] - unsat_throttle[2]);
    back_calc[3] = Kb * (throttle[3] - unsat_throttle[3]);

    // Calculate average back-calculation terms for x, y, and z
    float avg_back_calc_x = (back_calc[0] - back_calc[1] + back_calc[2] - back_calc[3]) / 4.0;
    float avg_back_calc_y = (-back_calc[0] - back_calc[1] + back_calc[2] + back_calc[3]) / 4.0;
    float avg_back_calc_z = (back_calc[0] + back_calc[1] + back_calc[2] + back_calc[3]) / 4.0;

    // Subtract average back-calculation terms from integral terms
    I_x -= avg_back_calc_x;
    I_y -= avg_back_calc_y;
    I_z -= avg_back_calc_z;




    // Run motors with calculated throttles
    if (!dry) {
      runMotors(throttle);
    }

    String payload = 
      "M1: " + String(throttle[0]) + " // " + 
      "M2: " + String(throttle[1]) + " // " + 
      "M3: " + String(throttle[2]) + " // " + 
      "M4: " + String(throttle[3]);

    if (dry) {
      payload = payload + " // " + 
        "P_X: " + formatFloat(P_x) + " // " + "P_Y: " + formatFloat(P_y) + " // " + "P_Z: " + formatFloat(P_z) + " // " +
        "I_x: " + formatFloat(I_x) + " // " + "I_y: " + formatFloat(I_y) + " // " + "I_z: " + formatFloat(I_z) + " // " +
        "D_x: " + formatFloat(D_x) + " // " + "D_y: " + formatFloat(D_y) + " // " + "D_z: " + formatFloat(D_z);

      payload = payload + " // " + 
        "error_x: " + formatFloat(error_x) + " // " + "error_y: " + formatFloat(error_y) + " // " + "error_z: " + formatFloat(error_z);
    }

    Serial.println(payload + " \n");
    bluetooth.println(payload);
    delay (10);
  }
}