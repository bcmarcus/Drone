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



// :: BLUETOOTH :: // 
// Define the RX and TX pins for the Bluetooth module
const int rxPin = 15;
const int txPin = 14;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

// baud
// const int baudRate = 460800;
const int baudRate = 230400;
// const int baudRate = 9600;

// timeout to kill
unsigned long lastReceivedTime = 0;
const unsigned long TIMEOUT = 10000; // 10 seconds
const int loopDelay = 1000 * 1000; // 1000 nanoseconds
// :: BLUETOOTH :: // 



// :: MOTORS :: //
// Motor pins
// const int motorPins[4] = {0, 29, 33, 19};
const int motorPins[4] = {33, 19, 0, 29};
const int numMotors = 4;
const int minThrottle = 1000;
const int maxThrottle = 2000;

// run at 15 percent power because it cant fly with that little thrust.
// const int baseThrottle = 1150;

//testing number
static int landThrottles[4] = {1050, 1050, 1050, 1050};
// static int takeoffThrottles[4] = {1050, 1050, 1050, 1050};
static int takeoffThrottles[4] = {1325, 1325, 1250, 1250};
static float currentThrottles[4];

Servo motors[numMotors];
// :: MOTORS :: //


// :: PID :: // 
float multiplier = loopDelay / 1000.0 / 1000.0;

// float Kp = 0.05 * multiplier;
// float Ki = 0.001 * multiplier;
// float Kd = 0.001 * multiplier;
// float Kb = 0.001 * multiplier;

float Kp = 0.05 * multiplier;
float Ki = 0;
float Kd = 0;
float Kb = 0;

float error_x, error_y, error_z, error_sum_x, error_sum_y, error_sum_z, error_prev_x, error_prev_y, error_prev_z;

float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;

float maxI = 100;
float minI = -100;

uint16_t calibrationCount = 500;

static boolean ready = false;
static boolean dry = true;
static boolean testNoPrint = false;
// :: PID :: // 



// :: SENSORS :: // 
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire1;

#define RETRIES 3
static boolean hasBMP = false;
static boolean hasMPU = false;
static boolean hasMAG = false;
// :: SENSORS :: // 


// :: BATTERY :: //
const int batteryPins[] = {20, 21, 22, 23};
const int numBatteries = 4;
const float maxVoltage = 2.08;
const float minVoltage = 1.7;
const float voltageRange = maxVoltage - minVoltage;
volatile bool battery20 = false;
volatile bool battery5 = false;
// :: BATTERY :: //

void customPrint (String data) {
  if (testNoPrint) {
    return;
  }
  if (battery20) {
    data = data + " // " + "BATTERY IS LOW, PLAN TO RETURN TO BASE.";
  }
  if (battery5) {
    data = data + " // " + "BATTERY AT 5, IF YOU DO NOT SEND AN OVERRIDE COMMAND WITHIN 30 SECONDS, THE DRONE WILL TURN ITSELF OFF.";
  }
  Serial.println (data);
  bluetooth.println (data);
}

void calibrateSensors()
{
  wire.begin();

  for (int i = 0; i < RETRIES; i++) {
    if (bmp.begin(BMP085_HIGHRES, &wire))
    {
      customPrint("BMP180 Found!");
      hasBMP = true;
      break;
    }
    delay (1000);
  }

  for (int i = 0; i < RETRIES; i++) {
    if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &wire, 0))
    {
      customPrint("MPU6050 Found!");
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
      customPrint("HMC5883L Found!");
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
  dtostrf(val, 7, 4, out);
  return String(out);
}

String arrtostr(int arr[], int count) {
  if (count == 0) {
    return "[]";
  }

  String out = "[";
  for (int i = 0; i < count - 1; i++) {
    out = out + arr[i] + ", ";
  }
  out = out + arr[count - 1] + "]";

  return out;
}

void calibrateESC() {
  customPrint("Calibration started. Now power the motors.");

  for (int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(maxThrottle);
  }
  delay(10000);

  for (int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }
  delay(2000);

  customPrint("Calibration complete. You can now control the motors.");
}

void runMotors(float throttle[]) {
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
  customPrint("PID Reset!");
}

void killMotors() {
  for(int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }
  resetPID();

  for (int i = 0; i < numMotors; i++) {
    currentThrottles[i] = takeoffThrottles[i];
  }
  
  customPrint("Kill command received. Motors shut down.");
  ready = false;
}

void resetBatteryData() {
  battery20 = false;
  battery5 = false;
}

void restartSequence() {
  killMotors();
  customPrint("Preparing motors...");
  ready = true;
  dry = false;
  customPrint("Motors starting!");
}

void drySequence() {
  customPrint("Preparing dry run...");
  ready = true;
  dry = true;
  customPrint("Dry run beginning!");
}

void status () {
  String payload = "";
  payload = payload + "Kp: " + formatFloat(Kp) + " // ";
  payload = payload + "Ki: " + formatFloat(Ki) + " // ";
  payload = payload + "Kd: " + formatFloat(Kd) + " // ";
  payload = payload + "Kb: " + formatFloat(Kb) + " // ";
  payload = payload + "Takeoff Throttles: " + arrtostr(takeoffThrottles, 4) + " // ";
  payload = payload + "MPU: " + (hasMPU ? "Yes" : "No") + " // ";
  payload = payload + "BMP: " + (hasBMP ? "Yes" : "No") + " // ";
  payload = payload + "MAG: " + (hasMAG ? "Yes" : "No") + " // ";
  payload = payload + "Ready: " + (ready ? "Yes" : "No") + " // ";
  payload = payload + "Dry: " + (dry ? "Yes" : "No") + " // ";
  
  for (int i = 0; i < numBatteries; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    int percentage = (voltage - minVoltage) / voltageRange * 100;
    payload = payload + "Battery " + String ((i + 1)) + ": " + formatFloat(percentage) + "%" + " // ";
  }

  // add battery status when available
  customPrint(payload);
}

void checkBattery() {
  for (int i = 0; i < numBatteries; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    int percentage = (voltage - minVoltage) / voltageRange * 100;
    if (percentage < 20) {
      battery20 = true;
    }
    if (percentage < 5) {
      battery5 = true;
    }
  }
}

void setTakeoffThrottle (int motor, float value) {
  takeoffThrottles [motor - 1] = value;
  customPrint("TakeoffThrottle set.");
}

void takeoff() {
  for (int i = 0; i < numMotors; i++) {
    currentThrottles[i] = takeoffThrottles[i];
  }

  customPrint("Attempting takeoff...");
}

void land() {
  for (int i = 0; i < numMotors; i++) {
    currentThrottles[i] = landThrottles[i];
  }
  customPrint("Attempting landing...");
}

float get_motor_speed(int motor) {
  return currentThrottles[motor - 1];
}

void calibrateBluetooth() {
  bluetooth.begin(230400);
  bluetooth.print("AT+NAMEDrone1");
  delay(1000);
  bluetooth.print("AT+BAUD4"); // AT command to set baud rate to 230400
  delay(1000);
  bluetooth.end();
  bluetooth.begin(9600);
}

void setup() {
  Serial.begin(baudRate);
  bluetooth.begin(9600);
  // calibrateBluetooth();

  customPrint("Setup Beginning!");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  for (int i = 0; i < numBatteries; i++) {
    pinMode(batteryPins[i], INPUT);
  }

  // bluetooth //
  calibrateESC();
  calibrateSensors();

  if (!hasMPU) {
    customPrint("No Mpu found");
    while (1){
      customPrint("No Mpu found");
      delay (1000);
    }
  }

  delay(100);
  customPrint("Setup Complete! On Standby");
  delay(1000);
  killMotors();
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
    else if (s.equalsIgnoreCase("PRINT")) {
      testNoPrint = false;
    }
    else if (s.equalsIgnoreCase("NOPRINT")) {
      testNoPrint = true;
    }
    else if (s.equalsIgnoreCase("RECALIBRATE")) {
      calibrateESC();
      calibrateSensors();
    }
    else if (s.equalsIgnoreCase("DRY")) {
      drySequence();
    }
    else {
      
      
      
      // change values
      String identifier = s.substring(0, 2);
      identifier.toUpperCase();

      int motorNumber = s.substring(0, s.indexOf(' ')).toInt();

      if (identifier == "KP" || identifier == "KI" || identifier == "KD" || identifier == "KB" || identifier == "TT") {
        String valueStr = s.substring(2);
        float value = valueStr.toFloat();

        if (identifier == "KP") {
          Kp = value;
          customPrint("Kp set.");
        } else if (identifier == "KI") {
          Ki = value;
          customPrint("Ki set.");
        } else if (identifier == "KD") {
          Kd = value;
          customPrint("Kd set.");
        } else if (identifier == "KB") {
          Kb = value;
          customPrint("Kb set.");
        } else if (identifier == "TT") {
          int motorNumber = s.substring(3, s.indexOf(' ', 3)).toInt();
          if(motorNumber < 1 || motorNumber > numMotors) {
            customPrint("Changing Takeoff Throttle: Invalid motor number.");
          }
          float value = s.substring(5).toFloat();
          if (value < 1000 || value > 2000) {
            customPrint("Changing Takeoff Throttle: Invalid throttle value.");
          }
          setTakeoffThrottle(motorNumber, value);
        }
      }
      else if(motorNumber >= 1 && motorNumber <= numMotors) {
        float throttlePercent = s.substring(s.indexOf(' ') + 1).toFloat();
        throttlePercent = constrain(throttlePercent, 0, 100);
        int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
        motors[motorNumber - 1].writeMicroseconds(throttleValue);
        customPrint("Motor " + String(motorNumber) + " set to throttle " + String((throttleValue - 1000) / 10) + "%");
      }
      else {
        killMotors();
      }
    }
  }
  




  // actual loop
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

    // Assuming you have a function to get the current motor speed for each motor
    float current_motor_speed_0 = get_motor_speed(1);
    float current_motor_speed_1 = get_motor_speed(2);
    float current_motor_speed_2 = get_motor_speed(3);
    float current_motor_speed_3 = get_motor_speed(4);

    // Calculate unsaturated output for each motor
    float unsat_throttle[numMotors];
    unsat_throttle[0] = current_motor_speed_0 - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z;
    unsat_throttle[1] = current_motor_speed_1 + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z;
    unsat_throttle[2] = current_motor_speed_2 + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z;
    unsat_throttle[3] = current_motor_speed_3 - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z;


    currentThrottles[0] = constrain(unsat_throttle[0], minThrottle, maxThrottle);
    currentThrottles[1] = constrain(unsat_throttle[1], minThrottle, maxThrottle);
    currentThrottles[2] = constrain(unsat_throttle[2], minThrottle, maxThrottle);
    currentThrottles[3] = constrain(unsat_throttle[3], minThrottle, maxThrottle);



    // // Calculate motor throttles
    // int throttle[numMotors];
    // throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);




    // back calculation for integral
    float back_calc[4];
    back_calc[0] = Kb * (currentThrottles[0] - unsat_throttle[0]);
    back_calc[1] = Kb * (currentThrottles[1] - unsat_throttle[1]);
    back_calc[2] = Kb * (currentThrottles[2] - unsat_throttle[2]);
    back_calc[3] = Kb * (currentThrottles[3] - unsat_throttle[3]);

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
      runMotors(currentThrottles);
    }

    String payload = 
      "M1: " + String(currentThrottles[0]) + " // " + 
      "M2: " + String(currentThrottles[1]) + " // " + 
      "M3: " + String(currentThrottles[2]) + " // " + 
      "M4: " + String(currentThrottles[3]);

    payload = payload + " // " + 
      "P_X: " + formatFloat(P_x) + " // " + "P_Y: " + formatFloat(P_y) + " // " + "P_Z: " + formatFloat(P_z) + " // " +
      "I_x: " + formatFloat(I_x) + " // " + "I_y: " + formatFloat(I_y) + " // " + "I_z: " + formatFloat(I_z) + " // " +
      "D_x: " + formatFloat(D_x) + " // " + "D_y: " + formatFloat(D_y) + " // " + "D_z: " + formatFloat(D_z);

    payload = payload + " // " + 
      "error_x: " + formatFloat(error_x) + " // " + "error_y: " + formatFloat(error_y) + " // " + "error_z: " + formatFloat(error_z);

    customPrint(payload);
    delayNanoseconds (loopDelay);
  }
}