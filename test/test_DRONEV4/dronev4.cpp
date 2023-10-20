// add feed-forward control


// first pid version

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883.h>
#include <SoftwareSerial.h>
#include <unordered_map>
#include <functional>
#include <sstream>


// :: BLUETOOTH :: // 
// Define the RX and TX pins for the Bluetooth module
const int rxPin = 15;
const int txPin = 14;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
// SoftwareSerial bluetooth(rxPin, txPin);
HardwareSerial bluetooth = Serial2;

// baud
// const int baudRate = 460800;
const int baudRate = 230400;
// const int baudRate = 9600;

// timeout to kill
uint32_t lastReceivedTime = 0;
const unsigned long killTime = 30000; // 10 seconds
// static int loopDelay = 1000; // 1micro second
static int loopDelay = 0;
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
static int takeoffThrottles[4] = {1050, 1050, 1050, 1050};
// static int takeoffThrottles[4] = {1300, 1300, 1250, 1250};
static float currentThrottles[4];

Servo motors[numMotors];
// :: MOTORS :: //


// :: PID :: // 
float multiplier = loopDelay / 1000.0 / 1000.0;

uint32_t counter = 0;
uint32_t maxCount = 10000;
uint32_t timer = 0;

// float Kp = 0.05 * multiplier;
// float Ki = 0.001 * multiplier;
// float Kd = 0.001 * multiplier;
// float Kb = 0.001 * multiplier;


// KP BETWEEN 0.5 AND 1

float Kp = 0.65;
float Ki = 0.015;
float Kd = 0;
float Kb = 0;

// YAW_MULT BETWEEN 0.01 AND 0.5
float yaw_multiplier = 0.08;


float error_x, error_y, error_z, error_sum_x, error_sum_y, error_sum_z, error_prev_x, error_prev_y, error_prev_z;


float P_roll, P_pitch, P_yaw;
float I_roll, I_pitch, I_yaw;
float D_roll, D_pitch, D_yaw;

float error_roll, error_pitch, error_yaw;
float error_sum_roll, error_sum_pitch, error_sum_yaw;
float error_prev_roll, error_prev_pitch, error_prev_yaw;

float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;

float maxI = 100;
float minI = -100;

uint16_t calibrationCount = 500;

static boolean displaySensors = false;
static boolean ready = false;
static boolean dry = true;
static boolean testNoPrint = false;
// :: PID :: // 



// :: SENSORS :: // 
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883 mag = Adafruit_HMC5883(12345);
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
uint32_t batteryTimer = 0;
// :: BATTERY :: //

// :: FUNCTIONS :: //
void kill();
void status();
void resetPID();
void displaySensor();
void restartSequence();
void takeoff();
void land();
void setPrint(boolean value);
void calibrateESC();
void calibrateSensors();
void drySequence();

void setKp(float value);
void setKi(float value);
void setKd(float value);
void setKb(float value);
void setLoopDelay(float value);
void setMaxCount(float value);
void setYawMultiplier(float value);
// :: FUNCTIONS :: //

// :: COMMANDS :: //
// Define the command map
std::unordered_map<std::string, std::function<void()>> commands = {
  {"KILL", kill},
  {"STATUS", status},
  {"RESETPID", resetPID},
  {"DISPLAYSENSORS", displaySensor},
  {"RESTART", restartSequence},
  {"TAKEOFF", takeoff},
  {"LAND", land},
  {"PRINT", std::bind(setPrint, false)},
  {"NOPRINT", std::bind(setPrint, true)},
  {"RECALIBRATEESC", calibrateESC},
  {"RECALIBRATESENSORS", calibrateSensors},
  {"DRY", drySequence},
};

std::unordered_map<std::string, std::function<void(float value)>> setCommands = {
  {"KP", setKp},
  {"KI", setKi},
  {"KD", setKd},
  {"KB", setKb},
  {"LD", setLoopDelay},
  {"CT", setMaxCount},
  {"YM", setYawMultiplier}
};

void customPrint (const char* data) {
  if (testNoPrint) {
    return;
  }

  if (Serial) {
    Serial.println (data);
  }

  if (bluetooth) {
    bluetooth.println (data);
  }
}

void displaySensor() {
  displaySensors = true;
}

void setPrint(bool value) {
  testNoPrint = value;
  std::stringstream payloadStream;
  payloadStream << "Printing turned " << (value ? "off." : "on.");
  customPrint(payloadStream.str().c_str());
}

void setKp (float value) {
  Kp = value;
  customPrint("Kp set.");
}

void setKi (float value) {
  Ki = value;
  customPrint("Ki set.");
}

void setKd (float value) {
  Kd = value;
  customPrint("Kd set.");
}

void setKb (float value) {
  Kb = value;
  customPrint("Kb set.");
}

void setLoopDelay (float value) {
  loopDelay = value;
  customPrint("Loop delay set.");
}

void setMaxCount (float value) {
  maxCount = value;
  customPrint("Max count set.");
}

void setYawMultiplier (float value) {
  yaw_multiplier = value;
  customPrint("Yaw multiplier set.");
}

void calibrateSensors()
{
  wire.begin();
  wire.setClock(600000000);

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
    if (mag.begin(HMC5883_MAGGAIN_1_3, &wire)) {
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

const std::string formatFloat(float val) {
  static char out[7];
  dtostrf(val, 7, 4, out);
  return out;
}

std::string arrtostr(int arr[], int count) {
  std::stringstream payloadStream;

  if (count == 0) {
    return "[]";
  }

  payloadStream << "[";
  for (int i = 0; i < count - 1; i++) {
    payloadStream << arr[i] << ", ";
  }
  payloadStream << arr[count - 1] << "]";
  return payloadStream.str();
}

void printPIDValues () {
  std::stringstream payloadStream;

  payloadStream 
    << "M1: " << currentThrottles[0] << " // "
    << "M2: " << currentThrottles[1] << " // "
    << "M3: " << currentThrottles[2] << " // "
    << "M4: " << currentThrottles[3] << " // "
    << "P_roll: " << formatFloat(P_roll) << " // " << "P_pitch: " << formatFloat(P_pitch) << " // " << "P_yaw: " << formatFloat(P_yaw) << " // "
    << "I_roll: " << formatFloat(I_roll) << " // " << "I_pitch: " << formatFloat(I_pitch) << " // " << "I_yaw: " << formatFloat(I_yaw) << " // "
    << "D_roll: " << formatFloat(D_roll) << " // " << "D_pitch: " << formatFloat(D_pitch) << " // " << "D_yaw: " << formatFloat(D_yaw) << " // "
    << "error_roll: " << formatFloat(error_roll) << " // " << "error_pitch: " << formatFloat(error_pitch) << " // " << "error_yaw: " << formatFloat(error_yaw);

  customPrint(payloadStream.str().c_str());
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
  error_roll = 0;
  error_pitch = 0;
  error_yaw = 0;
  error_sum_roll = 0;
  error_sum_pitch = 0;
  error_sum_yaw = 0;
  error_prev_roll = 0;
  error_prev_pitch = 0;
  error_prev_yaw = 0;
  customPrint("PID Reset!");
}

void kill() {
  for(int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }

  displaySensors = false;
  testNoPrint = false;
  customPrint("Kill command received. Motors shut down.");

  printPIDValues();
  takeoff();
  resetPID();
  
  counter = 0;
  ready = false;
}

void resetBatteryData() {
  battery20 = false;
  battery5 = false;
}

void restartSequence() {
  kill();
  customPrint("Preparing motors...");
  ready = true;
  dry = false;
  timer = millis();
  customPrint("Motors starting!");
  testNoPrint = true;
}

void drySequence() {
  customPrint("Preparing dry run...");
  ready = true;
  dry = true;
  timer = millis();
  customPrint("Dry run beginning!");
}

void status () {
  std::stringstream payloadStream;
  payloadStream 
    << "Kp: " << formatFloat(Kp) << " // "
    << "Ki: " << formatFloat(Ki) << " // "
    << "Kd: " << formatFloat(Kd) << " // "
    << "Kb: " << formatFloat(Kb) << " // "
    << "Takeoff Throttles: " << arrtostr(takeoffThrottles, 4) << " // "
    << "MPU: " << (hasMPU ? "Yes" : "No") << " // "
    << "BMP: " << (hasBMP ? "Yes" : "No") << " // "
    << "MAG: " << (hasMAG ? "Yes" : "No") << " // "
    << "Ready: " << (ready ? "Yes" : "No") << " // "
    << "Dry: " << (dry ? "Yes" : "No") << " // "
    << "Loop Delay: " << formatFloat (loopDelay) << "ns // "
    << "Max Count: " << formatFloat (maxCount) << " // ";

  for (int i = 0; i < numBatteries - 1; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    int percentage = (voltage - minVoltage) / voltageRange * 100;
    payloadStream << "Battery " << (i + 1) << ": " << formatFloat(percentage) << "%" << " // ";
  }

  float voltage = (analogRead(batteryPins[numBatteries - 1]) / 1023.0) * 3.3;
  int percentage = (voltage - minVoltage) / voltageRange * 100;
  payloadStream << "Battery " << (numBatteries - 1) << ": " << formatFloat(percentage) << "%";

  testNoPrint = false;
  // add battery status when available
  customPrint(payloadStream.str().c_str());
}

void listCommands () {
  std::stringstream payloadStream;
  payloadStream 
    << "Unsuccessful Command. Useable commands are: \n"
    << "KILL \n"
    << "STATUS \n"
    << "DISPLAYSENSORS\n"
    << "RESETPID\n"
    << "RESTART\n"
    << "TAKEOFF\n"
    << "LAND\n"
    << "RECALIBRATEESC\n"
    << "RECALIBRATESENSORS\n"
    << "DRY\n"
    << "PRINT\n"
    << "NOPRINT\n"
    << "{MOTOR NUM} {THROTTLE PERCENT}\n"
    << "KP {NEW VALUE}\n"
    << "KI {NEW VALUE}\n"
    << "KD {NEW VALUE}\n"
    << "KB {NEW VALUE}\n"
    << "TT {(OPTIONAL) MOTOR NUM} {NEW VALUE}\n"
    << "LD {NEW VALUE} (this is the loop delay in nano seconds)\n"
    << "CT {NEW VALUE} (this is the counter for pid increment)\n"
    << "YM {NEW VALUE} (this is the multiplier on the yaw component)\n";

  customPrint(payloadStream.str().c_str());
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

  customPrint("Settings throttles to takeoff...");
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
  Serial.begin(9600);
  bluetooth.begin(9600);
  lastReceivedTime = millis();

  // calibrateBluetooth();
  timer = millis();
  customPrint("Setup Beginning!");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  for (int i = 0; i < numBatteries; i++) {
    pinMode(batteryPins[i], INPUT);
  }

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
  takeoff();
  kill();
}

void loop() {
  static char identifier[3];
  identifier[0] = ' ';
  identifier[1] = ' ';
  identifier[2] = '\0';

  // Replace the existing Bluetooth command parsing code
  if (bluetooth.available()) {
    lastReceivedTime = millis();
    String s = bluetooth.readStringUntil('\n').trim().toUpperCase();
    // customPrint(s.c_str());

    strncpy(identifier, s.c_str(), 2);
    identifier[2] = '\0'; // Null terminate the string

    int motorNumber = -1;
    if (s.indexOf(' ') != -1) {
      motorNumber = s.substring(0, s.indexOf(' ')).toInt();
    }

    // Check if the command exists in the map
    auto it = commands.find(s.c_str());
    auto it2 = setCommands.find(identifier);

    // complex identifier found
    if (it != commands.end()) {
        // Execute the corresponding action
        it->second();
    }
    // simple identifier found
    else if (it2 != setCommands.end()) {
      float value = s.substring(2).toFloat();
      it2->second(value);
    } 
    
    // takeoff throttle controls
    else if (strcmp(identifier, "TT") == 0) {
      int motorNumber = s.substring(3, s.indexOf(' ', 3)).toInt();
      if(motorNumber < 1 || motorNumber > numMotors) {
        float value = s.substring(3).toFloat();
        if (value < 1000 || value > 2000) {
          customPrint("Changing Takeoff Throttle: Invalid motor number.");
        } else {
          setTakeoffThrottle(1, value);
          setTakeoffThrottle(2, value);
          setTakeoffThrottle(3, value);
          setTakeoffThrottle(4, value);
        }
      } else {
        float value = s.substring(5).toFloat();
        if (value < 1000 || value > 2000) {
          customPrint("Changing Takeoff Throttle: Invalid throttle value.");
        }
        setTakeoffThrottle(motorNumber, value);
      }
    } 
    
    // motor controls
    else if(motorNumber >= 1 && motorNumber <= numMotors) {
      float throttlePercent = s.substring(s.indexOf(' ') + 1).toFloat();
      throttlePercent = constrain(throttlePercent, 0, 100);
      int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
      motors[motorNumber - 1].writeMicroseconds(throttleValue);

      std::stringstream payloadStream;
      payloadStream << "Motor " << motorNumber << " set to throttle " << ((throttleValue - 1000) / 10) << "%";

      customPrint(payloadStream.str().c_str());
    }

    // kill everything
    else if (s.length() != 0){
      kill();
      listCommands();
    }
  } else if ((millis() - lastReceivedTime) > killTime && ready) {
    kill();
  }
  
  // actual loop
  if (ready) {

    // do all of the battery checks to make sure that the drone doesnt die.
    checkBattery();

    if (battery5) {
      if (batteryTimer == 0) {
        batteryTimer = millis();
      }
      // 30 seconds then kill
      if ((millis() - batteryTimer) / 1000 > 30) {
        batteryTimer = 0;
        customPrint("Battery 5% remaining");
        // kill();
      }
    }
    
    // // Get sensor data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate errors
    // error_roll = (g.gyro.x - gx);
    // error_pitch = (g.gyro.y - gy);
    // error_yaw = (g.gyro.z - gz) * yaw_multiplier;

    // // Calculate PID terms
    // float P_roll = Kp_roll * error_roll;
    // float P_pitch = Kp_pitch * error_pitch;
    // float P_yaw = Kp_yaw * error_yaw;

    // error_sum_roll += error_roll;
    // error_sum_pitch += error_pitch;
    // error_sum_yaw += error_yaw;

    // float I_roll = Ki_roll * error_sum_roll;
    // float I_pitch = Ki_pitch * error_sum_pitch;
    // float I_yaw = Ki_yaw * error_sum_yaw;

    // I_roll = constrain(I_roll, minI, maxI);
    // I_pitch = constrain(I_pitch, minI, maxI);
    // I_yaw = constrain(I_yaw, minI, maxI);

    // float D_roll = Kd_roll * (error_roll - error_prev_roll);
    // float D_pitch = Kd_pitch * (error_pitch - error_prev_pitch);
    // float D_yaw = Kd_yaw * (error_yaw - error_prev_yaw);

    // error_prev_roll = error_roll;
    // error_prev_pitch = error_pitch;
    // error_prev_yaw = error_yaw;



    // Calculate PID terms
    // P_roll = Kp * error_roll;
    // P_pitch = Kp * error_pitch;
    // P_yaw = Kp * error_yaw;

    // error_sum_roll += error_roll;
    // error_sum_pitch += error_pitch;
    // error_sum_yaw += error_yaw;

    // I_roll = Ki * error_sum_roll;
    // I_pitch = Ki * error_sum_pitch;
    // I_yaw = Ki * error_sum_yaw;

    // I_roll = constrain(I_roll, minI, maxI);
    // I_pitch = constrain(I_pitch, minI, maxI);
    // I_yaw = constrain(I_yaw, minI, maxI);

    // D_roll = Kd * (error_roll - error_prev_roll);
    // D_pitch = Kd * (error_pitch - error_prev_pitch);
    // D_yaw = Kd * (error_yaw - error_prev_yaw);

    // error_prev_roll = error_roll;
    // error_prev_pitch = error_pitch;
    // error_prev_yaw = error_yaw;

    // // Assuming you have a function to get the current motor speed for each motor
    // float current_motor_speed_0 = get_motor_speed(1);
    // float current_motor_speed_1 = get_motor_speed(2);
    // float current_motor_speed_2 = get_motor_speed(3);
    // float current_motor_speed_3 = get_motor_speed(4);

    // // Calculate unsaturated output for each motor
    // float unsat_throttle[numMotors];
    // unsat_throttle[0] = current_motor_speed_0 - P_roll - I_roll - D_roll + P_pitch + I_pitch + D_pitch + P_yaw + I_yaw + D_yaw; // Motor 1 (positive roll, positive pitch, clockwise)
    // unsat_throttle[1] = current_motor_speed_1 - P_roll - I_roll - D_roll - P_pitch - I_pitch - D_pitch - P_yaw - I_yaw - D_yaw; // Motor 2 (positive roll, negative pitch, counterclockwise)
    // unsat_throttle[2] = current_motor_speed_2 + P_roll + I_roll + D_roll - P_pitch - I_pitch - D_pitch + P_yaw + I_yaw + D_yaw; // Motor 3 (negative roll, negative pitch, clockwise)
    // unsat_throttle[3] = current_motor_speed_3 + P_roll + I_roll + D_roll + P_pitch + I_pitch + D_pitch - P_yaw - I_yaw - D_yaw; // Motor 4 (negative roll, positive pitch, counterclockwise)

    // currentThrottles[0] = constrain(unsat_throttle[0], minThrottle, maxThrottle);
    // currentThrottles[1] = constrain(unsat_throttle[1], minThrottle, maxThrottle);
    // currentThrottles[2] = constrain(unsat_throttle[2], minThrottle, maxThrottle);
    // currentThrottles[3] = constrain(unsat_throttle[3], minThrottle, maxThrottle);


    // test all kp values

    counter++;
    if (counter % 1000 == 0) {
      std::stringstream payloadStream;
      payloadStream 
        << counter;
      boolean testNoPrintPrev = testNoPrint;
      testNoPrint = false;
      customPrint(payloadStream.str().c_str());
      testNoPrint = testNoPrintPrev;
    }
    
    if (counter == maxCount) {
      // Kp += 0.01;
      // String payload = "KP INCREASING: " + formatFloat(Kp) + " // " + "TOTAL TIME: " + formatFloat((timer - micros()) / 1000000) + "ms";
      std::stringstream payloadStream;

      payloadStream 
        << "TOTAL TIME FOR " << maxCount << " ITERATIONS: " << formatFloat((millis() - timer)) << "ms";
      boolean testNoPrintPrev = testNoPrint;
      testNoPrint = false;
      customPrint(payloadStream.str().c_str());
  
      counter = 0;
      timer = millis();
      elapsedMicros();

      printPIDValues();
      testNoPrint = testNoPrintPrev;
    }



    // // Calculate motor throttles
    // int throttle[numMotors];
    // throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);




    // // back calculation for integral
    // float back_calc[4];
    // back_calc[0] = Kb * (currentThrottles[0] - unsat_throttle[0]);
    // back_calc[1] = Kb * (currentThrottles[1] - unsat_throttle[1]);
    // back_calc[2] = Kb * (currentThrottles[2] - unsat_throttle[2]);
    // back_calc[3] = Kb * (currentThrottles[3] - unsat_throttle[3]);

    // // Calculate average back-calculation terms for x, y, and z
    // float avg_back_calc_x = (back_calc[0] - back_calc[1] + back_calc[2] - back_calc[3]) / 4.0;
    // float avg_back_calc_y = (-back_calc[0] - back_calc[1] + back_calc[2] + back_calc[3]) / 4.0;
    // float avg_back_calc_z = (back_calc[0] + back_calc[1] + back_calc[2] + back_calc[3]) / 4.0;

    // // Subtract average back-calculation terms from integral terms
    // I_roll -= avg_back_calc_x;
    // I_pitch -= avg_back_calc_y;
    // I_yaw -= avg_back_calc_z;




    // Run motors with calculated throttles
    if (!dry) {
      runMotors(currentThrottles);
    }

    printPIDValues();
    
    delayNanoseconds (loopDelay);
  }

  if (displaySensors) {
    sensors_event_t a, g, temp, event;
    mpu.getEvent(&a, &g, &temp);
    mag.getEvent(&event);
    
    std::stringstream payloadStream;
    payloadStream 
      << "Acceleration (m/s^2) => X: " << formatFloat(a.acceleration.x - ax) << ", Y: " << formatFloat(a.acceleration.y - ay) << ", Z: " << formatFloat(a.acceleration.z - az) << " ||| "
      << "Gyro (deg/s^2) => X: " << formatFloat(g.gyro.x - gx) << ", Y: " << formatFloat(g.gyro.y - gy) << ", Z: " << formatFloat(g.gyro.z - gz) << " ||| "
      << "Magnetic (uT) => X: " << formatFloat(event.magnetic.x) << "  " << "Y: " << formatFloat(event.magnetic.y) << "  " << "Z: " << formatFloat(event.magnetic.z) << " ||| ";



    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    // float declinationAngle = 0.22;
    // heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    payloadStream
      << "Heading (degrees): " << formatFloat(headingDegrees)
      << "Temperature (*C): " << formatFloat(bmp.readTemperature()) << " ||| "
      << "Pressure (Pa): " << formatFloat(bmp.readPressure()) << ", Altitude (meters): " << formatFloat(bmp.readAltitude());

    customPrint(payloadStream.str().c_str());
    delay(100);
  }
}