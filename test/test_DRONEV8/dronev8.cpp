// add feed-forward control


// first pid version

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <unordered_map>
#include <functional>
#include <sstream>
#include <cstring>

#include <GY87.h>

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_BIT_I2C_BYPASS_EN 0x02

#define GYRO_COEFF 0.95
// :: BLUETOOTH :: // 
// Define the RX and TX pins for the Bluetooth module
const int rxPin = 15;
const int txPin = 14;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);
// HardwareSerial bluetooth = Serial3;

// baud
// const int baudRate = 460800;
// const int baudRate = 115200;
const int baudRate = 9600;

// timeout to kill
uint32_t lastReceivedTime = 0;
const unsigned long batteryKillTime = 30000; // 30 seconds
const unsigned long killTime = 60000; // 60 seconds
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
static int takeoffThrottles[4] = {1150, 1150, 1150, 1150};
// static int takeoffThrottles[4] = {1050, 1050, 1050, 1050};
static float currentThrottles[4];

Servo motors[numMotors];
// :: MOTORS :: //


// :: PID :: // 

// MagwickAHRS //
uint32_t loopTime, loopPrev;
float deltaTime;

uint32_t counter = 0;
uint32_t maxCount = 100000;
uint32_t timer = 0;

boolean imuOnly = true;

// float Kp = 0.05 * multiplier;
// float Ki = 0.001 * multiplier;
// float Kd = 0.001 * multiplier;
// float Kb = 0.001 * multiplier;

// KP BETWEEN 25 AND 50
float maxRateRoll = 360.0;
float maxRatePitch = 360.0;
float maxRateYaw = 360.0;
float maxAngularAccel = 300.0;

float multiplier = 0.0001;

float Kp = 4;
float Ki = 0.00;
float Kd = 0;
float Kb = 0;

float yaw_k_multiplier = 0.001;
float yaw_i_multiplier = 0.0;
float z_k_multiplier = 0;
float z_i_multiplier = 0;
float desired_z = 0;

float P_roll, P_pitch, P_yaw, P_z;
float I_roll, I_pitch, I_yaw, I_z;
float D_roll, D_pitch, D_yaw, D_z;

float error_roll, error_pitch, error_yaw, error_z;
float error_sum_roll, error_sum_pitch, error_sum_yaw, error_sum_z;
float error_prev_roll, error_prev_pitch, error_prev_yaw, error_prev_z;

float desiredRoll, desiredPitch, desiredYaw;

float maxI = 200;
float minI = -200;

uint16_t calibrationCount = 100;

static boolean displaySensors = false;
static boolean ready = false;
static boolean dry = true;
static boolean noPrint = false;
// :: PID :: // 



// :: SENSORS :: // 
TwoWire wire = Wire1;
GY87 gy87(wire);

#define RETRIES 3
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
void setIMUOnly(boolean value);
void calibrateESC();
boolean calibrateSensors(boolean calibrateMpu, boolean calibrateCompass);
void drySequence();

const std::string formatFloat(float val, boolean normalize = false);

void setKp(float value);
void setKi(float value);
void setKd(float value);
void setKb(float value);
void setMaxCount(float value);
void setYawKMultiplier(float value);
void setYawIMultiplier(float value);
void setZKMultiplier(float value);
void setZIMultiplier(float value);
void setDesiredRoll(float value);
void setDesiredPitch(float value);
void setDesiredYaw(float value);
void setDesiredZ(float value);
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
  {"IMUONLY", std::bind(setIMUOnly, true)},
  {"NOTIMUONLY", std::bind(setIMUOnly, false)},
  {"RECALIBRATEESC", calibrateESC},
  {"RECALIBRATESENSORS", std::bind(calibrateSensors, true, true)},
  {"RECALIBRATEMPU", std::bind(calibrateSensors, true, false)},
  {"RECALIBRATECOMPASS", std::bind(calibrateSensors, false, true)},
  {"DRY", drySequence},
};

std::unordered_map<std::string, std::function<void(float value)>> setCommands = {
  {"KP", setKp},
  {"KI", setKi},
  {"KD", setKd},
  {"KB", setKb},
  {"CT", setMaxCount},
  {"KY", setYawKMultiplier},
  {"IY", setYawIMultiplier},
  {"KZ", setZKMultiplier},
  {"IZ", setZIMultiplier},
  {"DR", setDesiredRoll},
  {"DP", setDesiredPitch},
  {"DY", setDesiredYaw},
  {"DZ", setDesiredZ}
};

void customPrint (const char* data, boolean overridePrint = false) {
  if (noPrint && !overridePrint) {
    return;
  }

  if (Serial) {
    Serial.println (data);
  }

  if (bluetooth) {
    bluetooth.println (data);
  }
}

void stabilize() {
  gy87.stabilize(imuOnly, 2500);

  // desiredRoll = gy87.getRoll();
  // desiredPitch = gy87.getPitch();
  // desiredYaw = gy87.getAbsoluteYaw();

  desiredRoll = 0;
  desiredPitch = 0;
  desiredYaw = 0;

  Serial.println(gy87.getRoll());
  Serial.println(gy87.getPitch());
  Serial.println(gy87.getAbsoluteYaw());

  loopTime = micros();
  loopPrev = micros();
}

void displaySensor() {
  kill();
  stabilize();
  displaySensors = true;
}

void setPrint(bool value) {
  noPrint = value;
  std::stringstream payloadStream;
  payloadStream << "Printing turned " << (value ? "off." : "on.");
  customPrint(payloadStream.str().c_str());
}

void setIMUOnly(bool value) {
  imuOnly = value;
  std::stringstream payloadStream;
  payloadStream << "IMUOnly " << (value ? "on." : "off.");
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

void setMaxCount (float value) {
  maxCount = value;
  customPrint("Max count set.");
}

void setYawKMultiplier (float value) {
  yaw_k_multiplier = value;
  customPrint("Yaw K multiplier set.");
}

void setYawIMultiplier (float value) {
  yaw_i_multiplier = value;
  customPrint("Yaw I multiplier set.");
}

void setZKMultiplier (float value) {
  z_k_multiplier = value;
  customPrint("Z K multiplier set.");
}

void setZIMultiplier (float value) {
  z_i_multiplier = value;
  customPrint("Z I multiplier set.");
}

void setDesiredRoll (float value) {
  desiredRoll = value;
  customPrint("Desired Roll set.");
}

void setDesiredPitch (float value) {
  desiredPitch = value;
  customPrint("Desired Pitch set.");
}

void setDesiredYaw (float value) {
  desiredYaw = value;
  customPrint("Desired Yaw set.");
}

void setDesiredZ (float value) {
  desired_z = value;
  customPrint("Desired Z set.");
}

const std::string formatFloat(float val, boolean normalize) {
  static char out[7];  // Increase the size of the out array to accommodate null terminator
  if (normalize) {
    val *= 1000;
  }
  std::memset(out, ' ', sizeof(out));  // Initialize the array with spaces
  dtostrf(val, 5, 4, out);  // Use 6 instead of 7 to leave space for the potential negative sign
  out[6] = '\0';  // Add null terminator at the end
  return out;
}

boolean calibrateSensors(boolean calibrateMpu, boolean calibrateCompass)
{
  delay (250);
  customPrint("Calibrating MPU, do not move sensor");
  gy87.calibrate(calibrateMpu, false);
  customPrint("Calibrating compass, move in a figure 8 pattern");
  delay (250);
  gy87.calibrate(false, calibrateCompass);
  customPrint("Calibration complete.");
  
  if (!noPrint && calibrateCompass) {
    float hardIronBiasX, hardIronBiasY, hardIronBiasZ;
    float softIronScaleX, softIronScaleY, softIronScaleZ;

    gy87.getHardIronBias(hardIronBiasX, hardIronBiasY, hardIronBiasZ);
    gy87.getSoftIronScale(softIronScaleX, softIronScaleY, softIronScaleZ);

    std::stringstream payloadStream;
      payloadStream
        << "HardIron: X " << formatFloat(hardIronBiasX) << " Y " << formatFloat(hardIronBiasY) << " Z " << formatFloat(hardIronBiasZ)
        << "SoftIron: X " << formatFloat(softIronScaleX) << " Y " << formatFloat(softIronScaleY) << " Z " << formatFloat(softIronScaleZ);

  }
  return false;
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

void printSensorValues (boolean override = false) {
  if ((noPrint && !override)) {
    return;
  }

  gy87.fetchData();

  std::stringstream payloadStream;
  payloadStream 
    << "Acceleration (m/s^2) => X: " << formatFloat(gy87.getAccX()) << ", Y: " << formatFloat(gy87.getAccY()) << ", Z: " << formatFloat(gy87.getAccZ()) << " ||| "
    << "Gyro (deg/s^2) => X: " << formatFloat(gy87.getGyroX()) << ", Y: " << formatFloat(gy87.getGyroY()) << ", Z: " << formatFloat(gy87.getGyroZ()) << " ||| "
    << "Angle (deg) => X: " << formatFloat(gy87.getAngleX()) << ", Y: " << formatFloat(gy87.getAngleY()) << ", Z: " << formatFloat(gy87.getAngleZ()) << " ||| "
    << "AccAngle (deg) => X: " << formatFloat(gy87.getAccAngleX()) << ", Y: " << formatFloat(gy87.getAccAngleY()) << " ||| "
    << "Magnetic (uT) => X: " << formatFloat(gy87.getMagX()) << "  " << "Y: " << formatFloat(gy87.getMagY()) << "  " << "Z: " << formatFloat(gy87.getMagZ()) << " ||| "
    << "Madgwick (deg) => X: " << formatFloat (gy87.getRoll()) << "  " << "Y: " << formatFloat (gy87.getPitch()) << "  " << "Z: " << formatFloat (gy87.getAbsoluteYaw());

  customPrint(payloadStream.str().c_str(), override);
}

void printPIDValues (boolean override = false) {
  if (noPrint && !override) {
    return;
  }
  
  std::stringstream payloadStream;

  payloadStream 
    << "M1: " << currentThrottles[0] << " | "
    << "M2: " << currentThrottles[1] << " | "
    << "M3: " << currentThrottles[2] << " | "
    << "M4: " << currentThrottles[3] << " | "
    << "P_roll: " << formatFloat(P_roll, true) << " | " << "P_pitch: " << formatFloat(P_pitch, true) << " | " << "P_yaw: " << formatFloat(P_yaw, true) << " | " << "P_z: " << formatFloat(P_z, true) << " | "
    << "I_roll: " << formatFloat(I_roll, true) << " | " << "I_pitch: " << formatFloat(I_pitch, true) << " | " << "I_yaw: " << formatFloat(I_yaw, true) << " | " << "I_z: " << formatFloat(I_z, true) << " | "
    << "D_roll: " << formatFloat(D_roll, true) << " | " << "D_pitch: " << formatFloat(D_pitch, true) << " | " << "D_yaw: " << formatFloat(D_yaw, true) << " | " << "D_z: " << formatFloat(D_z, true) << " | "
    << "error_roll: " << formatFloat(error_roll, true) << " | " << "error_pitch: " << formatFloat(error_pitch, true) << " | " << "error_yaw: " << formatFloat(error_yaw, true) <<  " | " <<  "error_z: " << formatFloat(error_z, true);
    
  customPrint(payloadStream.str().c_str(), override);
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

  desiredRoll = 0;
  desiredPitch = 0;
  desiredYaw = 0;
  desired_z = 0;

  gy87.resetFast();
  customPrint("PID Reset!");
}

void kill() {
  for(int i = 0; i < numMotors; i++) {
    motors[i].writeMicroseconds(minThrottle);
  }

  displaySensors = false;
  noPrint = false;

  printPIDValues();
  printSensorValues();

  customPrint("Kill command received. Motors shut down.");

  for (int i = 0; i < numMotors; i++) {
    currentThrottles[i] = takeoffThrottles[i];
  }

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
  printPIDValues();
  customPrint("Preparing motors...");
  stabilize();
  noPrint = true;
  ready = true;
  dry = false;
  timer = millis();
  customPrint("Motors prepared!");
  takeoff();
}

void drySequence() {
  customPrint("Preparing dry run...");
  kill();
  stabilize();

  ready = true;
  dry = true;
  timer = millis();
  customPrint("Dry run beginning!");
}

void status () {
  std::stringstream payloadStream;
  payloadStream 
    << "Kp: " << formatFloat(Kp) << " | "
    << "Ki: " << formatFloat(Ki) << " | "
    << "Kd: " << formatFloat(Kd) << " | "
    << "Kb: " << formatFloat(Kb) << " | "
    << "KY: " << formatFloat(yaw_k_multiplier) << " | "
    << "IY: " << formatFloat(yaw_i_multiplier) << " | "
    << "KZ: " << formatFloat(z_k_multiplier) << " | "
    << "IZ: " << formatFloat(z_i_multiplier) << " | "
    << "DR: " << formatFloat(desiredRoll) << " | "
    << "DP: " << formatFloat(desiredPitch) << " | "
    << "DY: " << formatFloat(desiredYaw) << " | "
    << "DZ: " << formatFloat(desired_z) << " | "
    << "IMUOnly: " << (imuOnly ? "Yes" : "No") << " | "
    << "Takeoff Throttles: " << arrtostr(takeoffThrottles, 4) << " | "
    << "Ready: " << (ready ? "Yes" : "No") << " | "
    << "Dry: " << (dry ? "Yes" : "No") << " | "
    << "Max Count: " << formatFloat (maxCount) << " | ";

  for (int i = 0; i < numBatteries - 1; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    float percentage = (voltage - minVoltage) / voltageRange * 100.0;
    payloadStream << "Battery " << (i + 1) << ": " << formatFloat(percentage) << "%" << " | ";
  }

  float voltage = (analogRead(batteryPins[numBatteries - 1]) / 1023.0) * 3.3;
  float percentage = (voltage - minVoltage) / voltageRange * 100;
  payloadStream << "Battery " << (numBatteries - 1) << ": " << formatFloat(percentage) << "%";

  noPrint = false;
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
    << "RECALIBRATEMPU\n"
    << "RECALIBRATECOMPASS\n"
    << "DRY\n"
    << "PRINT\n"
    << "NOPRINT\n"
    << "IMUONLY\n"
    << "NOTIMUONLY\n"
    << "{MOTOR NUM} {THROTTLE PERCENT}\n"
    << "========================================================\n"
    << "KP {NEW VALUE}\n"
    << "KI {NEW VALUE}\n"
    << "KD {NEW VALUE}\n"
    << "KB {NEW VALUE}\n"
    << "TT {(OPTIONAL) MOTOR NUM} {NEW VALUE}\n"
    << "CT {NEW VALUE} (this is the counter for pid increment)\n"
    << "KY {NEW VALUE} (this is the multiplier on the K yaw component)\n"
    << "IY {NEW VALUE} (this is the multiplier on the I yaw component)\n"
    << "KZ {NEW VALUE} (this is the multiplier on the K Z component)\n"
    << "IZ {NEW VALUE} (this is the multiplier on the I Z component)\n"
    << "DR {NEW VALUE} (desired Roll value)\n"
    << "DP {NEW VALUE} (desired Pitch value)\n"
    << "DY {NEW VALUE} (desired Yaw value)\n"
    << "DZ {NEW VALUE} (desired Z value)\n";

  customPrint(payloadStream.str().c_str());
}

void checkBattery() {
  for (int i = 0; i < numBatteries; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    float percentage = (voltage - minVoltage) / voltageRange * 100.0;
    if (percentage < 20) {
      battery20 = true;
    }
    if (percentage < 5) {
      customPrint(formatFloat(percentage).c_str(), true);
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

// void calibrateBluetooth() {
//   customPrint("Calibrating Bluetooth...");
//   delay(1000);
//   bluetooth.end();
//   delay(1000);
//   bluetooth.begin (115200);
//   while (!bluetooth) {};
//   Serial.println("AT+BAUD7");
//   bluetooth.print("AT+BAUD7"); // AT command to set baud rate to 57600
//   delay(1000);
//   bluetooth.end();
//   bluetooth.begin(57600);
//   while (!bluetooth) {};
//   customPrint("Bluetooth Calibrated");
// }

void setup() {
  lastReceivedTime = millis();
  timer = millis();
  Serial.begin(230400);
  bluetooth.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // bluetooth.begin(9600);

  customPrint("Setup Beginning!");

  for (int i = 0; i < numMotors; i++) {
    motors[i].attach(motorPins[i]);
  }

  for (int i = 0; i < numBatteries; i++) {
    pinMode(batteryPins[i], INPUT);
  }

  // calibrateBluetooth();
  calibrateESC();

  // calibrate gy87
  if (gy87.begin(1, 0, 3, true) != 0) {
    customPrint("Sensors failed");
  };

  gy87.setAccOffsets(0.04, 0.0, -0.11);
  gy87.setGyroOffsets(2.42, 0.05, -0.29);
  gy87.stabilize(true);

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

      if (throttlePercent <= 15) {
        motors[motorNumber - 1].writeMicroseconds(throttleValue);

        std::stringstream payloadStream;
        payloadStream << "Motor " << motorNumber << " set to throttle " << ((throttleValue - 1000) / 10) << "%";

        customPrint(payloadStream.str().c_str());
      } else {
        customPrint("Invalid motor value, too high.");
      }
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
    // checkBattery();

    if (battery5) {
      if (batteryTimer == 0) {
        batteryTimer = millis();
        customPrint("Battery 5% remaining, killing in 30 seconds", true);
      }
      // 30 seconds then kill
      if ((millis() - batteryTimer) > batteryKillTime) {
        batteryTimer = 0;
        customPrint("Battery too low.", true);
        kill();
      }
    }
    
    loopTime = micros();
    deltaTime = (float)(loopTime - loopPrev) / 1000000.0; // Convert to seconds

    if (imuOnly) {
      gy87.updateIMU();
    }
    else {
      gy87.update();
    }

    loopPrev = loopTime;


    float maxChangeRateRoll = maxRateRoll * deltaTime;
    float maxChangeRatePitch = maxRatePitch * deltaTime;
    float maxChangeRateYaw = maxRateYaw * deltaTime;
    float maxChangeAccel = maxAngularAccel * deltaTime * deltaTime;

    float newDesiredRoll = 0;
    float newDesiredPitch = 0;
    float newDesiredYaw = 0;

    // Constrain desired roll, pitch, and yaw considering rate limits and maximum angular acceleration
    for (int axis = 0; axis < 3; ++axis) {
      float* desiredValue;
      float newDesiredValue;
      float maxChangeRate;

      switch (axis) {
        case 0:
          desiredValue = &desiredRoll;
          newDesiredValue = newDesiredRoll;
          maxChangeRate = maxChangeRateRoll;
          break;
        case 1:
          desiredValue = &desiredPitch;
          newDesiredValue = newDesiredPitch;
          maxChangeRate = maxChangeRatePitch;
          break;
        case 2:
          desiredValue = &desiredYaw;
          newDesiredValue = newDesiredYaw;
          maxChangeRate = maxChangeRateYaw;
          break;
      }

      float change = newDesiredValue - *desiredValue;
      float change_abs = abs(change);
      float maxChange = min(maxChangeRate, maxChangeAccel);

      if (change_abs > maxChange) {
        change = (change > 0 ? 1 : -1) * maxChange;
        newDesiredValue = *desiredValue + change;
      }

      *desiredValue = newDesiredValue;
    }


    // ** MAKE A MAXIMUM ANGULAR VELOCITY AND ACCELERATION


    error_roll = (gy87.getRoll() - desiredRoll) * multiplier;
    error_pitch = (gy87.getPitch() - desiredPitch) * multiplier;
    error_yaw = (gy87.getAbsoluteYaw() - desiredYaw) * multiplier * yaw_k_multiplier;

    error_z = (gy87.getAccZ() - (1 + desired_z)) * z_k_multiplier * 0;

    // Calculate PID terms
    P_roll = Kp * error_roll;
    P_pitch = Kp * error_pitch;
    P_yaw = Kp * error_yaw;
    P_z = Kp * error_z;

    // Update the error_sum variables with the current error multiplied by deltaTime
    error_sum_roll += error_roll * deltaTime;
    error_sum_pitch += error_pitch * deltaTime;
    error_sum_yaw += error_yaw * deltaTime;
    error_sum_z += error_z * deltaTime;

    // Calculate the integral terms
    I_roll = Ki * error_sum_roll;
    I_pitch = Ki * error_sum_pitch;
    I_yaw = Ki * error_sum_yaw * yaw_i_multiplier;
    I_z = Ki * error_sum_z * z_i_multiplier;
    // I_z = 0;

    I_roll = constrain(I_roll, minI, maxI);
    I_pitch = constrain(I_pitch, minI, maxI);
    I_yaw = constrain(I_yaw, minI, maxI);
    I_z = constrain(I_z, minI, maxI);

    // Calculate the derivative terms, dividing by deltaTime
    if (deltaTime != 0) {
      D_roll = Kd * (error_roll - error_prev_roll) / deltaTime;
      D_pitch = Kd * (error_pitch - error_prev_pitch) / deltaTime;
      D_yaw = Kd * (error_yaw - error_prev_yaw) / deltaTime;
      D_z = Kd * (error_z - error_prev_z) / deltaTime;
    } else {
      D_roll = 0;
      D_pitch = 0;
      D_yaw = 0;
      D_z = 0;
    }

    // Store the previous errors for the next iteration
    error_prev_roll = error_roll;
    error_prev_pitch = error_pitch;
    error_prev_yaw = error_yaw;
    error_prev_z = error_z;

    // Assuming you have a function to get the current motor speed for each motor
    float current_motor_speed_0 = get_motor_speed(1);
    float current_motor_speed_1 = get_motor_speed(2);
    float current_motor_speed_2 = get_motor_speed(3);
    float current_motor_speed_3 = get_motor_speed(4);

    // Calculate unsaturated output for each motor
    float unsat_throttle[numMotors];
    unsat_throttle[0] = current_motor_speed_0 - P_roll - I_roll - D_roll + P_pitch + I_pitch + D_pitch - P_yaw - I_yaw - D_yaw - P_z - I_z - D_z; // Motor 1 (positive roll, positive pitch, clockwise)
    unsat_throttle[1] = current_motor_speed_1 - P_roll - I_roll - D_roll - P_pitch - I_pitch - D_pitch + P_yaw + I_yaw + D_yaw - P_z - I_z - D_z; // Motor 2 (positive roll, negative pitch, counterclockwise)
    unsat_throttle[2] = current_motor_speed_2 + P_roll + I_roll + D_roll - P_pitch - I_pitch - D_pitch - P_yaw - I_yaw - D_yaw - P_z - I_z - D_z; // Motor 3 (negative roll, negative pitch, clockwise)
    unsat_throttle[3] = current_motor_speed_3 + P_roll + I_roll + D_roll + P_pitch + I_pitch + D_pitch + P_yaw + I_yaw + D_yaw - P_z - I_z - D_z; // Motor 4 (negative roll, positive pitch, counterclockwise)

    currentThrottles[0] = constrain(unsat_throttle[0], minThrottle, maxThrottle);
    currentThrottles[1] = constrain(unsat_throttle[1], minThrottle, maxThrottle);
    currentThrottles[2] = constrain(unsat_throttle[2], minThrottle, maxThrottle);
    currentThrottles[3] = constrain(unsat_throttle[3], minThrottle, maxThrottle);


    // test all kp values

    counter++;
    if (counter == maxCount) {
      // Kp += 0.01;
      // String payload = "KP INCREASING: " + formatFloat(Kp) + " | " + "TOTAL TIME: " + formatFloat((timer - micros()) / 1000000) + "ms";
      std::stringstream payloadStream;

      payloadStream 
        << "TOTAL TIME FOR " << maxCount << " ITERATIONS: " << formatFloat((millis() - timer)) << "ms";

      customPrint(payloadStream.str().c_str(), true);

      counter = 0;
      elapsedMicros();

      printPIDValues(true);
      printSensorValues(true);
      timer = millis();
    }

    // // Calculate motor throttles
    // int throttle[numMotors];
    // throttle[0] = constrain(baseThrottle - P_x - I_x - D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[1] = constrain(baseThrottle + P_x + I_x + D_x - P_y - I_y - D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[2] = constrain(baseThrottle + P_x + I_x + D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);
    // throttle[3] = constrain(baseThrottle - P_x - I_x - D_x + P_y + I_y + D_y + P_z + I_z + D_z, minThrottle, maxThrottle);




    // // back calculation for integral
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
    // I_roll -= avg_back_calc_x;
    // I_pitch -= avg_back_calc_y;
    // I_yaw -= avg_back_calc_z;

    // if (dry) {
    //   std::stringstream payloadStream;

    //   payloadStream 
    //     << "BackCalc => X: " << formatFloat(avg_back_calc_x) << ", Y: " << formatFloat(avg_back_calc_y) << ", Z: " << formatFloat(avg_back_calc_z);

    //   customPrint(payloadStream.str().c_str(), true);
    // }


    // Run motors with calculated throttles
    if (!dry) {
      runMotors(currentThrottles);
    }

    printPIDValues();
  }

  if (displaySensors) {

    gy87.updateFast();
    if (imuOnly) {
      gy87.updateIMU();
    } else {
      gy87.update();
    }

    loopPrev = loopTime;
    printSensorValues();

    delay(10);
  }
}