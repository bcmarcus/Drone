// simple serial version

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

void loop () {
   if (Serial.available() > 0) {
    int throttlePercent = Serial.parseInt();
    throttlePercent = constrain(throttlePercent, 0, 100);
    int throttleValue = map(throttlePercent, 0, 100, minThrottle, maxThrottle);
    // Calculate motor throttles
    int throttle[numMotors];
    throttle[0] = throttleValue;
    throttle[1] = throttleValue;
    throttle[2] = throttleValue;
    throttle[3] = throttleValue;

    // Run motors with calculated throttles
    runMotors(throttle);

    // if(mpu.getMotionInterruptStatus()) {
  
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print("AccelX: ");
    Serial.print(a.acceleration.x - ax);
    Serial.print(",");
    Serial.print("AccelY: ");
    Serial.print(a.acceleration.y - ay);
    Serial.print(",");
    Serial.print("AccelZ: ");
    Serial.print(a.acceleration.z - az);
    Serial.print(", ");
    Serial.print("GyroX: ");
    Serial.print(g.gyro.x - gx);
    Serial.print(",");
    Serial.print("GyroY: ");
    Serial.print(g.gyro.y - gy);
    Serial.print(",");
    Serial.print("GyroZ: ");
    Serial.print(g.gyro.z - gz);
    Serial.println("");
    Serial.println("");

    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");

    sensors_event_t event;

    mag.getEvent(&event);
    Serial.print("X: ");
    Serial.print(event.magnetic.x);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(event.magnetic.y);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(event.magnetic.z);
    Serial.print("  ");
    Serial.println("uT");
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

    Serial.print("Heading (degrees): ");
    Serial.println(headingDegrees);
    Serial.println("");

    Serial.println("Enter desired throttle value between 0 and 100:");
  }
}
