#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <SPI.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TwoWire wire = Wire;

float ax, ay, az, gx, gy, gz, mx, my, mz, t, pres, alt;
uint16_t calibrationCount = 500;

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

  Serial.println("");
  Serial.println("Default Values");
  Serial.print("ax: ");
  Serial.print(ax);
  Serial.println(" m/s^2");
  Serial.print("ay: ");
  Serial.print(ay);
  Serial.println(" m/s^2");
  Serial.print("az: ");
  Serial.print(az);
  Serial.println(" m/s^2");
  Serial.print("gx: ");
  Serial.print(gx);
  Serial.println(" rad/s^2");
  Serial.print("gy: ");
  Serial.print(gy);
  Serial.println(" rad/s^2");
  Serial.print("gz: ");
  Serial.print(gz);
  Serial.println(" rad/s^2");
  Serial.print("mx: ");
  Serial.print(mx);
  Serial.println(" nT");
  Serial.print("my: ");
  Serial.print(my);
  Serial.println(" nT");
  Serial.print("mz: ");
  Serial.print(mz);
  Serial.println(" nT");
  Serial.print("t: ");
  Serial.print(t);
  Serial.println(" degrees C");
  Serial.print("pres: ");
  Serial.print(pres);
  Serial.println(" mBar");
  Serial.print("alt: ");
  Serial.print(alt);
  Serial.println(" Meters");
  Serial.println("");
}

void setup(void)
{
  Serial.begin(9600);
  wire.begin();

  while (!Serial)
  {
    delay(10);
  }

  if (!bmp.begin(BMP085_HIGHRES, &wire))
  {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("BMP180 Found!");

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &wire, 0))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setI2CBypass(true);

  // setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_2_5_HZ);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  if (!mag.begin(&wire))
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
    {
      delay(1000);
    };
  }
  Serial.println("HMC5883L Found!");

  calibrateSensors();
  delay(100);
}

void loop()
{
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
  delay(500);
}