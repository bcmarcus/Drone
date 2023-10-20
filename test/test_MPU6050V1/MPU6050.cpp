#include <Arduino.h>
#include <Wire.h>


uint32_t counter = 0;
uint32_t timer = 0;


#define MPU6050_ADDRESS 0x68
#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_STATUS_REGISTER 0x09

TwoWire wire = Wire1;

void setup() {
  wire.begin();
  Serial.begin(9600);

  // Wake up MPU6050
  wire.beginTransmission(MPU6050_ADDRESS);
  wire.write(0x6B); // PWR_MGMT_1 register
  wire.write(0x00); // Clear sleep mode bit to wake up MPU
  wire.endTransmission();

  wire.setClock(32000000);
}

void loop() {
  // Read MPU6050
  int16_t accelX, accelY, accelZ;
  wire.beginTransmission(MPU6050_ADDRESS);
  wire.write(0x3B); // ACCEL_XOUT_H register
  wire.endTransmission();
  wire.requestFrom(MPU6050_ADDRESS, 6);
  if(wire.available() == 6) {
    accelX = wire.read() << 8 | wire.read();
    accelY = wire.read() << 8 | wire.read();
    accelZ = wire.read() << 8 | wire.read();
  }
  
  // Serial.print("Accelerometer data - X: "); Serial.print(accelX);
  // Serial.print(" Y: "); Serial.print(accelY);
  // Serial.print(" Z: "); Serial.println(accelZ);
  // Serial.println(F("=====================================================\n"));
  // delay (1000);
  
  counter++;
  if (counter % 1000 == 0) {
    Serial.print(counter);
    Serial.print(" :: ");
    Serial.print(millis() / 1000.0);
    Serial.print(" :: ");
    Serial.println((millis() - timer) / 1000.0);
    timer = millis();
  }
}
