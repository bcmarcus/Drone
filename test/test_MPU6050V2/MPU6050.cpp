#include <Wire.h>
#include <MPU6050.h>

uint32_t counter = 0;
uint32_t timer = 0;


#define MPU6050_ADDRESS 0x68
#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_STATUS_REGISTER 0x09

MPU6050 mpu(Wire1);

void setup() {
  Serial.begin(9600);
  if (mpu.begin(1, 0) != 0){
    Serial.println ("Mpu failed to start.");
  };

  Serial.print(F("ACCEL OFFSETS     X: "));Serial.print(mpu.getAccXoffset());
  Serial.print("\tY: ");Serial.print(mpu.getAccYoffset());
  Serial.print("\tZ: ");Serial.println(mpu.getAccZoffset());
  
  Serial.print(F("GYRO OFFSETS     X: "));Serial.print(mpu.getGyroXoffset());
  Serial.print("\tY: ");Serial.print(mpu.getGyroYoffset());
  Serial.print("\tZ: ");Serial.println(mpu.getGyroZoffset());

  // mpu.setAccOffsets(0.03, 0.03, -0.08);
  // mpu.setGyroOffsets(1.47, -0.10, -0.18);

  // mpu.setAccOffsets(0.03, 0.03, -0.08);
  // mpu.setGyroOffsets(1.47, -0.10, -0.11 );
}

void loop() {
  // Read MPU6050

  mpu.updateFast();


  Serial.print(F("ACCEL     X: "));Serial.print(mpu.getAccX());
  Serial.print("\tY: ");Serial.print(mpu.getAccY());
  Serial.print("\tZ: ");Serial.println(mpu.getAccZ());

  Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
  Serial.print("\tY: ");Serial.print(mpu.getGyroY());
  Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
  Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
  Serial.print("\tY: ");Serial.print(mpu.getAngleY());
  Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());

  Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
  Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
  
  Serial.println(F("=====================================================\n"));
  delay (1000);
  
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
