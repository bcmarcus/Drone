#include <Wire.h>
#include "person_sensor.h"

#define SENSOR_I2C_ADDRESS 0x62

void setup() {
  Serial.begin(9600);
  Wire.begin();
  enableIDModel();
}

void loop() {
  readPersonSensorData();
  delay(1000);
}

void enableIDModel() {
  uint8_t registerAddress = 0x02; // Enable ID register address
  uint8_t enableValue = 0x01; // Enable the ID model

  Wire.beginTransmission(SENSOR_I2C_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(enableValue);
  Wire.endTransmission();
}

void addPerson(uint8_t personID) {
  uint8_t registerAddress = 0x04; // Label next ID register address
  uint8_t calibrationValue = personID; // Person ID to be calibrated (0 to 7)

  Wire.beginTransmission(SENSOR_I2C_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(calibrationValue);
  Wire.endTransmission();
}

bool verifyChecksum(const uint8_t *data, uint16_t dataSize, uint16_t checksum) {
  uint16_t computedChecksum = 0;
  for (int i = 0; i < dataSize; ++i) {
    computedChecksum += data[i];
  }
  return computedChecksum == checksum;
}

void readPersonSensorData() {
  uint8_t data[39];
  Wire.requestFrom(SENSOR_I2C_ADDRESS, (uint8_t)sizeof(data));

  for (int i = 0; i < sizeof(data); ++i) {
    data[i] = Wire.read();
  }

  uint16_t checksum = (data[37] << 8) | data[38];

  if (!verifyChecksum(data, 37, checksum)) {
    Serial.println("Checksum mismatch. Data may be corrupted.");
    return;
  }

  uint8_t numFaces = data[4];

  for (int i = 0; i < numFaces; ++i) {
    int faceOffset = 5 + i * 8;
    uint8_t boxConfidence = data[faceOffset];
    uint8_t boxLeft = data[faceOffset + 1];
    uint8_t boxTop = data[faceOffset + 2];
    uint8_t boxRight = data[faceOffset + 3];
    uint8_t boxBottom = data[faceOffset + 4];
    uint8_t recognitionConfidence = data[faceOffset + 5];
    int8_t recognitionID = data[faceOffset + 6];

    Serial.print("Face ");
    Serial.print(i);
    Serial.print(": Box Confidence=");
    Serial.print(boxConfidence);
    Serial.print(", Box Left=");
    Serial.print(boxLeft);
    Serial.print(", Box Top=");
    Serial.print(boxTop);
    Serial.print(", Box Right=");
    Serial.print(boxRight);
    Serial.print(", Box Bottom=");
    Serial.print(boxBottom);
    Serial.print(", Recognition Confidence=");
    Serial.print(recognitionConfidence);
    Serial.print(", Recognition ID=");
    Serial.println(recognitionID);
  }
}

bool isPersonRecognized(const person_sensor_face_t* face) {
  // Check if the face has a valid ID and confidence
  return (face->id >= 0) && (face->id_confidence > 0);
}