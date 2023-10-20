#include <Arduino.h>
#include <Wire.h>

uint32_t counter = 0;
uint32_t timer = 0;
boolean flag = false;
TwoWire wire = Wire1;

// CHANGE THIS FOR PRINTING OR NOT // 

boolean print = false;

// CHANGE THIS FOR PRINTING OR NOT // 

#define BMP180_ADDRESS 0x77
#define BMP180_CONTROL_REGISTER 0xF4
#define BMP180_DATA_REGISTER 0xF6
#define BMP180_TEMP_COMMAND 0x2E
#define BMP180_PRESSURE_COMMAND 0x34 // Add 0-3 to this value for different oversampling settings (0-3 correspond to ultra low power to ultra high resolution)
#define OVERSAMPLING_SETTING 3

enum class Bmp180State {
  IDLE,
  REQUESTED_TEMP,
  REQUESTED_PRESSURE
};

Bmp180State bmp180State = Bmp180State::IDLE;
unsigned long bmp180RequestTime;
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
int32_t b5;

uint8_t readBmp180Byte(uint8_t address) {
  wire.beginTransmission(BMP180_ADDRESS);
  wire.write(address);
  int error = wire.endTransmission(false);  // keep the connection active
  if (error != 0) {
    Serial.println("I2C transmission error: " + String(error));
    return 0;
  }

  wire.requestFrom(BMP180_ADDRESS, 1);
  if (wire.available()) {
    return wire.read();
  } else {
    Serial.println("No data available from BMP180.");
    return 0;
  }
}

int16_t readBmp180Word(uint8_t address) {
  uint8_t msb = readBmp180Byte(address);
  uint8_t lsb = readBmp180Byte(address + 1);
  return (msb << 8) | lsb;
}


void setup() {
  Serial.begin(9600);
  
  wire.begin();
  wire.setClock(20000000);
  // ... setup code for MPU6050 and HMC5883L ...
  
  // Read BMP180 calibration data
  ac1 = readBmp180Word(0xAA);
  ac2 = readBmp180Word(0xAC);
  ac3 = readBmp180Word(0xAE);
  ac4 = readBmp180Word(0xB0);
  ac5 = readBmp180Word(0xB2);
  ac6 = readBmp180Word(0xB4);
  b1 = readBmp180Word(0xB6);
  b2 = readBmp180Word(0xB8);
  mb = readBmp180Word(0xBA);
  mc = readBmp180Word(0xBC);
  md = readBmp180Word(0xBE);
}

void loop() {
  // Check the current state of the BMP180
  switch (bmp180State) {
    case Bmp180State::IDLE:
      // Request a temperature measurement
      wire.beginTransmission(BMP180_ADDRESS);
      wire.write(BMP180_CONTROL_REGISTER); 
      wire.write(BMP180_TEMP_COMMAND); 
      wire.endTransmission();
      bmp180State = Bmp180State::REQUESTED_TEMP;
      bmp180RequestTime = millis();
      break;

    case Bmp180State::REQUESTED_TEMP:
      if (millis() - bmp180RequestTime >= 5) { 
        // Read the temperature data
        int32_t temp = readBmp180Word(BMP180_DATA_REGISTER);

        int32_t x1 = ((temp - ac6) * ac5) >> 15;
        int32_t x2 = (mc << 11) / (x1 + md);
        b5 = x1 + x2;
        float temperature = ((b5 + 8) >> 4) / 10.0; // Temperature in Celsius

        // Request a pressure measurement
        wire.beginTransmission(BMP180_ADDRESS);
        wire.write(BMP180_CONTROL_REGISTER); 
        wire.write(BMP180_PRESSURE_COMMAND + (OVERSAMPLING_SETTING << 6)); // Use ultra high resolution mode (add 3 to command value)
        wire.endTransmission();
        bmp180State = Bmp180State::REQUESTED_PRESSURE;
        bmp180RequestTime = millis();

        if (print) {
          Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" // ");
        }
      }
      break;

    // Requested Pressure
    case Bmp180State::REQUESTED_PRESSURE:
      if (millis() - bmp180RequestTime >= 26) {

        // Read the raw pressure value
        // int32_t up = (readBmp180Word(BMP180_DATA_REGISTER) << 8) | readBmp180Byte(BMP180_DATA_REGISTER + 2);

        uint16_t msb = readBmp180Word(BMP180_DATA_REGISTER);
        uint8_t xlsb = readBmp180Byte(BMP180_DATA_REGISTER + 2);

        int32_t up = ((int32_t)msb << 8) | xlsb;
        up >>= (8 - OVERSAMPLING_SETTING);  // replace OVERSAMPLING_SETTING with your oversampling setting

        // Calculate the intermediate value b6
        int32_t b6 = b5 - 4000;

        // Calculate intermediate values x1, x2, x3
        int32_t x1 = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
        int32_t x2 = ((int32_t)ac2 * b6) >> 11;
        int32_t x3 = x1 + x2;

        // Calculate the intermediate value b3
        int32_t b3 = ((((int32_t)ac1 * 4 + x3) << OVERSAMPLING_SETTING) + 2) / 4;

        // Calculate new intermediate values x1, x2, x3
        x1 = ((int32_t)ac3 * b6) >> 13;
        x2 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
        uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> OVERSAMPLING_SETTING);

        // Calculate the pressure value p
        int32_t p;
        if (b7 < 0x80000000) {
            p = (b7 * 2) / b4;
        } else {
            p = (b7 / b4) * 2;
        }

        // Apply final corrections to the pressure value
        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        p += (x1 + x2 + 3791) >> 4;

        // Calculate altitude
        float altitude = 44330.0 * (1.0 - pow(p / 101325.0, 0.1903)); // Altitude in m

        bmp180State = Bmp180State::IDLE;

        if (print) {
          Serial.print("Pressure: "); Serial.print (p); Serial.print (" // ");
          Serial.print("Altitude: "); Serial.println(altitude);
        }

        counter++;
        flag = true;
      }
      break;
  }

  if (counter % 100 == 0 && flag) {
    Serial.print(counter);
    Serial.print(" :: ");
    Serial.print(millis() / 1000.0);
    Serial.print(" :: ");
    Serial.println((millis() - timer) / 1000.0);
    timer = millis();
    flag = false;
  }
}