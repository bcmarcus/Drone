#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>

byte start_address = 0;
byte end_address = 127;

TwoWire wire = Wire1;

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_BIT_I2C_BYPASS_EN 0x02

void setup()
{
  byte rc;
  wire.begin();

  Serial.begin(9600);
  Serial.println("\nI2C Scanner");

  Serial.print("Scanning I2C bus from ");
  Serial.print(start_address,DEC);  Serial.print(" to ");  Serial.print(end_address,DEC);
  Serial.println("...");

  setI2CBypass(true);

  for( byte addr  = start_address;
            addr <= end_address;
            addr++ ) {
      wire.beginTransmission(addr);
      rc = wire.endTransmission();

      if (addr<16) Serial.print("0");
      Serial.print(addr,HEX);
      if (rc==0) {
        Serial.print(" found!");
      } else {
        Serial.print(" "); Serial.print(rc); Serial.print("     ");
      }
      Serial.print( (addr%8)==7 ? "\n":" ");
  }

  Serial.println("\n-------------------------------\nPossible devices:");
  for( byte addr  = start_address;
            addr <= end_address;
            addr++ ) {
      wire.beginTransmission(addr);
      rc = wire.endTransmission();
      if (rc == 0) {
        Serial.print(addr,HEX); Serial.print(" = ");
        switch (addr) {
          case 0x1E: Serial.println("HMC5883L"); break;
          case 0x50: Serial.println("AT24C32/AT24C64 - EEPROM"); break;
          case 0x68: Serial.println("MPU6050"); break;
          case 0x77: Serial.println("BMP085"); break;
          default: Serial.println("Unknown"); break;
        }
      }
  }

  Serial.println("\ndone");
}

// standard Arduino loop()
void loop()
{
    // Nothing to do here, so we'll just blink the built-in LED
    delay(300);
    // digitalWrite(13,HIGH); delay(300);
    // digitalWrite(13,LOW);  delay(300);
}