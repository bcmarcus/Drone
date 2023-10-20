#include "HMC5883L.h"
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <unordered_map>
#include <functional>
#include <sstream>
#include <cstring>

uint32_t counter, timer;

boolean print = true;

TwoWire wire1 = Wire1;
HMC5883L compass = HMC5883L(wire1);

const int rxPin = 15;
const int txPin = 14;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

void customPrint (const char* data, boolean overridePrint = false) {
  if (Serial) {
    Serial.println (data);
  }

  if (bluetooth) {
    bluetooth.println (data);
  }
}

const std::string formatFloat(float val, boolean normalize = false) {
  static char out[7];  // Increase the size of the out array to accommodate null terminator
  if (normalize) {
    val *= 1000;
  }
  std::memset(out, ' ', sizeof(out));  // Initialize the array with spaces
  dtostrf(val, 5, 4, out);  // Use 6 instead of 7 to leave space for the potential negative sign
  out[6] = '\0';  // Add null terminator at the end
  return out;
}

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  compass.begin(true);
  while (!bluetooth.available()){};
  customPrint("Bluetooth found!");
  delay(10000);
  customPrint("Calibrating compass!");
  compass.calibrate();
  customPrint("Compass calibrated!");
  timer = millis();
}

void loop() {
  compass.fetchData();
  compass.calculateHeading();

  if (print) {
    std::stringstream payloadStream;
    payloadStream 
      << "Magnetic data - X: " << formatFloat(compass.getX()) << " Y: " << formatFloat(compass.getY()) << " Z: " << formatFloat(compass.getZ()) << " Z: "
      << " Heading: " << formatFloat(compass.getHeading()) << "   ";

    float hardIronBiasX, hardIronBiasY, hardIronBiasZ;
    float softIronScaleX, softIronScaleY, softIronScaleZ;

    compass.getHardIronBias(hardIronBiasX, hardIronBiasY, hardIronBiasZ);
    compass.getSoftIronScale(softIronScaleX, softIronScaleY, softIronScaleZ);

    payloadStream
        << "HardIron: X " << formatFloat(hardIronBiasX) << " Y " << formatFloat(hardIronBiasY) << " Z " << formatFloat(hardIronBiasZ)
        << "SoftIron: X " << formatFloat(softIronScaleX) << " Y " << formatFloat(softIronScaleY) << " Z " << formatFloat(softIronScaleZ);


    customPrint(payloadStream.str().c_str());
    delay(1000);
  }
  
  // counter++;
  // if (counter % 1000 == 0) {
  //   customPrint(counter);
  //   customPrint(" :: ");
  //   customPrint(millis() / 1000.0);
  //   customPrint(" :: ");
  //   customPrintln((millis() - timer) / 1000.0);
  //   timer = millis();
  // }
}
