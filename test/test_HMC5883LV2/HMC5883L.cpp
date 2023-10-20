#include "HMC5883L.h"

uint32_t counter, timer;

boolean print = true;

TwoWire wire1 = Wire1;
HMC5883L compass = HMC5883L(wire1);

void setup() {
  Serial.begin(9600);
  compass.begin(true);
  compass.calibrate();
  timer = millis();
}

void loop() {
  compass.fetchData();
  compass.calculateHeading();

  if (print) {
    Serial.print("Magnetic data - X: "); Serial.print(compass.getX());
    Serial.print(" Y: "); Serial.print(compass.getY());
    Serial.print(" Z: "); Serial.print(compass.getZ());
    Serial.print(" Heading: "); Serial.println(compass.getHeading()); 
    delay(1000);
  }
  
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
