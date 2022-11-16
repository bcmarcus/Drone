// #include <Arduino.h>
// #include "Wire.h"
// #include <Adafruit_Sensor.h>
// #include <Adafruit_I2CDevice.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <SPI.h>
#include <RHHardwareSPI.h>
#include <RHHardwareSPI1.h>

#include <Module.hpp>
#include <UltrasonicSensor.hpp>
#include <UltrasonicModule.hpp>

using namespace Robot::Modules;

UltrasonicModule* sm;
UltrasonicSensor* sm2;
void setup()
{
  sm = new UltrasonicModule (0, 2, 3, 4, 5, 6);
}

void loop()
{
  Serial.print (sm->pulse (1, 500000UL));
  Serial.println (" cm");
  Serial.print (sm->pulse (2, 500000UL));
  Serial.println (" cm");
  Serial.print (sm->pulse (3, 500000UL));
  Serial.println (" cm");
  Serial.print (sm->pulse (4, 500000UL));
  Serial.println (" cm");
  Serial.print (sm->pulse (5, 500000UL));
  Serial.println (" cm");
  Serial.print (sm->pulse (6, 500000UL));
  Serial.println (" cm");

  Serial.println ("\nhere\n");
  delay (250);
}