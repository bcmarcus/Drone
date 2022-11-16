/*!
 * @file UltrasonicModule.cpp
 */

#include "Arduino.h"
#include <UltrasonicModule.hpp>
#include "UltrasonicSensor.hpp"
#include "Sensor.hpp"

using namespace Robot::Modules;
using namespace Robot::Sensors;

UltrasonicModule::UltrasonicModule (int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4, int8_t pin5, int8_t pin6) 
: Module(
   new UltrasonicSensor (pin1), 
   new UltrasonicSensor (pin2), 
   new UltrasonicSensor (pin3), 
   new UltrasonicSensor (pin4), 
   new UltrasonicSensor (pin5), 
   new UltrasonicSensor (pin6)
)
{
   //**//
}

long UltrasonicModule::pulse (int pinPos, uint32_t ms) {
   Sensor* s = nullptr;
   switch (pinPos) {
      case 1:
         s = s1;
         break;
      case 2:
         s = s2;
         break;
      case 3:
         s = s3;
         break;
      case 4:
         s = s4;
         break;
      case 5:
         s = s5;
         break;
      case 6:
         s = s6;
         break;
   }

   if (s == nullptr) {
      return -1;
   }
   
   return ((UltrasonicSensor*) s)->MeasureInCentimeters(ms);
}

// long* UltrasonicModule::multiPulse (int* pins, int numpins, uint32_t ms) {
//    for (int i = 0; i < numpins; i++) {

//    }
//    this->
// }
