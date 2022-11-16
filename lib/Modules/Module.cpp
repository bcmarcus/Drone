/*!
 * @file Module.cpp
 */

#include "Arduino.h"
#include "Module.hpp"
#include "Sensor.hpp"

using namespace Robot::Modules;
using namespace Robot::Sensors;

Module::Module (Sensor* s1, Sensor* s2, Sensor* s3, Sensor* s4, Sensor* s5, Sensor* s6) {
   this->s1 = s1;
   this->s2 = s2;
   this->s3 = s3;
   this->s4 = s4;
   this->s5 = s5;
   this->s6 = s6;
}
