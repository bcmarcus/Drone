/*!
 * @file Sensor.cpp
 */

#include "Arduino.h"
#include "Sensor.hpp"

using namespace Robot::Sensors;

Sensor::Sensor (int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4, int8_t pin5, int8_t pin6) {
   this->pin1 = pin1;
   this->pin2 = pin2;
   this->pin3 = pin3;
   this->pin4 = pin4;
   this->pin5 = pin5;
   this->pin6 = pin6;
}
