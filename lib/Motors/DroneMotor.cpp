/*!
 * @file Module.cpp
 */

#include "Arduino.h"
#include "DroneMotor.hpp"

using namespace Robot::Motors;

DroneMotor::DroneMotor (int8_t pwmPin) 
: Sensor (pwmPin)
{

}

bool DroneMotor::drive (float dutyCycle, long frequency) {
   analogWriteFrequency(pin1, frequency);
   analogWrite(pin1, (int) dutyCycle * 255);
}

void DroneMotor::on () {
   digitalWrite(pin1, 1);
}

void DroneMotor::off () {
   digitalWrite(pin1, 0);
}
