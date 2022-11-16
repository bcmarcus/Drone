/*!
 * @file Module.hpp
 */
#ifndef __DRONE_MOTORS_HPP__
#define __DRONE_MOTORS_HPP__

#include "Arduino.h"
#include "Sensor.hpp"

using namespace Robot::Sensors;

namespace Robot {
   namespace Motors {
      class DroneMotor : public Sensor {
         public:
            DroneMotor (int8_t pwmPin = -1);
            bool drive (float dutyCycle, long frequency);
            void on ();
            void off ();
      };
   };
};
#endif