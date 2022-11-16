/*!
 * @file Sensor.hpp
 */
#ifndef __SENSOR_HPP__
#define __SENSOR_HPP__

#include "Arduino.h"

namespace Robot {
   namespace Sensors {
      class Sensor {
         public:
            Sensor (int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1, int8_t pin5 = -1, int8_t pin6 = -1);

            int8_t pin1, pin2, pin3, pin4, pin5, pin6;
      };
   };
};
#endif