/*!
 * @file UltrasonicModule.hpp
 */
#ifndef __ULTRASONIC_MODULE_HPP__
#define __ULTRASONIC_MODULE_HPP__

#include "Arduino.h"
#include <Module.hpp>
#include "UltrasonicSensor.hpp"

namespace Robot {
   namespace Modules {
      class UltrasonicModule : public Module {
         public:
            UltrasonicModule (int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1, int8_t pin5 = -1, int8_t pin6 = -1);

         private:
            long pulse (int pin, uint32_t ms); 
      };
   };
};
#endif