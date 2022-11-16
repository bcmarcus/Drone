/*!
 * @file Module.hpp
 */
#ifndef __MODULE_HPP__
#define __MODULE_HPP__

#include "Arduino.h"
#include "Sensor.hpp"

using namespace Robot::Sensors;

namespace Robot {
   namespace Modules {
      class Module {
         public:
            Module (Sensor* s1 = nullptr, Sensor* s2 = nullptr, Sensor* s3 = nullptr, Sensor* s4 = nullptr, Sensor* s5 = nullptr, Sensor* s6 = nullptr);

            Sensor* s1;
            Sensor* s2;
            Sensor* s3;
            Sensor* s4;
            Sensor* s5;
            Sensor* s6;
      };
   };
};
#endif