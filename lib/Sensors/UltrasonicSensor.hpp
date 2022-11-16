/*
    UltrasonicSensor.h
    A library for UltrasonicSensor ranger

    Copyright (c) 2012 seeed technology inc.
    Website    : www.seeed.cc
    Author     : LG, FrankieChu
    Create Time: Jan 17,2013
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#ifndef __ULTRASONIC_SENSOR_HPP__
#define __ULTRASONIC_SENSOR_HPP__

#include "Arduino.h"
#include "Sensor.hpp"

namespace Robot {
  namespace Sensors {
    class UltrasonicSensor : public Sensor {
      public:
        UltrasonicSensor(int pin);
        // long on ();
        // long off ();
        long MeasureInCentimeters(uint32_t timeout = 1000000L);
        long MeasureInMillimeters(uint32_t timeout = 1000000L);
        long MeasureInInches(uint32_t timeout = 1000000L);
      private:
        long duration(uint32_t timeout = 1000000L);
    };
  };
};

#endif