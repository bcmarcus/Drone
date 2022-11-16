/*
    UltrasonicSensor.cpp
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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "UltrasonicSensor.hpp"

using namespace Robot::Sensors;


#ifdef ARDUINO_ARCH_STM32F4

static uint32_t MicrosDiff(uint32_t begin, uint32_t end) {
    return end - begin;
}

static uint32_t pulseIn(uint32_t pin, uint32_t state, uint32_t timeout = 1000000L) {
    uint32_t begin = micros();

    // wait for any previous pulse to end
    while (digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
            return 0;
        }

    // wait for the pulse to start
    while (!digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
            return 0;
        }
    uint32_t pulseBegin = micros();

    // wait for the pulse to stop
    while (digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
            return 0;
        }
    uint32_t pulseEnd = micros();

    return MicrosDiff(pulseBegin, pulseEnd);
}

#endif // ARDUINO_ARCH_STM32F4

UltrasonicSensor::UltrasonicSensor(int pin)
: Sensor (pin)
{
    //do nothing
}

// long UltrasonicSensor::on () {
//     pinMode(pin1, OUTPUT);
//     digitalWrite(pin1, HIGH);
// }

// long UltrasonicSensor::offAndRead () {
//     pinMode(pin1, OUTPUT);
//     digitalWrite(pin1, LOW);
//     pinMode(pin1, INPUT);

// }

long UltrasonicSensor::duration(uint32_t timeout) {
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, LOW);
    delayMicroseconds(2);
    digitalWrite(pin1, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin1, LOW);
    pinMode(pin1, INPUT);
    long duration;
    duration = pulseIn(pin1, HIGH, timeout);
    return duration;
}

/*The measured distance from the range 0 to 400 Centimeters*/
long UltrasonicSensor::MeasureInCentimeters(uint32_t timeout) {
    long RangeInCentimeters;
    RangeInCentimeters = duration(timeout) / 29 / 2;
    return RangeInCentimeters;
}

/*The measured distance from the range 0 to 4000 Millimeters*/
long UltrasonicSensor::MeasureInMillimeters(uint32_t timeout) {
    long RangeInMillimeters;
    RangeInMillimeters = duration(timeout) * (10 / 2) / 29;
    return RangeInMillimeters;
}

/*The measured distance from the range 0 to 157 Inches*/
long UltrasonicSensor::MeasureInInches(uint32_t timeout) {
    long RangeInInches;
    RangeInInches = duration(timeout) / 74 / 2;
    return RangeInInches;
}