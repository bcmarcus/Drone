/*!
 * @file Module.hpp
 */
#ifndef __DRONE_MOTORS_HPP__
#define __DRONE_MOTORS_HPP__

#include "Arduino.h"

#include "../Custom/CustomSerialPrint.h"

// converts microseconds to tick (assumes prescale of 8)
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us))
// converts from ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned)_ticks) / clockCyclesPerMicrosecond())

#define ESC_PIN_1 5
#define ESC_PIN_2 6
#define ESC_PIN_3 9
#define ESC_PIN_4 10

enum MotorId { Motor0, Motor1, Motor2, Motor3 };

namespace Robot {
   namespace Motors {
      class DroneMotor {
         private:
            static const int nbMotors = 4;
            const int MIN_POWER = 1000;
            const int MAX_POWER = 2000;             // Max pwr available. Set to 1860 to reach max
            const int MAX_THROTTLE_PERCENT = 100.0; // Percent to restrain max motor power
            uint16_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0); // Restrained max power
            int IDLE_THRESHOLD = 1100;
            static uint16_t motorsTicks[nbMotors];
            static int currMotor;

         public:
            void Init();
            void UpdateSpeed(int _id, float _PWM);
            void Idle();
            static void ApplySpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
            const int GetMotorsMaxPower() {
               return MAX_POWER;
            }
            const int GetMotorsMinPower() {
               return MIN_POWER;
            }
            const int GetMotorsMaxThrottlePercent() {
               return MAX_THROTTLE_PERCENT;
            }
            const int GetMotorsMaxThrottle() {
               return MAX_THROTTLE;
            }
            int GetMotorsIdleThreshold() {
               return IDLE_THRESHOLD;
            }

         private:
            void InitTimer1();
      };
   }
};
#endif