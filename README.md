# Drone Controller

This project is a drone controller implemented in C++. It uses a modified version of the Servo library to control the drone's motors. The Servo library has been modified to allow for finer control over the pulse width sent to the servos, which in turn allows for more precise control over the drone's movements. It uses a custom PCB (https://oshwlab.com/tacosaurus/main-board-revamped_copy) which has a radio, built in motor drivers, the teensy, and some power distribution logic and some pinouts for a modular form factor.

## Description

The Drone Controller project is designed to provide a high level of control over a drone's movements. Eventually, the goal is to have the entire system be completely autonomous with all calculations on board, however for now, most camera processing happens on a server connected to the drone. 

## PWM Types

The higher the PWM type (where oneshot125 > oneshot42), the finer the fidelity. However, it cannot be polled nearly as often.

## Installation

Use platformio to run one of the files in the test folder. Tests exist for every single component on the PCB, as well as multiple version of the drone main code, which had a few flaws.

### Modifications

The following modifications were made to the Servo library:

- The `MIN_PULSE_WIDTH` was set to 5, the `MAX_PULSE_WIDTH` was set to 2400, and the `DEFAULT_PULSE_WIDTH` was set to 1500. This allows for a wider range of pulse widths to be sent to the servos.
- The `REFRESH_INTERVAL` was set to 100 microseconds. This is the minimum time to refresh servos.
- The `writeMicroseconds` function was modified to accept a double value. This allows for more precise control over the pulse width.
- The `attach` function was modified to set the minimum and maximum ticks based on the input parameters. This allows for more precise control over the range of pulse widths that can be sent to the servos.

The rest should simply work after plugging in.

## Usage

Plug it in and tune the PID controllers by locking it to one axis of rotation, for every axis, and then it should fly properly. For a simple test file, look at test_DRONEV8
