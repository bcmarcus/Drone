# Drone Controller

This project is a drone controller implemented in C++. It uses a modified version of the Servo library to control the drone's motors. The Servo library has been modified to allow for finer control over the pulse width sent to the servos, which in turn allows for more precise control over the drone's movements.

## Description

The Drone Controller project is designed to provide a high level of control over a drone's movements. It does this by modifying the Servo library to allow for a wider range of pulse widths to be sent to the servos. This allows for more precise control over the drone's motors, and therefore its movements.

## Modifications

The following modifications were made to the Servo library:

- The `MIN_PULSE_WIDTH` was set to 5, the `MAX_PULSE_WIDTH` was set to 2400, and the `DEFAULT_PULSE_WIDTH` was set to 1500. This allows for a wider range of pulse widths to be sent to the servos.
- The `REFRESH_INTERVAL` was set to 100 microseconds. This is the minimum time to refresh servos.
- The `writeMicroseconds` function was modified to accept a double value. This allows for more precise control over the pulse width.
- The `attach` function was modified to set the minimum and maximum ticks based on the input parameters. This allows for more precise control over the range of pulse widths that can be sent to the servos.

## PWM Types

The higher the PWM type (where oneshot125 > oneshot42), the finer the fidelity. However, it cannot be polled nearly as often.

## Installation

*Details about the installation process and required packages will be added soon.*

## Usage

To use the Drone Controller, you first need to include the modified Servo library in your C++ program. Here's a basic example of how to control a servo: