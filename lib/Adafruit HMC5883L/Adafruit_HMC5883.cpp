/*!
 * @file Adafruit_HMC5883.cpp
 *
 * @mainpage Adafruit HMC5883 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HMC5883 magnentometer/compass
 *
 * Designed specifically to work with the Adafruit HMC5883 Breakout
 * http://www.adafruit.com/products/1746
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

#include <limits.h>

#include "Adafruit_HMC5883.h"

static float _hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void Adafruit_HMC5883::write8(uint8_t a, uint8_t d) {
  // send d prefixed with a (a d [stop])
  i2c_dev->write(&d, 1, true, &a, 1);
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t Adafruit_HMC5883::read8(uint8_t a) {
  uint8_t ret;

  // send 1 byte, reset i2c, read 1 byte
  i2c_dev->write_then_read(&a, 1, &ret, 1, true);

  return ret;
}

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void Adafruit_HMC5883::read() {
  uint8_t add = HMC5883_REGISTER_MAG_OUT_X_H_M;
  i2c_dev->write(&add, 1, false);

  // // Note high before low (different than accel)
  uint8_t xhi, xlo, zhi, zlo, yhi, ylo;
  i2c_dev->read(&xhi, 1, false);
  i2c_dev->read(&xlo, 1, false);
  i2c_dev->read(&zhi, 1, false);
  i2c_dev->read(&zlo, 1, false);
  i2c_dev->read(&yhi, 1, false);
  i2c_dev->read(&ylo, 1, false);

  // uint8_t x, y, z;
  // i2c_dev->read(&x, 2, false);
  // i2c_dev->read(&z, 2, false);
  // i2c_dev->read(&y, 2, false);
  
  Serial.println (xlo);
  Serial.println (xhi);
  Serial.println (zhi);
  Serial.println (zlo);
  Serial.println (yhi);
  Serial.println (ylo);
  // _magData.x = (int16_t)x;
  // _magData.y = (int16_t)y;
  // _magData.z = (int16_t)z;

  // Shift values to create properly formed integer (low byte first)
  _magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
  _magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
  _magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));

  // ToDo: Calculate orientation
  _magData.orientation = 0.0;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_HMC5883 class
*/
/**************************************************************************/
Adafruit_HMC5883::Adafruit_HMC5883(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_HMC5883::begin(hmc5883MagGain gain, TwoWire *wire) {
    if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice((byte)HMC5883_ADDRESS_MAG, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  uint8_t add = HMC5883_ADDRESS_MAG;
  i2c_dev->write (&add, HMC5883_REGISTER_MAG_MR_REG_M, false);

  // Enable the magnetometer
  write8(HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

  // Set the gain to a known level
  setMagGain(gain);

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Adafruit_HMC5883::setMagGain(hmc5883MagGain gain) {
  write8(HMC5883_REGISTER_MAG_CRB_REG_M, (byte)gain);

  _magGain = gain;

  switch (gain) {
  case HMC5883_MAGGAIN_1_3:
    _hmc5883_Gauss_LSB_XY = 1100;
    _hmc5883_Gauss_LSB_Z = 980;
    break;
  case HMC5883_MAGGAIN_1_9:
    _hmc5883_Gauss_LSB_XY = 855;
    _hmc5883_Gauss_LSB_Z = 760;
    break;
  case HMC5883_MAGGAIN_2_5:
    _hmc5883_Gauss_LSB_XY = 670;
    _hmc5883_Gauss_LSB_Z = 600;
    break;
  case HMC5883_MAGGAIN_4_0:
    _hmc5883_Gauss_LSB_XY = 450;
    _hmc5883_Gauss_LSB_Z = 400;
    break;
  case HMC5883_MAGGAIN_4_7:
    _hmc5883_Gauss_LSB_XY = 400;
    _hmc5883_Gauss_LSB_Z = 255;
    break;
  case HMC5883_MAGGAIN_5_6:
    _hmc5883_Gauss_LSB_XY = 330;
    _hmc5883_Gauss_LSB_Z = 295;
    break;
  case HMC5883_MAGGAIN_8_1:
    _hmc5883_Gauss_LSB_XY = 230;
    _hmc5883_Gauss_LSB_Z = 205;
    break;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_HMC5883::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = 0;
  event->magnetic.x =
      _magData.x / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y =
      _magData.y / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z =
      _magData.z / _hmc5883_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_HMC5883::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "HMC5883", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 800;  // 8 gauss == 800 microTesla
  sensor->min_value = -800; // -8 gauss == -800 microTesla
  sensor->resolution = 0.2; // 2 milligauss == 0.2 microTesla
}
