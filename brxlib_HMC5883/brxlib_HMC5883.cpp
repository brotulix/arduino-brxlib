/***************************************************************************
  This is a library for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

/* Modified by Jørn Holm Aske to not use floats, and to yield values 1000x the
 *   SI units (to allow for some precision in spite of lacking float).
 */

#include "Arduino.h"

#include <Wire.h>
#include <limits.h>

#include "brxlib_HMC5883.h"

static int32_t _hmc5883_Gauss_LSB_XY = 1100;  // Varies with gain
static int32_t _hmc5883_Gauss_LSB_Z  = 980;   // Varies with gain

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
void brxlib_HMC5883::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte brxlib_HMC5883::read8(byte address, byte reg)
{
    byte value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

/**************************************************************************/
/*!
@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void brxlib_HMC5883::read()
{
    // Read the magnetometer
    Wire.beginTransmission((byte)HMC5883_ADDRESS_MAG);
    Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
    Wire.endTransmission();
    Wire.requestFrom((byte)HMC5883_ADDRESS_MAG, (byte)6);

    // Wait around until enough data is available
    while (Wire.available() < 6);

    // Note high before low (different than accel)  
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();

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
brxlib_HMC5883::brxlib_HMC5883(int32_t sensorID) {
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
bool brxlib_HMC5883::begin()
{
    // Enable I2C
    Wire.begin();

    // Enable the magnetometer
    write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

    // Set the gain to a known level
    setMagGain(HMC5883_MAGGAIN_1_3);

    return true;
}

/**************************************************************************/
/*!
@brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void brxlib_HMC5883::setMagGain(hmc5883MagGain gain)
{
    write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte)gain);

    _magGain = gain;

    switch(gain)
    {
        case HMC5883_MAGGAIN_1_3:
            _hmc5883_Gauss_LSB_XY = 1100;
            _hmc5883_Gauss_LSB_Z  = 980;
            break;
        case HMC5883_MAGGAIN_1_9:
            _hmc5883_Gauss_LSB_XY = 855;
            _hmc5883_Gauss_LSB_Z  = 760;
            break;
        case HMC5883_MAGGAIN_2_5:
            _hmc5883_Gauss_LSB_XY = 670;
            _hmc5883_Gauss_LSB_Z  = 600;
            break;
        case HMC5883_MAGGAIN_4_0:
            _hmc5883_Gauss_LSB_XY = 450;
            _hmc5883_Gauss_LSB_Z  = 400;
            break;
        case HMC5883_MAGGAIN_4_7:
            _hmc5883_Gauss_LSB_XY = 400;
            _hmc5883_Gauss_LSB_Z  = 255;
            break;
        case HMC5883_MAGGAIN_5_6:
            _hmc5883_Gauss_LSB_XY = 330;
            _hmc5883_Gauss_LSB_Z  = 295;
            break;
        case HMC5883_MAGGAIN_8_1:
            _hmc5883_Gauss_LSB_XY = 230;
            _hmc5883_Gauss_LSB_Z  = 205;
            break;
    } 
}

/**************************************************************************/
/*! 
@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool brxlib_HMC5883::getEvent(sensors_event_t *event)
{
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    /* Read new data */
    read();

    event->version   = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
    event->timestamp = 0;
    event->magnetic.x = (_magData.x  * SENSORS_GAUSS_TO_MICROTESLA) / _hmc5883_Gauss_LSB_XY;
    event->magnetic.y = (_magData.y * SENSORS_GAUSS_TO_MICROTESLA) / _hmc5883_Gauss_LSB_XY;
    event->magnetic.z = (_magData.z * SENSORS_GAUSS_TO_MICROTESLA) / _hmc5883_Gauss_LSB_Z;

    return true;
}

/**************************************************************************/
/*! 
@brief  Gets the sensor_t data
*/
/**************************************************************************/
void brxlib_HMC5883::getSensor(sensor_t *sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy (sensor->name, "HMC5883", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
    sensor->min_delay   = 0;
    sensor->max_value   = 800000; // 8 gauss == 800 microTesla
    sensor->min_value   = -800000; // -8 gauss == -800 microTesla
    sensor->resolution  = 20; // 2 milligauss == 0.2 microTesla
}
