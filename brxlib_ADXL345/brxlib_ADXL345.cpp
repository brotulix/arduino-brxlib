/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
    
    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/

/* Modified by Jørn Holm Aske to not use floats, and to yield values 1000x the
 *   SI units (to allow for some precision in spite of lacking float).
 */

#include "Arduino.h"
#include <Wire.h>
#include <limits.h>

#include "brxlib_ADXL345.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino Wire library
*/
/**************************************************************************/
inline uint8_t brxlib_ADXL345::i2cread(void) {
    return Wire.read();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino Wire library
*/
/**************************************************************************/
inline void brxlib_ADXL345::i2cwrite(uint8_t x) {
    Wire.write((uint8_t)x);
}

/**************************************************************************/
/*!
@brief  Abstract away SPI receiver & transmitter
*/
/**************************************************************************/
static uint8_t spixfer(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t data) {
    uint8_t reply = 0;
    for (int i=7; i>=0; i--) {
        reply <<= 1;
        digitalWrite(clock, LOW);
        digitalWrite(mosi, data & (1<<i));
        digitalWrite(clock, HIGH);
        if (digitalRead(miso)) 
            reply |= 1;
    }
    return reply;
}

/**************************************************************************/
/*!
@brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void brxlib_ADXL345::writeRegister(uint8_t reg, uint8_t value) {
    if (_i2c) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        i2cwrite((uint8_t)reg);
        i2cwrite((uint8_t)(value));
        Wire.endTransmission();
    } else {
        digitalWrite(_cs, LOW);
        spixfer(_clk, _di, _do, reg);
        spixfer(_clk, _di, _do, value);
        digitalWrite(_cs, HIGH);
    }
}

/**************************************************************************/
/*!
@brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t brxlib_ADXL345::readRegister(uint8_t reg) {
    if (_i2c) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        i2cwrite(reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
        return (i2cread());
    } else {
        reg |= 0x80; // read byte
        digitalWrite(_cs, LOW);
        spixfer(_clk, _di, _do, reg);
        uint8_t reply = spixfer(_clk, _di, _do, 0xFF);
        digitalWrite(_cs, HIGH);
        return reply;
    }  
}

/**************************************************************************/
/*!
@brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t brxlib_ADXL345::read16(uint8_t reg) {
    if (_i2c) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        i2cwrite(reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)2);
        return (uint16_t)(i2cread() | (i2cread() << 8));  
    } else {
        reg |= 0x80 | 0x40; // read byte | multibyte
        digitalWrite(_cs, LOW);
        spixfer(_clk, _di, _do, reg);
        uint16_t reply = spixfer(_clk, _di, _do, 0xFF)  | (spixfer(_clk, _di, _do, 0xFF) << 8);
        digitalWrite(_cs, HIGH);
        return reply;
    }    
}

/**************************************************************************/
/*! 
@brief  Reads the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t brxlib_ADXL345::getDeviceID(void) {
// Check device ID register
    return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*! 
@brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t brxlib_ADXL345::getX(void) {
    return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*! 
@brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t brxlib_ADXL345::getY(void) {
    return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*! 
@brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t brxlib_ADXL345::getZ(void) {
    return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
@brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
brxlib_ADXL345::brxlib_ADXL345(int32_t sensorID) {
    _sensorID = sensorID;
    _range = ADXL345_RANGE_2_G;
    _i2c = true;
}

/**************************************************************************/
/*!
@brief  Instantiates a new ADXL345 class in SPI mode
*/
/**************************************************************************/
brxlib_ADXL345::brxlib_ADXL345(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs, int32_t sensorID) {
    _sensorID = sensorID;
    _range = ADXL345_RANGE_2_G;
    _cs = cs;
    _clk = clock;
    _do = mosi;
    _di = miso;
    _i2c = false;
}

/**************************************************************************/
/*!
@brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool brxlib_ADXL345::begin(uint8_t i2caddr) {
    _i2caddr = i2caddr;

    if (_i2c)
        Wire.begin();
    else {
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        pinMode(_clk, OUTPUT);
        digitalWrite(_clk, HIGH);
        pinMode(_do, OUTPUT);
        pinMode(_di, INPUT);
    }

    /* Check connection */
    uint8_t deviceid = getDeviceID();
    if (deviceid != 0xE5)
    {
        /* No ADXL345 detected ... return false */
        return false;
    }

    // Enable measurements
    writeRegister(ADXL345_REG_POWER_CTL, 0x08);  

    return true;
}

/**************************************************************************/
/*!
@brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void brxlib_ADXL345::setRange(range_t range)
{
    /* Read the data format register to preserve bits */
    uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

    /* Update the data rate */
    format &= ~0x0F;
    format |= range;

    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(ADXL345_REG_DATA_FORMAT, format);

    /* Keep track of the current range (to avoid readbacks) */
    _range = range;
}

/**************************************************************************/
/*!
@brief  Gets the g range for the accelerometer
*/
/**************************************************************************/
range_t brxlib_ADXL345::getRange(void)
{
    /* Read the data format register to preserve bits */
    return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
@brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void brxlib_ADXL345::setDataRate(dataRate_t dataRate)
{
    /* Note: The LOW_POWER bits are currently ignored and we always keep
    the device in 'normal' mode */
    writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
@brief  Gets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t brxlib_ADXL345::getDataRate(void)
{
    return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*! 
@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool brxlib_ADXL345::getEvent(sensors_event_t *event) {
    /* Clear the event */
    int32_t c = 0;

    memset(event, 0, sizeof(sensors_event_t));

    event->version   = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type      = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp = 0;
    
    c = 0;
    c = getX();
    c *= ADXL345_MG2G_MULTIPLIER;
    c *= SENSORS_GRAVITY_STANDARD;
    c /= 1000;
    //event->acceleration.x = (int32_t)getX() * ((ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD)/1000);
    event->acceleration.x = c;
    
    c = 0;
    c = getY();
    c *= ADXL345_MG2G_MULTIPLIER;
    c *= SENSORS_GRAVITY_STANDARD;
    c /= 1000;
    event->acceleration.y = c;
    //event->acceleration.y = (int32_t)getY() * ((ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD)/1000);
    
    c = 0;
    c = getZ();
    c *= ADXL345_MG2G_MULTIPLIER;
    c *= SENSORS_GRAVITY_STANDARD;
    c /= 1000;
    event->acceleration.z = c;
    //event->acceleration.z = (int32_t)getZ() * ((ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD)/1000);

    return true;
}

/**************************************************************************/
/*! 
@brief  Gets the sensor_t data
*/
/**************************************************************************/
void brxlib_ADXL345::getSensor(sensor_t *sensor) {
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy (sensor->name, "ADXL345", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->type        = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay   = 0;
    sensor->max_value   = -156906; /* -16g = 156.9064 m/s^2  */
    sensor->min_value   = 156906;  /*  16g = 156.9064 m/s^2  */
    sensor->resolution  = 39;   /*  4mg = 0.0392266 m/s^2 */ 
}
