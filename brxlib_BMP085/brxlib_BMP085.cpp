/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
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
#include "brxlib_sensors.h"

#include <math.h>
#include <limits.h>

#include "brxlib_BMP085.h"

static bmp085_calib_data _bmp085_coeffs;   // Last read accelerometer data will be available here
static uint8_t           _bmp085Mode;

#define BMP085_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */



/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/



/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
static void writeCommand(byte reg, byte value)
{
    Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}



/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
static void read8(byte reg, uint8_t *value)
{
    Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)1);
    *value = Wire.read();
    Wire.endTransmission();
}



/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
static void read16(byte reg, uint16_t *value)
{
    Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)2);
    *value = (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();
}



/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
static void readS16(byte reg, int16_t *value)
{
    uint16_t i;
    read16(reg, &i);
    *value = (int16_t)i;
}



/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
static void readCoefficients(void)
{
#if BMP085_USE_DATASHEET_VALS
    _bmp085_coeffs.ac1 = 408;
    _bmp085_coeffs.ac2 = -72;
    _bmp085_coeffs.ac3 = -14383;
    _bmp085_coeffs.ac4 = 32741;
    _bmp085_coeffs.ac5 = 32757;
    _bmp085_coeffs.ac6 = 23153;
    _bmp085_coeffs.b1  = 6190;
    _bmp085_coeffs.b2  = 4;
    _bmp085_coeffs.mb  = -32768;
    _bmp085_coeffs.mc  = -8711;
    _bmp085_coeffs.md  = 2868;
    _bmp085Mode        = 0;
#else
    readS16(BMP085_REGISTER_CAL_AC1, &_bmp085_coeffs.ac1);
    readS16(BMP085_REGISTER_CAL_AC2, &_bmp085_coeffs.ac2);
    readS16(BMP085_REGISTER_CAL_AC3, &_bmp085_coeffs.ac3);
    read16(BMP085_REGISTER_CAL_AC4, &_bmp085_coeffs.ac4);
    read16(BMP085_REGISTER_CAL_AC5, &_bmp085_coeffs.ac5);
    read16(BMP085_REGISTER_CAL_AC6, &_bmp085_coeffs.ac6);
    readS16(BMP085_REGISTER_CAL_B1, &_bmp085_coeffs.b1);
    readS16(BMP085_REGISTER_CAL_B2, &_bmp085_coeffs.b2);
    readS16(BMP085_REGISTER_CAL_MB, &_bmp085_coeffs.mb);
    readS16(BMP085_REGISTER_CAL_MC, &_bmp085_coeffs.mc);
    readS16(BMP085_REGISTER_CAL_MD, &_bmp085_coeffs.md);
#endif
}



/**************************************************************************/
/*!

*/
/**************************************************************************/
static void readRawTemperature(int32_t *temperature)
{
#if BMP085_USE_DATASHEET_VALS
    *temperature = 27898;
#else
    uint16_t t;
    writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
    delay(5);
    read16(BMP085_REGISTER_TEMPDATA, &t);
    *temperature = t;
#endif
}



/**************************************************************************/
/*!

*/
/**************************************************************************/
static void readRawPressure(int32_t *pressure)
{
#if BMP085_USE_DATASHEET_VALS
    *pressure = 23843;
#else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;

    writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6));
    switch(_bmp085Mode)
    {
        case BMP085_MODE_ULTRALOWPOWER:
            delay(5);
            break;
        case BMP085_MODE_STANDARD:
            delay(8);
            break;
        case BMP085_MODE_HIGHRES:
            delay(14);
            break;
        case BMP085_MODE_ULTRAHIGHRES:
        default:
            delay(26);
            break;
    }

    read16(BMP085_REGISTER_PRESSUREDATA, &p16);
    p32 = (uint32_t)p16 << 8;
    read8(BMP085_REGISTER_PRESSUREDATA+2, &p8);
    p32 += p8;
    p32 >>= (8 - _bmp085Mode);

    *pressure = p32;
#endif
}


#if BRXLIB_COMPUTEB5_MACRO
#define computeB5(ut)   (uint32_t)(((((uint32_t)ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5)) >> 15) + ((int32_t)_bmp085_coeffs.mc << 11) / (((((uint32_t)ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5)) >> 15) + (int32_t)_bmp085_coeffs.md))
#else
/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t brxlib_BMP085::computeB5(int32_t ut) {
    int32_t X1 = ((ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5)) >> 15;
    int32_t X2 = ((int32_t)_bmp085_coeffs.mc << 11) / (X1 + (int32_t)_bmp085_coeffs.md);
    return X1 + X2;
}
#endif

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 


/**************************************************************************/
/*!
    @brief  Instantiates a new brxlib_BMP085 class
*/
/**************************************************************************/
brxlib_BMP085::brxlib_BMP085(int32_t sensorID) {
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
bool brxlib_BMP085::begin(bmp085_mode_t mode)
{
    // Enable I2C
    Wire.begin();

    /* Mode boundary check */
    if ((mode > BMP085_MODE_ULTRAHIGHRES) || (mode < 0))
    {
        mode = BMP085_MODE_ULTRAHIGHRES;
    }

    /* Make sure we have the right device */
    uint8_t id;
    read8(BMP085_REGISTER_CHIPID, &id);
    if(id != 0x55)
    {
        return false;
    }

    /* Set the mode indicator */
    _bmp085Mode = mode;

    /* Coefficients need to be read once */
    readCoefficients();

    return true;
}



/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
void brxlib_BMP085::getPressure(int32_t *pressure)
{
    int32_t ut = 0; // temperature data (16 bit)
    int32_t up = 0; // pressure data (16 to 19 bit)
    int32_t compp = 0;
    int32_t x1 = 0;
    int32_t x2 = 0;
    int32_t b5 = 0;
    int32_t b6 = 0;
    int32_t x3 = 0;
    int32_t b3 = 0;
    int32_t p = 0;
    uint32_t b4 = 0;
    uint32_t b7 = 0;

    /* Get the raw pressure and temperature values */
    readRawTemperature(&ut);
    readRawPressure(&up);

    /* Temperature compensation */
    b5 = computeB5(ut);

    /* Pressure compensation */
    b6 = (b5 + 8) >> 4;
    x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
    x3 = x1 + x2;
    // B3 = ((AC1*4+X3) << oss + 2) / 4
    //b3 = (((((int32_t) _bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
    b3 = ((((((int32_t) _bmp085_coeffs.ac1 ) * 4 ) + x3) << _bmp085Mode) + 2) >> 2;

    x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
    x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t)up - b3) * (50000 >> _bmp085Mode);

    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    /* Assign compensated pressure value */
    *pressure = compp;
}



/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius times 10
*/
/**************************************************************************/
void brxlib_BMP085::getTemperature(int32_t *temp)
{
    int32_t UT = 0;
    int32_t B5 = 0;     // following ds convention
    int32_t t = 0;

    readRawTemperature(&UT);

#if BMP085_USE_DATASHEET_VALS
    // use datasheet numbers!
    UT = 27898;
    _bmp085_coeffs.ac6 = 23153;
    _bmp085_coeffs.ac5 = 32757;
    _bmp085_coeffs.mc = -8711;
    _bmp085_coeffs.md = 2868;
#endif

    B5 = computeB5(UT);
    t = (B5+8) >> 4;
    
    // Leave it as 10x temperature since we don't have float.
    //t /= 10;

    *temp = t;
}


#if BRXLIB_ADVANCED_MATH // We don't need this functionality for now
/**************************************************************************/
/*!
Calculates the altitude (in meters) from the specified atmospheric
pressure (in hPa), and sea-level pressure (in hPa).

@param  seaLevelPressure      Sea-level pressure in hPa
@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float brxlib_BMP085::pressureToAltitude(float seaLevelPressure, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return 44330.0 * (1.0 - pow(atmospheric / seaLevelPressure, 0.1903));
}


/**************************************************************************/
/*!
Calculates the pressure at sea level (in hPa) from the specified altitude 
(in meters), and atmospheric pressure (in hPa).  

@param  altitude      Altitude in meters
@param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float brxlib_BMP085::seaLevelForAltitude(float altitude, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}
#endif // if BRXLIB_ADVANCED_MATH



/**************************************************************************/
/*!
@brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void brxlib_BMP085::getSensor(sensor_t *sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy (sensor->name, "BMP085", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->type        = SENSOR_TYPE_PRESSURE;
    sensor->min_delay   = 0;
    sensor->max_value   = 30000;               // 300..1100 hPa
    sensor->min_value   = 110000;
    sensor->resolution  = 1;                // Datasheet states 0.01 hPa resolution
}

/**************************************************************************/
/*!
@brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
bool brxlib_BMP085::getEvent(sensors_event_t *event)
{
    int32_t pressure_Pa;

    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version   = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type      = SENSOR_TYPE_PRESSURE;
    event->timestamp = 0;
    getPressure(&pressure_Pa);
    event->pressure = pressure_Pa;

    return true;
}
