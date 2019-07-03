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
#ifndef BRXLIB_BMP085_H
#define BRXLIB_BMP085_H

#include "Arduino.h"
#include <Wire.h>
#include "brxlib_sensors.h"

#define BRXLIB_ADVANCED_MATH    0
#define BRXLIB_COMPUTEB5_MACRO  0





#define BMP085_ADDRESS                (0x77)
enum
{
    BMP085_REGISTER_CAL_AC1            = 0xAA,  // 16 bit calibration data
    BMP085_REGISTER_CAL_AC2            = 0xAC,  // 16 bit calibration data
    BMP085_REGISTER_CAL_AC3            = 0xAE,  // 16 bit calibration data
    BMP085_REGISTER_CAL_AC4            = 0xB0,  // 16 bit calibration data
    BMP085_REGISTER_CAL_AC5            = 0xB2,  // 16 bit calibration data
    BMP085_REGISTER_CAL_AC6            = 0xB4,  // 16 bit calibration data
    BMP085_REGISTER_CAL_B1             = 0xB6,  // 16 bit calibration data
    BMP085_REGISTER_CAL_B2             = 0xB8,  // 16 bit calibration data
    BMP085_REGISTER_CAL_MB             = 0xBA,  // 16 bit calibration data
    BMP085_REGISTER_CAL_MC             = 0xBC,  // 16 bit calibration data
    BMP085_REGISTER_CAL_MD             = 0xBE,  // 16 bit calibration data
    BMP085_REGISTER_CHIPID             = 0xD0,
    BMP085_REGISTER_VERSION            = 0xD1,
    BMP085_REGISTER_SOFTRESET          = 0xE0,
    BMP085_REGISTER_CONTROL            = 0xF4,
    BMP085_REGISTER_TEMPDATA           = 0xF6,
    BMP085_REGISTER_PRESSUREDATA       = 0xF6,
    BMP085_REGISTER_READTEMPCMD        = 0x2E,
    BMP085_REGISTER_READPRESSURECMD    = 0x34
};

typedef enum
{
    BMP085_MODE_ULTRALOWPOWER          = 0,
    BMP085_MODE_STANDARD               = 1,
    BMP085_MODE_HIGHRES                = 2,
    BMP085_MODE_ULTRAHIGHRES           = 3
} bmp085_mode_t;

typedef struct
{
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
} bmp085_calib_data;

class brxlib_BMP085 : public brxlib_sensors
{
  public:
    brxlib_BMP085(int32_t sensorID = -1);
  
    bool begin(bmp085_mode_t mode = BMP085_MODE_ULTRAHIGHRES);
    void getTemperature(int32_t *temp);
    void getPressure(int32_t *pressure);
    int32_t pressureToAltitude(int32_t seaLvel, int32_t atmospheric);
    int32_t seaLevelForAltitude(int32_t altitude, int32_t atmospheric);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);

  private:
#if !BRXLIB_COMPUTEB5_MACRO
    int32_t computeB5(int32_t ut);
#endif
    int32_t _sensorID;
};

#endif // BRXLIB_BMP085_H
