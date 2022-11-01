/*!
 *  @file Adafruit_AGS02MA.h
 *
 * 	I2C Driver for the Adafruit AGS02MA Gas & TVOC Sensor library
 *
 * 	This is a library for the Adafruit AGS02MA breakout:
 * 	https://www.adafruit.com/products/5593
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_AGS02MA_H
#define _ADAFRUIT_AGS02MA_H

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define AGS02MA_I2CADDR_DEFAULT 0x1A  ///< AHT default i2c address
#define _AGS02MA_TVOCSTAT_REG 0x00    ///< Status and TVOC reading
#define _AGS02MA_VERSION_REG 0x11     ///< Rirmware version
#define _AGS02MA_GASRES_REG 0x20      ///< Raw gas resistance
#define _AGS02MA_SETADDR_REG 0x21     ///< Change I2C addr
#define _AGS02MA_CRC8_INIT 0xFF       ///< CRC8 init val
#define _AGS02MA_CRC8_POLYNOMIAL 0x31 ///< CRC8 polynomial

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the AGS02MA gas sensor
 */
class Adafruit_AGS02MA : public Adafruit_Sensor {
public:
  Adafruit_AGS02MA();
  ~Adafruit_AGS02MA();

  bool begin(TwoWire *wire = &Wire, uint8_t i2caddr = AGS02MA_I2CADDR_DEFAULT,
             int32_t sensor_id = 0);
  bool setAddress(uint8_t new_addr);

  uint32_t getFirmwareVersion(void);
  uint32_t getGasResistance(void);
  uint32_t getTVOC(void);

  bool getEvent(sensors_event_t *tvoc);
  void getSensor(sensor_t *sensor);

private:
  bool _readReg(uint8_t addr, uint16_t delayms, uint32_t *value);

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  uint32_t _sensorID = 0;
};

#endif
