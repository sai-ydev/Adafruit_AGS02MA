/*!
 *  @file Adafruit_AGS02MA.cpp
 *
 *  @mainpage Adafruit AGS02MA Gas Sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit AGS02MA Gas Sensor library
 *
 * 	This is a library for the Adafruit AGS02MA breakout:
 * 	https://www.adafruit.com/product/5593
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Limor Fried (Adafruit Industries)
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"

#include "Adafruit_AGS02MA.h"

static uint8_t crc8(const uint8_t *data, int len);

/*!
 *    @brief  Instantiates a new AGS02MA class
 */
Adafruit_AGS02MA::Adafruit_AGS02MA(void) {}

Adafruit_AGS02MA::~Adafruit_AGS02MA(void) {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  i2caddr The 7 bit address to look for the sensor (default 0x1A)
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_AGS02MA::begin(TwoWire *wire, uint8_t i2caddr,
                             int32_t sensor_id) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2caddr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  i2c_dev->setSpeed(20000);

  _sensorID = sensor_id;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the firmware version/datecode value
    @returns 0 on I2C failure, otherwise, the 32 bit version register
*/
/**************************************************************************/
uint32_t Adafruit_AGS02MA::getFirmwareVersion(void) {
  uint32_t vers = 0;
  if (!_readReg(_AGS02MA_VERSION_REG, 30, &vers)) {
    return 0;
  }
  return vers;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent gas resistance reading
    @returns 0 on I2C failure, otherwise, the resistance in Ohms
*/
/**************************************************************************/
uint32_t Adafruit_AGS02MA::getGasResistance(void) {
  uint32_t res = 0;
  if (!_readReg(_AGS02MA_GASRES_REG, 1500, &res)) {
    return 0;
  }
  return res * 100;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent TVOC reading
    @returns 0 on I2C failure, otherwise, the 24-bit ppb reading
*/
/**************************************************************************/
uint32_t Adafruit_AGS02MA::getTVOC(void) {
  uint32_t tvoc = 0;
  if (!_readReg(_AGS02MA_TVOCSTAT_REG, 1500, &tvoc)) {
    return 0;
  }
  return tvoc & 0xFFFFFF;
}

/**************************************************************************/
/*!
    @brief  Sets the non-volatile I2C address
    @param  new_addr The new 7 bit address to use
    @returns true on I2C success
*/
/**************************************************************************/
bool Adafruit_AGS02MA::setAddress(uint8_t new_addr) {
  uint8_t buf[6];

  new_addr &= 0x7F;

  buf[0] = _AGS02MA_SETADDR_REG;
  buf[3] = buf[1] = new_addr;
  buf[4] = buf[2] = ~new_addr;
  buf[5] = crc8(buf + 1, 4);

  return i2c_dev->write(buf, 6);
}

bool Adafruit_AGS02MA::_readReg(uint8_t addr, uint16_t delayms,
                                uint32_t *value) {
  uint8_t buf[5];

  buf[0] = addr;
  if (!i2c_dev->write(buf, 1)) {
    return false;
  }
  delay(delayms);
  if (!i2c_dev->read(buf, 5)) {
    return false;
  }

  if (crc8(buf, 4) != buf[4]) {
    return false;
  }

  uint32_t temp = buf[0];
  temp <<= 8;
  temp |= buf[1];
  temp <<= 8;
  temp |= buf[2];
  temp <<= 8;
  temp |= buf[3];
  *value = temp;
  return true;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
  /* CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent TVOC sensor event
    @param event The `sensors_event_t` to fill with event data
    @returns false if I2C communicatoin occured, true if data was available
*/
/**************************************************************************/
bool Adafruit_AGS02MA::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  uint32_t tvoc = getTVOC();
  if (tvoc == 0) {
    return false;
  }

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_TVOC;
  event->timestamp = millis();
  event->tvoc = tvoc;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t description for AGS02MA
    @param  sensor The unified sensor_t object we will populate
*/
/**************************************************************************/
void Adafruit_AGS02MA::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "AGS02MA", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_TVOC;
  sensor->min_delay = 0;
  sensor->min_value = 0;     // 0 ppb
  sensor->max_value = 99999; // 99,999 ppb
  sensor->resolution = 1;    // native units are ppb
}
