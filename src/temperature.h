#pragma once

#include <stdint.h>

#define TEMP_CONVERSION_T 750 /* Minimum temperature sample time */

typedef enum TEMP_INTF_ {
  TEMP_INTF_NONE,
  TEMP_INTF_ONEWIRE,
  TEMP_INTF_I2C,
  TEMP_INTF_OPA1,
  TEMP_INTF_OPA2
} TEMP_INTF_t;

typedef enum TempStatus_ {
  TEMP_OK,
  TEMP_OVERRUN,
  TEMP_NO_SENSORS,
  TEMP_FAILED,
  TEMP_NO_SAMPLE,
  TEMP_OUT_OF_RANGE,
  TEMP_UNUSED,
  TEMP_BAD_CRC,
  TEMP_BAD_SENSOR
} TempStatus_t;

typedef struct TempDev_ {
  TEMP_INTF_t intf;
  uint64_t    id;
} TempDev_t;

typedef struct TempRead_ {
  TempStatus_t status;
  int16_t      temp;
} TempRead_t;

/*! @brief Populate the table of OneWire devices
 *  @return pointer to the address table
 */
uint64_t *tempAddress1WGet(void);

/*! @brief Return the temperature as a float
 *  @param [in] intf : interface type
 *  @param [in] tFixed : fixed point temperature
 *  @return the temperature as a float
 */
float tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed);

/*! @brief Clear any saved temperature settings */
void tempInitClear(void);

/*! @brief Find and initialise sensors
 *  @param [in] intf : interface type
 *  @param [in] pParams : parameters for given interface type
 *  @return number of sensors found
 */
uint32_t tempInitSensors(const TEMP_INTF_t intf, const void *pParams);

/*! @brief Generate a mapping of physical to logical sensors
 *  @param [in] pAddr : pointer to array of logical sensors
 */
void tempMapDevices(const TEMP_INTF_t intf, const void *pAddr);

/*! @brief Get the map index for this sensor
 *  @param [in] dev : physical device index
 *  @return logical device index
 */
unsigned int tempMapToLogical(const TEMP_INTF_t intf, const size_t dev);

/*! @brief Read an existing temperature sample
 *  @param [in] intf : interface type
 *  @param [in] dev : device index
 *  @return TempRead struct
 */
TempRead_t tempReadSample(const TEMP_INTF_t intf, const size_t dev);

/*! @brief Read the temperature sensor's serial number
 *  @param [in] intf : interface type
 *  @param [in] dev : device index
 *  @return interface and device ID
 */
TempDev_t tempReadSerial(const TEMP_INTF_t intf, const uint8_t dev);

/*! @brief Start a temperature sample
 *  @param [in] intf : interface type
 *  @param [in] dev : device index
 *  @return status of the sample start
 */
TempStatus_t tempStartSample(const TEMP_INTF_t intf, const size_t dev);
