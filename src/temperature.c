#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"
#include "driver_TIME.h"
#include "emon32_assert.h"
#include "periph_DS18B20.h"
#include "temperature.h"

static bool    tempSampled               = false;
static int32_t numSensors                = 0;
static int32_t millisLastSample[NUM_OPA] = {0};

uint64_t *tempAddress1WGet(void) { return ds18b20AddressGet(); }

float tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed) {
  float ret = -1000.0;
  if (TEMP_INTF_ONEWIRE == intf) {
    ret = ds18b20SampleToCelsius(tFixed);
  }
  return ret;
}

void tempInitClear(void) { numSensors = 0; }

uint32_t tempInitSensors(const TEMP_INTF_t intf, const void *pParams) {
  EMON32_ASSERT(pParams);

  if (TEMP_INTF_ONEWIRE == intf) {
    int32_t numFound = ds18b20InitSensors((DS18B20_conf_t *)pParams);
    numSensors += numFound;
    return numFound;
  }

  return 0;
}

void tempMapDevices(const TEMP_INTF_t intf, const void *pAddr) {
  if (TEMP_INTF_ONEWIRE == intf) {
    ds18b20MapSensors((uint64_t *)pAddr);
  }
}

unsigned int tempMapToLogical(const TEMP_INTF_t intf, const uint8_t dev) {
  if (TEMP_INTF_ONEWIRE == intf) {
    return ds18b20MapToLogical(dev);
  }

  return 0;
}

TempRead_t tempReadSample(const TEMP_INTF_t intf, const uint8_t dev) {
  TempRead_t res = {TEMP_FAILED, INT16_MIN};

  if (!tempSampled) {
    res.status = TEMP_NO_SAMPLE;
    return res;
  }
  if (0 == numSensors) {
    res.status = TEMP_NO_SENSORS;
    return res;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    res = ds18b20ReadSample(dev);
  }

  return res;
}

TempDev_t tempReadSerial(const TEMP_INTF_t intf, const uint8_t dev) {
  if (TEMP_INTF_ONEWIRE == intf) {
    return ds18b20ReadSerial(dev);
  }
  return (TempDev_t){.id = 0, .intf = TEMP_INTF_NONE};
}

TempStatus_t tempStartSample(const TEMP_INTF_t intf, const uint32_t dev) {

  if (0 == numSensors) {
    return TEMP_NO_SENSORS;
  }

  if (timerMillisDelta(millisLastSample[dev]) < TEMP_CONVERSION_T) {
    return TEMP_OVERRUN;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    tempSampled           = true;
    millisLastSample[dev] = timerMillis();

    if (ds18b20StartSample(dev)) {
      return TEMP_OK;
    }
  }

  /* Default to failure */
  return TEMP_FAILED;
}
