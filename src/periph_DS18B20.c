#include "emon32_samd.h"

#include "board_def.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emon32_assert.h"
#include "periph_DS18B20.h"
#include "qfplib-m0-full.h"

/* Driver for DS18B20 OneWire temperature sensor
 * https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf
 *
 * OneWire search algorithm adapted from:
 * https://www.analog.com/en/app-notes/1wire-search-algorithm.html
 */

typedef struct __attribute__((__packed__)) Scratch_ {
  int16_t temp;
  uint8_t th;
  uint8_t tl;
  uint8_t cfg;
  uint8_t res_FF;
  uint8_t res_X;
  uint8_t res_10;
  uint8_t crc;
} Scratch_t;

_Static_assert(9 == sizeof(Scratch_t), "Scratch_t is not 9 bytes.");

/* OneWire pins and configuration */
static DS18B20_conf_t cfg[NUM_OPA];

/* Device address table */
static uint64_t devTableAddr[TEMP_MAX_ONEWIRE] = {0};
static uint8_t  devTableOpa[TEMP_MAX_ONEWIRE]  = {0};
static uint8_t  devRemap[TEMP_MAX_ONEWIRE]     = {0};

/* OneWire functions & state variables */
static uint8_t  calcCRC8(const uint8_t crc, const uint8_t value);
static bool     oneWireFirst(const int32_t opaIdx);
static bool     oneWireNext(const int32_t opaIdx);
static uint32_t oneWireReadBit(const int32_t opaIdx);
static void oneWireReadBytes(void *pDst, const uint8_t n, const int32_t opaIdx);
static bool oneWireReset(const int32_t opaIdx);
static bool oneWireSearch(const int32_t opaIdx);
static void oneWireWriteBit(uint32_t bit, const int32_t opaIdx);
static void oneWireWriteBytes(const void *pSrc, const uint8_t n,
                              const int32_t opaIdx);

uint64_t ROM_NO                = 0;
int32_t  lastDiscrepancy       = 0;
int32_t  lastFamilyDiscrepancy = 0;
int32_t  lastDeviceFlag        = 0;

static uint8_t calcCRC8(const uint8_t crc, const uint8_t value) {
  const uint8_t dscrc_table[] = {
      0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
      65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
      130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
      222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
      29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
      102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
      165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
      249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
      58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
      15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
      204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
      144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
      83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
      40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
      235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
      183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
      116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
      53};

  return dscrc_table[crc ^ value];
}

/*! @brief: Find the first device on the 1-Wire bus
 *  @return true if device found, ROM number in ROM_NO buffer; false otherwise
 */
static bool oneWireFirst(const int32_t opaIdx) {
  /* Reset the search state */
  lastDiscrepancy       = 0;
  lastDeviceFlag        = 0;
  lastFamilyDiscrepancy = 0;

  return oneWireSearch(opaIdx);
}

/*! @brief: Find the next device on the 1-Wire bus
 *  @return true if device found, ROM number in ROM_NO buffer; false otherwise
 */
static bool oneWireNext(const int32_t opaIdx) { return oneWireSearch(opaIdx); }

static uint32_t oneWireReadBit(const int32_t opaIdx) {
  uint32_t result = 0;

  __disable_irq();
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_OUT);
  timerDelay_us(cfg[opaIdx].t_wait_us);
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_IN);
  /* Max 15 us for read slot; leave 3 us slack */
  timerDelay_us(12u - cfg[opaIdx].t_wait_us);
  result = portPinValue(cfg[opaIdx].grp, cfg[opaIdx].pin);
  __enable_irq();

  /* Wait for the end of the read slot, t_RDV */
  timerDelay_us(60u);

  return result;
}

static void oneWireReadBytes(void *pDst, const uint8_t n,
                             const int32_t opaIdx) {
  EMON32_ASSERT(pDst);

  uint8_t *pData = (uint8_t *)pDst;

  for (uint8_t i = 0; i < n; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      /* Data received LSB first */
      *pData |= (oneWireReadBit(opaIdx) << j);
    }
    pData++;
  }
}

static bool oneWireReset(const int32_t opaIdx) {
  /* t_RSTL (min) = 480 us
   * t_RSTH (min) = 480 us
   * t_PDHIGH (max) = 60 us
   * t_PDLOW (max) = 240 us
   */

  bool     presence  = 0;
  uint32_t timeStart = 0;

  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_OUT);

  timerDelay_us(500u);
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_IN);
  /* Wait 75 us to ensure t_PDHIGH has elapsed, then wait the full t_RSTH
   * time +25 us slack to complete the reset sequence.
   */
  timerDelay_us(75u);

  timeStart = timerMicros();
  while (timerMicrosDelta(timeStart) < 425u) {
    /* Latch presence if found */
    if (0 == presence) {
      presence = !portPinValue(cfg[opaIdx].grp, cfg[opaIdx].pin);
    }
  }

  return presence;
}

static bool oneWireSearch(const int32_t opaIdx) {
  /* Initialise for search */
  const uint8_t CMD_SEARCH_ROM  = 0xF0u;
  int32_t       searchDirection = 0;
  int32_t       idBitNumber     = 1;
  int32_t       lastZero        = 0;
  uint8_t       romByteMask     = 1;
  bool          searchResult    = false;
  int32_t       idBit           = 0;
  int32_t       cmpidBit        = 0;
  uint8_t       crc8            = 0;
  uint8_t      *romBuffer       = (uint8_t *)&ROM_NO;

  /* If the last call was not the last one... */
  if (!lastDeviceFlag) {
    /* ... reset the OneWire bus... */
    if (!oneWireReset(opaIdx)) {
      /* Reset the search */
      lastDiscrepancy       = 0;
      lastDeviceFlag        = 0;
      lastFamilyDiscrepancy = 0;
      return 0;
    }

    /* ...issue the search command...*/
    oneWireWriteBytes(&CMD_SEARCH_ROM, 1, opaIdx);

    /* ...and commence the search! */
    for (uint32_t i = 0; i < 64; i++) {
      idBit    = oneWireReadBit(opaIdx);
      cmpidBit = oneWireReadBit(opaIdx);

      /* Check for no devices on OneWire */
      if (idBit && cmpidBit) {
        break;
      }

      if (idBit != cmpidBit) {
        searchDirection = idBit;
      } else {
        /* If this discrepancy is before the last discrepancy on a previous
         * next then pick the same as last time
         */
        searchDirection = (idBitNumber < lastDiscrepancy)
                              ? ((*romBuffer & romByteMask) > 0)
                              : (idBitNumber == lastDiscrepancy);

        /* If 0 was picked, record its position */
        if (0 == searchDirection) {
          lastZero = idBitNumber;
          /* and check for last discrepancy in Family */
          if (lastZero < 9) {
            lastFamilyDiscrepancy = lastZero;
          }
        }
      }

      /* Set or clear the bit in the ROM byte number with mask */
      if (0 == searchDirection) {
        *romBuffer &= ~romByteMask;
      } else {
        *romBuffer |= romByteMask;
      }

      /* Serial number search direction bit */
      oneWireWriteBit(searchDirection, opaIdx);
      idBitNumber++;
      romByteMask <<= 1;

      /* When the mask is 0, go to new serial number byte and reset */
      if (0 == romByteMask) {
        crc8 = calcCRC8(crc8, *romBuffer);
        romBuffer++;
        romByteMask = 1;
      }
    }
  }

  /* If the search was successful... */
  if (!((65 > idBitNumber) || (0 != crc8))) {
    lastDiscrepancy = lastZero;
    searchResult    = true;

    /* Check for last device */
    if (0 == lastDiscrepancy) {
      lastDeviceFlag = 1;
    }
  }

  return searchResult;
}

static void oneWireWriteBit(uint32_t bit, const int32_t opaIdx) {
  /* See timing diagrams in Figure 16. Interrupts are disabled in sections
   * where too long would break the OneWire protocol. At the end of a bit
   * transmission, a pending interrupt may be serviced, but this will only
   * extend the interbit timing, with no affect on the protocol.
   */

  __disable_irq();
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_OUT);
  timerDelay_us(cfg[opaIdx].t_wait_us);
  if (bit) {
    portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_IN);
  }
  timerDelay_us(75u - cfg[opaIdx].t_wait_us);
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DIR_IN);
  __enable_irq();
  timerDelay_us(5u);
}

static void oneWireWriteBytes(const void *pSrc, const uint8_t n,
                              const int32_t opaIdx) {
  EMON32_ASSERT(pSrc);

  uint8_t *pData = (uint8_t *)pSrc;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t byte = *pData++;
    for (uint8_t j = 0; j < 8; j++) {
      oneWireWriteBit((byte & 0x1), opaIdx);
      byte >>= 1;
    }
  }
}

uint64_t *ds18b20AddressGet(void) { return devTableAddr; }

uint32_t ds18b20InitSensors(const DS18B20_conf_t *pCfg) {
  EMON32_ASSERT(pCfg);

  const uint8_t DS18B_FAMILY_CODE = 0x28;

  uint8_t  opaIdx       = pCfg->opaIdx;
  uint32_t deviceCount  = 0;
  bool     searchResult = false;

  cfg[opaIdx].grp       = pCfg->grp;
  cfg[opaIdx].pin       = pCfg->pin;
  cfg[opaIdx].pinPU     = pCfg->pinPU;
  /* If not overridden, default to 5 us pull low */
  cfg[opaIdx].t_wait_us = pCfg->t_wait_us ? pCfg->t_wait_us : 5u;

  /* Enable the hard pull up, and search for devices */
  portPinDrv(cfg[opaIdx].grp, cfg[opaIdx].pinPU, PIN_DRV_SET);
  portPinDir(cfg[opaIdx].grp, cfg[opaIdx].pinPU, PIN_DIR_OUT);
  portPinDrv(cfg[opaIdx].grp, cfg[opaIdx].pin, PIN_DRV_CLR);
  searchResult = oneWireFirst(opaIdx);

  while (searchResult && (deviceCount < TEMP_MAX_ONEWIRE)) {

    /* Only count DS18B20 devices. */
    if (DS18B_FAMILY_CODE == (uint8_t)ROM_NO) {
      devTableAddr[deviceCount] = ROM_NO;
      devTableOpa[deviceCount]  = opaIdx;
      deviceCount++;
    }

    searchResult = oneWireNext(opaIdx);
  }

  return deviceCount;
}

void ds18b20MapSensors(const uint64_t *pAddr) {
  /* Default to physical == logical index. If a matching saved address is found,
   * then swap the remapping indices */
  for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    devRemap[i] = i;
    for (int j = 0; j < TEMP_MAX_ONEWIRE; j++) {
      if (devTableAddr[i] == pAddr[j]) {
        devRemap[j] = devRemap[i];
        devRemap[i] = j;
        break;
      }
    }
  }
}

uint8_t ds18b20MapToLogical(const unsigned int dev) { return devRemap[dev]; }

bool ds18b20StartSample(const int32_t opaIdx) {
  const uint8_t CMD_SKIP_ROM  = 0xCC;
  const uint8_t CMD_CONVERT_T = 0x44;
  const uint8_t cmds[2]       = {CMD_SKIP_ROM, CMD_CONVERT_T};

  /* Check for presence pulse before continuing */
  if (!oneWireReset(opaIdx)) {
    return false;
  }

  oneWireWriteBytes(cmds, 2u, opaIdx);
  return true;
}

TempRead_t ds18b20ReadSample(const uint32_t dev) {
  const uint8_t CMD_MATCH_ROM    = 0x55;
  const uint8_t CMD_READ_SCRATCH = 0xBE;
  const int16_t DS_T85DEG        = 1360;
  const int16_t DS_TNEG55DEG     = -880;
  const int16_t DS_T125DEG       = 2000;

  const uint64_t *addrDev  = &devTableAddr[dev];
  Scratch_t       scratch  = {0};
  const uint8_t  *pScratch = (uint8_t *)&scratch;
  uint8_t         crcDS    = 0;
  TempRead_t      tempRes  = {0};

  /* Check for presence pulse before continuing */
  if (!oneWireReset(devTableOpa[dev])) {
    tempRes.status = TEMP_NO_SENSORS;
    return tempRes;
  }

  oneWireWriteBytes(&CMD_MATCH_ROM, 1, devTableOpa[dev]);
  oneWireWriteBytes(addrDev, 8, devTableOpa[dev]);
  oneWireWriteBytes(&CMD_READ_SCRATCH, 1, devTableOpa[dev]);
  oneWireReadBytes(&scratch, 9, devTableOpa[dev]);

  /* Check CRC for received data */
  for (int32_t i = 0; i < 8; i++) {
    calcCRC8(crcDS, pScratch[i]);
  }
  if (crcDS != scratch.crc) {
    tempRes.status = TEMP_BAD_CRC;
  }

  /* scratch[4] is the DS18B20's configuration register, must not be 0. See
   * Figure 10. Configuration Register. */
  if (0 == scratch.cfg) {
    tempRes.status = TEMP_BAD_SENSOR;
    return tempRes;
  }

  /* Check for spurious 85°C reading. This could be caused by e.g. a power
   * glitch after the sample was requested. */
  if ((0x0C == scratch.res_X) && (DS_T85DEG == tempRes.temp)) {
    tempRes.status = TEMP_BAD_SENSOR;
    return tempRes;
  }

  /* Flag values < -55°C and > +125°C as out of range  */
  if ((DS_TNEG55DEG > scratch.temp) || (DS_T125DEG < scratch.temp)) {
    tempRes.status = TEMP_OUT_OF_RANGE;
    return tempRes;
  }

  tempRes.temp   = scratch.temp;
  tempRes.status = TEMP_OK;
  return tempRes;
}

TempDev_t ds18b20ReadSerial(const uint32_t dev) {
  TempDev_t device;

  device.id   = devTableAddr[dev];
  device.intf = (0 == devTableOpa[dev]) ? TEMP_INTF_OPA1 : TEMP_INTF_OPA2;

  return device;
}

float ds18b20SampleToCelsius(const int16_t fix) {
  return qfp_fix2float(fix, 4);
}
