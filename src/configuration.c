#include <inttypes.h>
#include <string.h>

#include "emon32_assert.h"

#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "eeprom.h"
#include "emon32.h"
#include "emon32_build_info.h"
#include "emon_CM.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

/*************************************
 * Types
 *************************************/

/* Async confirmation state machine */
typedef enum {
  CONFIRM_IDLE,
  CONFIRM_BOOTLOADER,
  CONFIRM_ZERO_ACCUM,
  CONFIRM_ZERO_ACCUM_INDIVIDUAL,
  CONFIRM_NVM_OVERWRITE,
  CONFIRM_RESET
  /* CONFIRM_RESTORE_DEFAULTS - Removed pending OEM decision on restore defaults
     confirmation */
} ConfirmState_t;

typedef enum {
  RCAUSE_SYST  = 0x40,
  RCAUSE_WDT   = 0x20,
  RCAUSE_EXT   = 0x10,
  RCAUSE_BOD33 = 0x04,
  RCAUSE_BOD12 = 0x02,
  RCAUSE_POR   = 0x01
} RCAUSE_t;

/*************************************
 * Prototypes
 *************************************/

static void     configDefault(void);
static void     configEchoQueueChar(const uint8_t c);
static void     configEchoQueueStr(const char *s);
static void     configInitialiseNVM(void);
static uint16_t configTimeToCycles(const float time, const uint32_t mainsFreq);
static bool     configureAnalog(void);
static bool     configureAssumed(void);
static void     configureBackup(void);
static bool     configureDatalog(void);
static bool     configureGroupID(void);
static bool     configureJSON(void);
static bool     configureLineFrequency(void);
static bool     configure1WAddr(void);
static bool     configure1WAddrClear(void);
static void     configure1WFind(void);
static bool     configure1WFreeze(void);
static void     configure1WList(void);
static void     configure1WListSaved(void);
static bool     configure1WSave(void);
static bool     configureOPA(void);
static bool     configureNodeID(void);
static void     configureReconfigureAll(void);
static void     configureRestore(void);
static bool     configureRFEnable(void);
static bool     configureRF433(void);
static bool     configureRFPower(void);
static bool     configureSerialLog(void);
static void     enterBootloader(void);
static uint32_t getBoardRevision(void);
static char    *getLastReset(void);
static void     handleConfirmation(char c);
static void     inBufferClear(const size_t n);
static size_t   inBufferTok(void);
static void     printSettingCT(const size_t ch);
static void     printSettingDatalog(void);
static void     printSettingJSON(void);
static void     printSettingOPA(const size_t ch);
static void     printSettingRF(void);
static void     printSettingRFFreq(void);
static void     printSettingV(const size_t ch);
static void     printSettings(void);
static void     printSettingsHR(void);
static void     printSettingsKV(void);
static void     printUptime(void);
static void     putFloat(float val, const size_t flt_len);
static void     resetRequest(void);
static void     zeroAccumulators(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W  64u
#define ERROR_PREFIX "> Error: "

static void serialPutsError(const char *msg) {
  serialPuts(ERROR_PREFIX);
  serialPuts(msg);
  serialPuts("\r\n");
}

static void printfError(const char *fmt, ...) {
  va_list args;
  serialPuts(ERROR_PREFIX);
  va_start(args, fmt);
  vprintf_(fmt, args);
  va_end(args);
  serialPuts("\r\n");
}

static Emon32Config_t config;
static char           inBuffer[IN_BUFFER_W];

/* Async confirmation state */
static volatile ConfirmState_t confirmState        = CONFIRM_IDLE;
static volatile uint32_t       confirmStartTime_ms = 0;
static uint8_t                 clearAccumIdx =
    UINT8_MAX; /* UINT8_MAX=all, 0-11=E1-E12, 12-13=P1-P2 */
static size_t inBufferIdx   = 0;
static bool   cmdPending    = false;
static bool   unsavedChange = false;

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  (void)memset(&config, 0, sizeof(config));

  config.key = CONFIG_NVM_KEY;

  /* Single phase, 50 Hz, 240 VAC, 10 s report period */
  config.baseCfg.nodeID     = NODE_ID_DEF;
  config.baseCfg.mainsFreq  = MAINS_FREQ_DEF;
  config.baseCfg.reportTime = REPORT_TIME_DEF;
  config.baseCfg.reportCycles =
      configTimeToCycles(REPORT_TIME_DEF, MAINS_FREQ_DEF);
  config.baseCfg.assumedVrms  = ASSUMED_VRMS_DEF;
  config.baseCfg.epDeltaStore = DELTA_EP_STORE_DEF;
  config.baseCfg.dataGrp      = GROUP_ID_DEF;
  config.baseCfg.logToSerial  = true;
  config.baseCfg.useJson      = false;
  config.baseCfg.debugSerial  = false;
  config.dataTxCfg.useRFM     = true;
  config.dataTxCfg.rfmPwr     = RFM_PALEVEL_DEF;
  config.dataTxCfg.rfmFreq    = RFM_FREQ_DEF;

  for (size_t idxV = 0u; idxV < NUM_V; idxV++) {
    config.voltageCfg[idxV].voltageCal = 100.0f;
    config.voltageCfg[idxV].vActive    = (0 == idxV);
    config.voltageCfg[idxV].phase      = 0.0f;
  }

  /* Initialize ALL slots including reserved. */
  for (size_t idxCT = 0u; idxCT < (NUM_CT + CT_RES); idxCT++) {
    config.ctCfg[idxCT].ctCal    = 100.0f;
    config.ctCfg[idxCT].phase    = CT_LEAD_DEF;
    config.ctCfg[idxCT].vChan1   = 0;
    config.ctCfg[idxCT].vChan2   = 0;
    config.ctCfg[idxCT].ctActive = (idxCT < NUM_CT_ACTIVE_DEF);
  }

  /* OneWire/Pulse configuration:
   * OPA1
   *   - OneWire input
   *   - Enabled
   */
  config.opaCfg[0].func      = 'o';
  config.opaCfg[0].opaActive = true;
  config.opaCfg[0].period    = 0;
  config.opaCfg[0].puEn      = true;

  /* OPA2
   *   - OneWire input
   *   - Enabled
   */
  config.opaCfg[1].func      = 'o';
  config.opaCfg[1].opaActive = true;
  config.opaCfg[1].period    = 0;
  config.opaCfg[1].puEn      = true;

  /* OPA3
   *   - Pulse input
   *   - Disabled
   */
  config.opaCfg[2].func      = 'r';
  config.opaCfg[2].opaActive = false;
  config.opaCfg[2].period    = 100;
  config.opaCfg[2].puEn      = false;

  config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2u));
}

/*! @brief Write the configuration values to index 0, and zero the
 *         accumulator space to.
 */
static void configInitialiseNVM(void) {

  serialPuts("  - Initialising NVM... ");

  configDefault();
  eepromInitBlock(0, 0, EEPROM_WL_OFFSET);
  eepromInitConfig(&config, sizeof(config));
  eepromWLClear();
  serialPuts("Done!\r\n");
}

static bool configureAnalog(void) {
  /* String format: k<x> <a> <y.y> <z.z> v1 v2
   * Find space delimiters, then convert to null and a->i/f
   */
  ConvFloat_t convF     = {false, 0.0f};
  ConvUint_t  convU     = {false, {0}};
  uint32_t    ch        = 0;
  bool        active    = false;
  float       calAmpl   = 0.0f;
  float       calPhase  = 0.0f;
  uint8_t     vCh1      = 0;
  uint8_t     vCh2      = 0;
  uint32_t    posActive = 0;
  uint32_t    posCalib  = 0;
  uint32_t    posPhase  = 0;
  uint32_t    posV1     = 0;
  uint32_t    posV2     = 0;
  ECMCfg_t   *ecmCfg    = 0;

  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
      if (0 == posActive) {
        posActive = i + 1u;
      } else if (0 == posCalib) {
        posCalib = i + 1u;
      } else if (0 == posPhase) {
        posPhase = i + 1u;
      } else if (0 == posV1) {
        posV1 = i + 1u;
      } else if (0 == posV2) {
        posV2 = i + 1u;
        break;
      }
    }
  }

  /* All or no parameters must be specified */
  if ((0 == posCalib) ^ (0 == posActive) ^ (0 == posPhase)) {
    serialPutsError("Missing required parameters.");
    return false;
  }

  /* Voltage channels are [1..3], CTs are [4..] but 0 indexed internally. All
   * fields must be present for a given channel type.
   */
  convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid channel number.");
    return false;
  }

  if (--convU.val.u32 >= VCT_TOTAL) {
    printfError("Channel out of range (valid: 1-%d).", VCT_TOTAL);
    return false;
  }

  ch = convU.val.u32;

  convU = utilAtoui(inBuffer + posActive, ITOA_BASE10);
  if (!convU.valid || (convU.val.u32 > 1u)) {
    serialPutsError("Invalid active value (valid: 0 or 1).");
    return false;
  }
  active = (bool)convU.val.u8;

  ecmCfg = ecmConfigGet();

  /* Exit early if just activating or deactivating */
  if (0 == posCalib) {
    if (ch < NUM_V) {
      ecmCfg->vCfg[ch].vActive      = active;
      config.voltageCfg[ch].vActive = active;
      printSettingV(ch);
    } else {
      ecmCfg->ctCfg[ch - NUM_V].active  = active;
      config.ctCfg[ch - NUM_V].ctActive = active;
      printSettingCT(ch - NUM_V);
    }
    ecmConfigChannel(ch);
    return true;
  }

  /* CT requires at least V1 */
  if (ch >= NUM_V) {
    if (0 == posV1) {
      serialPutsError("CT requires voltage channel reference.");
      return false;
    }
  }

  convF = utilAtof(inBuffer + posCalib);
  if (!convF.valid) {
    serialPutsError("Invalid calibration value.");
    return false;
  }
  calAmpl = convF.val;

  convF = utilAtof(inBuffer + posPhase);
  if (!convF.valid) {
    serialPutsError("Invalid phase value.");
    return false;
  }
  calPhase = convF.val;

  if (NUM_V > ch) {

    if ((calAmpl <= 25.0f) || (calAmpl >= 150.0f)) {
      serialPutsError("vCal out of range (valid: 25-150).");
      return false;
    }

    bool reconfigureCT = calPhase != ecmCfg->vCfg[ch].phase;

    config.voltageCfg[ch].vActive    = active;
    config.voltageCfg[ch].voltageCal = calAmpl;
    config.voltageCfg[ch].phase      = calPhase;
    ecmCfg->vCfg[ch].vActive         = active;
    ecmCfg->vCfg[ch].voltageCalRaw   = calAmpl;
    ecmCfg->vCfg[ch].phase           = calPhase;

    printSettingV(ch);

    ecmConfigChannel(ch);

    /* If the voltage phase was changed reconfigure all CTs as well */
    if (reconfigureCT) {
      for (size_t i = 0; i < NUM_CT; i++) {
        ecmConfigChannel(i + NUM_V);
      }
    }

    return true;
  }

  convU = utilAtoui(inBuffer + posV1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid v1 value.");
    return false;
  }
  if (!convU.val.u32 || convU.val.u32 > NUM_V) {
    printfError("vChan out of range (valid: 1-%d).", NUM_V);
    return false;
  }
  vCh1 = convU.val.u8;

  /* V2 is optional */
  if (posV2) {
    convU = utilAtoui(inBuffer + posV2, ITOA_BASE10);
    if (!convU.valid) {
      serialPutsError("Invalid v2 value.");
      return false;
    }
    if (!convU.val.u32 || convU.val.u32 > NUM_V) {
      printfError("vChan out of range (valid: 1-%d).", NUM_V);
      return false;
    }
    vCh2 = convU.val.u8;
  } else {
    vCh2 = vCh1;
  }

  /* CT configuration - assume 10/200 A min/max CTs */
  if ((calAmpl < 10.0f) || (calAmpl) > 200.0f) {
    serialPutsError("iCal out of range (valid: 10-200).");
    return false;
  }

  ch -= NUM_V;
  config.ctCfg[ch].ctActive = active;
  ecmCfg->ctCfg[ch].active  = active;

  config.ctCfg[ch].ctCal     = calAmpl;
  ecmCfg->ctCfg[ch].ctCalRaw = calAmpl;

  config.ctCfg[ch].phase  = calPhase;
  ecmCfg->ctCfg[ch].phCal = calPhase;

  config.ctCfg[ch].vChan1  = vCh1 - 1;
  ecmCfg->ctCfg[ch].vChan1 = vCh1 - 1;

  config.ctCfg[ch].vChan2  = vCh2 - 1;
  ecmCfg->ctCfg[ch].vChan2 = vCh2 - 1;

  printSettingCT(ch);
  ecmConfigChannel(ch + NUM_V);
  return true;
}

static bool configureAssumed(void) {
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (convU.valid) {
    ECMCfg_t *pEcmCfg          = ecmConfigGet();
    pEcmCfg->assumedVrms       = qfp_uint2float(convU.val.u32);
    config.baseCfg.assumedVrms = convU.val.u16;
    return true;
  }
  return false;
}

static void configureBackup(void) {
  /* Send all configuration values as JSON over the serial link. */
  char strBuf[8] = {0};

  /* Open JSON block */
  serialPuts("{");
  /* {board_info} dict */
  serialPuts("\"board_info\":{");
  printf_("\"revision\":%ld,", getBoardRevision());
  printf_("\"serial\":\"0x%02lx%02lx%02lx%02lx\",", getUniqueID(0),
          getUniqueID(1), getUniqueID(2), getUniqueID(3));
  printf_("\"fw\":\"%d.%d.%d\"},", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);

  /* {board_config} dict */
  utilFtoa(strBuf, config.baseCfg.reportTime);
  printf_("\"board_config\":{\"rfmFreq\":%d,\"f_mains\":%d,\"t_report\":%s},",
          config.dataTxCfg.rfmFreq, config.baseCfg.mainsFreq, strBuf);

  printf_("\"assumedV\":%d,", config.baseCfg.assumedVrms);
  /* {v_config} list of dicts */
  serialPuts("\"v_config\":[");
  for (size_t i = 0; i < NUM_V; i++) {
    utilFtoa(strBuf, config.voltageCfg[i].voltageCal);
    printf_("{\"active\":%s,\"cal\":%s}",
            (config.voltageCfg[i].vActive ? "true" : "false"), strBuf);
    if (i != (NUM_V - 1)) {
      serialPuts(",");
    }
  }
  serialPuts("],");

  /* {ct_config} list of dicts */
  serialPuts("\"ct_config\":[");
  for (size_t i = 0; i < NUM_CT; i++) {
    utilFtoa(strBuf, config.ctCfg[i].ctCal);
    printf_("{\"active\":%s,\"cal\":%s,",
            (config.ctCfg[i].ctActive ? "true" : "false"), strBuf);
    utilFtoa(strBuf, config.ctCfg[i].phase);
    printf_("\"phase\":%s,\"vChan1\":%d,\"vChan2\":%d}", strBuf,
            (config.ctCfg[i].vChan1 + 1), (config.ctCfg[i].vChan2 + 1));
    if (i != (NUM_CT - 1)) {
      serialPuts(",");
    }
  }
  serialPuts("]");

  /* Close JSON block*/
  serialPuts("}\r\n");
}

static bool configureDatalog(void) {
  ConvFloat_t convF = utilAtof(inBuffer + 1);
  /* Set the datalog period (s) in range 0.5 <= t <= 600 */
  if (!convF.valid) {
    serialPutsError("Invalid datalog value.");
    return false;
  }

  if ((convF.val < 0.5f) || (convF.val > 600.0f)) {
    serialPutsError("Datalog out of range (valid: 0.5-600).");
    return false;
  }

  config.baseCfg.reportTime = convF.val;
  config.baseCfg.reportCycles =
      configTimeToCycles(convF.val, config.baseCfg.mainsFreq);
  ecmConfigReportCycles(config.baseCfg.reportCycles);

  printSettingDatalog();
  return true;
}

static bool configureGroupID(void) {
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);

  if (!convU.valid) {
    serialPutsError("Invalid group ID value.");
    return false;
  }

  if (convU.val.u32 > 255u) {
    serialPutsError("Group ID out of range (valid: 0-255).");
    return false;
  }

  config.baseCfg.dataGrp = convU.val.u8;
  printf_("rfGroup = %u\r\n", convU.val.u8);
  return true;
}

static bool configureJSON(void) {
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);

  if (!convU.valid) {
    serialPutsError("Invalid JSON value.");
    return false;
  }

  if (convU.val.u32 > 1) {
    serialPutsError("JSON value must be 0 or 1.");
    return false;
  }

  config.baseCfg.useJson = (bool)convU.val.u8;
  printSettingJSON();
  return true;
}

static bool configureLineFrequency(void) {
  /* f<n>
   * n must be 50 or 60
   */
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid frequency value.");
    return false;
  }

  if (!((50 == convU.val.u32) || (60 == convU.val.u32))) {
    serialPutsError("Frequency must be 50 or 60.");
    return false;
  }

  config.baseCfg.mainsFreq = convU.val.u8;

  /* Recalculate all CT interpolation values */
  ECMCfg_t *ecmCfg  = ecmConfigGet();
  ecmCfg->mainsFreq = convU.val.u8;
  for (size_t i = 0; i < NUM_CT; i++) {
    ecmConfigChannel(i + NUM_V);
  }
  ecmFlush();

  printf_("> Mains frequency set to: %d\r\n", config.baseCfg.mainsFreq);
  return true;
}

static bool configure1WAddr(void) {
  char c1 = *(inBuffer + 1);
  switch (c1) {
  case 'c':
    return configure1WAddrClear();
    break;
  case 'f':
    configure1WFind();
    return false;
  case 'l':
    configure1WList();
    return false;
  case 'n':
    configure1WListSaved();
    return false;
  case 's':
    configure1WFreeze();
    return true;
  default:
    return configure1WSave();
  }

  return false;
}

static bool configure1WAddrClear(void) {

  /* Clear _all_ saved addresses */
  if ('a' == inBuffer[2]) {
    memset(&config.oneWireAddr.addr, 0, sizeof(config.oneWireAddr.addr));
    serialPuts("> Cleared all saved 1-Wire addresses.\r\n");
    emon32EventSet(EVT_OPA_INIT);
    return true;
  }

  ConvUint_t convU = utilAtoui(inBuffer + 2, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid 1-Wire channel value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > (TEMP_MAX_ONEWIRE))) {
    printfError("1-Wire channel out of range (valid: 1-%d).", TEMP_MAX_ONEWIRE);
    return false;
  }

  size_t ch = convU.val.u32 - 1u;
  memset(&config.oneWireAddr.addr[ch], 0, sizeof(*config.oneWireAddr.addr));
  printf_("> Cleared saved 1-Wire address for channel %u\r\n", (ch + 1u));

  emon32EventSet(EVT_OPA_INIT);
  return true;
}

static void configure1WFind(void) {
  serialPuts("> Searching for 1-Wire devices...\r\n");
  emon32EventSet(EVT_OPA_INIT);
}

static bool configure1WFreeze(void) {
  uint64_t *pAddrDev = tempAddress1WGet();
  memcpy(&config.oneWireAddr, pAddrDev, sizeof(config.oneWireAddr));
  serialPuts("> 1-Wire addresses saved to config.\r\n");
  return true;
}

static void configure1WList(void) {
  uint64_t *pAddr = tempAddress1WGet();

  for (size_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {

    /* Only list DS18B20 devices */
    uint8_t id = (uint8_t)(pAddr[i] & 0xFF);
    if (0x28 == id) {
      printf_("%d [->%d] ", (i + 1),
              (tempMapToLogical(TEMP_INTF_ONEWIRE, i) + 1));
      for (size_t j = 0; j < 8; j++) {
        printf_("%02x%s", (uint8_t)((pAddr[i] >> (8 * j)) & 0xFF),
                ((j == 7) ? "\r\n" : " "));
      }
    }
  }
}

static void configure1WListSaved(void) {
  for (size_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    uint64_t as; /* Ensure 8byte alignment */
    memcpy(&as, &config.oneWireAddr.addr[i], sizeof(as));
    printf_("[%u] ", i);
    for (size_t j = 0; j < 8; j++) {
      printf_("%02x%s", (uint8_t)((as >> (8 * j)) & 0xFF),
              ((j == 7) ? "\r\n" : " "));
    }
  }
}

static bool configure1WSave(void) {
  size_t ch;

  if (8 != inBufferTok()) {
    serialPutsError("1-Wire save requires 8 address bytes.");
    return false;
  }

  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid 1-Wire channel value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > (TEMP_MAX_ONEWIRE))) {
    printfError("1-Wire channel out of range (valid: 1-%d).", TEMP_MAX_ONEWIRE);
    return false;
  }
  ch = convU.val.u32 - 1u;

  /* Find the position of the bytes in the string */
  size_t  tcnt = 0;
  uint8_t pos[8];
  for (uint8_t i = 0; (i < IN_BUFFER_W) && (tcnt != 8u); i++) {
    if ('\0' == inBuffer[i]) {
      pos[tcnt++] = i + 1u;
    }
  }

  uint64_t addr = 0;
  for (size_t i = 0; i < 8; i++) {
    convU = utilAtoui(&inBuffer[pos[i]], ITOA_BASE16);
    if (!convU.valid) {
      serialPutsError("Invalid 1-Wire address byte.");
      return false;
    }
    addr |= ((uint64_t)convU.val.u8 << (8u * i));
  }

  /* If this is an existing address, then zero the previous one */
  for (size_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    uint64_t as; /* Ensure 8byte alignment */
    memcpy(&as, &config.oneWireAddr.addr[i], sizeof(as));
    if (as == addr) {
      config.oneWireAddr.addr[i] = 0;
    }
  }

  config.oneWireAddr.addr[ch] = addr;

  printf_("> 1W_addr%d = ", (ch + 1));
  for (size_t i = 0; i < 8; i++) {
    printf_("%02x%s", (uint8_t)((addr >> (8 * i)) & 0xFF),
            ((i == 7) ? "\r\n" : " "));
  }
  return true;
}

static bool configureOPA(void) {
  /* String format in inBuffer:
   *  m<v> <w> <x> <y> <z>
   *      v[1] -> ch
   *      w[3] -> active
   *      x[5] -> function / edge
   *      (ignore below for OneWire/analog)
   *      y[7] -> pull up enabled
   *      z[9] -> NULL: hysteresis
   */
  const int32_t posCh     = 1;
  const int32_t posActive = 3;
  const int32_t posFunc   = 5;
  const int32_t posPu     = 7;
  const int32_t posPeriod = 9;

  ConvUint_t convU;
  uint8_t    ch     = 0;
  bool       active = 0;
  char       func   = 0;
  bool       pu     = false;
  uint8_t    period = 0;

  inBufferTok();

  /* Channel index */
  convU = utilAtoui(inBuffer + posCh, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid OPA channel value.");
    return false;
  }

  if (!convU.val.u32 || convU.val.u32 > NUM_OPA) {
    printfError("OPA channel out of range (valid: 1-%d).", NUM_OPA);
    return false;
  }

  ch = convU.val.u8 - 1;

  /* Check if the channel is active or inactive */
  convU = utilAtoui(inBuffer + posActive, ITOA_BASE10);
  if ((!convU.valid) || (convU.val.u32 > 1u)) {
    serialPutsError("Invalid OPA active value.");
    return false;
  }

  active = (bool)convU.val.u8;

  if (!active || ('\0' == inBuffer[posFunc])) {
    config.opaCfg[ch].opaActive = active;
    printSettingOPA(ch);
    return true;
  }
  config.opaCfg[ch].opaActive = true;

  /* Check for the function. Must be a valid type and if a pulse must also have
   * a hysteresis period applied. */
  func = inBuffer[posFunc];

  bool isPulse   = ('b' == func) || ('f' == func) || ('r' == func);
  bool isOneWire = ('o' == func);

  if (!(isPulse || isOneWire)) {
    serialPutsError("Invalid OPA function (valid: b/f/r/o).");
    return false;
  }

  if (isPulse) {
    convU = utilAtoui((inBuffer + posPu), ITOA_BASE10);
    if (!convU.valid) {
      serialPutsError("Invalid OPA pull-up value.");
      return false;
    }

    if (convU.val.u32 > 1) {
      serialPutsError("Invalid OPA pull-up value.");
      return false;
    }

    pu = (bool)convU.val.u8;

    convU = utilAtoui((inBuffer + posPeriod), ITOA_BASE10);
    if (!convU.valid) {
      serialPutsError("Invalid OPA period value.");
      return false;
    }
    period = convU.val.u8;
  }

  /* OPA3 can only be a pulse or analog input */
  if ((2 == ch) && !isPulse) {
    serialPutsError("OPA3 only supports pulse input.");
    return false;
  }

  if (isOneWire) {
    config.opaCfg[ch].func = 'o';
    printSettingOPA(ch);
    return true;
  }

  config.opaCfg[ch].func   = func;
  config.opaCfg[ch].period = period;
  config.opaCfg[ch].puEn   = pu;

  printSettingOPA(ch);

  return true;
}

static bool configureNodeID(void) {
  /* n<n>
   * Valid range is 1..60.
   */
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid node ID value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > 60)) {
    serialPutsError("Node ID out of range (valid: 1-60).");
    return false;
  }

  config.baseCfg.nodeID = convU.val.u8;
  printf_("rfNode = %d\r\n", config.baseCfg.nodeID);

  return true;
}

static void configureReconfigureAll(void) {
  rfmSetFrequency(config.dataTxCfg.rfmFreq);
  rfmSetPowerLevel(config.dataTxCfg.rfmPwr);

  config.baseCfg.reportCycles =
      configTimeToCycles(config.baseCfg.reportTime, config.baseCfg.mainsFreq);
  ecmConfigure();
  ecmFlush();

  emon32EventSet(EVT_OPA_INIT);
}

static void configureRestore(void) {
  if ('\0' == inBuffer[1]) {
    configDefault();
    configureReconfigureAll();

    serialPuts("> Restored default values.\r\n");
    unsavedChange = true;
  } else if ('s' == inBuffer[1]) {
    (void)configLoadFromNVM();
    configureReconfigureAll();

    serialPuts("> Restored values from NVM.\r\n");
    unsavedChange = false;
  }
}

static bool configureRFEnable(void) {
  int32_t val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    serialPutsError("RF enable must be 0 or 1.");
    return false;
  }

  config.dataTxCfg.useRFM = (bool)val;
  printf_("RF = %s\r\n", (bool)val ? "on" : "off");

  return true;
}

static bool configureRF433(void) {
  int32_t val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    serialPutsError("RF 433 value must be 0 or 1.");
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2) || (config.dataTxCfg.rfmFreq == 3))) {
    serialPutsError("RF 433 only applies to 433 MHz band.");
    return false;
  }

  config.dataTxCfg.rfmFreq = val ? 2 : 3;

  serialPuts("rfBand = ");
  printSettingRFFreq();
  serialPuts(" MHz\r\n");

  return true;
}

static bool configureRFPower(void) {
  /* p<n>
   * n is in range: 0-31
   */
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid RF power value.");
    return false;
  }

  if (convU.val.u32 > 31) {
    serialPutsError("RF power out of range (valid: 0-31).");
    return false;
  }

  config.dataTxCfg.rfmPwr = convU.val.u8;
  printf_("rfPower = %lu\r\n", convU.val.u32);
  return true;
}

static bool configureSerialLog(void) {
  /* Log to serial output, default TRUE
   * Format: c0 | c1
   */
  ConvUint_t convU = utilAtoui(inBuffer + 1, ITOA_BASE10);

  if (!convU.valid) {
    serialPutsError("Invalid serial log value.");
    return false;
  }

  if (convU.val.u32 > 1) {
    serialPutsError("Serial log must be 0 or 1.");
    return false;
  }

  config.baseCfg.logToSerial = (bool)convU.val.u8;
  return true;
}

static void enterBootloader(void) {
  /* Set confirmation state and prompt user
   * Response will be handled asynchronously by handleConfirmation() */
  serialPuts("> Enter bootloader? All unsaved changes will be lost. 'y' to "
             "proceed.\r\n");
  __disable_irq();
  confirmStartTime_ms = timerMillis();
  confirmState        = CONFIRM_BOOTLOADER;
  __enable_irq();
}

/*! @brief Get the board revision, software visible changes only
 *  @return board revision, 0-7
 */
static uint32_t getBoardRevision(void) {
  uint32_t boardRev = 0;
  boardRev |= portPinValue(GRP_REV0, PIN_REV0);
  boardRev |= portPinValue(GRP_REV1, PIN_REV1) << 1;
  boardRev |= portPinValue(GRP_REV2, PIN_REV2) << 2;
  return boardRev;
}

/*! @brief Get the last reset cause (16.8.14)
 *  @return null-terminated string with the last cause.
 */
static char *getLastReset(void) {
  const RCAUSE_t lastReset = (RCAUSE_t)PM->RCAUSE.reg;
  switch (lastReset) {
  case RCAUSE_SYST:
    return "Reset request";
    break;
  case RCAUSE_WDT:
    return "Watchdog timeout";
    break;
  case RCAUSE_EXT:
    return "External reset";
    break;
  case RCAUSE_BOD33:
    return "3V3 brownout";
    break;
  case RCAUSE_BOD12:
    return "1V2 brownout";
    break;
  case RCAUSE_POR:
    return "Power on cold reset";
    break;
  }
  return "Unknown";
}

uint32_t getUniqueID(const size_t idx) {
  /* Section 10.3.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(const size_t n) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, n);
}

static size_t inBufferTok(void) {
  /* Form a group of null-terminated strings */
  size_t tokCount = 0;
  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    if ('\0' == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
      tokCount++;
    }
  }
  return tokCount;
}

static void printSettingCT(const size_t ch) {
  printf_("iCal%u = ", (ch + 1));
  putFloat(config.ctCfg[ch].ctCal, 0);
  printf_(", iLead%u = ", (ch + 1));
  putFloat(config.ctCfg[ch].phase, 0);
  printf_(", iActive%u = %s", (ch + 1),
          config.ctCfg[ch].ctActive ? "on" : "off");
  printf_(", v1Chan%u = %d, v2Chan%u = %d\r\n", (ch + 1),
          (config.ctCfg[ch].vChan1 + 1), (ch + 1),
          (config.ctCfg[ch].vChan2 + 1));
}

static void printSettingDatalog(void) {
  serialPuts("datalog = ");
  putFloat(config.baseCfg.reportTime, 0);
  serialPuts("\r\n");
}

static void printSettingJSON(void) {
  printf_("json = %s\r\n", config.baseCfg.useJson ? "on" : "off");
}

static void printSettingOPA(const size_t ch) {
  printf_("opa%d ", (ch + 1));

  /* OneWire */
  if ('o' == config.opaCfg[ch].func) {
    printf_("active = %s, onewire\r\n",
            config.opaCfg[ch].opaActive ? "on" : "off");
    return;
  }

  /* Pulse - show edge type */
  const char *edgeStr = ('r' == config.opaCfg[ch].func)   ? "rising"
                        : ('f' == config.opaCfg[ch].func) ? "falling"
                                                          : "both";

  printf_("active = %s, pulse = %s, pullUp = %s, pulsePeriod = %d\r\n",
          (config.opaCfg[ch].opaActive ? "on" : "off"), edgeStr,
          config.opaCfg[ch].puEn ? "on" : "off", config.opaCfg[ch].period);
}

static void printSettingRF(void) {
  printf_("RF = %s, ", config.dataTxCfg.useRFM ? "on" : "off");
  serialPuts("rfBand = ");
  printSettingRFFreq();
  serialPuts(" MHz, ");
  printf_("rfGroup = %d, ", config.baseCfg.dataGrp);
  printf_("rfNode = %d, ", config.baseCfg.nodeID);
  printf_("rfPower = %d, ", config.dataTxCfg.rfmPwr);
  serialPuts("rfFormat = LowPowerLabs\r\n");
}

static void printSettingRFFreq(void) {
  switch (config.dataTxCfg.rfmFreq) {
  case 0:
    serialPuts("868");
    break;
  case 1:
    serialPuts("915");
    break;
  case 2:
    serialPuts("433.00");
    break;
  case 3:
    serialPuts("433.92");
    break;
  }
}

static void printSettingV(const size_t ch) {
  printf_("vCal%u = ", (ch + 1));
  putFloat(config.voltageCfg[ch].voltageCal, 0);
  printf_(", vLead%u = ", (ch + 1));
  putFloat(config.voltageCfg[ch].phase, 0);
  printf_(", vActive%u = %s\r\n", (ch + 1),
          config.voltageCfg[ch].vActive ? "on" : "off");
}

static void printAccumulators(void) {
  Emon32Cumulative_t cumulative;
  eepromWLStatus_t   status;
  bool               eepromOK;
  uint32_t           idx;

  status   = eepromReadWL(&cumulative, &idx);
  eepromOK = (EEPROM_WL_OK == status);

  serialPuts("Accumulators (can be updated by command 'u')");
  if (status == EEPROM_WL_BUSY) {
    serialPuts(" (write in progress)");
  } else if (!eepromOK) {
    serialPuts(" (no valid NVM data)");
  }
  printf_(" [%lu]:\r\n", idx);

  for (size_t i = 0; i < NUM_CT; i++) {
    int32_t wh = eepromOK ? cumulative.wattHour[i] : 0;
    printf_("  E%u = %lu Wh\r\n", (i + 1), wh);
  }
  for (size_t i = 0; i < NUM_OPA; i++) {
    uint32_t pulse = eepromOK ? cumulative.pulseCnt[i] : 0;
    printf_("  pulse%u = %lu\r\n", (i + 1), pulse);
  }
  serialPuts("\r\n");
}

static void printSettings(void) {
  if ('h' == inBuffer[1]) {
    printSettingsHR();
    /* Only show accumulators with 'lh' command */
    printAccumulators();
  } else {
    printSettingsKV();
  }

  if (unsavedChange) {
    serialPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  } else {
    serialPuts("All settings saved.\r\n\r\n");
  }
}

static void printSettingsHR(void) {
  serialPuts("\r\n\r\n==== Settings ====\r\n\r\n");
  printf_("Mains frequency (Hz):      %d\r\n", config.baseCfg.mainsFreq);
  serialPuts("Data log time (s):         ");
  putFloat(config.baseCfg.reportTime, 0);
  serialPuts("\r\nData transmission:         ");
  if (config.dataTxCfg.useRFM) {
    serialPuts("RFM69, ");
    printSettingRFFreq();
    printf_(" MHz @ %ddBm\r\n", (-18 + config.dataTxCfg.rfmPwr));
    printf_("  - Data group:            %d\r\n", config.baseCfg.dataGrp);
    printf_("  - Node ID:               %d\r\n", config.baseCfg.nodeID);
  } else {
    serialPuts("Serial only\r\n");
  }
  printf_("Data format:               %s\r\n",
          config.baseCfg.useJson ? "JSON" : "Key:Value");
  serialPuts("\r\n");

  for (size_t i = 0; i < NUM_OPA; i++) {
    bool enabled = config.opaCfg[i].opaActive;
    printf_("OPA %u (%sactive)\r\n", (i + 1), enabled ? "" : "in");
    if ('o' == config.opaCfg[i].func) {
      serialPuts("  - OneWire interface\r\n");
    } else {
      printf_("  - Hysteresis (ms): %d\r\n", config.opaCfg[i].period);
      serialPuts("  - Edge:            ");
      switch (config.opaCfg[i].func) {
      case 'b':
        serialPuts("Both");
        break;
      case 'f':
        serialPuts("Falling");
        break;
      case 'o':
        break;
      case 'r':
        serialPuts("Rising");
        break;
      default:
        serialPuts("Unknown");
      }
      printf_("\r\n  - Pull up:         %s\r\n",
              config.opaCfg[i].puEn ? "Yes" : "No");
    }
    serialPuts("\r\n");
  }

  printf_("Assumed RMS voltage: %d V\r\n\r\n", config.baseCfg.assumedVrms);

  serialPuts(
      "| Ref | Channel | Active | Calibration |  Phase  | In 1 | In 2 |\r\n");
  serialPuts(
      "+=====+=========+========+=============+=========+======+======+\r\n");
  for (size_t i = 0; i < NUM_V; i++) {
    printf_("| %2d  |  V %2d   | %c      | ", (i + 1), (i + 1),
            (config.voltageCfg[i].vActive ? 'Y' : 'N'));
    putFloat(config.voltageCfg[i].voltageCal, 6);
    serialPuts("      |  ");
    putFloat(config.voltageCfg[i].phase, 6);
    serialPuts(" |      |      |\r\n");
  }
  for (size_t i = 0; i < NUM_CT; i++) {
    printf_("| %2d  | CT %2d   | %c      | ", (i + 1 + NUM_V), (i + 1),
            (config.ctCfg[i].ctActive ? 'Y' : 'N'));
    putFloat(config.ctCfg[i].ctCal, 6);
    serialPuts("      |  ");
    putFloat(config.ctCfg[i].phase, 6);
    printf_(" | %d    | %d    |\r\n", (config.ctCfg[i].vChan1 + 1),
            (config.ctCfg[i].vChan2 + 1));
  }
  serialPuts("\r\n");
}

static void printSettingsKV(void) {
  serialPuts("hardware = emonPi3\r\n");
  printf_("hardware_rev = %lu\r\n", getBoardRevision());
  printf_("version = %d.%d.%d\r\n", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);
  printf_("commit = %s\r\n", emon32_build_info().revision);
  printf_("assumedV = %d\r\n", config.baseCfg.assumedVrms);
  for (size_t i = 0; i < NUM_V; i++) {
    printSettingV(i);
  }
  for (size_t i = 0; i < NUM_CT; i++) {
    printSettingCT(i);
  }
  for (size_t i = 0; i < NUM_OPA; i++) {
    printSettingOPA(i);
  }
  printSettingRF();
  printSettingDatalog();
  printSettingJSON();
}

static void putFloat(float val, const size_t flt_len) {
  char   strBuffer[16];
  size_t ftoalen = utilFtoa(strBuffer, val);

  while (ftoalen++ <= flt_len) {
    serialPuts(" ");
  }

  serialPuts(strBuffer);
}

static void printUptime(void) {

  uint32_t tSeconds = timerUptime();
  uint32_t tMinutes = tSeconds / 60;
  uint32_t tHours   = tMinutes / 60;
  uint32_t tDays    = tHours / 24;

  tSeconds = tSeconds % 60;
  tMinutes = tMinutes % 60;
  tHours   = tHours % 24;

  printf_("%lud %luh %lum %lus\r\n", tDays, tHours, tMinutes, tSeconds);
}

static void resetRequest(void) {
  serialPuts(
      "> Reset system? All unsaved changes will be lost. 'y' to proceed.\r\n");
  __disable_irq();
  confirmStartTime_ms = timerMillis();
  confirmState        = CONFIRM_RESET;
  __enable_irq();
}

/*! @brief Check if waiting for confirmation and handle if yes
 *  @param [in] c : character received
 *  @return true if character was handled as confirmation, false otherwise
 */
bool configHandleConfirmation(const uint8_t c) {
  if (CONFIRM_IDLE == confirmState) {
    return false; /* Not waiting for confirmation */
  }

  /* Reject CR/LF so the double line ending doesn't clear confirmation */
  if (('\n' == c) || ('\r' == c)) {
    return false;
  }

  /* We're waiting for confirmation - handle it */
  handleConfirmation((char)c);
  return true;
}

/*! @brief Handle confirmation response (async, internal)
 *  @param [in] c : character received ('y' or 'n' expected)
 */
static void handleConfirmation(char c) {
  volatile uint32_t *p_blsm;
  const uint32_t     blsm_key = 0xF01669EF;

  switch (confirmState) {
  case CONFIRM_BOOTLOADER:
    if ('y' == c) {
      p_blsm  = (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
      *p_blsm = blsm_key;
      NVIC_SystemReset();
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    __disable_irq();
    confirmState        = CONFIRM_IDLE;
    confirmStartTime_ms = 0;
    __enable_irq();
    break;

  case CONFIRM_RESET:
    if ('y' == c) {
      NVIC_SystemReset();
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    __disable_irq();
    confirmState        = CONFIRM_IDLE;
    confirmStartTime_ms = 0;
    __enable_irq();
    break;

  case CONFIRM_ZERO_ACCUM:
    if ('y' == c) {
      serialPuts("    - Clearing accumulators...\r\n");
      emon32EventSet(EVT_CLEAR_ACCUM);
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    __disable_irq();
    confirmState        = CONFIRM_IDLE;
    confirmStartTime_ms = 0;
    __enable_irq();
    break;

  case CONFIRM_ZERO_ACCUM_INDIVIDUAL:
    if ('y' == c) {
      Emon32Cumulative_t cumulative;
      uint32_t           idx;
      /* Read current NVM data */
      if (EEPROM_WL_OK == eepromReadWL(&cumulative, &idx)) {
        /* Clear the specific accumulator */
        if (clearAccumIdx < NUM_CT) {
          cumulative.wattHour[clearAccumIdx] = 0;
          ecmClearEnergyChannel(clearAccumIdx);
          printf_("    - Accumulator E%d cleared.\r\n", clearAccumIdx + 1);
        } else {
          cumulative.pulseCnt[clearAccumIdx - NUM_CT] = 0;
          pulseSetCount(clearAccumIdx - NUM_CT, 0);
          printf_("    - Accumulator pulse%d cleared.\r\n",
                  clearAccumIdx - NUM_CT + 1);
        }
        /* Write back to NVM */
        eepromWriteWLAsync(&cumulative, &idx);
      } else {
        serialPuts("    - Failed to read NVM.\r\n");
      }
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    __disable_irq();
    confirmState        = CONFIRM_IDLE;
    confirmStartTime_ms = 0;
    clearAccumIdx       = UINT8_MAX;
    __enable_irq();
    break;

  case CONFIRM_NVM_OVERWRITE:
    /* Reserved for future use: NVM corruption check during startup currently
     * uses a semi-blocking approach (see configLoadFromNVM) because it happens
     * before the main loop starts. This case is ready if startup is refactored
     * to use the async confirmation system.
     *
     * Future implementation:
     *   if ('y' == c) {
     *     configInitialiseNVM();
     *     serialPuts("    - NVM overwritten with defaults.\r\n");
     *   } else {
     *     serialPuts("    - Using potentially corrupt NVM.\r\n");
     *   }
     *   confirmState = CONFIRM_IDLE;
     *   confirmStartTime_ms = 0;
     */
    break;

    /* Removed pending OEM decision on restore defaults confirmation
     *
     * case CONFIRM_RESTORE_DEFAULTS:
     *   if ('y' == c) {
     *     configDefault();
     *     serialPuts("    - Restored default values.\r\n");
     *     unsavedChange = true;
     *     resetReq      = true;
     *
     *   } else {
     *     serialPuts("    - Cancelled.\r\n");
     *   }
     *   __disable_irq();
     *   confirmState        = CONFIRM_IDLE;
     *   confirmStartTime_ms = 0;
     *   __enable_irq();
     *   break;
     */

  case CONFIRM_IDLE:
    /* Not waiting for confirmation, ignore */
    break;
  }
}

/*! @brief Check for confirmation timeout and cancel if expired
 *  @details Call this periodically from main loop to check if a confirmation
 *           has been pending for too long (30 seconds). If timeout occurs,
 *           the confirmation is automatically cancelled.
 */
void configCheckConfirmationTimeout(void) {
  if (CONFIRM_IDLE == confirmState) {
    return; /* Not waiting for confirmation */
  }

  /* Check if timeout has expired */
  uint32_t elapsed_ms = timerMillis() - confirmStartTime_ms;
  if (elapsed_ms >= CONFIRM_TIMEOUT_MS) {
    serialPuts("    - Confirmation timeout, cancelled.\r\n");
    confirmState = CONFIRM_IDLE;
  }
}

/* Removed pending OEM decision on restore defaults confirmation
 *
 * static bool restoreDefaults(void) {
 *   serialPuts("> Restore default values? Unsaved changes will be lost. 'y' to
 * " "proceed.\r\n");
 *   __disable_irq();
 *   confirmStartTime_ms = timerMillis();
 *   confirmState        = CONFIRM_RESTORE_DEFAULTS;
 *   __enable_irq();
 *   return true;
 * }
 */

/*! @brief Zero all accumulators (async confirmation) */
static void zeroAccumulators(void) {
  clearAccumIdx = UINT8_MAX; /* UINT8_MAX means all accumulators */
  serialPuts(
      "> Zero accumulators. This can not be undone. 'y' to proceed.\r\n");
  __disable_irq();
  confirmStartTime_ms = timerMillis();
  confirmState        = CONFIRM_ZERO_ACCUM;
  __enable_irq();
}

/*! @brief Zero individual accumulator (async confirmation)
 *  @param [in] idx : accumulator index (0-11=E1-E12, 12-13=P1-P2)
 */
static void zeroAccumulatorIndividual(uint8_t idx) {
  clearAccumIdx = idx;
  if (idx < NUM_CT) {
    printf_(
        "> Zero accumulator E%d. This can not be undone. 'y' to proceed.\r\n",
        idx + 1);
  } else {
    printf_("> Zero accumulator pulse%d. This can not be undone. 'y' to "
            "proceed.\r\n",
            idx - NUM_CT + 1);
  }
  __disable_irq();
  confirmStartTime_ms = timerMillis();
  confirmState        = CONFIRM_ZERO_ACCUM_INDIVIDUAL;
  __enable_irq();
}

/*! @brief Parse z command and zero accumulators (z, ze1-12, zp1-3) */
static void parseAndZeroAccumulator(void) {
  /* z - zero all */
  if (inBuffer[1] == '\0') {
    zeroAccumulators();
    return;
  }

  /* ze1-12 - zero energy accumulator */
  if (inBuffer[1] == 'e' && inBuffer[2] >= '1' && inBuffer[2] <= '9') {
    union {
      int     i;
      uint8_t u8;
    } digit;
    digit.i     = inBuffer[2] - '0';
    uint8_t num = digit.u8;
    /* Check for two-digit number (ze10-12) */
    if (inBuffer[3] >= '0' && inBuffer[3] <= '9') {
      num *= 10;
      digit.i = inBuffer[3] - '0';
      num += digit.u8;
    }
    if (num >= 1 && num <= NUM_CT) {
      zeroAccumulatorIndividual(num - 1); /* Convert to 0-indexed */
    } else {
      printfError("Invalid energy accumulator index (valid: ze1-%d).", NUM_CT);
    }
    return;
  }

  /* zp1-3 - zero pulse accumulator */
  if (inBuffer[1] == 'p' && inBuffer[2] >= '1' &&
      inBuffer[2] <= '0' + NUM_OPA) {
    uint8_t num = inBuffer[2] - '0';
    if (num >= 1 && num <= NUM_OPA) {
      zeroAccumulatorIndividual(NUM_CT + num - 1); /* Pulse index starts after
                                                       energy */
    } else {
      printfError("Invalid pulse accumulator index (valid: zp1-%d).", NUM_OPA);
    }
    return;
  }

  /* Invalid format */
  serialPutsError("Invalid command. Use z, ze1-12, or zp1-3.");
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      configEchoQueueStr("\r\n");
      cmdPending = true;
      emon32EventSet(EVT_PROCESS_CMD);
    }
  } else if ('\b' == c) {
    configEchoQueueStr("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < (IN_BUFFER_W - 1)) && utilCharPrintable(c)) {
    configEchoQueueChar(c);
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    configEchoQueueStr("\r\n");
  }
  emon32EventSet(EVT_ECHO);
}

void configFirmwareBoardInfo(void) {
  serialPuts("                           ▄▄▄▄▄▄▄  ▄▄▄▄▄▄▄ \r\n");
  serialPuts("                           ▀▀▀▀████ ▀▀▀▀████\r\n");
  serialPuts("▄█▀█▄ ███▄███▄ ▄███▄ ████▄   ▄▄██▀     ▄██▀ \r\n");
  serialPuts("██▄█▀ ██ ██ ██ ██ ██ ██ ██     ███▄  ▄███▄▄▄\r\n");
  serialPuts("▀█▄▄▄ ██ ██ ██ ▀███▀ ██ ██ ███████▀ ████████\r\n\r\n");

  serialPuts("> Board:\r\n");
  printf_("  - emonPi3/emonTx6 (arch. rev. %lu)\r\n", getBoardRevision());
  printf_("  - Serial    : 0x%02lx%02lx%02lx%02lx\r\n", getUniqueID(0),
          getUniqueID(1), getUniqueID(2), getUniqueID(3));
  printf_("  - Last reset: %s\r\n", getLastReset());
  serialPuts("  - Uptime    : ");
  printUptime();
  serialPuts("\r\n");

  serialPuts("> Firmware:\r\n");
  printf_("  - Version:    %d.%d.%d\r\n", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);
  serialPuts("  - Build:      ");
  serialPuts(emon32_build_info_string());
  serialPuts("\r\n\r\n");
  serialPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  serialPuts("  - emon32 Copyright (C) 2023-26 Angus Logan\r\n");
  serialPuts("  - See CONTRIBUTORS.md\r\n");
  serialPuts("  - For Bear and Moose\r\n\r\n");
}

Emon32Config_t *configLoadFromNVM(void) {

  const uint32_t cfgSize     = sizeof(config);
  uint16_t       crc16_ccitt = 0;

  /* Load from "static" part of EEPROM. If the key does not match
   * CONFIG_NVM_KEY as this is the first time it has been run, run the built
   * in self test, write the default configuration to the EEPROM and zero wear
   * levelled portion before resetting.
   */
  eepromRead(0, &config, cfgSize);

  if (CONFIG_NVM_KEY != config.key) {
    configInitialiseNVM();
    NVIC_SystemReset();
  } else {
    /* Check the CRC and raise a warning if not matched. -2 from the base
     * size to account for the stored 16 bit CRC.
     */
    crc16_ccitt = calcCRC16_ccitt(&config, cfgSize - 2u);
    if (crc16_ccitt != config.crc16_ccitt) {
      serialPuts(
          "  - NVM corrupt. Loading defaults (save with 's' to fix).\r\n");
      configDefault();
      unsavedChange = true;
    }
  }

  return &config;
}

void configProcessCmd(void) {
  uint32_t arglen    = 0;
  bool     termFound = false;

  /* Help text - serves as documentation interally as well */
  const char helpText[] =
      "\r\n"
      "emon32 information and configuration commands\r\n\r\n"
      " - ?           : show this text again\r\n"
      " - a<n>        : set the assumed RMS voltage as integer\r\n"
      " - b           : backup to serial\r\n"
      " - c<n>        : log to serial output. n = 0: OFF, n = 1: ON\r\n"
      " - d<x.x>      : data log period (s)\r\n"
      " - e           : enter bootloader\r\n"
      " - f<n>        : line frequency (Hz)\r\n"
      " - g<n>        : set network group (default = 210)\r\n"
      " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - k<x> <a> <y.y> <z.z> v1 v2 : Configure an analog input\r\n"
      "   - x:        : channel (1-3 -> V; 4... -> CT)\r\n"
      "   - a:        : channel active. a = 0: DISABLED, a = 1: ENABLED\r\n"
      "   - y.y       : V/CT calibration constant\r\n"
      "   - z.z       : V/CT phase calibration value\r\n"
      "   - v1        : voltage 1 (for CT only)\r\n"
      "   - v2        : voltage 2 (for CT only, optional)\r\n"
      " - l           : list settings\r\n"
      " - lh          : list settings and accumulators (human readable)\r\n"
      " - m<v> <w> <x> <y> <z> : Configure OPA1-3 for OneWire or Pulse\r\n"
      "   - v : OPA index. [1-3]\r\n"
      "   - w : OPA active. w = 0: DISABLED, w = 1: ENABLED\r\n"
      "   - x : function select. x = [b,f,r]: pulse, x = o: OneWire.\r\n"
      "   - y : pull-up, only for pulse. y = 0: OFF, y = 1: ON\r\n"
      "   - z : minimum period (ms), only for pulse.\r\n"
      " - n<n>        : set node ID [1..60]\r\n"
      " - o<x>        : configure OneWire addressing\r\n"
      "   - x = ca  : clear all saved OneWire addresses\r\n"
      "   - x = c<n>: clear saved address for channel n\r\n"
      "   - x = f   : reset and find OneWire devices\r\n"
      "   - x = l   : list current addresses\r\n"
      "   - x = n   : list all saved addresses\r\n"
      "   - x = s   : save found indexes (overrides manually set values)\r\n"
      "   - x = <n> : save address to index n\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - q           : reset system (requires confirmation)\r\n"
      " - r           : restore defaults\r\n"
      " - rs          : restore saved settings from NVM\r\n"
      " - s           : save settings to NVM\r\n"
      " - t           : trigger report on next cycle\r\n"
      " - u           : store current accumulator values to NVM\r\n"
      " - v           : firmware and board information\r\n"
      " - w<n>        : RF active. n = 0: OFF, n = 1: ON\r\n"
      " - x<n>        : 433 MHz compatibility. n = 0: 433.92 MHz, n = 1: "
      "433.00 MHz\r\n"
      " - z           : zero all accumulators (E1-E12, pulse1-3)\r\n"
      " - ze<n>       : zero individual energy accumulator (n=1-12)\r\n"
      " - zp<n>       : zero individual pulse accumulator (n=1-3)\r\n\r\n";

  /* Convert \r or \n to 0, and get the length until then. */
  while (!termFound && (arglen < IN_BUFFER_W)) {
    if (0 == inBuffer[arglen]) {
      termFound = true;
      break;
    }
    arglen++;
  }

  if (!termFound) {
    return;
  }

  /* Decode on first character in the buffer */
  switch (inBuffer[0]) {
  case '?':
    /* Print help text */
    serialPuts(helpText);
    break;
  case 'a':
    if (configureAssumed()) {
      unsavedChange = true;
    }
    break;
  case 'b':
    configureBackup();
    break;
  case 'c':
    if (configureSerialLog()) {
      unsavedChange = true;
    }
    break;
  case 'd':
    if (configureDatalog()) {
      unsavedChange = true;
    }
    break;
  case 'e':
    /* Enter the bootloader through firmware */
    enterBootloader();
    break;
  case 'f':
    /* Set line frequency.
     * Format: f50 | f60
     */
    if (configureLineFrequency()) {
      unsavedChange = true;
    }
    break;
  case 'g':
    if (configureGroupID()) {
      rfmSetGroupID(config.baseCfg.dataGrp);
      unsavedChange = true;
    }
    break;
  case 'j':
    if (configureJSON()) {
      unsavedChange = true;
    }
    break;
  case 'k':
    if (configureAnalog()) {
      unsavedChange = true;
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    if (configureOPA()) {
      unsavedChange = true;
      emon32EventSet(EVT_OPA_INIT);
    }
    break;
  case 'o':
    if (configure1WAddr()) {
      unsavedChange = true;
      emon32EventSet(EVT_OPA_INIT);
    }
    break;
  case 'n':
    /* Set the node ID */
    if (configureNodeID()) {
      unsavedChange = true;
    }
    break;
  case 'p':
    /* Configure RF power */
    if (configureRFPower()) {
      rfmSetPowerLevel(config.dataTxCfg.rfmPwr);
      unsavedChange = true;
    }
    break;
  case 'q':
    resetRequest();
    break;
  case 'r':
    configureRestore();
    break;
  case 's':
    /* Save to EEPROM config space after recalculating CRC and indicate if a
     * reset is required.
     */
    config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2));

    serialPuts("> Saving configuration to NVM... ");
    eepromInitConfig(&config, sizeof(config));
    serialPuts("Done!\r\n");

    unsavedChange = false;
    break;
  case 't':
    emon32EventSet(EVT_ECM_TRIG);
    break;
  case 'u':
    emon32EventSet(EVT_STORE_ACCUM);
    break;
  case 'v':
    configFirmwareBoardInfo();
    break;
  case 'w':
    if (configureRFEnable()) {
      unsavedChange = true;
    }
    break;

  case 'x':
    if (configureRF433()) {
      rfmSetFrequency(config.dataTxCfg.rfmFreq);
      unsavedChange = true;
    }
    break;
  case 'z':
    parseAndZeroAccumulator();
    break;
  }

  cmdPending = false;
  inBufferClear(arglen + 1);
}

bool configUnsavedChanges(void) { return unsavedChange; }

uint16_t configTimeToCycles(const float time, const uint32_t mainsFreq) {
  return (uint16_t)qfp_float2uint(qfp_fmul(time, qfp_uint2float(mainsFreq)));
}

VersionInfo_t configVersion(void) {
  struct Emon32BuildInfo binfo = emon32_build_info();
  return (VersionInfo_t){.version = binfo.version, .revision = binfo.revision};
}

/* =======================
 * UART Interrupt handler
 * ======================= */

#define ECHO_BUF_DEPTH 16u
#define ECHO_IDX_MASK  (ECHO_BUF_DEPTH - 1u)
#define ECHO_FMASK     ((ECHO_IDX_MASK << 1) + 1u)

static size_t  idxEchoWr               = 0;
static size_t  idxEchoRd               = 0;
static uint8_t echoBuf[ECHO_BUF_DEPTH] = {0};

static void configEchoQueueChar(const uint8_t c) {
  echoBuf[(idxEchoWr & ECHO_IDX_MASK)] = c;
  idxEchoWr                            = (idxEchoWr + 1u) & ECHO_FMASK;
}

static void configEchoQueueStr(const char *s) {
  while (*s) {
    configEchoQueueChar(*s++);
  }
}

uint8_t configEchoChar(void) {
  uint8_t c = 0;
  if (idxEchoRd != idxEchoWr) {
    c         = echoBuf[idxEchoRd & ECHO_IDX_MASK];
    idxEchoRd = (idxEchoRd + 1u) & ECHO_FMASK;
  }
  return c;
}

void SERCOM_UART_INTERACTIVE_HANDLER {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */
  if (uartGetcReady(SERCOM_UART_INTERACTIVE)) {
    uint8_t rx_char = uartGetc(SERCOM_UART_INTERACTIVE);

    if (!configHandleConfirmation(rx_char)) {
      configCmdChar(rx_char);
    }
  }

  /* Revisit : need to handle the Error interrupt? */
}
