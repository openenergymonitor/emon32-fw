#include <inttypes.h>
#include <string.h>

#include "emon32_assert.h"

#include "driver_ADC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_USB.h"

#include "configuration.h"
#include "eeprom.h"
#include "emon32.h"
#include "emon32_build_info.h"
#include "emon_CM.h"
#include "periph_rfm69.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

/*************************************
 * Types
 *************************************/

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
static void     configInitialiseNVM(void);
static int      configTimeToCycles(const float time, const int mainsFreq);
static bool     configureAnalog(void);
static bool     configureAssumed(void);
static void     configureBackup(void);
static bool     configureDatalog(void);
static bool     configureGroupID(void);
static bool     configureJSON(void);
static bool     configureLineFrequency(void);
static bool     configureOPA(void);
static bool     configureNodeID(void);
static bool     configureRFEnable(void);
static bool     configureRF433(void);
static bool     configureRFPower(void);
static bool     configureSerialLog(void);
static void     enterBootloader(void);
static uint32_t getBoardRevision(void);
static char    *getLastReset(void);
static void     inBufferClear(int n);
static void     printSettingCT(const int ch);
static void     printSettingDatalog(void);
static void     printSettingJSON(void);
static void     printSettingOPA(const int ch);
static void     printSettingRF(void);
static void     printSettingRFFreq(void);
static void     printSettingV(const int ch);
static void     printSettings(void);
static void     printSettingsHR(void);
static void     printSettingsKV(void);
static void     printUptime(void);
static void     putFloat(float val, int flt_len);
static char     waitForChar(void);
static bool     zeroAccumulators(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W 64
static Emon32Config_t config;
static char           inBuffer[IN_BUFFER_W];
static int            inBufferIdx   = 0;
static bool           cmdPending    = false;
static bool           resetReq      = false;
static bool           unsavedChange = false;

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  config.key = CONFIG_NVM_KEY;

  /* Single phase, 50 Hz, 240 VAC, 10 s report period */
  config.baseCfg.nodeID       = NODE_ID_DEF;
  config.baseCfg.mainsFreq    = MAINS_FREQ_DEF;
  config.baseCfg.reportTime   = REPORT_TIME_DEF;
  config.baseCfg.assumedVrms  = ASSUMED_VRMS_DEF;
  config.baseCfg.whDeltaStore = DELTA_WH_STORE_DEF;
  config.baseCfg.dataGrp      = GROUP_ID_DEF;
  config.baseCfg.logToSerial  = true;
  config.baseCfg.useJson      = false;
  config.dataTxCfg.useRFM     = true;
  config.dataTxCfg.rfmPwr     = RFM_PALEVEL_DEF;
  config.dataTxCfg.rfmFreq    = RFM_FREQ_DEF;

  for (int idxV = 0u; idxV < NUM_V; idxV++) {
    config.voltageCfg[idxV].voltageCal = 100.0f;
    config.voltageCfg[idxV].vActive    = (0 == idxV);
  }

  /* 4.2 degree shift @ 50 Hz */
  for (int idxCT = 0u; idxCT < NUM_CT; idxCT++) {
    config.ctCfg[idxCT].ctCal    = 100.0f;
    config.ctCfg[idxCT].phase    = 4.2f;
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
  ConvInt_t   convI     = {false, 0};
  int         ch        = 0;
  bool        active    = false;
  float       calAmpl   = 0.0f;
  float       calPhase  = 0.0f;
  int         vCh1      = 0;
  int         vCh2      = 0;
  int         posActive = 0;
  int         posCalib  = 0;
  int         posPhase  = 0;
  int         posV1     = 0;
  int         posV2     = 0;
  ECMCfg_t   *ecmCfg    = 0;

  for (int i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
      if (0 == posActive) {
        posActive = i + 1;
      } else if (0 == posCalib) {
        posCalib = i + 1;
      } else if (0 == posPhase) {
        posPhase = i + 1;
      } else if (0 == posV1) {
        posV1 = i + 1;
      } else if (0 == posV2) {
        posV2 = i + 1;
        break;
      }
    }
  }

  /* Voltage channels are [1..3], CTs are [4..] but 0 indexed internally. All
   * fields must be present for a given channel type.
   */
  convI = utilAtoi(inBuffer + 1u, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  ch = convI.val - 1;

  if ((ch < 0) || (ch >= VCT_TOTAL)) {
    return false;
  }

  if ((0 == posCalib) || (0 == posActive)) {
    return false;
  }
  if (ch >= NUM_V) {
    if ((0 == posPhase) || (0 == posV1) || (0 == posV2)) {
      return false;
    }
  }

  ecmCfg = ecmConfigGet();
  EMON32_ASSERT(ecmCfg);

  convI = utilAtoi(inBuffer + posActive, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  active = (bool)convI.val;

  convF = utilAtof(inBuffer + posCalib);
  if (!convF.valid) {
    return false;
  }
  calAmpl = convF.val;

  if (NUM_V > ch) {

    if ((calAmpl <= 25.0f) || (calAmpl >= 150.0f)) {
      return false;
    }

    config.voltageCfg[ch].vActive    = active;
    config.voltageCfg[ch].voltageCal = calAmpl;
    ecmCfg->vCfg[ch].vActive         = active;
    ecmCfg->vCfg[ch].voltageCalRaw   = calAmpl;

    printSettingV(ch);

    ecmConfigChannel(ch);
    return true;
  }

  convF = utilAtof(inBuffer + posPhase);
  if (!convF.valid) {
    return false;
  }
  calPhase = convF.val;

  convI = utilAtoi(inBuffer + posV1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  vCh1 = convI.val;

  convI = utilAtoi(inBuffer + posV2, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  vCh2 = convI.val;

  if ((vCh1 < 1) || (vCh1 > NUM_V) || (vCh2 < 1) || (vCh2 > NUM_V)) {
    return false;
  }

  /* CT configuration - assume 10/200 A min/max CTs */
  if ((calAmpl < 10.0f) || (calAmpl) > 200.0f) {
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
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (convI.valid) {
    ECMCfg_t *pEcmCfg          = ecmConfigGet();
    pEcmCfg->assumedVrms       = qfp_uint2float(convI.val);
    config.baseCfg.assumedVrms = convI.val;
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
  printf_("\"revision\":%d,", (int)getBoardRevision());
  printf_("\"serial\":\"0x%02x%02x%02x%02x\",", (unsigned int)getUniqueID(0),
          (unsigned int)getUniqueID(1), (unsigned int)getUniqueID(2),
          (unsigned int)getUniqueID(3));
  printf_("\"fw\":\"%d.%d.%d\"},", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);

  /* {board_config} dict */
  utilFtoa(strBuf, config.baseCfg.reportTime);
  printf_("\"board_config\":{\"rfmFreq\":%d,\"f_mains\":%d,\"t_report\":%s},",
          config.dataTxCfg.rfmFreq, config.baseCfg.mainsFreq, strBuf);

  printf_("\"assumedV\":%d,", config.baseCfg.assumedVrms);
  /* {v_config} list of dicts */
  serialPuts("\"v_config\":[");
  for (int i = 0; i < NUM_V; i++) {
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
  for (int i = 0; i < NUM_CT; i++) {
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
  if (convF.valid) {
    if ((convF.val < 0.5f) || (convF.val > 600.0f)) {
      serialPuts("> Log report time out of range.\r\n");
    } else {
      config.baseCfg.reportTime = convF.val;
      ecmConfigReportCycles(
          configTimeToCycles(convF.val, config.baseCfg.mainsFreq));

      printSettingDatalog();
      return true;
    }
  }
  return false;
}

static bool configureGroupID(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (!convI.valid) {
    return false;
  }

  if ((convI.val < 0) || (convI.val > 255)) {
    return false;
  }

  config.baseCfg.dataGrp = convI.val;
  printf_("rfGroup = %d\r\n", (int)convI.val);
  return true;
}

static bool configureJSON(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (!convI.valid) {
    return false;
  }

  if (!((0 == convI.val) || (1 == convI.val))) {
    return false;
  }

  config.baseCfg.useJson = (bool)convI.val;
  printSettingJSON();
  return true;
}

static bool configureLineFrequency(void) {
  /* f<n>
   * n must be 50 or 60
   */
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  if (!((50 == convI.val) || (60 == convI.val))) {
    return false;
  }

  printf_("> Mains frequency set to: %d\r\n", config.baseCfg.mainsFreq);
  config.baseCfg.mainsFreq = convI.val;
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
  const int posCh     = 1;
  const int posActive = 3;
  const int posFunc   = 5;
  const int posPu     = 7;
  const int posPeriod = 9;

  ConvInt_t convI;
  int       ch     = 0;
  bool      active = 0;
  char      func   = 0;
  bool      pu     = false;
  int       period = 0;

  /* Form a group of null-terminated strings */
  for (int i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
    }
  }

  /* Channel index */
  convI = utilAtoi(inBuffer + posCh, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  ch = convI.val - 1;

  if ((ch < 0) || (ch >= NUM_OPA)) {
    return false;
  }

  /* Check if the channel is active or inactive */
  convI = utilAtoi(inBuffer + posActive, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  active = (bool)convI.val;

  if (!active) {
    config.opaCfg[ch].opaActive = false;
    printSettingOPA(ch);
    return true;
  }
  config.opaCfg[ch].opaActive = true;

  /* Check for the function. Must be a valid type and if a pulse must also have
   * a hysteresis period applied. */
  func = inBuffer[posFunc];
  if (!(('b' == func) || ('f' == func) || ('o' == func) || ('r' == func))) {
    return false;
  }

  if ('o' != func) {
    convI = utilAtoi((inBuffer + posPu), ITOA_BASE10);
    if (!convI.valid) {
      return false;
    }
    pu = (bool)convI.val;

    convI = utilAtoi((inBuffer + posPeriod), ITOA_BASE10);
    if (!convI.valid) {
      return false;
    }
    period = convI.val;
  }

  /* OneWire requires a reset if changed to find any OneWire sensors. */
  if ('o' == func) {
    config.opaCfg[ch].func = 'o';
    if ('o' != config.opaCfg[ch].func) {
      resetReq = true;
    }
    printSettingOPA(ch);
    return true;
  }

  config.opaCfg[ch].func   = func;
  config.opaCfg[ch].period = period;
  config.opaCfg[ch].puEn   = pu;

  printSettingOPA(ch);
  resetReq = true;
  return true;
}

static bool configureNodeID(void) {
  /* n<n>
   * Valid range is 1..60.
   */
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  if ((convI.val < 1) || (convI.val > 60)) {
    return false;
  }
  config.baseCfg.nodeID = convI.val;
  return true;
}

static bool configureRFEnable(void) {
  int val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    return false;
  }

  config.dataTxCfg.useRFM = (bool)val;
  printf_("RF = %s\r\n", (bool)val ? "on" : "off");

  return true;
}

static bool configureRF433(void) {
  int val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2) || (config.dataTxCfg.rfmFreq == 3))) {
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
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  if ((convI.val < 0) || (convI.val > 31)) {
    return false;
  }

  config.dataTxCfg.rfmPwr = convI.val;
  printf_("rfPower = %d\r\n", (int)convI.val);
  return true;
}

static bool configureSerialLog(void) {
  /* Log to serial output, default TRUE
   * Format: c0 | c1
   */
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (!convI.valid) {
    return false;
  }

  if (!((0 == convI.val) || (1 == convI.val))) {
    return false;
  }

  config.baseCfg.logToSerial = (bool)convI.val;
  return true;
}

static void enterBootloader(void) {
  /* Linker reserves 4 bytes at the bottom of the stack and write the UF2
   * bootloader key followed by reset. Will enter bootloader upon reset. */
  char               c;
  volatile uint32_t *p_blsm =
      (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
  // Key is uf2-samdx1/inc/uf2.h:DBL_TAP_MAGIC
  const uint32_t blsm_key = 0xF01669EF;
  serialPuts("> Enter bootloader? All unsaved changes will be lost. 'y' to "
             "proceed.\r\n");
  c = waitForChar();
  if ('y' == c) {
    *p_blsm = blsm_key;
    NVIC_SystemReset();
  } else {
    serialPuts("    - Cancelled.");
  }
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

uint32_t getUniqueID(int idx) {
  /* Section 10.3.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(int n) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, n);
}

static void printSettingCT(const int ch) {
  printf_("iCal%d = ", (ch + 1));
  putFloat(config.ctCfg[ch].ctCal, 0);
  printf_(", iLead%d = ", (ch + 1));
  putFloat(config.ctCfg[ch].phase, 0);
  printf_(", iActive%d = %s", (ch + 1), config.ctCfg[ch].ctActive ? "1" : "0");
  printf_(", v1Chan%d = %d, v2Chan%d = %d\r\n", (ch + 1),
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

static void printSettingOPA(const int ch) {
  printf_("opa%d = ", (ch + 1));

  /* OneWire */
  if ('o' == config.opaCfg[ch].func) {
    printf_("onewire, active = %s\r\n",
            config.opaCfg[ch].opaActive ? "1" : "0");
    return;
  }

  /* Pulse */
  printf_("pulse, pulsePeriod = %d, pullUp = %s, active = %s\r\n",
          config.opaCfg[ch].period, config.opaCfg[ch].puEn ? "on" : "off",
          config.opaCfg[ch].opaActive ? "1" : "0");
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

static void printSettingV(const int ch) {
  printf_("vCal%d = ", (ch + 1));
  putFloat(config.voltageCfg[ch].voltageCal, 0);
  printf_(", vActive%d = %s\r\n", (ch + 1),
          config.voltageCfg[ch].vActive ? "1" : "0");
}

static void printSettings(void) {
  if ('h' == inBuffer[1]) {
    printSettingsHR();
  } else {
    printSettingsKV();
  }

  if (unsavedChange) {
    serialPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
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
    printf_(" MHz @ %ddb\r\n", (-18 + config.dataTxCfg.rfmPwr));
    printf_("  - Data group:            %d\r\n", config.baseCfg.dataGrp);
    printf_("  - Node ID:               %d\r\n", config.baseCfg.nodeID);
  } else {
    serialPuts("Serial only\r\n");
  }
  printf_("Data format:               %s\r\n",
          config.baseCfg.useJson ? "JSON" : "Key:Value");
  serialPuts("\r\n");

  for (unsigned int i = 0; i < NUM_OPA; i++) {
    bool enabled = config.opaCfg[i].opaActive;
    printf_("OPA %d (%sactive)\r\n", (i + 1), enabled ? "" : "in");
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
      "| Ref | Channel | Active | Calibration | Phase  | In 1 | In 2 |\r\n");
  serialPuts(
      "+=====+=========+========+=============+========+======+======+\r\n");
  for (int i = 0; i < NUM_V; i++) {
    printf_("| %2d  |  V %2d   | %c      | ", (i + 1), (i + 1),
            (config.voltageCfg[i].vActive ? 'Y' : 'N'));
    putFloat(config.voltageCfg[i].voltageCal, 6);
    serialPuts("      |        |      |      |\r\n");
  }
  for (int i = 0; i < NUM_CT; i++) {
    printf_("| %2d  | CT %2d   | %c      | ", (i + 1 + NUM_V), (i + 1),
            (config.ctCfg[i].ctActive ? 'Y' : 'N'));
    putFloat(config.ctCfg[i].ctCal, 6);
    serialPuts("      | ");
    putFloat(config.ctCfg[i].phase, 6);
    printf_(" | %d    | %d    |\r\n", (config.ctCfg[i].vChan1 + 1),
            (config.ctCfg[i].vChan2 + 1));
  }
  serialPuts("\r\n");
}

static void printSettingsKV(void) {
  serialPuts("hardware = emonPi3\r\n");
  printf_("hardware_rev = %" PRIu32 "\r\n", getBoardRevision());
  printf_("version = %d.%d.%d\r\n", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);
  printf_("commit = %s\r\n", emon32_build_info().revision);
  printf_("assumedV = %d\r\n", config.baseCfg.assumedVrms);
  for (int i = 0; i < NUM_V; i++) {
    printSettingV(i);
  }
  for (int i = 0; i < NUM_CT; i++) {
    printSettingCT(i);
  }
  for (int i = 0; i < NUM_OPA; i++) {
    printSettingOPA(i);
  }
  printSettingRF();
  printSettingDatalog();
  printSettingJSON();
}

static void putFloat(float val, int flt_len) {
  char strBuffer[16];
  int  ftoalen = utilFtoa(strBuffer, val);

  if (flt_len) {
    int fillSpace = flt_len - ftoalen;

    while (fillSpace--) {
      serialPuts(" ");
    }
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

  printf_("%" PRIu32 "d %" PRIu32 "h %" PRIu32 "m %" PRIu32 "s\r\n", tDays,
          tHours, tMinutes, tSeconds);
}

/*! @brief Blocking wait for a key from the serial link. If the USB CDC is
 *         connected the key will come from here.
 */
static char waitForChar(void) {
  /* Disable the NVIC for the interrupt if needed while waiting for the
   * character otherwise it is handled by the configuration buffer.
   */
  char c;
  if (usbCDCIsConnected()) {
    while (!usbCDCRxAvailable())
      ;
    c = usbCDCRxGetChar();
  } else {
    int irqEnabled = (NVIC->ISER[0] &
                      (1 << ((uint32_t)(SERCOM_UART_INTERACTIVE_IRQn) & 0x1F)))
                         ? 1
                         : 0;
    if (irqEnabled) {
      NVIC_DisableIRQ(SERCOM_UART_INTERACTIVE_IRQn);
    }

    while (0 == (uartInterruptStatus(SERCOM_UART) & SERCOM_USART_INTFLAG_RXC))
      ;
    c = uartGetc(SERCOM_UART);

    if (irqEnabled) {
      NVIC_EnableIRQ(SERCOM_UART_INTERACTIVE_IRQn);
    }
  }

  return c;
}

/*! @brief Zero the accumulator portion of the NVM
 *  @return true if cleared, false if cancelled
 */
static bool zeroAccumulators(void) {
  char c;
  serialPuts(
      "> Zero accumulators. This can not be undone. 'y' to proceed.\r\n");

  c = waitForChar();
  if ('y' == c) {
    eepromInitBlock(EEPROM_WL_OFFSET, 0, (1024 - EEPROM_WL_OFFSET));
    serialPuts("    - Accumulators cleared.\r\n");
    return true;
  } else {
    serialPuts("    - Cancelled.\r\n");
    return false;
  }
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      serialPuts("\r\n");
      cmdPending = true;
      emon32EventSet(EVT_PROCESS_CMD);
    }
  } else if ('\b' == c) {
    serialPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < (IN_BUFFER_W - 1)) && utilCharPrintable(c)) {
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    serialPuts("\r\n");
  }
}

void configFirmwareBoardInfo(void) {
  serialPuts("\033c==== emonPi3 | emonTx6 ====\r\n\r\n");

  serialPuts("> Board:\r\n");
  printf_("  - emonPi3/emonTx6 (arch. rev. %" PRIu32 ")\r\n",
          getBoardRevision());
  printf_("  - Serial    : 0x%02x%02x%02x%02x\r\n",
          (unsigned int)getUniqueID(0), (unsigned int)getUniqueID(1),
          (unsigned int)getUniqueID(2), (unsigned int)getUniqueID(3));
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
  serialPuts("  - emon32 Copyright (C) 2023-25 Angus Logan\r\n");
  serialPuts("  - For Bear and Moose\r\n\r\n");
}

Emon32Config_t *configLoadFromNVM(void) {

  const uint32_t cfgSize     = sizeof(config);
  uint16_t       crc16_ccitt = 0;
  char           c           = 0;

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
      serialPuts("  - NVM may be corrupt. Overwrite with default? (y/n)\r\n");
      while ('y' != c && 'n' != c) {
        c = waitForChar();
      }
      if ('y' == c) {
        configInitialiseNVM();
      }
    }
  }

  config.baseCfg.reportCycles =
      configTimeToCycles(config.baseCfg.reportTime, config.baseCfg.mainsFreq);

  return &config;
}

void configProcessCmd(void) {
  unsigned int arglen    = 0;
  bool         termFound = false;

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
      " - k<x> <a> <y.y> <z.z> v1 v2\r\n"
      "   - Configure an analog input\r\n"
      "   - x:        : channel (1-3 -> V; 4... -> CT)\r\n"
      "   - a:        : channel active. a = 0: DISABLED, a = 1: ENABLED\r\n"
      "   - y.y       : V/CT calibration constant\r\n"
      "   - z.z       : CT phase calibration value\r\n"
      "   - v1        : CT voltage channel 1\r\n"
      "   - v2        : CT voltage channel 2\r\n"
      " - l           : list settings\r\n"
      " - m<v> <w> <x> <y> <z>\r\n"
      "   - Configure a OneWire/pulse input.\r\n"
      "     - v : channel index\r\n"
      "     - w : channel active. a = 0: DISABLED, a = 1: ENABLED\r\n"
      "     - x : function select. w = [b,f,r]: pulse, w = o: OneWire.\r\n"
      "     - y : pull-up. y = 0: OFF, y = 1: ON. Ignored if x = o\r\n"
      "     - z : minimum period (ms). Ignored if x = o\r\n"
      " - n<n>        : set node ID [1..60]\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - r           : restore defaults\r\n"
      " - s           : save settings to NVM\r\n"
      " - t           : trigger report on next cycle\r\n"
      " - v           : firmware and board information\r\n"
      " - w<n>        : RF active. n = 0: OFF, n = 1: ON\r\n"
      " - x<n>        : 433 MHz compatibility. n = 0: 433.92 MHz, n = 1: "
      "433.00 MHz\r\n"
      " - z           : zero energy accumulators\r\n\r\n";

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
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'b':
    configureBackup();
    break;
  case 'c':
    if (configureSerialLog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'd':
    if (configureDatalog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
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
      resetReq      = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'g':
    if (configureGroupID()) {
      rfmSetGroupID(config.baseCfg.dataGrp);
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'j':
    if (configureJSON()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'k':
    if (configureAnalog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    if (configureOPA()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'o':
    /* Start auto calibration of CT<x> lead */
    serialPuts("> Reserved for auto calibration. Not yet implemented.\r\n");
    break;
  case 'n':
    /* Set the node ID */
    if (configureNodeID()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'p':
    /* Configure RF power */
    if (configureRFPower()) {
      rfmSetPowerLevel(config.dataTxCfg.rfmPwr);
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'r':
    configDefault();

    serialPuts("> Restored default values.\r\n");

    unsavedChange = true;
    resetReq      = true;
    emon32EventSet(EVT_CONFIG_CHANGED);
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
    if (!resetReq) {
      emon32EventSet(EVT_CONFIG_SAVED);
    } else {
      emon32EventSet(EVT_SAFE_RESET_REQ);
    }
    break;
  case 't':
    emon32EventSet(EVT_ECM_TRIG);
    break;
  case 'v':
    configFirmwareBoardInfo();
    break;
  case 'w':
    if (configureRFEnable()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;

  case 'x':
    if (configureRF433()) {
      rfmSetFrequency(config.dataTxCfg.rfmFreq);
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'z':
    if (zeroAccumulators()) {
      emon32EventSet(EVT_CLEAR_ACCUM);
    }
    break;
  }

  cmdPending = false;
  inBufferClear(arglen + 1);
}

int configTimeToCycles(const float time, const int mainsFreq) {
  return qfp_float2uint(qfp_fmul(time, qfp_int2float(mainsFreq)));
}

VersionInfo_t configVersion(void) {
  struct Emon32BuildInfo binfo = emon32_build_info();
  return (VersionInfo_t){.version = binfo.version, .revision = binfo.revision};
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_INTERACTIVE_HANDLER {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */
  if (uartGetcReady(SERCOM_UART_INTERACTIVE)) {
    uint8_t rx_char = uartGetc(SERCOM_UART_INTERACTIVE);
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(SERCOM_UART_INTERACTIVE, rx_char);
    }
  }

  /* Revisit : need to handle the Error interrupt? */
}
