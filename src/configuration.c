#include <inttypes.h>
#include <string.h>

#include "emon32_assert.h"

#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "configuration_help.h"
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
  CONFIRM_RESET,
  CONFIRM_SHUTDOWN_PI
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

typedef struct {
  char  *argv[10];
  size_t argc;
} CmdArgs_t;

/* SAM-BA bootloader has "uino" at word aligned address 0xCA8 (part of
 * "Arduino"). This is a little fragile as it assumes that the bootloader will
 * not change - I don't intend to change, but need to be aware as the offset
 * is likely to change for different bootloader or compiler. */
#define BL_SERIAL_MAGIC_ADDR 0x00000CA8u
#define BL_SERIAL_MAGIC_WORD 0x6F6E6975u

/*************************************
 * Prototypes
 *************************************/

static bool      configCheckUnsaved(void);
static void      configDefault(void);
static void      configEchoQueueChar(const uint8_t c);
static void      configEchoQueueStr(const char *s);
static void      configInitialiseNVM(void);
static uint16_t  configTimeToCycles(const float time, const uint32_t mainsFreq);
static bool      configureVCTChannel(void);
static bool      configureAssumed(void);
static bool      configureAuto(void);
static void      configureAccumulatorSet(void);
static void      configureBackup(void);
static bool      configureDatalog(void);
static bool      configureGroupID(void);
static bool      configureJSON(void);
static bool      configureLineFrequency(void);
static bool      configure1WAddr(void);
static bool      configure1WAddrClear(void);
static void      configure1WFind(void);
static bool      configure1WFreeze(void);
static void      configure1WList(void);
static void      configure1WListSaved(void);
static bool      configure1WRemap(void);
static bool      configure1WSave(void);
static bool      configureOPA(void);
static bool      configureNodeID(void);
static void      configureReconfigureAll(void);
static void      configureRestore(void);
static bool      configureRFEnable(void);
static bool      configureRF433(void);
static bool      configureRFPower(void);
static bool      configureSerialLog(void);
static void      confirmationClear(void);
static void      confirmationStart(ConfirmState_t state);
static void      enterBootloader(void);
static uint32_t  getBoardRevision(void);
static char     *getLastReset(void);
static void      handleConfirmation(char c);
static void      inBufferClear(const size_t n);
static CmdArgs_t inBufferTok(void);
static void      parseAndZeroAccumulator(void);
static void      printSettingCT(const size_t ch, bool fromNvm);
static void      printSettingDatalog(bool fromNvm);
static void      printSettingJSON(bool fromNvm);
static void      printSettingOPA(const size_t ch, bool fromNvm);
static void      printSettingRF(bool fromNvm);
static void      printSettingRFFreq(bool fromNvm);
static void      printSettingSerial(bool fromNvm);
static void      printSettingV(const size_t ch, bool fromNvm);
static void      printSettings(void);
static void      printSettingsHR(bool fromNvm);
static void      printSettingsKV(bool fromNvm);
static void      printUptime(void);
static uint32_t  readWordAtAddress(uintptr_t address);
static void      resetRequest(void);
static void      saveToNVM(void);
static void      shutdownPi(void);
static void      zeroAccumulators(void);

/*************************************
 * Constants
 *************************************/

#define IN_BUFFER_W  64u
#define ERROR_PREFIX "> Error: "

/*************************************
 * Error output
 *************************************/

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

/*************************************
 * Module state
 *************************************/

static Emon32Config_t config;
static Emon32Config_t config_nvm;
static char           inBuffer[IN_BUFFER_W];
static CmdArgs_t     cmdArgs;

static AutoConfig_t autocfg = {0};

/* Async confirmation state */
static volatile ConfirmState_t confirmState        = CONFIRM_IDLE;
static volatile uint32_t       confirmStartTime_ms = 0;
static uint8_t                 clearAccumIdx =
    UINT8_MAX; /* UINT8_MAX=all, 0-11=E1-E12, 12-13=P1-P2 */
static size_t inBufferIdx   = 0;
static bool   cmdPending    = false;
static bool   unsavedChange = false;

static bool configCheckUnsaved(void) {
  return (0 != memcmp(&config, &config_nvm, sizeof(config)));
}

/*************************************
 * Defaults and NVM initialisation
 *************************************/

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
  for (size_t idxCT = 0u; idxCT < NUM_CT; idxCT++) {
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
  config.opaCfg[2].period    = PULSE_BLANK_DEF;
  config.opaCfg[2].puEn      = false;

  config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2u));
}

/*! @brief Write the configuration values to index 0, and clear accumulators */
static void configInitialiseNVM(void) {

  serialPuts("  - Initialising NVM... ");

  configDefault();
  eepromInitBlock(0, 0, EEPROM_WL_OFFSET);
  eepromInitConfig(&config, sizeof(config));
  eepromWLClear();
  serialPuts("Done!\r\n");
}

/*************************************
 * Measurement channel commands
 *************************************/

static bool configureVCTChannel(void) {
  /* String format: k<x> <a> <y.y> <z.z> v1 v2
   * Find space delimiters, then convert to null and a->i/f
   */
  ConvFloat_t convF    = {false, 0.0f};
  ConvUint_t  convU    = {false, {0}};
  uint32_t    ch       = 0;
  bool        active   = false;
  float       calAmpl  = 0.0f;
  float       calPhase = 0.0f;
  uint8_t     vCh1     = 0;
  uint8_t     vCh2     = 0;
  ECMCfg_t   *ecmCfg   = 0;

  /* All or no parameters must be specified */
  if ((cmdArgs.argc != 2u) && (cmdArgs.argc != 4u) && (cmdArgs.argc != 5u) &&
      (cmdArgs.argc != 6u)) {
    serialPutsError("Missing required parameters.");
    return false;
  }

  /* Voltage channels are [1..3], CTs are [4..] but 0 indexed internally. All
   * fields must be present for a given channel type.
   */
  convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid channel number.");
    return false;
  }

  if (--convU.val.u32 >= VCT_TOTAL) {
    printfError("Channel out of range (valid: 1-%d).", VCT_TOTAL);
    return false;
  }

  ch = convU.val.u32;

  convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
  if (!convU.valid || (convU.val.u32 > 1u)) {
    serialPutsError("Invalid active value (valid: 0 or 1).");
    return false;
  }
  active = (bool)convU.val.u8;

  ecmCfg = ecmConfigGet();

  /* Exit early if just activating or deactivating */
  if (2u == cmdArgs.argc) {
    if (ch < NUM_V) {
      ecmCfg->vCfg[ch].vActive      = active;
      config.voltageCfg[ch].vActive = active;
      printSettingV(ch, false);
    } else {
      ecmCfg->ctCfg[ch - NUM_V].active  = active;
      config.ctCfg[ch - NUM_V].ctActive = active;
      printSettingCT(ch - NUM_V, false);
    }
    ecmConfigChannel(ch);
    return true;
  }

  /* CT requires at least V1 */
  if (ch >= NUM_V) {
    if (cmdArgs.argc < 5u) {
      serialPutsError("CT requires voltage channel reference.");
      return false;
    }
  }

  convF = utilAtof(cmdArgs.argv[2]);
  if (!convF.valid) {
    serialPutsError("Invalid calibration value.");
    return false;
  }
  calAmpl = convF.val;

  convF = utilAtof(cmdArgs.argv[3]);
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

    printSettingV(ch, false);

    ecmConfigChannel(ch);

    /* If the voltage phase was changed reconfigure all CTs as well */
    if (reconfigureCT) {
      for (size_t i = 0; i < NUM_CT; i++) {
        ecmConfigChannel(i + NUM_V);
      }
    }

    return true;
  }

  convU = utilAtoui(cmdArgs.argv[4], ITOA_BASE10);
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
  if (cmdArgs.argc > 5u) {
    convU = utilAtoui(cmdArgs.argv[5], ITOA_BASE10);
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

  printSettingCT(ch, false);
  ecmConfigChannel(ch + NUM_V);
  return true;
}

/*************************************
 * General configuration commands
 *************************************/

static bool configureAssumed(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid assumed voltage value.");
    return false;
  }

  ECMCfg_t *pEcmCfg          = ecmConfigGet();
  pEcmCfg->assumedVrms       = qfp_uint2float(convU.val.u32);
  config.baseCfg.assumedVrms = convU.val.u16;
  printf_("assumedV = %d\r\n", config.baseCfg.assumedVrms);
  return true;
}

static bool configureAuto(void) {
  if (3u != cmdArgs.argc) {
    serialPutsError("Auto calibration requires channel, mode, and value.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1u, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid channel index.");
    return false;
  }
  if (0 == convU.val.u8 || convU.val.u8 > VCT_TOTAL) {
    printfError("OPA channel out of range (valid: 1-%d).", VCT_TOTAL);
    return false;
  }

  if (autocfg.inProgress) {
    printfError("Automatic %s configuration in progress on channel %d.",
                ((autocfg.mode == 'a') ? "amplitude" : "phase"),
                (autocfg.ch + 1u));
    return false;
  }

  const uint32_t ch   = convU.val.u8 - 1u;
  const char     mode = cmdArgs.argv[1][0];

  if (('a' != mode) && ('p' != mode)) {
    serialPutsError("Automatic calibration must be a or p.");
    return false;
  }

  /* Select V/CT calibration, exit if inactive. */
  if (ch > (NUM_V - 1u)) {
    if (!config.ctCfg[ch - NUM_V].ctActive) {
      printfError("CT%d is not active.\r\n", ch + 1u - NUM_V);
      return false;
    }
    autocfg.isCT = true;
    autocfg.ch   = ch - NUM_V;
  } else {
    if (!config.voltageCfg[ch].vActive) {
      printfError("V%d is not active.\r\n", ch + 1u);
      return false;
    }
    autocfg.isCT = false;
    autocfg.ch   = ch;
  }

  if ('a' == mode) {
    ConvFloat_t convF = utilAtof(cmdArgs.argv[2]);
    if (!convF.valid) {
      serialPutsError("Invalid calibration value.");
      return false;
    }

    autocfg.mode   = 'a';
    autocfg.target = convF.val;
    autocfg.iter   = 0u;
    autocfg.accum  = 0.0f;

    autocfg.inProgress = true;
    serialPuts("> Calibration in progress.\r\n");
    return true;
  }

  if ('p' == mode) {
    /* Revisit : automatic phase calibration */
    return false;
  }

  return false;
}

AutoConfig_t *configAutoStatus(void) { return &autocfg; }

static void configureAccumulatorSet(void) {
  char   ep = cmdArgs.argv[0][1];
  size_t ch = 0;

  if (ep != 'e' && ep != 'p') {
    serialPutsError("Invalid accumulator selection.");
    return;
  }

  if (2u != cmdArgs.argc) {
    serialPutsError("Accumulator set requires an index and value.");
    return;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 2, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid index.");
    return;
  }
  ch = convU.val.u32;

  if ('e' == ep) {
    if (convU.val.u32 < 1u || convU.val.u32 > NUM_CT) {
      serialPutsError("Index out of range.");
      return;
    }

    ConvInt_t convI = utilAtoi(cmdArgs.argv[1], ITOA_BASE10);
    if (!convI.valid) {
      serialPutsError("Invalid energy value.");
      return;
    }

    ecmEnergySetChannel((ch - 1u), convI.val.i32);
    printf_("E%d: %ld\r\n", ch, convI.val.i32);

  } else {
    if (convU.val.u32 < 1u || convU.val.u32 > NUM_OPA) {
      serialPutsError("Index out of range.");
      return;
    }

    convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
    if (!convU.valid) {
      serialPutsError("Invalid pulse count value.");
      return;
    }
    pulseSetCount((ch - 1u), convU.val.u32);
    printf_("pulse%d: %lu\r\n", ch, convU.val.u32);
  }
}

static void configureBackup(void) {
  if (1u != cmdArgs.argc || 1u != strlen(cmdArgs.argv[0])) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  /* Send all configuration values as JSON over the serial link. */
  char strBuf[8] = {0};

  /* Open JSON block */
  serialPuts("{");
  /* {board_info} dict */
  serialPuts("\"board_info\":{");
  printf_("\"revision\":%ld,", getBoardRevision());
  printf_("\"serial\":\"0x%02lx%02lx%02lx%02lx\",", getUniqueID(0),
          getUniqueID(1), getUniqueID(2), getUniqueID(3));
  printf_("\"fw\":\"%s\"},", emon32_build_info().release);

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
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvFloat_t convF = utilAtof(cmdArgs.argv[0] + 1);
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

  printSettingDatalog(false);
  return true;
}

static bool configureGroupID(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);

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
  rfmSetGroupID(config.baseCfg.dataGrp);
  return true;
}

static bool configureJSON(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);

  if (!convU.valid) {
    serialPutsError("Invalid JSON value.");
    return false;
  }

  if (convU.val.u32 > 1) {
    serialPutsError("JSON value must be 0 or 1.");
    return false;
  }

  config.baseCfg.useJson = (bool)convU.val.u8;
  printSettingJSON(false);
  return true;
}

static bool configureLineFrequency(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
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

/*************************************
 * OneWire commands
 *************************************/

static bool configure1WAddr(void) {
  char c1 = cmdArgs.argv[0][1];
  switch (c1) {
  case 'c':
    return configure1WAddrClear();
  case 'f':
    configure1WFind();
    return false;
  case 'l':
    configure1WList();
    return false;
  case 'h':
    configure1WFreeze();
    return true;
  case 'n':
    configure1WListSaved();
    return false;
  case 'r':
    return configure1WRemap();
  case '1': case '2': case '3': case '4': case '5':
  case '6': case '7': case '8': case '9':
    return configure1WSave();
  default:
    return false;
  }
}

static bool configure1WAddrClear(void) {

  /* Clear _all_ saved addresses */
  if ('a' == cmdArgs.argv[0][2] && '\0' == cmdArgs.argv[0][3]) {
    memset(&config.oneWireAddr.addr, 0, sizeof(config.oneWireAddr.addr));
    serialPuts("> Cleared all saved 1-Wire addresses.\r\n");
    emon32EventSet(EVT_OPA_INIT);
    return true;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 2, ITOA_BASE10);
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
  serialPuts("[end]\r\n");
}

static void configure1WListSaved(void) {
  for (size_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    uint64_t as; /* Ensure 8byte alignment */
    memcpy(&as, &config.oneWireAddr.addr[i], sizeof(as));
    printf_("[%u] ", i + 1u);
    for (size_t j = 0; j < 8; j++) {
      printf_("%02x%s", (uint8_t)((as >> (8 * j)) & 0xFF),
              ((j == 7) ? "\r\n" : " "));
    }
  }
  serialPuts("[end]\r\n");
}

static bool configure1WRemap(void) {
  if (3u != cmdArgs.argc) {
    serialPutsError("1-Wire remap requires two parameters.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid 1-Wire channel value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > (TEMP_MAX_ONEWIRE))) {
    printfError("1-Wire channel out of range (valid: 1-%d).", TEMP_MAX_ONEWIRE);
    return false;
  }
  const size_t ch = convU.val.u32 - 1u;

  convU = utilAtoui(cmdArgs.argv[2], ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid 1-Wire remap channel value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > (TEMP_MAX_ONEWIRE))) {
    printfError("1-Wire remap channel out of range (valid: 1-%d).",
                TEMP_MAX_ONEWIRE);
    return false;
  }
  const size_t chRemap = convU.val.u32 - 1u;

  const uint64_t *pAddr = tempAddress1WGet();
  const uint64_t  ar    = pAddr[ch];

  memcpy(&config.oneWireAddr.addr[chRemap], &ar, sizeof(ar));

  printf_("> 1W_addr%d = ", (chRemap + 1));
  for (size_t i = 0; i < 8; i++) {
    printf_("%02x%s", (uint8_t)((ar >> (8 * i)) & 0xFF),
            ((i == 7) ? "\r\n" : " "));
  }

  return true;
}

static bool configure1WSave(void) {
  size_t ch;

  if (9u != cmdArgs.argc) {
    serialPutsError("1-Wire save requires 8 address bytes.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    serialPutsError("Invalid 1-Wire channel value.");
    return false;
  }

  if ((convU.val.u32 < 1) || (convU.val.u32 > (TEMP_MAX_ONEWIRE))) {
    printfError("1-Wire channel out of range (valid: 1-%d).", TEMP_MAX_ONEWIRE);
    return false;
  }
  ch = convU.val.u32 - 1u;

  uint64_t addr = 0;
  for (size_t i = 0; i < 8; i++) {
    convU = utilAtoui(cmdArgs.argv[i + 1u], ITOA_BASE16);
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

/*************************************
 * OPA commands
 *************************************/

static bool configureOPA(void) {
  ConvUint_t convU;
  uint8_t    ch     = 0;
  bool       active = 0;
  char       func   = 0;
  bool       pu     = false;
  uint8_t    period = 0;

  if (cmdArgs.argc < 2u) {
    serialPutsError("OPA requires channel and active value.");
    return false;
  }

  /* Channel index */
  convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
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
  convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
  if ((!convU.valid) || (convU.val.u32 > 1u)) {
    serialPutsError("Invalid OPA active value.");
    return false;
  }

  active = (bool)convU.val.u8;

  if (!active || (cmdArgs.argc < 3u)) {
    config.opaCfg[ch].opaActive = active;
    printSettingOPA(ch, false);
    return true;
  }
  config.opaCfg[ch].opaActive = true;

  /* Check for the function. Must be a valid type and if a pulse must also have
   * a hysteresis period applied. */
  func = cmdArgs.argv[2][0];

  bool isAnalog  = ('a' == func);
  bool isOneWire = ('o' == func);
  bool isPulse   = ('b' == func) || ('f' == func) || ('r' == func);

  if (!(isAnalog || isPulse || isOneWire)) {
    serialPutsError("Invalid OPA function (valid: a/b/f/r/o).");
    return false;
  }

  /* OPA1,2 can only be pulse or OneWire */
  if ((ch < 2u) && isAnalog) {
    printfError("OPA%d only supports pulse and OneWire.", ch + 1u);
    return false;
  }

  /* OPA3 can only be a pulse or analog input */
  if ((2u == ch) && isOneWire) {
    serialPutsError("OPA3 only supports pulse and analog input.");
    return false;
  }

  if (isPulse) {
    if (cmdArgs.argc < 5u) {
      serialPutsError("OPA pulse requires pull-up and period values.");
      return false;
    }

    convU = utilAtoui(cmdArgs.argv[3], ITOA_BASE10);
    if (!convU.valid || (convU.val.u32 > 1u)) {
      serialPutsError("Invalid OPA pull-up value.");
      return false;
    }
    pu = (bool)convU.val.u8;

    convU = utilAtoui(cmdArgs.argv[4], ITOA_BASE10);
    if (!convU.valid) {
      serialPutsError("Invalid OPA period value.");
      return false;
    }

    period = convU.val.u8;

    /* For OPA3 only, if it _was_ an analog input before then the ADC must be
     * reconfigured for normal operation.
     */
    if (2u == ch) {
      if (config.opaCfg[2].func == 'a') {
        emon32EventSet(EVT_SMP_CFG_START);
      }
    }

    config.opaCfg[ch].period = period;
    config.opaCfg[ch].puEn   = pu;
  }

  if (isAnalog) {
    emon32EventSet(EVT_SMP_CFG_START);
  }

  config.opaCfg[ch].func = func;
  printSettingOPA(ch, false);
  return true;
}

/*************************************
 * RF and runtime configuration commands
 *************************************/

static bool configureNodeID(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
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
  if (1u != cmdArgs.argc || strlen(cmdArgs.argv[0]) > 2u) {
    return;
  }

  if ('\0' == cmdArgs.argv[0][1]) {
    configDefault();
    configureReconfigureAll();

    serialPuts("> Restored default values.\r\n");
    unsavedChange = true;
  } else if ('s' == cmdArgs.argv[0][1]) {
    (void)configLoadFromNVM();
    configureReconfigureAll();

    serialPuts("> Restored values from NVM.\r\n");
    unsavedChange = false;
  }
}

static bool configureRFEnable(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t conv = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!conv.valid || conv.val.u32 > 1u) {
    serialPutsError("RF enable must be 0 or 1.");
    return false;
  }

  config.dataTxCfg.useRFM = (bool)conv.val.u32;
  printf_("RF = %s\r\n", conv.val.u32 ? "on" : "off");

  return true;
}

static bool configureRF433(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t conv = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!conv.valid || conv.val.u32 > 1u) {
    serialPutsError("RF 433 value must be 0 or 1.");
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2) || (config.dataTxCfg.rfmFreq == 3))) {
    serialPutsError("RF 433 only applies to 433 MHz band.");
    return false;
  }

  config.dataTxCfg.rfmFreq = conv.val.u32 ? 2 : 3;

  serialPuts("rfBand = ");
  printSettingRFFreq(false);
  serialPuts(" MHz\r\n");
  rfmSetFrequency(config.dataTxCfg.rfmFreq);
  return true;
}

static bool configureRFPower(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
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
  rfmSetPowerLevel(config.dataTxCfg.rfmPwr);
  return true;
}

static bool configureSerialLog(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);

  if (!convU.valid) {
    serialPutsError("Invalid serial log value.");
    return false;
  }

  if (convU.val.u32 > 2) {
    serialPutsError("Serial log must be 0, 1, or 2.");
    return false;
  }

  config.baseCfg.logToSerial = convU.val.u8;
  printSettingSerial(false);
  return true;
}

/*************************************
 * Confirmation state
 *************************************/

static void confirmationClear(void) {
  __disable_irq();
  confirmState        = CONFIRM_IDLE;
  confirmStartTime_ms = 0;
  __enable_irq();
}

static void confirmationStart(ConfirmState_t state) {
  __disable_irq();
  confirmStartTime_ms = timerMillis();
  confirmState        = state;
  __enable_irq();
}

static void enterBootloader(void) {
  if (1u != cmdArgs.argc || 1u != strlen(cmdArgs.argv[0])) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  serialPuts("> Enter bootloader? All unsaved changes will be lost. 'y' to "
             "proceed.\r\n");
  confirmationStart(CONFIRM_BOOTLOADER);
}

/*************************************
 * Board information helpers
 *************************************/

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
  case RCAUSE_WDT:
    return "Watchdog timeout";
  case RCAUSE_EXT:
    return "External reset";
  case RCAUSE_BOD33:
    return "3V3 brownout";
  case RCAUSE_BOD12:
    return "1V2 brownout";
  case RCAUSE_POR:
    return "Power on cold reset";
  }
  return "Unknown";
}

uint32_t getUniqueID(const size_t idx) {
  /* Section 10.3.3 Serial Number */
  static const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                          0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

/*************************************
 * Input buffer parsing
 *************************************/

static void inBufferClear(const size_t n) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, n);
}

static CmdArgs_t inBufferTok(void) {
  CmdArgs_t result = {0};
  bool      inTok  = false;

  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    if ('\0' == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = '\0';
      inTok       = false;
      continue;
    }

    if (!inTok) {
      if (result.argc < (sizeof(result.argv) / sizeof(result.argv[0]))) {
        result.argv[result.argc++] = &inBuffer[i];
      }
      inTok = true;
    }
  }

  return result;
}

/*************************************
 * Settings output
 *************************************/

static void printSettingCT(const size_t ch, bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("iCal%u = ", (ch + 1));
  putFloat(pCfg->ctCfg[ch].ctCal, 0);
  printf_(", iLead%u = ", (ch + 1));
  putFloat(pCfg->ctCfg[ch].phase, 0);
  printf_(", iActive%u = %s", (ch + 1),
          pCfg->ctCfg[ch].ctActive ? "on" : "off");
  printf_(", v1Chan%u = %d, v2Chan%u = %d\r\n", (ch + 1),
          (pCfg->ctCfg[ch].vChan1 + 1), (ch + 1), (pCfg->ctCfg[ch].vChan2 + 1));
}

static void printSettingDatalog(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  serialPuts("datalog = ");
  putFloat(pCfg->baseCfg.reportTime, 0);
  serialPuts("\r\n");
}

static void printSettingJSON(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("json = %s\r\n", pCfg->baseCfg.useJson ? "on" : "off");
}

static void printSettingOPA(const size_t ch, bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("opa%d ", (ch + 1));

  /* OneWire */
  if ('o' == pCfg->opaCfg[ch].func) {
    printf_("active = %s, onewire\r\n",
            pCfg->opaCfg[ch].opaActive ? "on" : "off");
    return;
  }

  if ('a' == pCfg->opaCfg[ch].func) {
    printf_("active = %s, analog\r\n",
            pCfg->opaCfg[ch].opaActive ? "on" : "off");
    return;
  }

  /* Pulse - show edge type */
  const char *edgeStr = ('r' == pCfg->opaCfg[ch].func)   ? "rising"
                        : ('f' == pCfg->opaCfg[ch].func) ? "falling"
                                                         : "both";

  printf_("active = %s, pulse = %s, pullUp = %s, pulsePeriod = %d\r\n",
          (pCfg->opaCfg[ch].opaActive ? "on" : "off"), edgeStr,
          pCfg->opaCfg[ch].puEn ? "on" : "off", pCfg->opaCfg[ch].period);
}

static void printSettingRF(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("RF = %s, ", pCfg->dataTxCfg.useRFM ? "on" : "off");
  serialPuts("rfBand = ");
  printSettingRFFreq(fromNvm);
  serialPuts(" MHz, ");
  printf_("rfGroup = %d, ", pCfg->baseCfg.dataGrp);
  printf_("rfNode = %d, ", pCfg->baseCfg.nodeID);
  printf_("rfPower = %d, ", pCfg->dataTxCfg.rfmPwr);
  serialPuts("rfFormat = LowPowerLabs\r\n");
}

static void printSettingRFFreq(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  switch (pCfg->dataTxCfg.rfmFreq) {
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

static void printSettingSerial(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("serial = %s\r\n",
          (2u == pCfg->baseCfg.logToSerial
               ? "verbose"
               : ((1u == pCfg->baseCfg.logToSerial) ? "on" : "off")));
}

static void printSettingV(const size_t ch, bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  printf_("vCal%u = ", (ch + 1));
  putFloat(pCfg->voltageCfg[ch].voltageCal, 0);
  printf_(", vLead%u = ", (ch + 1));
  putFloat(pCfg->voltageCfg[ch].phase, 0);
  printf_(", vActive%u = %s\r\n", (ch + 1),
          pCfg->voltageCfg[ch].vActive ? "on" : "off");
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
    printf_("  E%u = %ld Wh\r\n", (i + 1), wh);
  }
  for (size_t i = 0; i < NUM_OPA; i++) {
    uint32_t pulse = eepromOK ? cumulative.pulseCnt[i] : 0;
    printf_("  pulse%u = %lu\r\n", (i + 1), pulse);
  }
  serialPuts("\r\n");
}

static void printSettings(void) {
  const size_t len = strlen(cmdArgs.argv[0]);

  if (len > 3u) {
    return;
  }

  if ('h' == cmdArgs.argv[0][1]) {
    if ('s' == cmdArgs.argv[0][2]) {
      printSettingsHR(true);
    } else if ('\0' == cmdArgs.argv[0][2]) {
      printSettingsHR(false);
    } else {
      return;
    }
    printAccumulators();
  } else if ('s' == cmdArgs.argv[0][1]) {
    printSettingsKV(true);
  } else if ('\0' == cmdArgs.argv[0][1]) {
    printSettingsKV(false);
  } else {
    return;
  }

  if (unsavedChange) {
    serialPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  } else {
    serialPuts("All settings saved.\r\n\r\n");
  }
}

static void printSettingsHR(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  serialPuts("\r\n\r\n==== Settings ====\r\n\r\n");
  printf_("Mains frequency (Hz):      %d\r\n", pCfg->baseCfg.mainsFreq);
  serialPuts("Data log time (s):         ");
  putFloat(pCfg->baseCfg.reportTime, 0);
  serialPuts("\r\nData transmission:         ");
  if (pCfg->dataTxCfg.useRFM) {
    serialPuts("RFM69, ");
    printSettingRFFreq(fromNvm);
    printf_(" MHz @ %ddBm\r\n", (-18 + pCfg->dataTxCfg.rfmPwr));
    printf_("  - Data group:            %d\r\n", pCfg->baseCfg.dataGrp);
    printf_("  - Node ID:               %d\r\n", pCfg->baseCfg.nodeID);
  }
  printf_("Serial:                    %s\r\n",
          (2u == pCfg->baseCfg.logToSerial
               ? "Verbose"
               : ((1u == pCfg->baseCfg.logToSerial) ? "On" : "Off")));
  printf_("Data format:               %s\r\n",
          pCfg->baseCfg.useJson ? "JSON" : "Key:Value");
  serialPuts("\r\n");

  for (size_t i = 0; i < NUM_OPA; i++) {
    bool enabled = pCfg->opaCfg[i].opaActive;
    printf_("OPA %u (%sactive)\r\n", (i + 1), enabled ? "" : "in");
    if ('o' == pCfg->opaCfg[i].func) {
      serialPuts("  - OneWire interface\r\n");
    } else {
      printf_("  - Hysteresis (ms): %d\r\n", pCfg->opaCfg[i].period);
      serialPuts("  - Edge:            ");
      switch (pCfg->opaCfg[i].func) {
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
              pCfg->opaCfg[i].puEn ? "Yes" : "No");
    }
    serialPuts("\r\n");
  }

  printf_("Assumed RMS voltage: %d V\r\n\r\n", pCfg->baseCfg.assumedVrms);

  serialPuts(
      "| Ref | Channel | Active | Calibration |  Phase  | In 1 | In 2 |\r\n");
  serialPuts(
      "+=====+=========+========+=============+=========+======+======+\r\n");
  for (size_t i = 0; i < NUM_V; i++) {
    printf_("| %2d  |  V %2d   | %c      | ", (i + 1), (i + 1),
            (pCfg->voltageCfg[i].vActive ? 'Y' : 'N'));
    putFloat(pCfg->voltageCfg[i].voltageCal, 6);
    serialPuts("      |  ");
    putFloat(pCfg->voltageCfg[i].phase, 6);
    serialPuts(" |      |      |\r\n");
  }
  for (size_t i = 0; i < NUM_CT; i++) {
    printf_("| %2d  | CT %2d   | %c      | ", (i + 1 + NUM_V), (i + 1),
            (pCfg->ctCfg[i].ctActive ? 'Y' : 'N'));
    putFloat(pCfg->ctCfg[i].ctCal, 6);
    serialPuts("      |  ");
    putFloat(pCfg->ctCfg[i].phase, 6);
    printf_(" | %d    | %d    |\r\n", (pCfg->ctCfg[i].vChan1 + 1),
            (pCfg->ctCfg[i].vChan2 + 1));
  }
  serialPuts("\r\n");
}

static void printSettingsKV(bool fromNvm) {
  Emon32Config_t *pCfg = fromNvm ? &config_nvm : &config;

  serialPuts("hardware = emonPi3\r\n");
  printf_("hardware_rev = %lu\r\n", getBoardRevision());
  printf_("version = %s\r\n", emon32_build_info().release);
  printf_("commit = %s\r\n", emon32_build_info().revision);
  printf_("bootloader = %s\r\n",
          (readWordAtAddress(BL_SERIAL_MAGIC_ADDR) == BL_SERIAL_MAGIC_WORD)
              ? "uart"
              : "usb");
  printf_("assumedV = %d\r\n", pCfg->baseCfg.assumedVrms);
  for (size_t i = 0; i < NUM_V; i++) {
    printSettingV(i, fromNvm);
  }
  for (size_t i = 0; i < NUM_CT; i++) {
    printSettingCT(i, fromNvm);
  }
  for (size_t i = 0; i < NUM_OPA; i++) {
    printSettingOPA(i, fromNvm);
  }
  printSettingRF(fromNvm);
  printSettingSerial(fromNvm);
  printSettingDatalog(fromNvm);
  printSettingJSON(fromNvm);
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

static uint32_t readWordAtAddress(uintptr_t address) {
  uint32_t value;

  /* Read from an absolute flash address without provoking GCC's bounds check */
  __asm__ volatile("ldr %0, [%1]" : "=r"(value) : "r"(address) : "memory");

  return value;
}

/*************************************
 * Runtime actions
 *************************************/

static void resetRequest(void) {
  if (1u != cmdArgs.argc || 1u != strlen(cmdArgs.argv[0])) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  serialPuts(
      "> Reset system? All unsaved changes will be lost. 'y' to proceed.\r\n");
  confirmationStart(CONFIRM_RESET);
}

static void saveToNVM(void) {
  if (1u != cmdArgs.argc || 1u != strlen(cmdArgs.argv[0])) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  if (unsavedChange) {
    memcpy(&config_nvm, &config, sizeof(config));
    config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2));

    serialPuts("> Saving configuration to NVM... ");
    eepromInitConfig(&config, sizeof(config));
    serialPuts("Done!\r\n");

    unsavedChange = false;
  } else {
    serialPuts("> No changes to save.\r\n");
  }
}

static void shutdownPi(void) {
  if (1u != cmdArgs.argc || 1u != strlen(cmdArgs.argv[0])) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  serialPuts("> Shut down Raspberry Pi? 'y' to proceed.\r\n");
  if (1u == getBoardRevision()) {
    serialPuts("> This will not remove power, only indicate when safe.\r\n");
  }
  confirmationStart(CONFIRM_SHUTDOWN_PI);
}

/*************************************
 * Confirmation handling
 *************************************/

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
  volatile uint32_t    *p_blsm;
  static const uint32_t blsm_key = 0xF01669EF;

  switch (confirmState) {
  case CONFIRM_BOOTLOADER:
    if ('y' == c) {
      p_blsm  = (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
      *p_blsm = blsm_key;
      NVIC_SystemReset();
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    confirmationClear();
    break;

  case CONFIRM_RESET:
    if ('y' == c) {
      NVIC_SystemReset();
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    confirmationClear();
    break;

  case CONFIRM_ZERO_ACCUM:
    if ('y' == c) {
      serialPuts("    - Clearing accumulators...\r\n");
      emon32EventSet(EVT_CLEAR_ACCUM);
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    confirmationClear();
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
          ecmEnergyClearChannel(clearAccumIdx);
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
    clearAccumIdx = UINT8_MAX;
    confirmationClear();
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

  case CONFIRM_SHUTDOWN_PI:
    if ('y' == c) {
      serialPuts("    - Shutting down.\r\n");
      emon32EventSet(EVT_PI_SHUTDOWN);
    } else {
      serialPuts("    - Cancelled.\r\n");
    }
    confirmationClear();
    break;

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
    confirmationClear();
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
  confirmationStart(CONFIRM_ZERO_ACCUM);
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
  confirmationStart(CONFIRM_ZERO_ACCUM_INDIVIDUAL);
}

/*! @brief Parse z command and zero accumulators (z, ze1-12, zp1-3) */
static void parseAndZeroAccumulator(void) {
  if (1u != cmdArgs.argc) {
    serialPutsError("Unexpected arguments.");
    return;
  }

  const char *cmd = cmdArgs.argv[0];

  /* z - zero all */
  if (cmd[1] == '\0') {
    zeroAccumulators();
    return;
  }

  /* ze<N> - zero energy accumulator */
  if (cmd[1] == 'e') {
    ConvUint_t conv = utilAtoui(cmd + 2, ITOA_BASE10);
    if (!conv.valid || conv.val.u32 < 1u || conv.val.u32 > NUM_CT) {
      printfError("Invalid energy accumulator index (valid: ze1-%d).", NUM_CT);
      return;
    }
    zeroAccumulatorIndividual(conv.val.u32 - 1u);
    return;
  }

  /* zp<N> - zero pulse accumulator */
  if (cmd[1] == 'p') {
    ConvUint_t conv = utilAtoui(cmd + 2, ITOA_BASE10);
    if (!conv.valid || conv.val.u32 < 1u || conv.val.u32 > NUM_OPA) {
      printfError("Invalid pulse accumulator index (valid: zp1-%d).", NUM_OPA);
      return;
    }
    zeroAccumulatorIndividual(NUM_CT + conv.val.u32 - 1u);
    return;
  }

  /* Invalid format */
  serialPutsError("Invalid command. Use z, ze1-12, or zp1-3.");
}

/*************************************
 * Command input
 *************************************/

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

/*************************************
 * Board and firmware report
 *************************************/

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
  printf_("  - Bootloader: %s\r\n",
          (readWordAtAddress(BL_SERIAL_MAGIC_ADDR) == BL_SERIAL_MAGIC_WORD)
              ? "UART (.bin)"
              : "USB (.uf2)");
  printf_("  - Version:    %s\r\n", emon32_build_info().release);
  serialPuts("  - Build:      ");
  serialPuts(emon32_build_info_string());
  serialPuts("\r\n\r\n");
  serialPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  serialPuts("  - emon32 Copyright (C) 2023-26 Angus Logan\r\n");
  serialPuts("  - See CONTRIBUTORS.md\r\n");
  serialPuts("  - For Bear and Moose\r\n\r\n");
}

/*************************************
 * Public configuration API
 *************************************/

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

  /* Copy loaded NVM config into shadow for shared value access */
  memcpy(&config_nvm, &config, sizeof(config));
  return &config;
}

void configProcessCmd(void) {
  cmdArgs = inBufferTok();

  if (0 == cmdArgs.argc) {
    cmdPending = false;
    inBufferClear(IN_BUFFER_W);
    return;
  }

  switch (cmdArgs.argv[0][0]) {
  case '?':
    if (1u == cmdArgs.argc && 1u == strlen(cmdArgs.argv[0])) {
      serialPuts(configHelpText);
    } else {
      serialPutsError("Unexpected arguments.");
    }
    break;
  case 'a':
    unsavedChange = configureAssumed();
    break;
  case 'b':
    configureBackup();
    break;
  case 'c':
    unsavedChange = configureSerialLog();
    break;
  case 'd':
    unsavedChange = configureDatalog();
    break;
  case 'e':
    enterBootloader();
    break;
  case 'f':
    unsavedChange = configureLineFrequency();
    break;
  case 'g':
    unsavedChange = configureGroupID();
    break;
  case 'h':
    shutdownPi();
    break;
  case 'i':
    unsavedChange = configureAuto();
    break;
  case 'j':
    unsavedChange = configureJSON();
    break;
  case 'k':
    unsavedChange = configureVCTChannel();
    break;
  case 'l':
    if (1u == cmdArgs.argc) {
      printSettings();
    } else {
      serialPutsError("Unexpected arguments.");
    }
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
    unsavedChange = configureNodeID();
    break;
  case 'p':
    unsavedChange = configureRFPower();
    break;
  case 'q':
    resetRequest();
    break;
  case 'r':
    configureRestore();
    break;
  case 's':
    saveToNVM();
    break;
  case 't':
    if (1u == cmdArgs.argc && 1u == strlen(cmdArgs.argv[0])) {
      emon32EventSet(EVT_ECM_TRIG);
    } else {
      serialPutsError("Unexpected arguments.");
    }
    break;
  case 'u':
    if (1u == cmdArgs.argc && 1u == strlen(cmdArgs.argv[0])) {
      emon32EventSet(EVT_STORE_ACCUM);
    } else {
      serialPutsError("Unexpected arguments.");
    }
    break;
  case 'v':
    if (1u == cmdArgs.argc && 1u == strlen(cmdArgs.argv[0])) {
      configFirmwareBoardInfo();
    } else {
      serialPutsError("Unexpected arguments.");
    }
    break;
  case 'w':
    unsavedChange = configureRFEnable();
    break;
  case 'x':
    unsavedChange = configureRF433();
    break;
  case 'y':
    configureAccumulatorSet();
    break;
  case 'z':
    parseAndZeroAccumulator();
    break;
  default:
    break;
  }

  unsavedChange = configCheckUnsaved();
  cmdPending    = false;
  inBufferClear(IN_BUFFER_W);
}

bool configUnsavedChanges(void) { return unsavedChange; }

uint16_t configTimeToCycles(const float time, const uint32_t mainsFreq) {
  return (uint16_t)qfp_float2uint(qfp_fmul(time, qfp_uint2float(mainsFreq)));
}

VersionInfo_t configVersion(void) {
  struct Emon32BuildInfo binfo = emon32_build_info();
  return (VersionInfo_t){.release = binfo.release, .revision = binfo.revision};
}

/*************************************
 * RX and echo queues
 *************************************/

#define ECHO_BUF_DEPTH 16u
#define ECHO_IDX_MASK  (ECHO_BUF_DEPTH - 1u)
#define ECHO_FMASK     ((ECHO_IDX_MASK << 1) + 1u)
#define RX_BUF_DEPTH   64u
#define RX_IDX_MASK    (RX_BUF_DEPTH - 1u)
#define RX_FMASK       ((RX_IDX_MASK << 1) + 1u)

static size_t          idxEchoWr               = 0;
static size_t          idxEchoRd               = 0;
static uint8_t         echoBuf[ECHO_BUF_DEPTH] = {0};
static volatile size_t idxRxWr                 = 0;
static volatile size_t idxRxRd                 = 0;
static uint8_t         rxBuf[RX_BUF_DEPTH]     = {0};

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

void configRxQueueChar(const uint8_t c) {
  __disable_irq();
  const size_t nextWr = (idxRxWr + 1u) & RX_FMASK;
  if (nextWr != idxRxRd) {
    rxBuf[idxRxWr & RX_IDX_MASK] = c;
    idxRxWr                      = nextWr;
  }
  __enable_irq();

  emon32EventSet(EVT_PROCESS_RX_CHAR);
}

void configRxProcess(void) {
  for (;;) {
    bool    charPending = false;
    uint8_t c           = 0;

    __disable_irq();
    if (idxRxRd != idxRxWr) {
      c           = rxBuf[idxRxRd & RX_IDX_MASK];
      idxRxRd     = (idxRxRd + 1u) & RX_FMASK;
      charPending = true;
    }
    __enable_irq();

    if (!charPending) {
      break;
    }

    if (!configHandleConfirmation(c)) {
      configCmdChar(c);
    }
  }
}
