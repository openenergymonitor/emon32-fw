#include <stddef.h>
#include <string.h>

#include "emon32_samd.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EIC.h"
#include "driver_EVSYS.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_USB.h"

#include "configuration.h"
#include "dataPack.h"
#include "eeprom.h"
#include "emon32.h"
#include "emon32_assert.h"
#include "emon_CM.h"
#include "periph_DS18B20.h"
#include "periph_SSD1306.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "ui.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

typedef struct EPAccum_ {
  int32_t  E; /* Energy */
  uint32_t P; /* Pulse */
} EPAccum_t;

typedef struct TransmitOpt_ {
  bool    json;      /* Use JSON format */
  bool    useRFM;    /* Use wireless */
  bool    logSerial; /* Log to serial */
  uint8_t node;      /*  Node ID */
} TransmitOpt_t;

typedef struct TxBlink_ {
  bool     txIndicate; /* Tx in progress */
  uint32_t timeBlink;  /* Time to blink LED for */
} TxBlink_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t evtPend       = 0;
AssertInfo_t             g_assert_info = {0};
static EPAccum_t         lastStoredEP  = {0};
static TxBlink_t         txBlink       = {0};
Emon32Config_t          *pConfig       = 0;

/*************************************
 * Static function prototypes
 *************************************/

static void cumulativeNVMLoad(Emon32Cumulative_t *pPkt, Emon32Dataset_t *pData);
static void cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                               const Emon32Dataset_t *pData, bool blocking);
static void cumulativeProcess(Emon32Cumulative_t    *pPkt,
                              const Emon32Dataset_t *pData,
                              const uint32_t         epDeltaStore);
static void datasetAddPulse(Emon32Dataset_t *pDst);
static void ecmConfigure(void);
static void ecmDmaCallback(void);
static void evtKiloHertz(void);
static bool evtPending(EVTSRC_t evt);
static void pulseConfigure(void);
void        putchar_(char c);
static void rfmConfigure(void);
static void ssd1306Setup(void);
static void tempReadEvt(Emon32Dataset_t *pData, const uint32_t numT);
static uint32_t tempSetup(Emon32Dataset_t *pData);
static void     totalEnergy(const Emon32Dataset_t *pData, EPAccum_t *pAcc);
static void transmitData(const Emon32Dataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void ucSetup(void);
static void waitWithUSB(uint32_t t_ms);

/*************************************
 * Functions
 *************************************/

/*! @brief Load cumulative energy and pulse values from NVM
 *  @param [in] pPkt : pointer to cumulative energy structure
 *  @param [out] pData : pointer to current dataset
 *  @return total Wh stored in NVM
 */
static void cumulativeNVMLoad(Emon32Cumulative_t *pPkt,
                              Emon32Dataset_t    *pData) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  uint32_t  totalP   = 0;
  uint32_t  totalWh  = 0;
  bool      eepromOK = false;
  ECMCfg_t *ecmCfg   = ecmConfigGet();

  eepromWLReset(sizeof(*pPkt));
  eepromOK = (EEPROM_WL_OK == eepromReadWL(pPkt, 0));

  for (uint32_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
    uint32_t wh = eepromOK ? pPkt->wattHour[idxCT] : 0;

    ecmCfg->ctCfg[idxCT].wattHourInit = wh;
    totalWh += wh;
  }

  for (uint32_t idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    uint32_t pulse = eepromOK ? pPkt->pulseCnt[idxPulse] : 0;

    pData->pulseCnt[idxPulse] = pulse;
    totalP += pulse;
  }

  lastStoredEP.E = totalWh;
  lastStoredEP.P = totalP;
}

/*! @brief Store cumulative energy and pulse values
 *  @param [in] pPkt : pointer to cumulative values
 *  @param [in] pData : pointer to current dataset
 */
static void cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                               const Emon32Dataset_t *pData, bool blocking) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  for (uint32_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
    pPkt->wattHour[idxCT] = pData->pECM->CT[idxCT].wattHour;
  }

  for (uint32_t idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    pPkt->pulseCnt[idxPulse] = pData->pulseCnt[idxPulse];
  }

  if (blocking) {
    /* Blocking write - use before system reset to ensure data is saved */
    eepromWriteWL(pPkt, 0);
  } else {
    /* Async write with hardware timer callbacks to avoid blocking */
    eepromWrStatus_t status = eepromWriteWLAsync(pPkt, 0);
    if (status == EEPROM_WR_BUSY) {
      /* Previous async write still in progress - this is expected occasionally
       */
      debugPuts("EEPROM async write skipped (busy)\r\n");
    } else if (status == EEPROM_WR_FAIL) {
      /* Failed to start async write - callback queue full or other error */
      debugPuts("EEPROM async write failed!\r\n");
    }
    /* EEPROM_WR_PEND is success - write has been queued */
  }
}

/*! @brief Calculate the cumulative energy consumption and store if the delta
 *         since last storage is greater than a configurable threshold
 *  @param [in] pPkt : pointer to an NVM packet
 *  @param [in] pData : pointer to the current dataset
 *  @param [in] epDeltaStore : Wh/pulse delta between stores to NVM
 */
static void cumulativeProcess(Emon32Cumulative_t    *pPkt,
                              const Emon32Dataset_t *pData,
                              const uint32_t         epDeltaStore) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  EPAccum_t ep = {0};
  totalEnergy(pData, &ep);

  /* Catch overflow of energy. This corresponds to ~2 GWh(!), so unlikely to
   * but handle safely.
   */
  const uint32_t absE     = utilAbs(ep.E);
  const uint32_t absELast = utilAbs(lastStoredEP.E);

  const bool epOverflow =
      ((absE + absELast) > INT32_MAX) || (ep.P < lastStoredEP.P);

  /* Treat 1 pulse == 1 Wh; not true in general case but reasonable criterion
   * for storage threshold. */
  const uint32_t deltaPulse = ep.P - lastStoredEP.P;
  const uint32_t deltaWh    = absE - absELast;

  /* Store cumulative values if over thresholds or overflow */
  if ((deltaWh >= epDeltaStore) || (deltaPulse >= epDeltaStore) || epOverflow) {
    cumulativeNVMStore(pPkt, pData, false);
    lastStoredEP.E = ep.E;
    lastStoredEP.P = ep.P;
  }
}

/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void datasetAddPulse(Emon32Dataset_t *pDst) {
  EMON32_ASSERT(pDst);
  for (uint32_t i = 0; i < NUM_OPA; i++) {
    pDst->pulseCnt[i] = pulseGetCount(i);
  }
}

void debugPuts(const char *s) {
  if (pConfig->baseCfg.debugSerial) {
    char tBuf[12];
    serialPuts("DBG:");
    utilItoa(tBuf, timerMillis(), ITOA_BASE10);
    serialPuts(tBuf);
    serialPuts(":");
    serialPuts(s);
  }
}

/*! @brief Configure the continuous energy monitoring system
 *  @param [in] pCfg : pointer to the configuration struct
 */
void ecmConfigure(void) {
  /* Makes the continuous monitoring setup agnostic to the data strcuture
   * used for storage, and avoids any awkward alignment from packing.
   */

  extern const int_fast8_t ainRemap[NUM_CT];

  ECMCfg_t *ecmCfg = ecmConfigGet();

  ecmCfg->reportCycles  = pConfig->baseCfg.reportCycles;
  ecmCfg->mainsFreq     = pConfig->baseCfg.mainsFreq;
  ecmCfg->samplePeriod  = timerADCPeriod();
  ecmCfg->reportTime_us = (1000000 / ecmCfg->mainsFreq) * ecmCfg->reportCycles;
  ecmCfg->assumedVrms   = qfp_uint2float(pConfig->baseCfg.assumedVrms);
  ecmCfg->timeMicros    = &timerMicros;
  ecmCfg->timeMicrosDelta = &timerMicrosDelta;

  if (adcCorrectionValid()) {
    ecmCfg->correction.valid  = true;
    ecmCfg->correction.gain   = adcCorrectionGain();
    ecmCfg->correction.offset = adcCorrectionOffset();
  } else {
    ecmCfg->correction.valid = false;
  }

  for (uint32_t i = 0; i < NUM_V; i++) {
    ecmCfg->vCfg[i].voltageCalRaw = pConfig->voltageCfg[i].voltageCal;
    ecmCfg->vCfg[i].vActive       = pConfig->voltageCfg[i].vActive;
  }

  for (uint32_t i = 0; i < NUM_CT; i++) {
    ecmCfg->ctCfg[i].phCal    = pConfig->ctCfg[i].phase;
    ecmCfg->ctCfg[i].ctCalRaw = pConfig->ctCfg[i].ctCal;
    ecmCfg->ctCfg[i].active   = pConfig->ctCfg[i].ctActive;
    ecmCfg->ctCfg[i].vChan1   = pConfig->ctCfg[i].vChan1;
    ecmCfg->ctCfg[i].vChan2   = pConfig->ctCfg[i].vChan2;
  }

  for (int32_t i = 0; i < NUM_CT; i++) {
    ecmCfg->mapCTLog[i] = ainRemap[i];
  }

  ecmConfigInit();
}

void ecmDmaCallback(void) {
  ECM_STATUS_t injectStatus;
  ecmDataBufferSwap();
  injectStatus = ecmInjectSample();
  switch (injectStatus) {
  case ECM_REPORT_COMPLETE:
    emon32EventSet(EVT_ECM_SET_CMPL);
    break;
  case ECM_PEND_1S:
    emon32EventSet(EVT_ECM_PEND_1S);
    break;
  default:
    break;
  }
}

void emon32EventClr(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = ~(1u << evt);
  __disable_irq();
  evtPend &= evtDecode;
  __enable_irq();
}

void emon32EventSet(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = (1u << evt);
  __disable_irq();
  evtPend |= evtDecode;
  __enable_irq();
}

/*! @brief This function is called when the 1 ms timer fires.
 *         Latency is not guaranteed, so only non-timing critical things
 *         should be done here (UI update, watchdog etc)
 */
static void evtKiloHertz(void) {
  uint32_t                 msDelta;
  static volatile uint32_t msLast = 0;

  /* Update the pulse counters, looking on different edges */
  pulseUpdate();

  /* Blink LED red for TX_INDICATE_T before going back to green */
  if (txBlink.txIndicate) {
    if (timerMillisDelta(txBlink.timeBlink) > TX_INDICATE_T) {
      txBlink.txIndicate = false;
    }
  } else {
    if (configUnsavedChanges()) {
      uiLedColour(LED_YELLOW);
    } else {
      uiLedColour(LED_GREEN);
    }
  }

  /* Track milliseconds to indicate uptime */
  msDelta = timerMillisDelta(msLast);
  if (msDelta >= 1000) {
    timerUptimeIncr();
    msLast = timerMillis();
    /* Account for any jitter in the 1 ms tick */
    if (msDelta > 1000) {
      msDelta -= 1000;
      msLast -= msDelta;
    }
  }
}

/*! @brief Check if an event source is active
 *  @param [in] : event source to check
 *  @return true if pending, false otherwise
 */
static bool evtPending(EVTSRC_t evt) { return (evtPend & (1u << evt)) != 0; }

/*! @brief Configure any pulse counter interfaces
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void pulseConfigure(void) {

  uint8_t pinsPulse[][NUM_OPA] = {
      {GRP_OPA, PIN_OPA1}, {GRP_OPA, PIN_OPA2}, {GRP_OPA, PIN_OPA3}};

  for (uint32_t i = 0; i < NUM_OPA; i++) {
    PulseCfg_t *pulseCfg = pulseGetCfg(i);

    EMON32_ASSERT(pulseCfg);

    if (('o' != pConfig->opaCfg[i].func) && (pConfig->opaCfg[i].opaActive)) {
      pulseCfg->edge    = (PulseEdge_t)pConfig->opaCfg[i].func;
      pulseCfg->grp     = pinsPulse[i][0];
      pulseCfg->pin     = pinsPulse[i][1];
      pulseCfg->periods = pConfig->opaCfg[i].period;
      pulseCfg->puEn    = pConfig->opaCfg[i].puEn;
      pulseCfg->active  = true;

      pulseInit(i);
    }
  }
}

/*! @brief Allows the printf function to print to the debug console. If the
 * USB CDC is connected, characters should be routed there.
 */
void putchar_(char c) {
  if (usbCDCIsConnected()) {
    usbCDCTxChar(c);
  }
  uartPutcBlocking(SERCOM_UART, c);
}

static void rfmConfigure(void) {
  RFMOpt_t rfmOpt = {0};
  rfmOpt.freq     = (RFM_Freq_t)pConfig->dataTxCfg.rfmFreq;
  rfmOpt.group    = pConfig->baseCfg.dataGrp;
  rfmOpt.nodeID   = pConfig->baseCfg.nodeID;
  rfmOpt.paLevel  = pConfig->dataTxCfg.rfmPwr;

  if (rfmInit(&rfmOpt)) {
    rfmSetAESKey("89txbe4p8aik5kt3"); /* Default OEM AES key */
  }
}

void serialPuts(const char *s) {
  EMON32_ASSERT(s);

  if (usbCDCIsConnected()) {
    usbCDCPutsBlocking(s);
  }
  uartPutsBlocking(SERCOM_UART, s);
}

/*! @brief Setup the SSD1306 display, if present. Display a basic message */
static void ssd1306Setup(void) {

  if (SSD1306_SUCCESS == ssd1306Init(SERCOM_I2CM_EXT)) {
    VersionInfo_t vInfo  = configVersion();
    int32_t       offset = 0;
    for (size_t i = 0; i < strlen(vInfo.revision); i++) {
      if ('-' == vInfo.revision[i]) {
        offset = -20;
      }
    }

    ssd1306SetPosition((PosXY_t){.x = 44, .y = 0});
    ssd1306DrawString("emonPi3");
    ssd1306SetPosition((PosXY_t){.x = 46, .y = 1});
    ssd1306DrawString(vInfo.version);
    ssd1306SetPosition((PosXY_t){.x = (44 + offset), .y = 2});
    ssd1306DrawString(vInfo.revision);
    ssd1306DisplayUpdate();
  }
}

static void tempReadEvt(Emon32Dataset_t *pData, const uint32_t numT) {
  static uint32_t tempRdCount;

  static uint8_t cntSinceLastValid[TEMP_MAX_ONEWIRE] = {0};
  static int16_t lastValidTemp[TEMP_MAX_ONEWIRE]     = {0};
  static bool    validTempRead[TEMP_MAX_ONEWIRE]     = {0};

  if (numT > 0) {
    TempRead_t tempValue  = tempReadSample(TEMP_INTF_ONEWIRE, tempRdCount);
    size_t     mapLogical = tempMapToLogical(TEMP_INTF_ONEWIRE, tempRdCount);

    int16_t tempData = 0;

    if (TEMP_OK == tempValue.status) {
      validTempRead[mapLogical]     = true;
      tempData                      = tempValue.temp;
      lastValidTemp[mapLogical]     = tempData;
      cntSinceLastValid[mapLogical] = 0;
    } else {
      if ((cntSinceLastValid[mapLogical] < 10) && validTempRead[mapLogical]) {
        tempData = lastValidTemp[mapLogical];
        cntSinceLastValid[mapLogical]++;
      } else {
        if (TEMP_OUT_OF_RANGE == tempValue.status) {
          tempData = 4832; /* 302°C */
        } else {
          tempData = 4864; /* 304°C */
        }
      }
    }

    pData->temp[mapLogical] = tempData;
    tempRdCount++;
  }

  if ((0 == numT) || (numT == tempRdCount)) {
    emon32EventSet(EVT_PROCESS_DATASET);
    emon32EventClr(EVT_TEMP_READ);
    tempRdCount = 0;
  }
}

/*! @brief Initialises the temperature sensors
 *  @param [in] pData : pointer to dataset to initialise
 *  @return number of temperature sensors found
 */
static uint32_t tempSetup(Emon32Dataset_t *pData) {
  const uint8_t opaPins[NUM_OPA] = {PIN_OPA1, PIN_OPA2};
  const uint8_t opaPUs[NUM_OPA]  = {PIN_OPA1_PU, PIN_OPA2_PU};

  uint32_t       numTempSensors = 0;
  DS18B20_conf_t dsCfg          = {0};
  dsCfg.grp                     = GRP_OPA;
  dsCfg.t_wait_us               = 5;

  tempInitClear();

  for (size_t i = 0; i < NUM_OPA; i++) {
    if (('o' == pConfig->opaCfg[i].func)) {

      /* If configured as OneWire device always enable the PU even if inactive
       * as an external device may be handling the port */
      portPinDrv(GRP_OPA, opaPUs[i], PIN_DRV_SET);
      portPinDir(GRP_OPA, opaPUs[i], PIN_DIR_OUT);
      portPinCfg(GRP_OPA, opaPins[i], PORT_PINCFG_PULLEN, PIN_CFG_CLR);
      portPinDrv(GRP_OPA, opaPins[i], PIN_DRV_CLR);

      if (pConfig->opaCfg[i].opaActive) {
        dsCfg.opaIdx = i;
        dsCfg.pin    = opaPins[i];
        dsCfg.pinPU  = opaPUs[i];
        numTempSensors += tempInitSensors(TEMP_INTF_ONEWIRE, &dsCfg);
      }
    }
  }

  /* 64 bit values must be 8byte aligned, not guaranteed from a packed struct */
  uint64_t addrAlign8[TEMP_MAX_ONEWIRE];
  memcpy(addrAlign8, pConfig->oneWireAddr.addr,
         sizeof(pConfig->oneWireAddr.addr));

  tempMapDevices(TEMP_INTF_ONEWIRE, addrAlign8);

  /* Set all unused temperature slots to 300°C (4800 as Q4 fixed point) */
  for (int32_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    pData->temp[i] = 4800;
  }

  return numTempSensors;
}

/*! @brief Total energy across all CTs
 *  @param [in] pData : pointer to data setup
 *  @param [out] pAcc : pointer to E/P accumulator
 */
static void totalEnergy(const Emon32Dataset_t *pData, EPAccum_t *pAcc) {
  EMON32_ASSERT(pData);
  EMON32_ASSERT(pAcc);

  pAcc->E = 0;
  pAcc->P = 0;

  for (uint32_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
    pAcc->E += pData->pECM->CT[idxCT].wattHour;
  }
  for (uint32_t idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    pAcc->P += pData->pulseCnt[idxPulse];
  }
}

static void transmitData(const Emon32Dataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  CHActive_t chsActive;

  for (size_t i = 0; i < NUM_V; i++) {
    chsActive.V[i] = pConfig->voltageCfg[i].vActive;
  }

  for (size_t i = 0; i < NUM_CT; i++) {
    chsActive.CT[i] = pConfig->ctCfg[i].ctActive;
  }

  for (size_t i = 0; i < NUM_OPA; i++) {
    uint8_t func       = pConfig->opaCfg[i].func;
    chsActive.pulse[i] = ('r' == func) || ('f' == func) || ('b' == func);
  }

  (void)dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json, chsActive);

  if (pOpt->useRFM) {

    if (pOpt->logSerial) {
      serialPuts(txBuffer);
    }

    if (sercomExtIntfEnabled()) {
      int32_t     retryCount = 0;
      RFMSend_t   rfmResult;
      int_fast8_t nPacked = dataPackPacked(pSrc, rfmGetBuffer(), PACKED_LOWER);

      rfmSetAddress(pOpt->node);

      rfmResult = rfmSendBuffer(nPacked, RFM_RETRIES, &retryCount);

      if (RFM_SUCCESS == rfmResult) {
        nPacked = dataPackPacked(pSrc, rfmGetBuffer(), PACKED_UPPER);
        rfmSetAddress(pOpt->node + 1);

        rfmResult = rfmSendBuffer(nPacked, RFM_RETRIES, &retryCount);
      }
    }

  } else {
    serialPuts(txBuffer);
  }
}

/*! @brief Setup the microcontroller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void ucSetup(void) {
  clkSetup();
  timerSetup();
  portSetup();
  eicSetup();
  dmacSetup();
  sercomSetup();
  adcSetup();
  evsysSetup();
  usbSetup();
}

static void waitWithUSB(uint32_t t_ms) {
  uint32_t t_start    = timerMillis();
  uint32_t t_last_usb = t_start;
  while (timerMillisDelta(t_start) < t_ms) {
    if (1 == timerMillisDelta(t_last_usb)) {
      tud_task();
      usbCDCTask();
      t_last_usb = timerMillis();
    }
  }
}

int main(void) {

  Emon32Dataset_t    dataset               = {0};
  uint32_t           numTempSensors        = 0;
  Emon32Cumulative_t nvmCumulative         = {0};
  char               txBuffer[TX_BUFFER_W] = {0};

  ucSetup();
  uiLedColour(LED_RED);

  /* Pause to allow any external pins to settle */
  waitWithUSB(100);
  spiConfigureExt();

  /* If the system is booted while it is connected to an active Pi, do not
   * write to the OLED or setup the RFM module. */
  if (sercomExtIntfEnabled()) {
    ssd1306Setup();
  }

  eicEnable();
  uartEnableTx(SERCOM_UART);

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration and 0 energy accumulator area.
   */
  serialPuts("> Reading configuration and accumulators from NVM...\r\n");
  pConfig = configLoadFromNVM();

  /* Load the accumulated energy and pulse values from NVM. */
  cumulativeNVMLoad(&nvmCumulative, &dataset);

  if (sercomExtIntfEnabled()) {
    rfmConfigure();
  }

  /* Set up pulse and temperature sensors, if present. */
  pulseConfigure();
  numTempSensors = tempSetup(&dataset);

  /* Wait 1s to allow USB to enumerate as serial. Not always possible, but
   * gives the possibility. The board information can be accessed through the
   * serial console later. */
  waitWithUSB(1000);
  configFirmwareBoardInfo();

  /* Set up buffers for ADC data, configure energy processing, and start */
  ecmConfigure();
  dmacCallbackBufferFill(&ecmDmaCallback);
  ecmFlush();
  adcDMACStart();
  uartEnableRx(SERCOM_UART, SERCOM_UART_INTERACTIVE_IRQn);

  if (configUnsavedChanges()) {
    uiLedColour(LED_YELLOW);
  } else {
    uiLedColour(LED_GREEN);
  }

  for (;;) {

    /* While there is an event pending (may be set while another is
     * handled), keep looping. Enter sleep (WFI) when done.
     */
    while (0 != evtPend) {

      /* External interface disable */
      if (evtPending(EVT_EXT_DISABLE)) {
        sercomExtIntfDisable();
        emon32EventClr(EVT_EXT_DISABLE);
      }

      /* 1 ms timer flag */
      if (evtPending(EVT_TICK_1kHz)) {
        tud_task();
        usbCDCTask();

        /* Process any timer callbacks that are ready */
        timerProcessPendingCallbacks();

        /* Check for confirmation timeout (30s) */
        configCheckConfirmationTimeout();

        evtKiloHertz();
        emon32EventClr(EVT_TICK_1kHz);
      }

      /* Pending character(s) in echo queue */
      if (evtPending(EVT_ECHO)) {
        uint8_t c = configEchoChar();
        while (c) {
          putchar_((char)c);
          c = configEchoChar();
        }
        emon32EventClr(EVT_ECHO);
      }

      /* Configuration request to store accumulator values to NVM on demand. */
      if (evtPending(EVT_STORE_ACCUM)) {
        cumulativeNVMStore(&nvmCumulative, &dataset, false);
        printf_("> Storing...\r\n");
        emon32EventClr(EVT_STORE_ACCUM);
      }

      /* Configuration request to clear all accumulator values (energy and
       * pulse count). The NVM is overwritten with 0s and the index of the
       * next read/write is reset. Clear the running counters in the main
       * loop, any residual energy in the dataset, and all pulse counters.
       */
      if (evtPending(EVT_CLEAR_ACCUM)) {
        lastStoredEP.E = 0;
        lastStoredEP.P = 0;
        /* REVISIT : may need to make this asynchronous as it will take 240 ms
         * (worst case) to clear the whole area for a 1KB EEPROM.
         */
        eepromWLClear();
        eepromWLReset(sizeof(nvmCumulative));
        ecmClearEnergy();
        for (size_t i = 0; i < NUM_OPA; i++) {
          pulseSetCount(i, 0);
        }
        serialPuts("    - Accumulators cleared.\r\n");
        emon32EventClr(EVT_CLEAR_ACCUM);
      }

      /* There has been a trigger request externally; the CM buffers will be
       * swapped on the next cycle. If there has been sufficient time between
       * the last temperature sample, start a temperature sample as well.
       */
      if (evtPending(EVT_ECM_TRIG)) {
        if (numTempSensors > 0) {
          for (size_t i = 0; i < NUM_OPA; i++) {
            if (('o' == pConfig->opaCfg[i].func) &&
                pConfig->opaCfg[i].opaActive) {
              (void)tempStartSample(TEMP_INTF_ONEWIRE, i);
            }
          }
        }
        ecmProcessSetTrigger();
        emon32EventClr(EVT_ECM_TRIG);
      }

      /* Trigger a temperature sample 1 s before the report is due. */
      if (evtPending(EVT_ECM_PEND_1S)) {
        if (numTempSensors > 0) {
          for (size_t i = 0; i < NUM_OPA; i++) {
            if (('o' == pConfig->opaCfg[i].func) &&
                pConfig->opaCfg[i].opaActive) {
              (void)tempStartSample(TEMP_INTF_ONEWIRE, i);
            }
          }
        }
        emon32EventClr(EVT_ECM_PEND_1S);
      }

      /* Readout has been requested, trigger a temperature read. */
      if (evtPending(EVT_ECM_SET_CMPL)) {
        emon32EventSet(EVT_TEMP_READ);
        emon32EventClr(EVT_ECM_SET_CMPL);
      }

      /* Read back samples from each DS18B20 present. This is a blocking
       * routine, and is lower priority than processing a cycle, so read
       * one sensor on each loop. When all are complete, send the data out
       * through the configured interface.
       */
      if (evtPending(EVT_TEMP_READ)) {
        tempReadEvt(&dataset, numTempSensors);
      }

      /* Report period elapsed; generate, pack, and send through the
       * configured channels.
       */
      if (evtPending(EVT_PROCESS_DATASET)) {
        TransmitOpt_t opt;
        opt.useRFM    = pConfig->dataTxCfg.useRFM;
        opt.logSerial = pConfig->baseCfg.logToSerial;
        opt.node      = pConfig->baseCfg.nodeID;
        opt.json      = pConfig->baseCfg.useJson;

        dataset.msgNum++;
        dataset.pECM = ecmProcessSet();
        datasetAddPulse(&dataset);
        transmitData(&dataset, &opt, txBuffer);

        /* If the energy used since the last storage is greater than the
         * configured energy delta then save the accumulated energy to NVM.
         */
        cumulativeProcess(&nvmCumulative, &dataset,
                          pConfig->baseCfg.epDeltaStore);

        /* Blink the STATUS LED, and clear the event. */
        uiLedColour(LED_RED);
        txBlink.timeBlink  = timerMillis();
        txBlink.txIndicate = true;
        emon32EventClr(EVT_PROCESS_DATASET);
      }

      /* Configuration:
       *   - Process command
       *   - Change (set PROG LED)
       *   - Save (clear PROG LED)
       */
      if (evtPending(EVT_PROCESS_CMD)) {
        configProcessCmd();
        emon32EventClr(EVT_PROCESS_CMD);
      }
      if (evtPending(EVT_OPA_INIT)) {
        pulseConfigure();
        numTempSensors = tempSetup(&dataset);
        emon32EventClr(EVT_OPA_INIT);
      }
      if (evtPending(EVT_CONFIG_CHANGED)) {
        emon32EventClr(EVT_CONFIG_CHANGED);
      }
      if (evtPending(EVT_CONFIG_SAVED)) {
        emon32EventClr(EVT_CONFIG_SAVED);
      }
    }

    samdSleepIdle();
  };
}
