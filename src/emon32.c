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

typedef struct TransmitOpt_ {
  bool    json;
  bool    useRFM;
  bool    logSerial;
  uint8_t node;
} TransmitOpt_t;

typedef struct TxBlink_ {
  bool         txIndicate;
  unsigned int timeBlink;
} TxBlink_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t evtPend;
AssertInfo_t             g_assert_info;
static unsigned int      lastStoredWh;
static TxBlink_t         txBlink;
Emon32Config_t          *pConfig = 0;

/*************************************
 * Static function prototypes
 *************************************/

static unsigned int cumulativeNVMLoad(Emon32Cumulative_t *pPkt,
                                      Emon32Dataset_t    *pData);
static void         cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                                       const Emon32Dataset_t *pData);
static void         cumulativeProcess(Emon32Cumulative_t    *pPkt,
                                      const Emon32Dataset_t *pData,
                                      const unsigned int     whDeltaStore);
static void         datasetAddPulse(Emon32Dataset_t *pDst);
static void         ecmConfigure(void);
static void         ecmDmaCallback(void);
static void         evtKiloHertz(void);
static bool         evtPending(EVTSRC_t evt);
static void         pulseConfigure(void);
void                putchar_(char c);
static void         serialPutsNonBlocking(const char *const s, uint16_t len);
static void         rfmConfigure(void);
static void         ssd1306Setup(void);
static uint32_t     tempSetup(void);
static uint32_t     totalEnergy(const Emon32Dataset_t *pData);
static void transmitData(const Emon32Dataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void ucSetup(void);

/*************************************
 * Functions
 *************************************/

/*! @brief Load cumulative energy and pulse values from NVM
 *  @param [in] pPkt : pointer to cumulative energy structure
 *  @param [out] pData : pointer to current dataset
 *  @return total Wh stored in NVM
 */
static unsigned int cumulativeNVMLoad(Emon32Cumulative_t *pPkt,
                                      Emon32Dataset_t    *pData) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  unsigned int totalWh  = 0;
  bool         eepromOK = false;
  ECMCfg_t    *ecmCfg   = ecmConfigGet();

  eepromWLReset(sizeof(*pPkt));
  eepromOK = (EEPROM_WL_OK == eepromReadWL(pPkt, 0));

  for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    uint32_t wh = eepromOK ? pPkt->wattHour[idxCT] : 0;

    ecmCfg->ctCfg[idxCT].wattHourInit = wh;
    totalWh += wh;
  }

  for (unsigned int idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    pData->pulseCnt[idxPulse] = eepromOK ? pPkt->pulseCnt[idxPulse] : 0;
  }

  return totalWh;
}

/*! @brief Store cumulative energy and pulse values
 *  @param [in] pRes : pointer to cumulative values
 */
static void cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                               const Emon32Dataset_t *pData) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  for (int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    pPkt->wattHour[idxCT] = pData->pECM->CT[idxCT].wattHour;
  }

  for (int idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    pPkt->pulseCnt[0] = pData->pulseCnt[0];
  }

  (void)eepromWriteWL(pPkt, 0);
}

/*! @brief Calculate the cumulative energy consumption and store if the delta
 *         since last storage is greater than a configurable threshold
 *  @param [in] pPkt : pointer to an NVM packet
 *  @param [in] pData : pointer to the current dataset
 *  @param [in] whDeltaStore : Wh delta between stores to NVM
 */
static void cumulativeProcess(Emon32Cumulative_t    *pPkt,
                              const Emon32Dataset_t *pData,
                              const unsigned int     whDeltaStore) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  bool     energyOverflow;
  uint32_t latestWh;
  uint32_t deltaWh;

  /* Store cumulative values if over threshold */
  latestWh = totalEnergy(pData);

  /* Catch overflow of energy. This corresponds to ~4 MWh(!), so unlikely to
   * but handle safely.
   */
  energyOverflow = (latestWh < lastStoredWh);
  deltaWh        = latestWh - lastStoredWh;
  if ((deltaWh >= whDeltaStore) || energyOverflow) {
    cumulativeNVMStore(pPkt, pData);
    lastStoredWh = latestWh;
  }
}

/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void datasetAddPulse(Emon32Dataset_t *pDst) {
  EMON32_ASSERT(pDst);
  for (unsigned int i = 0; i < NUM_OPA; i++) {
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

  ecmCfg->downsample    = DOWNSAMPLE_DSP;
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

  for (unsigned int i = 0; i < NUM_V; i++) {
    ecmCfg->vCfg[i].voltageCalRaw = pConfig->voltageCfg[i].voltageCal;
    ecmCfg->vCfg[i].vActive       = pConfig->voltageCfg[i].vActive;
  }

  for (unsigned int i = 0; i < NUM_CT; i++) {
    ecmCfg->ctCfg[i].phCal    = pConfig->ctCfg[i].phase;
    ecmCfg->ctCfg[i].ctCalRaw = pConfig->ctCfg[i].ctCal;
    ecmCfg->ctCfg[i].active   = pConfig->ctCfg[i].ctActive;
    ecmCfg->ctCfg[i].vChan1   = pConfig->ctCfg[i].vChan1;
    ecmCfg->ctCfg[i].vChan2   = pConfig->ctCfg[i].vChan2;
  }

  for (int i = 0; i < NUM_CT; i++) {
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
  if (txBlink.txIndicate &&
      (timerMillisDelta(txBlink.timeBlink) > TX_INDICATE_T)) {
    txBlink.txIndicate = false;
    uiLedColour(LED_GREEN);
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

  uint8_t pinsPulse[][2] = {{GRP_OPA, PIN_OPA1}, {GRP_OPA, PIN_OPA2}};

  for (unsigned int i = 0; i < NUM_OPA; i++) {
    PulseCfg_t *pulseCfg = pulseGetCfg(i);

    if ((0 != pulseCfg) && ('o' != pConfig->opaCfg[i].func) &&
        (pConfig->opaCfg[i].opaActive)) {
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

/*! @brief Allows the printf function to print to the debug console. If the USB
 *         CDC is connected, characters should be routed there.
 */
void putchar_(char c) {
  if (usbCDCIsConnected()) {
    usbCDCTxChar(c);
  }
  uartPutcBlocking(SERCOM_UART, c);
}

static void serialPutsNonBlocking(const char *const s, uint16_t len) {
  if (usbCDCIsConnected()) {
    usbCDCPutsBlocking(s);
  }
  uartPutsNonBlocking(DMA_CHAN_UART, s, len);
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
    int           offset = 0;
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

/*! @brief Initialises the temperature sensors
 *  @return number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  const uint8_t opaPins[NUM_OPA] = {PIN_OPA1, PIN_OPA2};
  const uint8_t opaPUs[NUM_OPA]  = {PIN_OPA1_PU, PIN_OPA2_PU};

  unsigned int   numTempSensors = 0;
  DS18B20_conf_t dsCfg          = {0};
  dsCfg.grp                     = GRP_OPA;
  dsCfg.t_wait_us               = 5;

  for (int i = 0; i < NUM_OPA; i++) {
    if (('o' == pConfig->opaCfg[i].func) && pConfig->opaCfg[i].opaActive) {
      dsCfg.opaIdx = i;
      dsCfg.pin    = opaPins[i];
      dsCfg.pinPU  = opaPUs[i];
      numTempSensors += tempInitSensors(TEMP_INTF_ONEWIRE, &dsCfg);
    }
  }

  return numTempSensors;
}

/*! @brief Total energy across all CTs
 *  @param [in] pData : pointer to data setup
 *  @return sum of Wh for all CTs
 */
static uint32_t totalEnergy(const Emon32Dataset_t *pData) {
  EMON32_ASSERT(pData);

  uint32_t totalEnergy = 0;
  for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    totalEnergy += pData->pECM->CT[idxCT].wattHour;
  }
  return totalEnergy;
}

static void transmitData(const Emon32Dataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  int nSerial = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);

  if (pOpt->useRFM) {

    if (pOpt->logSerial) {
      serialPutsNonBlocking(txBuffer, nSerial);
    }

    if (sercomExtIntfEnabled()) {
      int         retryCount = 0;
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
    serialPutsNonBlocking(txBuffer, nSerial);
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

int main(void) {

  Emon32Dataset_t    dataset               = {0};
  unsigned int       numTempSensors        = 0;
  Emon32Cumulative_t nvmCumulative         = {0};
  unsigned int       tempCount             = 0;
  char               txBuffer[TX_BUFFER_W] = {0};

  ucSetup();
  uiLedColour(LED_YELLOW);

  /* If the system is booted while it is connected to an active Pi, do not write
   * to the OLED. */
  if (sercomExtIntfEnabled()) {
    ssd1306Setup();
  }

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration and 0 energy accumulator area.
   */
  serialPuts("> Reading configuration and accumulators from NVM...\r\n");
  pConfig = configLoadFromNVM();

  /* Load the accumulated energy and pulse values from NVM. */
  lastStoredWh = cumulativeNVMLoad(&nvmCumulative, &dataset);

  /* Set up RFM module. Even if not used, this will put it in sleep mode. If
   * successful, set OEM's AES key. */
  if (sercomExtIntfEnabled()) {
    rfmConfigure();
  }

  /* Set up pulse and temperature sensors, if present. */
  pulseConfigure();
  numTempSensors = tempSetup();

  /* Wait 1s to allow USB to enumerate as serial. Not always possible, but gives
   * the possibility. The board information can be accessed through the serial
   * console later. */
  timerDelay_ms(1000);
  configFirmwareBoardInfo();

  /* Set up buffers for ADC data, configure energy processing, and start */
  uiLedColour(LED_GREEN);
  ecmConfigure();
  dmacCallbackBufferFill(&ecmDmaCallback);
  ecmFlush();
  adcDMACStart();

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
        evtKiloHertz();
        emon32EventClr(EVT_TICK_1kHz);
      }

      /* Configuration request to clear all accumulator values (energy and pulse
       * count). The NVM is overwritten with 0s and the index of the next
       * read/write is reset. Clear the running counters in the main loop, any
       * residual energy in the dataset, and all pulse counters.
       */
      if (evtPending(EVT_CLEAR_ACCUM)) {
        lastStoredWh = 0;
        /* REVISIT : may need to make this asynchronous as it will take 240 ms
         * (worst case) to clear the whole area for a 1KB EEPROM.
         */
        eepromWLClear();
        eepromWLReset(sizeof(nvmCumulative));
        ecmClearEnergy();
        for (int i = 0; i < NUM_OPA; i++) {
          pulseSetCount(i, 0);
        }
        emon32EventClr(EVT_CLEAR_ACCUM);
      }

      /* There has been a trigger request externally; the CM buffers will be
       * swapped on the next cycle. If there has been sufficient time between
       * the last temperature sample, start a temperature sample as well.
       */
      if (evtPending(EVT_ECM_TRIG)) {
        if (numTempSensors > 0) {
          for (int i = 0; i < NUM_OPA; i++) {
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
          for (int i = 0; i < NUM_OPA; i++) {
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
        if (numTempSensors > 0) {
          TempRead_t tempValue = tempReadSample(TEMP_INTF_ONEWIRE, tempCount);

          if (TEMP_OK == tempValue.status) {
            dataset.temp[tempCount] = tempValue.temp;
          }

          tempCount++;
          if (tempCount == numTempSensors) {
            emon32EventSet(EVT_PROCESS_DATASET);
            emon32EventClr(EVT_TEMP_READ);
            tempCount = 0;
          }
        } else {
          emon32EventSet(EVT_PROCESS_DATASET);
          emon32EventClr(EVT_TEMP_READ);
        }
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
                          pConfig->baseCfg.whDeltaStore);

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
      if (evtPending(EVT_CONFIG_CHANGED)) {
        emon32EventClr(EVT_CONFIG_CHANGED);
      }
      if (evtPending(EVT_CONFIG_SAVED)) {
        emon32EventClr(EVT_CONFIG_SAVED);
      }

      if (evtPending(EVT_SAFE_RESET_REQ)) {
        cumulativeNVMStore(&nvmCumulative, &dataset);
        NVIC_SystemReset();
      }
    }

    samdSleepIdle();
  };
}
