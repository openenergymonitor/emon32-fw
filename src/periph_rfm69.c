#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"
#include "emon32_samd.h"
#include "periph_rfm69.h"

#include "RFM69.h"

typedef struct RFMRx_ {
  uint16_t targetID;
  uint16_t senderID;
  int16_t  rxRSSI;
  uint8_t  payloadLen;
  uint8_t  dataLen;
  bool     ackRecv;
  bool     ackReq;
} RFMRx_t;

static bool      rfmAckRecv(uint16_t fromId);
static void      rfmFreqToBand(const RFM_Freq_t freq, uint8_t *band);
static void      rfmPacketHandler(void); /* LPL: interruptHandler */
static uint8_t   rfmReadReg(const unsigned int addr);
static int16_t   rfmReadRSSI(void);
static void      rfmReset(void);
static void      rfmRxBegin(void); /* LPL: receiveBegin */
static bool      rfmRxDone(void);  /* LPL: receiveDone */
static RFMSend_t rfmSendWithRetry(uint8_t n, const uint8_t retries,
                                  int *pRetryCount);
static void      rfmSetMode(RFMMode_t mode);
static bool      rfmTxAvailable(void); /* LPL: canSend */
static void      rfmWriteReg(const unsigned int addr, const uint8_t data);
static uint8_t   spiRx(void);
static void      spiTx(const uint8_t b);

static uint16_t      address       = 0;
static uint8_t       rxData[64]    = {0};
static volatile bool rxRdy         = false;
static const Pin_t   sel           = {GRP_SERCOM_SPI, PIN_SPI_RFM_SS};
static const Pin_t   rst           = {GRP_RFM_INTF, PIN_RFM_RST};
static bool          initDone      = false;
static uint8_t       rfmBuffer[64] = {0};
static int_fast8_t   rfmMode       = 0;
static RFMRx_t       rfmRx         = {0};

static bool rfmAckRecv(uint16_t fromId) {
  if (rfmRxDone()) {
    return (fromId == rfmRx.senderID) && rfmRx.ackRecv;
  }
  return false;
}

static void rfmFreqToBand(const RFM_Freq_t freq, uint8_t *band) {
  switch (freq) {
  case 0:
    band[2] = RFM_FRFMSB_868;
    band[1] = RFM_FRFMID_868;
    band[0] = RFM_FRFLSB_868;
    break;
  case 1:
    band[2] = RFM_FRFMSB_915;
    band[1] = RFM_FRFMID_915;
    band[0] = RFM_FRFLSB_915;
    break;
  case 2:
    band[2] = RFM_FRFMSB_433;
    band[1] = RFM_FRFMID_433_00;
    band[0] = RFM_FRFLSB_433_00;
    break;
  case 3:
    band[2] = RFM_FRFMSB_433;
    band[1] = RFM_FRFMID_433_92;
    band[0] = RFM_FRFLSB_433_92;
    break;
  }
}

static void rfmPacketHandler(void) {
  if ((RFM69_MODE_RX == rfmMode) &&
      (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PAYLOADREADY)) {

    rfmSetMode(RFM69_MODE_STANDBY);
    spiSelect(sel);
    spiTx(REG_FIFO & 0x7F);
    rfmRx.payloadLen = spiRx();
    /* Prevent any overflow */
    if (rfmRx.payloadLen > 66) {
      rfmRx.payloadLen = 66;
    }
    rfmRx.targetID = spiRx();
    rfmRx.senderID = spiRx();
    uint16_t ctl   = (uint16_t)spiRx();

    rfmRx.targetID |= (ctl & 0x0C) << 6;
    rfmRx.senderID |= (ctl & 0x03) << 8;

    if (!(address == rfmRx.targetID ||
          RFM69_BROADCAST_ADDR == rfmRx.targetID) ||
        (rfmRx.payloadLen < 3)) {
      rfmRx.payloadLen = 0;
      spiDeSelect(sel);
      rfmRxBegin();
      return;
    }

    rfmRx.dataLen = rfmRx.payloadLen - 3;
    rfmRx.ackRecv = ctl & RFM69_CTL_SENDACK;
    rfmRx.ackReq  = ctl & RFM69_CTL_REQACK;

    for (int i = 0; i < rfmRx.dataLen; i++) {
      rxData[i] = spiRx();
    }
    spiDeSelect(sel);
    rxData[rfmRx.dataLen] = 0;
    rfmSetMode(RFM69_MODE_RX);
  }
  rfmRx.rxRSSI = rfmReadRSSI();
}

static uint8_t rfmReadReg(const unsigned int addr) {
  uint8_t rdByte;
  spiSelect(sel);
  spiTx((uint8_t)addr);
  rdByte = spiRx();
  spiDeSelect(sel);
  return rdByte;
}

static bool rfmTxAvailable(void) {
  bool mode    = (RFM69_MODE_RX == rfmMode);
  bool len     = (0 == rfmRx.payloadLen);
  bool rssi    = (rfmReadRSSI() < RFM69_CSMA_LIMIT);
  bool canSend = mode && len && rssi;

  if (canSend) {
    rfmSetMode(RFM69_MODE_STANDBY);
  }
  return canSend;
}

static void rfmWriteReg(const unsigned int addr, const uint8_t data) {
  spiSelect(sel);
  /* Datasheet 5.2.1, Figure 24: "wnr is 1 for write" */
  spiTx((uint8_t)addr | 0x80);
  spiTx(data);
  spiDeSelect(sel);
}

static uint8_t spiRx(void) { return spiSendByte(SERCOM_SPI, 0x00); }

static void spiTx(const uint8_t b) { (void)spiSendByte(SERCOM_SPI, b); }

void rfmSetAESKey(const char *aes) {

  /* There is potential for buffer overflow in the original LPL library, which
   * only checks that the length is not 0. */
  bool key = (0 != aes) && (16 == strlen(aes));

  rfmSetMode(RFM69_MODE_STANDBY);

  if (key) {
    spiSelect(sel);
    spiTx(REG_AESKEY1 | 0x80);
    spiSendBuffer(SERCOM_SPI, aes, 16);
    spiDeSelect(sel);
  }

  rfmWriteReg(REG_PACKETCONFIG2,
              ((rfmReadReg(REG_PACKETCONFIG2) & 0xFE) | key));
}

void rfmSetFrequency(const RFM_Freq_t freq) {
  uint8_t band[3];
  rfmFreqToBand(freq, band);

  rfmWriteReg(REG_FRFMSB, band[2]);
  rfmWriteReg(REG_FRFMID, band[1]);
  rfmWriteReg(REG_FRFLSB, band[0]);
}

void rfmSetGroupID(const uint8_t grpID) {
  rfmSetMode(RFM69_MODE_STANDBY);
  rfmWriteReg(REG_SYNCVALUE2, grpID);
}

void rfmSetPowerLevel(const uint8_t paLevel) {
  rfmSetMode(RFM69_MODE_STANDBY);
  rfmWriteReg(REG_PALEVEL, (RFM_PALEVEL_PA0_ON | paLevel));
}

static int16_t rfmReadRSSI(void) {
  int16_t rssi = -rfmReadReg(REG_RSSIVALUE);
  return rssi >>= 1;
}

static void rfmReset(void) {
  /* Datasheet 7.2.2 Manual Reset: >100 us HIGH, then >5 ms before ready. */
  portPinDir(rst.grp, rst.pin, PIN_DIR_OUT);
  portPinDrv(rst.grp, rst.pin, PIN_DRV_SET);
  timerDelay_us(250u);
  portPinDir(rst.grp, rst.pin, PIN_DIR_IN);
  timerDelay_ms(6);
}

static void rfmRxBegin(void) {
  memset(&rfmRx, 0, sizeof(rfmRx));
  // Avoids Rx deadlocks
  if (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PAYLOADREADY) {
    rfmWriteReg(REG_PACKETCONFIG2, ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) |
                                    RFM_PACKET2_RXRESTART));
  }

  rfmWriteReg(REG_DIOMAPPING1, RFM_DIOMAPPING1_DIO0_01); // "PAYLOADREADY" in Rx
  rfmSetMode(RFM69_MODE_RX);
}

static bool rfmRxDone(void) {

  rfmPacketHandler();

  if ((RFM69_MODE_RX == rfmMode) && (rfmRx.payloadLen > 0)) {
    rfmSetMode(RFM69_MODE_STANDBY);
    return true;
  }

  rfmRxBegin();
  return false;
}

/*! @brief Send packet with optional retrys
 *  @param [in] n : number of bytes to send
 *  @param [in] retries : number of retries to attempt
 *  @param [out] pRetryCount : pointer to store number of retries for logging
 *  @return status of sending the packet
 */
static RFMSend_t rfmSendWithRetry(uint8_t n, const uint8_t retries,
                                  int *pRetryCount) {

  for (int r = 0; r < retries; r++) {
    uint32_t tNow;
    uint32_t tSent;

    *pRetryCount = *pRetryCount + 1;

    /* LPL send */
    rfmWriteReg(REG_PACKETCONFIG2, ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) |
                                    RFM_PACKET2_RXRESTART));
    tNow = timerMillis();
    while (!rfmTxAvailable() &&
           (timerMillisDelta(tNow) < RFM69_CSMA_LIMIT_MS)) {
      (void)rfmRxDone();
    }

    /* LPL sendFrame */
    rfmSetMode(RFM69_MODE_STANDBY); // Turn off Rx while filling FIFO
    while (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY))
      ;
    spiSelect(sel);
    spiTx(REG_FIFO | 0x80);
    spiTx(n + 3);
    spiTx(5u); // from OEM Tx
    spiTx((uint8_t)address);
    spiTx(RFM69_CTL_REQACK); // CTL byte
    spiSendBuffer(SERCOM_SPI, rfmBuffer, n);
    spiDeSelect(sel);

    /* Enter Tx mode, no need to wait for the mode to be ready as the Tx will
     * empty the FIFO when ready. */
    rfmSetMode(RFM69_MODE_TX);
    while (0 == (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PACKETSENT))
      ;
    rfmSetMode(RFM69_MODE_STANDBY);

    /* Listen for the acknowledgement */
    tSent = timerMillis();
    while (timerMillisDelta(tSent) < RFM_TIMEOUT) {
      if (rfmAckRecv(5)) {
        return RFM_SUCCESS;
      }
    }
  }
  return RFM_TIMED_OUT;
}

static void rfmSetMode(RFMMode_t mode) {
  if (rfmMode == mode) {
    return;
  }
  uint8_t rOpMode = rfmReadReg(REG_OPMODE) & 0xE3;

  switch (mode) {
  case RFM69_MODE_TX:
    rOpMode |= RFM_OPMODE_TRANSMITTER;
    break;
  case RFM69_MODE_RX:
    rOpMode |= RFM_OPMODE_RECEIVER;
    break;
  case RFM69_MODE_SYNTH:
    rOpMode |= RFM_OPMODE_SYNTHESIZER;
    break;
  case RFM69_MODE_STANDBY:
    rOpMode |= RFM_OPMODE_STANDBY;
    break;
  case RFM69_MODE_SLEEP:
    rOpMode |= RFM_OPMODE_SLEEP;
    break;
  default:
    return;
  }

  rfmWriteReg(REG_OPMODE, rOpMode);
  /* When coming from SLEEP, wait until FIFO is ready */
  if (RFM69_MODE_SLEEP == rfmMode) {
    while ((rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY) == 0)
      ;
  }
  rfmMode = mode;
}

uint8_t *rfmGetBuffer(void) { return rfmBuffer; }

void rfmInterrupt(void) { rxRdy = true; }

bool rfmInit(const RFMOpt_t *pOpt) {

  /* Immediately return if the interfaces are being externally controlled. */
  if (!sercomExtIntfEnabled()) {
    return false;
  }

  uint8_t band[3];
  rfmFreqToBand(pOpt->freq, band);

  /* Configuration parameters */
  const uint8_t config[][2] = {
      {REG_OPMODE, 0x04},    /* OPMODE: Sequencer, standby, listen off */
      {REG_DATAMODUL, 0x00}, /* DataModul: Packet, FSK, no shaping */
      {REG_BITRATEMSB, RFM_BITRATEMSB_55555},
      {REG_BITRATELSB, RFM_BITRATELSB_55555},
      {REG_FDEVMSB, RFM_FDEVMSB_50000},
      {REG_FDEVLSB, RFM_FDEVLSB_50000},
      {REG_FRFMSB, band[2]},
      {REG_FRFMID, band[1]},
      {REG_FRFLSB, band[0]},
      {REG_RXBW, (RFM_RXBW_DCCFREQ_010 | RFM_RXBW_MANT_16 | RFM_RXBW_EXP_2)},
      {REG_DIOMAPPING1, RFM_DIOMAPPING1_DIO0_01},
      {REG_DIOMAPPING2, RFM_DIOMAPPING2_CLKOUT_OFF},
      {REG_IRQFLAGS2, RFM_IRQFLAGS2_FIFOOVERRUN},
      {REG_RSSITHRESH, 0xDC},
      {REG_SYNCCONFIG, (RFM_SYNC_ON | RFM_SYNC_FIFOFILL_AUTO | RFM_SYNC_SIZE_2 |
                        RFM_SYNC_TOL_0)},
      {REG_SYNCVALUE1, 0x2D}, /* Make compatible with RFM12B library */
      {REG_SYNCVALUE2, pOpt->group},
      {REG_PACKETCONFIG1, RFM_PACKET1_FORMAT_VARIABLE | RFM_PACKET1_DCFREE_OFF |
                              RFM_PACKET1_CRC_ON | RFM_PACKET1_CRCAUTOCLEAR_ON |
                              RFM_PACKET1_ADRSFILTERING_OFF},
      {REG_PAYLOADLENGTH, 66},
      {REG_FIFOTHRESH,
       (RFM_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RFM_FIFOTHRESH_VALUE)},
      {REG_PACKETCONFIG2,
       (RFM_PACKET2_RXRESTARTDELAY_2BITS | RFM_PACKET2_AUTORXRESTART_OFF |
        RFM_PACKET2_AES_OFF)},
      {REG_TESTDAGC, RFM_DAGC_IMPROVED_LOWBETA0}};

  rfmReset();
  uint_fast8_t tStart = timerMillis();

  /* Initialise RFM69 */
  while (0xAA != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0xAAu);
    if (timerMillisDelta(tStart) > 25) {
      return false;
    }
  }

  tStart = timerMillis();
  while (0x55u != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0x55u);
    if (timerMillisDelta(tStart) > 25) {
      return false;
    }
  }

  /* Configuration */
  for (size_t idxCfg = 0; idxCfg < (sizeof(config) / sizeof(*config));
       idxCfg++) {
    rfmWriteReg(config[idxCfg][0], config[idxCfg][1]);
  }

  rfmSetAESKey(0);
  rfmWriteReg(REG_PALEVEL, (RFM_PALEVEL_PA0_ON | pOpt->paLevel));
  rfmSetMode(RFM69_MODE_STANDBY);

  tStart = timerMillis();
  while (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY)) {
    if (timerMillisDelta(tStart) > 25) {
      return false;
    }
  }

  initDone = true;
  return true;
}

RFMSend_t rfmSendBuffer(const int_fast8_t n, const uint8_t retries,
                        int *pRetryCount) {
  if (n > 61) {
    return RFM_N_TOO_LARGE;
  }

  if (!initDone) {
    return RFM_NO_INIT;
  }

  return rfmSendWithRetry(n, retries, pRetryCount);
}

void rfmSetAddress(const uint16_t addr) {
  address = addr;
  rfmWriteReg(REG_NODEADRS, addr);
}
