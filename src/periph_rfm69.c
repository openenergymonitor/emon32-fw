#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_def.h"
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

typedef struct RFMTx_ {
  uint32_t tSPI_us;     /* Time of last SPI access (us) */
  uint32_t tTimeout_ms; /* Time of operation start (ms) */
  uint8_t  retryNum;    /* Which retry are we on */
  uint8_t  retryNumMax; /* Maximum number of attempts */
  uint8_t  n;           /* Number of bytes in Tx buffer */
} RFMTx_t;

static bool      rfmAckRecv(uint16_t fromId);
static bool      rfmCheckCSMA(void);
static bool      rfmCheckSendComplete(void);
static void      rfmFreqToBand(const RFM_Freq_t freq, uint8_t *band);
static void      rfmPacketHandler(void); /* LPL: interruptHandler */
static uint8_t   rfmReadReg(const uint8_t addr);
static int16_t   rfmReadRSSI(void);
static void      rfmReset(void);
static void      rfmRxBegin(void); /* LPL: receiveBegin */
static bool      rfmRxDone(void);  /* LPL: receiveDone */
static void      rfmRxRestart(void);
static RFMSend_t rfmSendPacket(void);
static bool      rfmSetMode(RFMMode_t mode);
static bool      rfmTxAvailable(void); /* LPL: canSend */
static bool      rfmTxIsIdle(void);
static void      rfmWriteReg(const uint8_t addr, const uint8_t data);
static uint8_t   spiRx(void);
static void      spiTx(const uint8_t b);

static uint8_t       address       = 0;
static bool          initDone      = false;
static uint8_t       rfmBuffer[64] = {0};
static int8_t        rfmMode       = 0;
static RFMRx_t       rfmRx         = {0};
static RFMTx_t       rfmTx         = {0};
static RFMTxState_t  rfmTxState    = RFM_TX_IDLE;
static uint8_t       rxData[64]    = {0};
static volatile bool rxRdy         = false;
static const Pin_t   rst           = {GRP_RFM_INTF, PIN_RFM_RST};
static const Pin_t   sel           = {GRP_SERCOM_SPI, PIN_SPI_RFM_SS};

static bool rfmAckRecv(uint16_t fromId) {
  if (timerMicrosDelta(rfmTx.tSPI_us > 1000u)) {
    rfmTx.tSPI_us = timerMicros();
    if (rfmRxDone()) {
      return (fromId == rfmRx.senderID) && rfmRx.ackRecv;
    }
  }
  return false;
}

static bool rfmCheckCSMA(void) {

  /* LPL send */
  if (timerMicrosDelta(rfmTx.tSPI_us) > 1000u) {
    rfmTx.tSPI_us = timerMicros();

    if (!rfmTxAvailable()) {
      (void)rfmRxDone();
      return false;
    } else {
      return true;
    }
  }

  return false;
}

static bool rfmCheckSendComplete(void) {
  if (timerMicrosDelta(rfmTx.tSPI_us) > 1000) {
    rfmTx.tSPI_us = timerMicros();
    return (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PACKETSENT);
  }
  return false;
}

static void rfmFreqToBand(const RFM_Freq_t freq, uint8_t *band) {
  /* Default to 433.92 MHz */
  band[2] = RFM_FRFMSB_433;
  band[1] = RFM_FRFMID_433_92;
  band[0] = RFM_FRFLSB_433_92;

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

    (void)rfmSetMode(RFM69_MODE_STANDBY);
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

    for (size_t i = 0; i < rfmRx.dataLen; i++) {
      rxData[i] = spiRx();
    }
    spiDeSelect(sel);
    rxData[rfmRx.dataLen] = 0;
    (void)rfmSetMode(RFM69_MODE_RX);
  }
  rfmRx.rxRSSI = rfmReadRSSI();
}

static uint8_t rfmReadReg(const uint8_t addr) {
  uint8_t rdByte;
  spiSelect(sel);
  spiTx(addr);
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
    (void)rfmSetMode(RFM69_MODE_STANDBY);
  }
  return canSend;
}

static void rfmWriteReg(const uint8_t addr, const uint8_t data) {
  spiSelect(sel);
  /* Datasheet 5.2.1, Figure 24: "wnr is 1 for write" */
  spiTx(addr | 0x80);
  spiTx(data);
  spiDeSelect(sel);
}

static uint8_t spiRx(void) { return spiSendByte(SERCOM_SPI, 0x00); }

static void spiTx(const uint8_t b) { (void)spiSendByte(SERCOM_SPI, b); }

void rfmSetAESKey(const char *aes) {

  /* There is potential for buffer overflow in the original LPL library, which
   * only checks that the length is not 0. */
  bool key = (0 != aes) && (16 == strlen(aes));

  (void)rfmSetMode(RFM69_MODE_STANDBY);

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
  (void)rfmSetMode(RFM69_MODE_STANDBY);
  rfmWriteReg(REG_SYNCVALUE2, grpID);
}

void rfmSetPowerLevel(const uint8_t paLevel) {
  (void)rfmSetMode(RFM69_MODE_STANDBY);
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
  (void)rfmSetMode(RFM69_MODE_RX);
}

static bool rfmRxDone(void) {

  rfmPacketHandler();

  if ((RFM69_MODE_RX == rfmMode) && (rfmRx.payloadLen > 0)) {
    (void)rfmSetMode(RFM69_MODE_STANDBY);
    return true;
  }

  rfmRxBegin();
  return false;
}

static void rfmRxRestart(void) {
  rfmWriteReg(REG_PACKETCONFIG2,
              ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) | RFM_PACKET2_RXRESTART));

  rfmTx.tSPI_us = timerMicros();
}

static RFMSend_t rfmSendPacket(void) {
  uint32_t tStart = timerMillis();
  (void)rfmSetMode(RFM69_MODE_STANDBY); // Turn off Rx while filling FIFO
  while (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY)) {
    if (timerMillisDelta(tStart) > 100u) {
      return RFM_FUNCTIONAL_FAILURE;
    }
  }
  spiSelect(sel);
  spiTx(REG_FIFO | 0x80);
  spiTx(rfmTx.n + 3);
  spiTx(5u); // from OEM Tx
  spiTx((uint8_t)address);
  spiTx(RFM69_CTL_REQACK); // CTL byte, request acknowledgement.
  spiSendBuffer(SERCOM_SPI, rfmBuffer, rfmTx.n);
  spiDeSelect(sel);

  /* Enter Tx mode, no need to wait for the mode to be ready as the Tx will
   * empty the FIFO when ready. */
  (void)rfmSetMode(RFM69_MODE_TX);

  return RFM_SUCCESS;
}

static bool rfmSetMode(RFMMode_t mode) {
  if (rfmMode == mode) {
    return true;
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
    return false;
  }

  rfmWriteReg(REG_OPMODE, rOpMode);

  /* When coming from SLEEP, wait until FIFO is ready */
  const uint32_t tStart = timerMillis();
  if (RFM69_MODE_SLEEP == rfmMode) {
    while ((rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY) == 0) {
      if (timerMillisDelta(tStart) > 25u) {
        return false;
      }
    }
  }
  rfmMode = mode;
  return true;
}

uint8_t *rfmGetBuffer(void) { return rfmBuffer; }

bool rfmInit(const RFMOpt_t *pOpt) {

  /* Immediately return if the interfaces are being externally controlled. */
  if (!sercomExtIntfEnabled()) {
    return false;
  }

  initDone   = false;
  rfmTxState = RFM_TX_IDLE;

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
  uint32_t tStart = timerMillis();

  /* Initialise RFM69 */
  while (0xAA != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0xAAu);
    if (timerMillisDelta(tStart) > 25u) {
      return false;
    }
  }

  tStart = timerMillis();
  while (0x55u != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0x55u);
    if (timerMillisDelta(tStart) > 25u) {
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

  /* This is the only point where the RFM module is in SLEEP mode, so the
   * timeout would apply here. If the RFM fails to come out of SLEEP, indicate
   * the initialisation has failed. */
  if (!rfmSetMode(RFM69_MODE_STANDBY)) {
    return false;
  }

  initDone = true;
  return true;
}

RFMSend_t rfmSendBuffer(const uint8_t n, const uint8_t retries) {
  if (n > 61) {
    return RFM_N_TOO_LARGE;
  }

  if (!initDone) {
    return RFM_NO_INIT;
  }

  if (!rfmTxIsIdle()) {
    return RFM_NOT_IDLE;
  }

  /* RFM Tx SETUP */
  rfmTx.n           = n;
  rfmTx.retryNum    = 1u;
  rfmTx.retryNumMax = retries;
  rfmTx.tTimeout_ms = timerMillis();
  rfmTxState        = RFM_TX_AWAIT_CMSA;

  /* Start state machine running */
  rfmRxRestart();
  (void)rfmTxAdvance();

  return RFM_SUCCESS;
}

void rfmSetAddress(const uint8_t addr) {
  address = addr;
  rfmWriteReg(REG_NODEADRS, addr);
}

RFMTxState_t rfmTxAdvance(void) {
  switch (rfmTxState) {
  case RFM_TX_AWAIT_CMSA:
    /* Wait for clear air. If there is none in the time limit, proceed anyway */
    if (rfmCheckCSMA() ||
        (timerMillisDelta(rfmTx.tTimeout_ms) > RFM69_CSMA_LIMIT_MS)) {
      if (RFM_FUNCTIONAL_FAILURE == rfmSendPacket()) {
        rfmTxState = RFM_TX_ABORT;
      } else {
        rfmTx.tTimeout_ms = timerMillis();
        rfmTxState        = RFM_TX_AWAIT_TX;
      }
    }
    break;
  case RFM_TX_AWAIT_TX:
    if (timerMillisDelta(rfmTx.tTimeout_ms) > 100u) {
      rfmTxState = RFM_TX_ABORT;
    } else if (rfmCheckSendComplete()) {
      (void)rfmSetMode(RFM69_MODE_STANDBY);
      rfmTx.tTimeout_ms = timerMillis();
      rfmTxState        = RFM_TX_AWAIT_ACK;
    }
    break;
  case RFM_TX_AWAIT_ACK:
    /* Wait for acknowledgement up to limit */
    if (timerMillisDelta(rfmTx.tTimeout_ms) > RFM_TIMEOUT_ACK) {
      rfmTx.retryNum++;
      if (rfmTx.retryNum > rfmTx.retryNumMax) {
        rfmTxState = RFM_TX_IDLE;
      } else {
        rfmTx.tTimeout_ms = timerMillis();
        rfmTxState        = RFM_TX_AWAIT_CMSA;
        rfmRxRestart();
      }
    } else {
      if (rfmAckRecv(5u)) {
        rfmTxState = RFM_TX_IDLE;
      }
    }
    break;
  case RFM_TX_IDLE:
  case RFM_TX_ABORT:
    break;
  }

  return rfmTxState;
}

static bool rfmTxIsIdle(void) { return (RFM_TX_IDLE == rfmTxState); }
