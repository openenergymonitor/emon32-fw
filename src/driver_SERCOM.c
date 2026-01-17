#include "emon32_samd.h"

#include "configuration.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"

#define I2CM_ACTIVATE_TIMEOUT_US 200u /* Time to wait for I2C address phase */
#define I2CM_DATA_TIMEOUT_US     200u /* Time to wait for I2C data byte */

static void i2cmCommon(Sercom *pSercom);
static void i2cmExtPinsSetup(void);
static void sercomSetupSPI(void);
static void spiExtPinsSetup(bool enable);

static void uartInterruptEnable(Sercom *sercom, uint8_t interrupt);
static void uartSetup(void);

static volatile bool extIntfEnabled = true;

static void i2cmCommon(Sercom *pSercom) {
  /* For 400 kHz I2C (fast mode) with asymmetric timing:
   * At 8 MHz (125 ns/tick):
   *   T_LOW  = (BAUDLOW + 5) * 125 = (8 + 5) * 125 = 1625 ns
   *   T_HIGH = (BAUD + 5) * 125    = (2 + 5) * 125 =  875 ns
   * Resulting f_SCL ~ 357 kHz
   */
  pSercom->I2CM.BAUD.reg =
      SERCOM_I2CM_BAUD_BAUDLOW(8u) | SERCOM_I2CM_BAUD_BAUD(2u);

  /* SDAHOLD(3): Extended hold time for marginal timing (SMBus requirement)
   */
  pSercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER |
                            SERCOM_I2CM_CTRLA_SDAHOLD(3u) |
                            SERCOM_I2CM_CTRLA_ENABLE;
  while (pSercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;

  /* After enabling the I2C SERCOM, the bus state is UNKNOWN (Table 28-13)
   * Force into IDLE state, with sync
   */
  pSercom->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(0x1u);
  while (pSercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;

  pSercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB |
                               SERCOM_I2CM_INTENSET_SB |
                               SERCOM_I2CM_INTENSET_ERROR;
}

static void i2cmExtPinsSetup(void) {
  portPinMux(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SDA, PMUX_I2CM_EXT);
  portPinMux(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SCL, PMUX_I2CM_EXT);
}

static void spiExtPinsSetup(bool enable) {
  if (enable) {
    portPinMux(GRP_SERCOM_SPI, PIN_SPI_MISO, PMUX_SPI);
    portPinMux(GRP_SERCOM_SPI, PIN_SPI_MOSI, PMUX_SPI);
    portPinMux(GRP_SERCOM_SPI, PIN_SPI_SCK, PMUX_SPI);
    portPinDir(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DIR_OUT);
  } else {
    portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_MISO);
    portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_MOSI);
    portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_SCK);

    portPinDir(GRP_SERCOM_SPI, PIN_SPI_MISO, PIN_DIR_IN);
    portPinDir(GRP_SERCOM_SPI, PIN_SPI_MOSI, PIN_DIR_IN);
    portPinDir(GRP_SERCOM_SPI, PIN_SPI_SCK, PIN_DIR_IN);
    portPinDir(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DIR_IN);
  }
}

void sercomExtIntfDisable(void) {
  extIntfEnabled = false;
  spiExtPinsSetup(false);
}

bool sercomExtIntfEnabled(void) { return extIntfEnabled; }

void sercomSetup(void) {
  /*****************
   * Debug UART setup
   ******************/

  uartSetup();

  /*****************
   * I2C Setup
   ******************/

  portPinMux(GRP_SERCOM_I2C_INT, PIN_I2C_INT_SDA, PMUX_I2CM_INT);
  portPinMux(GRP_SERCOM_I2C_INT, PIN_I2C_INT_SCL, PMUX_I2CM_INT);

  PM->APBCMASK.reg |= SERCOM_I2CM_INT_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_I2CM_INT_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  i2cmCommon(SERCOM_I2CM);

  PM->APBCMASK.reg |= SERCOM_I2CM_EXT_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_I2CM_EXT_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  i2cmExtPinsSetup();
  i2cmCommon(SERCOM_I2CM_EXT);

  /*****************
   * SPI Setup
   ******************/
  sercomSetupSPI();
}

static void uartSetup(void) {

  uint16_t baud;
  // const uint64_t br_dbg = (uint64_t)65536 * (F_PERIPH - 16 * pCfg->baud) /
  // F_PERIPH;
  switch (UART_BAUD) {
  case (UART_BAUD_9600):
    baud = 64279;
    break;
  case (UART_BAUD_19200):
    baud = 63020;
    break;
  case (UART_BAUD_28800):
    baud = 61762;
    break;
  case (UART_BAUD_38400):
    baud = 60504u;
    break;
  case (UART_BAUD_57600):
    baud = 57987;
    break;
  case (UART_BAUD_76800):
    baud = 55471;
    break;
  case (UART_BAUD_115200):
    baud = 50438u;
    break;
  default:
    /* Default to 9600 if a non-standard baud is entered */
    baud = 64279;
  }

  portPinMux(GRP_SERCOM_UART, PIN_UART_TX, PMUX_UART);
  portPinMux(GRP_SERCOM_UART, PIN_UART_RX, PMUX_UART);

  /* Configure clocks - runs from the OSC8M clock on gen 3 */
  PM->APBCMASK.reg |= SERCOM_UART_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_UART_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Reset the USART fully to flush any state */
  SERCOM_UART->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (SERCOM_UART->USART.CTRLA.reg & SERCOM_USART_CTRLA_SWRST)
    ;

  /* Configure the USART */
  SERCOM_UART->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |
                                 SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                                 SERCOM_USART_CTRLA_RXPO(UART_PAD_RX) |
                                 SERCOM_USART_CTRLA_TXPO(UART_PAD_TX);

  /* TX/RX enable requires synchronisation */
  SERCOM_UART->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
                                 SERCOM_USART_CTRLB_TXEN |
                                 SERCOM_USART_CTRLB_CHSIZE(0);
  while (SERCOM_UART->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    ;

  SERCOM_UART->USART.BAUD.reg = baud;
}

static void sercomSetupSPI(void) {

  /* Configure clocks - runs from the OSC8M clock on gen 3 */
  PM->APBCMASK.reg |= SERCOM_SPI_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_SPI_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Table 25-2 - driven @ F_REF = F_PERIPH. BAUD = F_REF / 2F_BAUD - 1
   * RFM69 maximum SCK is 10 MHz, so can go at maximum 4 MHz SCK easily.
   */
  SERCOM_SPI->SPI.BAUD.reg = 0;

  /* SPI mode 0: CPOL == 0, CPHA == 0 */
  SERCOM_SPI->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                              SERCOM_SPI_CTRLA_DIPO(0x0) |
                              SERCOM_SPI_CTRLA_DOPO(0x2);

  /* Enable TX and RX interrupts (complete and empty), not routed to NVIC */
  SERCOM_SPI->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC |
                                 SERCOM_SPI_INTENSET_TXC |
                                 SERCOM_SPI_INTENSET_DRE;

  /* While disabled, RXEN will be set immediately. When the SPI SERCOM is
   * enabled, this requires synchronisation before the SPI is ready. See
   * field description in 27.8.2
   */
  SERCOM_SPI->SPI.CTRLB.reg        = SERCOM_SPI_CTRLB_RXEN;
  SERCOM_SPI->SPI.CTRLA.bit.ENABLE = 1;
  while (0 != SERCOM_SPI->SPI.SYNCBUSY.reg)
    ;
}

/*
 * =====================================
 * UART Functions
 * =====================================
 */

void uartPutcBlocking(Sercom *sercom, char c) {

  uint32_t t_start         = timerMillis();
  bool     uartNotTimedOut = true;

  while (!(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) &&
         uartNotTimedOut) {
    uartNotTimedOut = (timerMillisDelta(t_start) < 10);
  };

  /* Assume at this point the UART is in error state, so reset and proceed. */
  if (!uartNotTimedOut) {
    uartSetup();
  }

  sercom->USART.DATA.reg    = c;
  sercom->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_DRE;
}

void uartPutsBlocking(Sercom *sercom, const char *s) {
  while (*s) {
    uartPutcBlocking(sercom, *s++);
  }
}

void uartEnableRx(Sercom *sercom, const uint32_t irqn) {
  uartInterruptEnable(sercom, SERCOM_USART_INTENSET_RXC);
  NVIC_EnableIRQ(irqn);

  sercom->USART.CTRLB.bit.RXEN = 1;

  /* Enable requires synchronisation (26.6.6) */
  if (!(sercom->USART.CTRLA.reg & SERCOM_USART_CTRLA_ENABLE)) {
    sercom->USART.CTRLA.bit.ENABLE = 1;
    while (sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_ENABLE)
      ;
  }
}

void uartEnableTx(Sercom *sercom) {
  sercom->USART.CTRLB.bit.TXEN = 1;
  /* Enable requires synchronisation (26.6.6) */
  if (!(sercom->USART.CTRLA.reg & SERCOM_USART_CTRLA_ENABLE)) {
    sercom->USART.CTRLA.bit.ENABLE = 1;
    while (sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_ENABLE)
      ;
  }
}

char uartGetc(Sercom *sercom) { return (char)sercom->USART.DATA.reg; }

bool uartGetcReady(const Sercom *sercom) {
  return (bool)(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC);
}

static void uartInterruptEnable(Sercom *sercom, uint8_t interrupt) {
  sercom->USART.INTENSET.reg = interrupt;
}

uint32_t uartInterruptStatus(const Sercom *sercom) {
  return sercom->USART.INTFLAG.reg;
}

/*
 * =====================================
 * I2C Functions
 * =====================================
 */

void i2cBusRecovery(Sercom *sercom, const uint8_t grp, const uint8_t sdaPin,
                    const uint8_t sclPin, const uint8_t pmux) {
  /* Disable I2C peripheral */
  sercom->I2CM.CTRLA.bit.ENABLE = 0;
  while (sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE)
    ;

  /* Clear pin mux - make them GPIO */
  portPinMuxClear(grp, sdaPin);
  portPinMuxClear(grp, sclPin);

  /* Configure SCL as output (high), SDA as input */
  portPinDir(grp, sclPin, PIN_DIR_OUT);
  portPinDrv(grp, sclPin, PIN_DRV_SET);
  portPinDir(grp, sdaPin, PIN_DIR_IN);
  portPinCfg(grp, sdaPin, PORT_PINCFG_INEN | PORT_PINCFG_PULLEN, PIN_CFG_SET);
  portPinDrv(grp, sdaPin, PIN_DRV_SET); /* Enable pull-up */

  /* Toggle SCL up to 9 times to release stuck slave */
  for (size_t i = 0; i < 9; i++) {
    if (portPinValue(grp, sdaPin)) {
      break; /* SDA released, we're done */
    }
    portPinDrv(grp, sclPin, PIN_DRV_CLR);
    timerDelay_us(5);
    portPinDrv(grp, sclPin, PIN_DRV_SET);
    timerDelay_us(5);
  }

  /* Generate STOP condition: SDA low->high while SCL high */
  portPinDir(grp, sdaPin, PIN_DIR_OUT);
  portPinDrv(grp, sdaPin, PIN_DRV_CLR);
  timerDelay_us(5);
  portPinDrv(grp, sclPin, PIN_DRV_SET);
  timerDelay_us(5);
  portPinDrv(grp, sdaPin, PIN_DRV_SET);
  timerDelay_us(5);

  /* Reconfigure pins for I2C */
  portPinMux(grp, sdaPin, pmux);
  portPinMux(grp, sclPin, pmux);

  /* Reinitialize I2C peripheral */
  i2cmCommon(sercom);
}

I2CM_Status_t i2cActivate(Sercom *sercom, const uint32_t addr) {
  uint32_t      t = timerMicros();
  I2CM_Status_t s = I2CM_SUCCESS;

  if (!(sercom->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_ENABLE)) {
    return I2CM_DISABLED;
  }

  sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_ADDR(addr);

  /* MB: master on bus, SB: slave on bus */
  while (!(sercom->I2CM.INTFLAG.reg &
           (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB))) {
    if (timerMicrosDelta(t) > I2CM_ACTIVATE_TIMEOUT_US) {
      return I2CM_TIMEOUT;
    }
  }

  /* Check for bus errors (BUSERR, ARBLOST) */
  if (sercom->I2CM.STATUS.reg &
      (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST)) {
    return I2CM_ERROR;
  }

  /* Check for NoAck response from client (28.6.2.4.2) */
  if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
    s = I2CM_NOACK;
  }

  return s;
}

void i2cAck(Sercom *sercom, const I2CM_Ack_t ack, const I2CM_AckCmd_t cmd) {
  sercom->I2CM.CTRLB.reg =
      (ack << SERCOM_I2CM_CTRLB_ACKACT_Pos) | SERCOM_I2CM_CTRLB_CMD(cmd);
  while (sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;
}

I2CM_Status_t i2cDataWrite(Sercom *sercom, const uint8_t data) {
  uint32_t t = timerMicros();

  sercom->I2CM.DATA.reg = data;

  /* Wait for MB (master on bus) flag */
  while (!(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB)) {
    if (timerMicrosDelta(t) > I2CM_DATA_TIMEOUT_US) {
      return I2CM_TIMEOUT;
    }
  }

  /* Check for bus errors (BUSERR, ARBLOST) */
  if (sercom->I2CM.STATUS.reg &
      (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST)) {
    return I2CM_ERROR;
  }

  /* Check for NACK from slave */
  if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
    return I2CM_NOACK;
  }

  return I2CM_SUCCESS;
}

I2CM_Status_t i2cDataRead(Sercom *sercom, uint8_t *pData) {
  uint32_t t = timerMicros();

  /* Wait for SB (slave on bus) or MB (error condition) */
  while (!(sercom->I2CM.INTFLAG.reg &
           (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB))) {
    if (timerMicrosDelta(t) > I2CM_DATA_TIMEOUT_US) {
      return I2CM_TIMEOUT;
    }
  }

  /* Check for bus errors (BUSERR, ARBLOST) */
  if (sercom->I2CM.STATUS.reg &
      (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST)) {
    return I2CM_ERROR;
  }

  *pData = sercom->I2CM.DATA.reg;
  return I2CM_SUCCESS;
}

/*
 * =====================================
 * SPI Functions
 * =====================================
 */

void spiConfigureExt(void) {
  extIntfEnabled = !portPinValue(GRP_DISABLE_EXT, PIN_DISABLE_EXT);
  spiExtPinsSetup(extIntfEnabled);
}

void spiDeSelect(const Pin_t nSS) {
  if (!extIntfEnabled) {
    return;
  }
  portPinDrv(nSS.grp, nSS.pin, PIN_DRV_SET);
}

void spiSelect(const Pin_t nSS) {
  if (!extIntfEnabled) {
    return;
  }
  portPinDrv(nSS.grp, nSS.pin, PIN_DRV_CLR);
}

void spiSendBuffer(Sercom *sercom, const void *pSrc, size_t n) {
  if (!extIntfEnabled) {
    return;
  }

  uint8_t *pData = (uint8_t *)pSrc;

  while (n--) {
    spiSendByte(sercom, *pData++);
  }
}

uint8_t spiSendByte(Sercom *sercom, const uint8_t b) {
  if (!extIntfEnabled) {
    return 0;
  }

  while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_DRE))
    ;
  sercom->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_RXC;
  sercom->SPI.DATA.reg    = b;

  while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC))
    ;

  /* Reading SPI.DATA clears the RXC interrupt. */
  return (uint8_t)sercom->SPI.DATA.reg;
}
