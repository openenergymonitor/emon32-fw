#include "emon32_samd.h"

#include "configuration.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"

#define I2CM_ACTIVATE_TIMEOUT_US 200u /* Time to wait for I2C bus */

static void i2cmCommon(Sercom *pSercom);
static void i2cmExtPinsSetup(void);
static void sercomSetupSPI(void);
static void spiExtPinsSetup(bool enable);

static void uartConfigureDMA(void);
static void uartInterruptEnable(Sercom *sercom, uint32_t interrupt);
static void uartSetup(const UART_Cfg_t *pCfg);

static volatile bool extIntfEnabled = true;

static void i2cmCommon(Sercom *pSercom) {
  /* For 400 kHz I2C, SCL T_high >= 0.6 us, T_low >= 1.3 us, with
   * (T_high + T_low) <= 2.5 us, and T_low / T_high ~ 1.8.
   * From I2C->Clock generation (28.6.2.4.1):
   * BAUD.BAUDLOW = (T_low * f_clk) - 5 (1.625 us -> 8 @ 8 MHz)
   * BAUD.BAUD = (T_high * f_clk) - 5 (0.875 us -> 2 @ 8 MHz)
   */
  pSercom->I2CM.BAUD.reg =
      SERCOM_I2CM_BAUD_BAUDLOW(8u) | SERCOM_I2CM_BAUD_BAUD(2u);

  pSercom->I2CM.CTRLA.reg =
      SERCOM_I2CM_CTRLA_MODE_I2C_MASTER | SERCOM_I2CM_CTRLA_ENABLE;
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

  UART_Cfg_t uart_dbg_cfg;
  uart_dbg_cfg.sercom    = SERCOM_UART;
  uart_dbg_cfg.baud      = UART_BAUD;
  uart_dbg_cfg.apbc_mask = SERCOM_UART_APBCMASK;
  uart_dbg_cfg.gclk_id   = SERCOM_UART_GCLK_ID;
  uart_dbg_cfg.gclk_gen  = 3u;
  uart_dbg_cfg.pad_tx    = UART_PAD_TX;
  uart_dbg_cfg.pad_rx    = UART_PAD_RX;

  uart_dbg_cfg.port_grp = GRP_SERCOM_UART;
  uart_dbg_cfg.pin_tx   = PIN_UART_TX;
  uart_dbg_cfg.pin_rx   = PIN_UART_RX;
  uart_dbg_cfg.pmux     = PMUX_UART;

  uart_dbg_cfg.dmaChannel   = DMA_CHAN_UART;
  uart_dbg_cfg.dmaCfg.ctrlb = DMAC_CHCTRLB_LVL(1u) |
                              DMAC_CHCTRLB_TRIGSRC(SERCOM_UART_DMAC_ID_TX) |
                              DMAC_CHCTRLB_TRIGACT_BEAT;
  uartSetup(&uart_dbg_cfg);

  /* Setup DMAC for non-blocking UART (this is optional, unlike ADC) */
  uartConfigureDMA();

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

static void uartSetup(const UART_Cfg_t *pCfg) {
  uint16_t baud;
  // const uint64_t br_dbg = (uint64_t)65536 * (F_PERIPH - 16 * pCfg->baud) /
  // F_PERIPH;
  switch (pCfg->baud) {
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

  portPinMux(pCfg->port_grp, pCfg->pin_tx, pCfg->pmux);
  portPinMux(pCfg->port_grp, pCfg->pin_rx, pCfg->pmux);

  /* Configure clocks - runs from the OSC8M clock on gen 3 */
  PM->APBCMASK.reg |= pCfg->apbc_mask;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(pCfg->gclk_id) |
                      GCLK_CLKCTRL_GEN(pCfg->gclk_gen) | GCLK_CLKCTRL_CLKEN;

  /* Configure the USART */
  pCfg->sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |
                                  SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                                  SERCOM_USART_CTRLA_RXPO(pCfg->pad_rx) |
                                  SERCOM_USART_CTRLA_TXPO(pCfg->pad_tx);

  /* TX/RX enable requires synchronisation */
  pCfg->sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
                                  SERCOM_USART_CTRLB_TXEN |
                                  SERCOM_USART_CTRLB_CHSIZE(0);
  while (pCfg->sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    ;

  pCfg->sercom->USART.BAUD.reg = baud;

  /* Configure DMA */
  dmacChannelConfigure(pCfg->dmaChannel, &pCfg->dmaCfg);
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
  SERCOM_SPI->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;
  SERCOM_SPI->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
  while (0 != SERCOM_SPI->SPI.SYNCBUSY.reg)
    ;
}

/*
 * =====================================
 * UART Functions
 * =====================================
 */
void uartPutcBlocking(Sercom *sercom, char c) {
  while (!(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE))
    ;
  sercom->USART.DATA.reg    = c;
  sercom->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_DRE;
}

void uartPutsBlocking(Sercom *sercom, const char *s) {
  while (*s)
    uartPutcBlocking(sercom, *s++);
}

static void uartConfigureDMA(void) {
  volatile DmacDescriptor *dmacDesc = dmacGetDescriptor(DMA_CHAN_UART);
  dmacDesc->BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT_NOACT |
                         DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC |
                         DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE;

  dmacDesc->DSTADDR.reg  = (uint32_t)&SERCOM_UART->USART.DATA;
  dmacDesc->DESCADDR.reg = 0u;
}

void uartPutsNonBlocking(unsigned int dma_chan, const char *const s,
                         uint16_t len) {
  volatile DmacDescriptor *dmacDesc = dmacGetDescriptor(dma_chan);
  /* Valid bit is cleared when a channel is complete */
  dmacDesc->BTCTRL.reg |= DMAC_BTCTRL_VALID;
  dmacDesc->BTCNT.reg   = len;
  dmacDesc->SRCADDR.reg = (uint32_t)s + len;
  dmacChannelEnable(dma_chan);
}

void uartEnable(Sercom *sercom, const uint32_t interrupts,
                const uint32_t irqn) {
  uartInterruptEnable(sercom, interrupts);
  NVIC_EnableIRQ(irqn);

  /* Enable requires synchronisation (26.6.6) */
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
  while (sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_ENABLE)
    ;
}

char uartGetc(Sercom *sercom) { return sercom->USART.DATA.reg; }

bool uartGetcReady(const Sercom *sercom) {
  return (bool)(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC);
}

static void uartInterruptEnable(Sercom *sercom, uint32_t interrupt) {
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
I2CM_Status_t i2cActivate(Sercom *sercom, uint8_t addr) {
  unsigned int  t = timerMicros();
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

  /* Check for NoAck response from client (28.6.2.4.2) */
  if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
    s = I2CM_NOACK;
  }

  return s;
}

void i2cAck(Sercom *sercom, I2CM_Ack_t ack, I2CM_AckCmd_t cmd) {
  sercom->I2CM.CTRLB.reg =
      (ack << SERCOM_I2CM_CTRLB_ACKACT_Pos) | SERCOM_I2CM_CTRLB_CMD(cmd);
  while (sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;
}

void i2cDataWrite(Sercom *sercom, uint8_t data) {
  sercom->I2CM.DATA.reg = data;
  while (!(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB))
    ;
}

uint8_t i2cDataRead(Sercom *sercom) {
  while (!(sercom->I2CM.INTFLAG.reg &
           (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB)))
    ;
  return sercom->I2CM.DATA.reg;
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

void spiSendBuffer(Sercom *sercom, const void *pSrc, int n) {
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
