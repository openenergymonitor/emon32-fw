#include "driver_DMAC.h"
#include "driver_ADC.h"
#include "emon32_samd.h"

#include "emon32.h"

static void irqHandlerADCCommon(void);

static volatile DmacDescriptor dmacs[NUM_CHAN_DMA];
static DmacDescriptor          dmacs_wb[NUM_CHAN_DMA];

static void (*cbBufferFill)(void);

/* Useful ref: https://aykevl.nl/2019/09/samd21-dma */

void dmacSetup(void) {
  /* Clocking - AHB and APB are both enabled at reset (16.8.8, 16.8.10) */
  DMAC->BASEADDR.reg = (uint32_t)dmacs;
  DMAC->WRBADDR.reg  = (uint32_t)dmacs_wb;
  DMAC->CTRL.reg     = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xFu);

  /* CRC module - CRC16-CCITT byte wise access from IO */
  DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCSRC_IO;

  /* Enable the DMAC interrupt in the NVIC, but leave the channel interrupt
   * enable/disable for each channel to the peripheral */
  NVIC_EnableIRQ(DMAC_IRQn);
}

volatile DmacDescriptor *dmacGetDescriptor(unsigned int ch) {
  return &dmacs[ch];
}

void dmacCallbackBufferFill(void (*cb)(void)) { cbBufferFill = cb; }

void dmacChannelDisable(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void dmacChannelEnable(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void dmacEnableChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg       = ch;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
}

void dmacDisableChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg       = ch;
  DMAC->CHINTENCLR.reg = DMAC_CHINTENCLR_TCMPL;
}

void dmacClearChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg      = ch;
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
}

void dmacChannelConfigure(unsigned int ch, const DMACCfgCh_t *pCfg) {
  DMAC->CHID.reg    = ch;
  DMAC->CHCTRLB.reg = pCfg->ctrlb;
}

static void irqHandlerADCCommon(void) { (*cbBufferFill)(); }

void irq_handler_dmac(void) {
  /* Check which channel has triggered the interrupt, set the event, and
   * clear the interrupt source
   */
  DMAC->CHID.reg = DMA_CHAN_ADC0;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
    dmacChannelEnable(DMA_CHAN_ADC0);
    irqHandlerADCCommon();
  }

  /* REVISIT : is this branch hit? Should all be in channel 0 */
  DMAC->CHID.reg = DMA_CHAN_ADC1;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
    dmacChannelEnable(DMA_CHAN_ADC1);
    irqHandlerADCCommon();
  }

  DMAC->CHID.reg = DMA_CHAN_UART;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
  }
}

uint16_t calcCRC16_ccitt(const void *pSrc, unsigned int n) {
  const uint8_t *pData = (uint8_t *)pSrc;

  /* CCITT is 0xFFFF initial value */
  DMAC->CRCCHKSUM.reg = 0xFFFF;
  DMAC->CTRL.reg |= DMAC_CTRL_CRCENABLE;

  /* Input into CRC data byte wise. Byte beats convert in a single cycle, so
   * using a DSB to ensure the previous store is complete is sufficient.
   */
  while (n--) {
    DMAC->CRCDATAIN.reg = *pData++;
    __DSB();
  }

  /* Clear status and disable CRC module before returning CRC */
  DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
  DMAC->CTRL.reg &= ~DMAC_CTRL_CRCENABLE;

  return DMAC->CRCCHKSUM.reg;
}
