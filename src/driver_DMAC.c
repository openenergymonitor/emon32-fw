#include "driver_DMAC.h"
#include "emon32_samd.h"

#include "emon32.h"

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

volatile DmacDescriptor *dmacGetDescriptor(uint8_t ch) { return &dmacs[ch]; }

void dmacCallbackBufferFill(void (*cb)(void)) { cbBufferFill = cb; }

void dmacChannelDisable(uint8_t ch) {
  DMAC->CHID.reg           = ch;
  DMAC->CHCTRLA.bit.ENABLE = 0;
}

void dmacChannelEnable(uint8_t ch) {
  DMAC->CHID.reg           = ch;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}

void dmacEnableChannelInterrupt(uint8_t ch) {
  DMAC->CHID.reg       = ch;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
}

void dmacDisableChannelInterrupt(uint8_t ch) {
  DMAC->CHID.reg       = ch;
  DMAC->CHINTENCLR.reg = DMAC_CHINTENCLR_TCMPL;
}

void dmacClearChannelInterrupt(uint8_t ch) {
  DMAC->CHID.reg      = ch;
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
}

void dmacChannelConfigure(uint8_t ch, const DMACCfgCh_t *pCfg) {
  DMAC->CHID.reg    = ch;
  DMAC->CHCTRLB.reg = pCfg->ctrlb;
}

void irq_handler_dmac(void) {
  /* Check which channel has triggered the interrupt, set the event, and
   * clear the interrupt source
   */
  DMAC->CHID.reg = DMA_CHAN_ADC0;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
    dmacChannelEnable(DMA_CHAN_ADC0);
    (*cbBufferFill)();
  }
}

uint16_t calcCRC16_ccitt(const void *pSrc, size_t n) {
  const uint8_t *pData = (uint8_t *)pSrc;

  /* CCITT is 0xFFFF initial value */
  DMAC->CRCCHKSUM.reg      = 0xFFFF;
  DMAC->CTRL.bit.CRCENABLE = 1;

  /* Input into CRC data byte wise. Byte beats convert in a single cycle, so
   * using a DSB to ensure the previous store is complete is sufficient.
   */
  while (n--) {
    DMAC->CRCDATAIN.reg = *pData++;
    __DSB();
  }

  /* Clear status and disable CRC module before returning CRC */
  DMAC->CRCSTATUS.reg      = DMAC_CRCSTATUS_CRCBUSY;
  DMAC->CTRL.bit.CRCENABLE = 0;

  return (uint16_t)DMAC->CRCCHKSUM.reg;
}
