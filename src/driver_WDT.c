#include "emon32_samd.h"

#include "board_def.h"
#include "driver_WDT.h"

void wdtEnable(void) {
  /* Enable and synchronise (18.6.5) */
  WDT->CTRL.bit.ENABLE = 1u;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY)
    ;
}

void wdtFeed(void) {
  /* Write key (18.6.2.4) to clear */
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

void wdtSetup(void) {
  /* OSCULP32 is enabled and connected to Generator 2, undivided
   * Connect Gen 2 -> WDT
   */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(WDT_GCLK_ID) | GCLK_CLKCTRL_GEN(2u) | GCLK_CLKCTRL_CLKEN;

  WDT->EWCTRL.reg |= WDT_EWCTRL_EWOFFSET(EMON32_WDT_EW);
  WDT->CONFIG.reg |= WDT_CONFIG_PER(EMON32_WDT_PER);
  while (WDT->STATUS.bit.SYNCBUSY)
    ;

  /* Enable the early warning interrupt. If this is hit and the debugger is
   * attached, it can be stepped. */
  WDT->INTENSET.bit.EW = 1u;
  NVIC_EnableIRQ(WDT_IRQn);
}

void irq_handler_wdt(void) {
  WDT->INTFLAG.reg = WDT_INTFLAG_EW;

  /* Halt if there is an attached debugger */
  if (DSU->STATUSB.bit.DBGPRES) {
    __asm("bkpt 0");
  }
}
