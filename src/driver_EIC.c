#include <stdbool.h>

#include "emon32_samd.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "periph_rfm69.h"

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (16.8.8)
   * GCLK required for edge detection */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(EIC_GCLK_ID) | GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* EXTINT[0] is nDISABLE_EXT_INTF */
  portPinMux(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT, PORT_PMUX_PMUXE_A);
  EIC->CONFIG[0].reg = EIC_CONFIG_FILTEN0 | EIC_CONFIG_SENSE0_BOTH;
  EIC->INTENSET.reg  = EIC_INTENSET_EXTINT0;

  EIC->CTRL.reg = EIC_CTRL_ENABLE;
  while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY)
    ;

  NVIC_EnableIRQ(EIC_IRQn);
}

void irq_handler_eic(void) {
  if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT0) {
    if (portPinValue(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT)) {
      sercomExtIntfEnable();
    } else {
      sercomExtIntfDisable();
    }
    EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT0;
  }

  if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT14) {
    rfmInterrupt();
    EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT14;
  }
}
