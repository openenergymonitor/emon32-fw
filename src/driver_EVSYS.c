#include "board_def.h"
#include "emon32_samd.h"

void evsysSetup(void) {
  /* Enable APB clock, connect to generator 3 (OSC8M @ F_PERIPH)
   * Each EVSYS channel has a separate GCLK channel
   */
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EVSYS_GCLK_ID_0) |
                      GCLK_CLKCTRL_GEN(GCLK_PERIPH) | GCLK_CLKCTRL_CLKEN;

  /* Connect TC1 -> ADC (Section 24.6.2.1) - select channel N-1 */
  EVSYS->USER.reg =
      EVSYS_USER_USER(EVSYS_ID_USER_ADC_START) | EVSYS_USER_CHANNEL(1u);

  /* ADC path must be async
   * NB : the channel ID in the CHANNEL register is the channel number; for
   * the USER register it is (n - 1).
   */
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0u) |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(TIMER_ADC_EVT_SRC);
}
