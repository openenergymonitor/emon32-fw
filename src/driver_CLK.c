#include "driver_SAMD.h"
#include "emon32_samd.h"
#include "fuses.h"

static inline void SYNC_DFLL(void) {
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY))
    ;
}

void clkSetup(void) {
  /* Set up brown out detector to catch:
   *   - Brown in at startup before setting waiting states @ 48 MHz
         Suspend setting up the 48 MHz clock until the power is established
   *   - Brown out trigger reset
   * https://blog.thea.codes/sam-d21-brown-out-detector/
   */

  /* Disable BOD during configuration to avoid spurious reset */
  SYSCTRL->BOD33.bit.ENABLE = 0;
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY))
    ;

  /* Configure BOD to watch the system voltage @3V3
   *   - Vmin = 3.07 V (typical, Table 37-21)
   *   - Enable hysteresis
   */
  SYSCTRL->BOD33.reg =
      SYSCTRL_BOD33_LEVEL(48) | SYSCTRL_BOD33_ACTION_NONE | SYSCTRL_BOD33_HYST;

  SYSCTRL->BOD33.bit.ENABLE = 1;
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY))
    ;

  /* PCLKSR.BOD33DET is 1 when the voltage is too low */
  while (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_BOD33DET)
    ;

  /* Now at ~3V3, set BOD33 to reset the micro on brown out */
  SYSCTRL->BOD33.bit.ENABLE = 0;
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY))
    ;
  SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ACTION_RESET;
  SYSCTRL->BOD33.bit.ENABLE = 1;
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY))
    ;

  /* Boost OSC8M to 8 MHz from initial 1 MHz */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val;

  /* 48 MHz DFLL for core with USB clock recovery
   *  1. Enable OSC32K clock (assuming no external crystal)
   *  2. Set GCLK Gen 1 source as OSC32K
   *  3. Set GCLK Gen 1 as source for GCLK Multiplexor 0 (DFLL48M reference)
   *  4. Enable DFLL48M CLK
   *  5. Set flash wait state to 1 for 48 MHz core
   *  6. Switch GCLK Gen 0 to DFLL48M - core will run at 48 MHz
   */

  /* 1. OSC32K setup */
  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(samdCalibration(CAL_OSC32K)) |
                        SYSCTRL_OSC32K_STARTUP(0x6u) | SYSCTRL_OSC32K_EN32K |
                        SYSCTRL_OSC32K_ENABLE;
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY))
    ;

  /* Reset clock system; 32K sources are not affected (Section 14.7)
   * CTRL.SWRST and STATUS.SYNCBUSY will be cleared simultaneously
   * (Section 15.8.1)
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;
  while ((GCLK->CTRL.reg & GCLK_CTRL_SWRST) &&
         (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY))
    ;

  /* 2. OSC32K -> generator 1 */
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_ID(1u) | GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  /* Gen 1 -> Mux 0 */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(0u) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  /* 4. Enable DFLL48M in closed loop mode (Section 17.6.7.1) */
  /* ERRATA 9905: DFLL48 must be requested before configuration */
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  SYNC_DFLL();

  const uint32_t coarse = OTP4_FUSES->FUSES0.bit.DFLL48M_COARSE_CAL;
  const uint32_t fine   = OTP4_FUSES->FUSES1.bit.DFLL48M_FINE_CAL;

  SYSCTRL->DFLLVAL.reg =
      SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);
  SYNC_DFLL();

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_CCDIS;
  SYNC_DFLL();

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000) | SYSCTRL_DFLLMUL_CSTEP(1) |
                         SYSCTRL_DFLLMUL_FSTEP(0xA);
  SYNC_DFLL();

  SYSCTRL->DFLLCTRL.bit.MODE = 1;
  SYNC_DFLL();
  SYSCTRL->DFLLCTRL.bit.ENABLE = 1;
  SYNC_DFLL();

  /* 5. Flash wait - 1 @ 3V3/48 MHz (37.12 NVM Characteristics) */
  NVMCTRL->CTRLB.bit.RWS = 1;

  /* 6. DFLL48M -> generator 0 */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(0u);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0u) | GCLK_GENCTRL_SRC_DFLL48M |
                      GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  /* Connect OSC8M to generator 3 */
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_ID(3u) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}
