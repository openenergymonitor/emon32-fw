#include "driver_PORT.h"
#include "board_def.h"
#include "emon32_samd.h"

static void input(const uint8_t grp, const uint8_t pin, const bool high);

static void input(const uint8_t grp, const uint8_t pin, const bool high) {
  portPinDir(grp, pin, PIN_DIR_IN);
  portPinCfg(grp, pin, PORT_PINCFG_PULLEN, PIN_CFG_SET);
  portPinDrv(grp, pin, (high ? PIN_DRV_SET : PIN_DRV_CLR));
  portPinCfg(grp, pin, PORT_PINCFG_INEN, PIN_CFG_SET);
}

void portPinCfg(const uint8_t grp, const uint8_t pin, const uint8_t cfg,
                const PINCFG_t cs) {
  if (PIN_CFG_SET == cs) {
    PORT->Group[grp].PINCFG[pin].reg |= cfg;
  } else {
    PORT->Group[grp].PINCFG[pin].reg &= ~cfg;
  }
}

void portPinDir(const uint8_t grp, const uint8_t pin, const PINDIR_t mode) {
  if (PIN_DIR_IN == mode) {
    PORT->Group[grp].DIRCLR.reg = (1u << pin);
  } else {
    PORT->Group[grp].DIRSET.reg = (1u << pin);
  }
  PORT->Group[grp].PINCFG[pin].bit.INEN = 1;
}

void portPinDrv(const uint8_t grp, const uint8_t pin, const PINDRV_t drv) {
  switch (drv) {
  case PIN_DRV_CLR:
    PORT->Group[grp].OUTCLR.reg = (1u << pin);
    break;
  case PIN_DRV_SET:
    PORT->Group[grp].OUTSET.reg = (1u << pin);
    break;
  case PIN_DRV_TGL:
    PORT->Group[grp].OUTTGL.reg = (1u << pin);
    break;
  }
}

void portPinMux(const uint8_t grp, const uint8_t pin, const uint8_t mux) {
  PORT->Group[grp].PINCFG[pin].bit.PMUXEN = 1;
  if (pin & 1u) {
    PORT->Group[grp].PMUX[pin >> 1].bit.PMUXO = (uint8_t)(mux & 0x0Fu);
  } else {
    PORT->Group[grp].PMUX[pin >> 1].bit.PMUXE = (uint8_t)(mux & 0x0Fu);
  }
}

void portPinMuxClear(const uint8_t grp, const uint8_t pin) {
  PORT->Group[grp].PINCFG[pin].bit.PMUXEN = 0;
}

bool portPinValue(const uint8_t grp, const uint8_t pin) {
  return (0u == (PORT->Group[grp].IN.reg & (1u << pin))) ? false : true;
}

void portSetup(void) {
  extern const uint8_t pinsGPIO_Out[][2];
  extern const uint8_t pinsGPIO_In[][2];
  extern const uint8_t pinsUnused[][2];

  /* GPIO outputs - also enable read buffer */
  for (uint32_t i = 0; pinsGPIO_Out[i][0] != 0xFF; i++) {
    portPinDir(pinsGPIO_Out[i][0], pinsGPIO_Out[i][1], PIN_DIR_OUT);
    /* RFM69 !SS must be HIGH */
    if ((GRP_SERCOM_SPI == pinsGPIO_Out[i][0]) &&
        (PIN_SPI_RFM_SS == pinsGPIO_Out[i][1])) {
      portPinDrv(GRP_PINA, PIN_SPI_RFM_SS, PIN_DRV_SET);
    }
    portPinCfg(pinsGPIO_Out[i][0], pinsGPIO_Out[i][1], PORT_PINCFG_INEN,
               PIN_CFG_SET);
  }

  /* GPIO inputs - all inputs currently need pull ups, so default enable */
  for (uint32_t i = 0; pinsGPIO_In[i][0] != 0xFF; i++) {
    if (GRP_OPA == pinsGPIO_In[i][0]) {
      uint8_t p = pinsGPIO_In[i][1];
      if ((PIN_OPA1 == p) || (PIN_OPA2 == p) || (PIN_OPA1_PU == p) ||
          (PIN_OPA2_PU == p)) {
        input(pinsGPIO_In[i][0], pinsGPIO_In[i][1], false);
      }
    } else {
      input(pinsGPIO_In[i][0], pinsGPIO_In[i][1], true);
    }
  }

  /* External interface disable is idle low, invert pull */
  input(GRP_DISABLE_EXT, PIN_DISABLE_EXT, false);

  /* Unused pins: input, pull down (Table 23-2) */
  for (uint32_t i = 0; pinsUnused[i][0] != 0xFF; i++) {
    portPinDir(pinsUnused[i][0], pinsUnused[i][1], PIN_DIR_IN);
    portPinCfg(pinsUnused[i][0], pinsUnused[i][1], PORT_PINCFG_PULLEN,
               PIN_CFG_SET);
  }
}
