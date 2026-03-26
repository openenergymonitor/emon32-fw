#include <stddef.h>

#include "board_def.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emon32.h"
#include "emon32_samd.h"
#include "pulse.h"

typedef enum PulseLvl_ { PULSE_LVL_LOW, PULSE_LVL_HIGH } PulseLvl_t;

static uint32_t   pulseCount[NUM_OPA];
static PulseCfg_t pulseCfg[NUM_OPA];
static PulseLvl_t pulseLvlLast[NUM_OPA];

PulseCfg_t *pulseGetCfg(const size_t index) { return &pulseCfg[index]; }

void pulseInit(const size_t index) {
  static const uint8_t opaPUs[] = {PIN_OPA1_PU, PIN_OPA2_PU};

  const uint8_t pin = pulseCfg[index].pin;

  /* Enable pull up if configured and allow a delay to charge RC. OPA1/2 have an
   * external pull-up; for OPA3 enable the weak internal pull-up */
  if (pulseCfg[index].puEn) {
    if (index < 2u) {
      portPinDir(GRP_OPA, opaPUs[index], PIN_DIR_OUT);
      portPinDrv(GRP_OPA, opaPUs[index], PIN_DRV_SET);
    } else {
      portPinCfg(GRP_OPA, pin, PORT_PINCFG_PULLEN, PIN_CFG_SET);
      portPinDrv(GRP_OPA, pin, PIN_DRV_SET);
    }
    timerDelay_ms(1);
  } else {
    /* Disable external pull ups */
    if (index < 2u) {
      portPinDir(GRP_OPA, opaPUs[index], PIN_DIR_IN);
      portPinCfg(GRP_OPA, opaPUs[index], PORT_PINCFG_PULLEN, PIN_CFG_CLR);
    }

    /* Enable weak pull down */
    portPinCfg(GRP_OPA, pin, PORT_PINCFG_PULLEN, PIN_CFG_SET);
    portPinDrv(GRP_OPA, pin, PIN_DRV_CLR);
  }

  pulseLvlLast[index] = (uint8_t)portPinValue(GRP_OPA, pin);
}

void pulseSetCount(const size_t index, const uint32_t value) {
  pulseCount[index] = value;
}

uint32_t pulseGetCount(const size_t index) { return pulseCount[index]; }

void pulseUpdate(void) {
  PulseLvl_t      level               = PULSE_LVL_LOW;
  static uint32_t tLastPulse[NUM_OPA] = {0};

  for (size_t i = 0; i < NUM_OPA; i++) {
    if (pulseCfg[i].active) {
      bool incrPulse = false;
      level          = pulseLvlLast[i];

      /* features/pulse_noshift - remove the internal 8 ms debounce */
      level = (uint8_t)portPinValue(pulseCfg[i].grp, pulseCfg[i].pin)
                  ? PULSE_LVL_HIGH
                  : PULSE_LVL_LOW;

      switch (pulseCfg[i].edge) {
      case PULSE_EDGE_RISING:
        if ((PULSE_LVL_LOW == pulseLvlLast[i]) && (PULSE_LVL_HIGH == level)) {
          incrPulse = true;
        }
        break;
      case PULSE_EDGE_FALLING:
        if ((PULSE_LVL_HIGH == pulseLvlLast[i]) && (PULSE_LVL_LOW == level)) {
          incrPulse = true;
        }
        break;
      case PULSE_EDGE_BOTH:
        if (pulseLvlLast[i] != level) {
          incrPulse = true;
        }
      }

      if (incrPulse) {
        if (timerMillisDelta(tLastPulse[i]) >= pulseCfg[i].periods) {
          pulseCount[i]++;
          tLastPulse[i] = timerMillis();
        }
      }

      pulseLvlLast[i] = level;
    }
  }
}
