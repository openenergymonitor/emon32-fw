#include "pulse.h"
#include "board_def.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emon32.h"

typedef enum PulseLvl_ { PULSE_LVL_LOW, PULSE_LVL_HIGH } PulseLvl_t;

static uint64_t     pulseCount[NUM_OPA];
static PulseCfg_t   pulseCfg[NUM_OPA];
static unsigned int pinValue[NUM_OPA];
static PulseLvl_t   pulseLvlLast[NUM_OPA];

PulseCfg_t *pulseGetCfg(const unsigned int index) {
  /* If no pulse counters attached or index out of range, return 0 */
  if ((0 == NUM_OPA) || (index > (NUM_OPA - 1u))) {
    return 0;
  }

  return &pulseCfg[index];
}

void pulseInit(const unsigned int index) {
  const uint_fast8_t opaPUs[] = {PIN_OPA1_PU, PIN_OPA2_PU};

  const unsigned int pin = pulseCfg[index].pin;

  /* Enable pull up if configured and allow a delay to charge RC */
  if (pulseCfg[index].puEn) {
    portPinDir(GRP_OPA, opaPUs[index], PIN_DIR_OUT);
    portPinDrv(GRP_OPA, opaPUs[index], PIN_DRV_SET);
    timerDelay_ms(1);
  }
  pinValue[index] = (unsigned int)portPinValue(GRP_OPA, pin);

  /* Use the first read value as the current state */
  pulseLvlLast[index] = (PulseLvl_t)pinValue[index];
}

void pulseSetCount(const unsigned int index, const uint64_t value) {
  pulseCount[index] = value;
}

uint64_t pulseGetCount(const unsigned int index) { return pulseCount[index]; }

void pulseUpdate(void) {
  unsigned int mask;
  PulseLvl_t   level;

  for (unsigned int i = 0; i < NUM_OPA; i++) {
    if (pulseCfg[i].active) {
      mask  = (1 << pulseCfg[i].periods) - 1u;
      level = pulseLvlLast[i];

      pinValue[i] <<= 1;
      pinValue[i] +=
          (unsigned int)portPinValue(pulseCfg[i].grp, pulseCfg[i].pin);

      if (0 == (pinValue[i] & mask)) {
        level = PULSE_LVL_LOW;
      } else if (mask == (pinValue[i] & mask)) {
        level = PULSE_LVL_HIGH;
      }

      switch (pulseCfg[i].edge) {
      case PULSE_EDGE_RISING:
        if ((PULSE_LVL_LOW == pulseLvlLast[i]) && (PULSE_LVL_HIGH == level)) {
          pulseCount[i]++;
        }
        break;
      case PULSE_EDGE_FALLING:
        if ((PULSE_LVL_HIGH == pulseLvlLast[i]) && (PULSE_LVL_LOW == level)) {
          pulseCount[i]++;
        }
        break;
      case PULSE_EDGE_BOTH:
        if (pulseLvlLast[i] != level) {
          pulseCount[i]++;
        }
      }

      pulseLvlLast[i] = level;
    }
  }
}
