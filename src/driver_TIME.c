#include "emon32_samd.h"

#include "driver_TIME.h"
#include "driver_USB.h"
#include "driver_WDT.h"
#include "emon32.h"

/*! @brief Common setup for 1 us resolution timer
 *  @param [in] delay : delay in us
 */
static void commonSetup(uint32_t delay);
static void timerSync(Tc *tc);

static volatile uint32_t timeMillisCounter  = 0;
static volatile uint32_t timeSecondsCounter = 0;

static bool TIMER_DELAYInUse = false;

/* Queue sized for expected callback load:
 * - 1x EEPROM async write (needs up to 6 callbacks: header + multi-page data +
 * retries)
 * - Additional spare for concurrent operations
 */
#define TIMER_CALLBACK_QUEUE_SIZE 8

typedef struct {
  TimerCallback_t callback;
  uint32_t        target_us;
  bool            active;
  bool            pending_exec; /* Set in ISR, cleared when executed */
} TimerCallbackEntry_t;

static volatile TimerCallbackEntry_t callbackQueue[TIMER_CALLBACK_QUEUE_SIZE] =
    {0};
static volatile uint32_t nextScheduledEvent_us = UINT32_MAX;

static void commonSetup(uint32_t delay) {
  /* Unmask match interrrupt, zero counter, set compare value */
  TIMER_DELAY->COUNT32.INTENSET.reg = TC_INTENSET_MC0;
  TIMER_DELAY->COUNT32.COUNT.reg    = 0u;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CC[0].reg = delay;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);
}

static void timerSync(Tc *tc) {
  /* All STATUS registers are at the same offset, use COUNT32 for access */
  while (tc->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY)
    ;
}

uint16_t timerADCPeriod(void) {
  return (F_TIMER_ADC / SAMPLE_RATE / VCT_TOTAL);
}

bool timerDelay_ms(uint16_t delay) { return timerDelay_us(delay * 1000u); }

bool timerDelay_us(uint32_t delay) {
  /* Return -1 if timer is already in use */
  if (TIMER_DELAYInUse) {
    return false;
  }

  TIMER_DELAYInUse = true;
  commonSetup(delay);

  /* Wait for timer to complete, then disable */
  while (0 == (TIMER_DELAY->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0))
    ;
  TIMER_DELAY->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
  TIMER_DELAY->COUNT32.INTFLAG.reg  = TC_INTFLAG_MC0;
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

  TIMER_DELAYInUse = 0;
  return true;
}

void timerDisable(void) {
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  NVIC_DisableIRQ(TIMER_DELAY_IRQn);
}

bool timerElapsedStart(void) {
  /* Return -1 if timer is already in use */
  if (TIMER_DELAYInUse) {
    return false;
  }

  TIMER_DELAYInUse = true;

  /* Mask match interrupt, zero counter, and start */
  TIMER_DELAY->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
  TIMER_DELAY->COUNT32.COUNT.reg    = 0u;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);

  TIMER_DELAYInUse = false;
  return true;
}

uint32_t timerElapsedStop(void) {
  /* Disable timer, and return value of COUNT */
  __disable_irq();
  const uint32_t elapsed = TIMER_DELAY->COUNT32.COUNT.reg;
  __enable_irq();
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);
  return elapsed;
}

uint32_t timerMicros(void) {
  /* Resynchronise COUNT32.COUNT, and then return the result */
  TIMER_TICK->COUNT32.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);
  timerSync(TIMER_TICK);
  return TIMER_TICK->COUNT32.COUNT.reg;
}

uint32_t timerMicrosDelta(const uint32_t prevMicros) {
  uint32_t delta         = 0;
  uint32_t timeMicrosNow = timerMicros();

  /* Check for wrap (every ~1 h) */
  if (prevMicros > timeMicrosNow) {
    delta = (UINT32_MAX - prevMicros) + timeMicrosNow + 1u;
    ;
  } else {
    delta = timeMicrosNow - prevMicros;
  }

  return delta;
}

uint32_t timerMillis(void) { return timeMillisCounter; }

uint32_t timerMillisDelta(const uint32_t prevMillis) {
  uint32_t delta = 0;

  /* Check for wrap around (every 49 days, so rare!) */
  if (prevMillis > timeMillisCounter) {
    delta = (UINT32_MAX - prevMillis) + timeMillisCounter;
  } else {
    delta = timeMillisCounter - prevMillis;
  }
  return delta;
}

void timerSetup(void) {
  /* TIMER_ADC is used to trigger ADC sampling at constant rate. Enable APB
   * clock, run from generator 3 (OSC8M @ F_PERIPH).
   */
  PM->APBCMASK.reg |= TIMER_ADC_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_ADC_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Configure as 16bit counter (F_PERIPH / 8) -> F_TIMER_ADC.
   * In MFRQ mode, the CC0 register is used as the period.
   */
  TIMER_ADC->COUNT16.CTRLA.reg =
      TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_WAVEGEN_MFRQ |
      TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC_RESYNC;

  /* TIMER_ADC MC0 match event to trigger ADC */
  TIMER_ADC->COUNT16.EVCTRL.reg |= TC_EVCTRL_MCEO0;

  /* TIMER_ADC is running at 1 MHz, each tick is 1 us
   * PER, COUNT, and Enable require synchronisation (30.6.6)
   * COUNT is -1 to account for the wrap around
   */
  TIMER_ADC->COUNT16.CC[0].reg = timerADCPeriod() - 1u;
  timerSync(TIMER_ADC);
  TIMER_ADC->COUNT16.COUNT.reg = 0u;
  timerSync(TIMER_ADC);
  TIMER_ADC->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_ADC);

  /* TIMER_DELAY is used as the delay and elapsed time counter
   * Enable APB clock, set TIMER_DELAY to generator 3 @ F_PERIPH
   * Enable the interrupt for Compare Match, do not route to NVIC
   */
  PM->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_DELAY_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;
  TIMER_DELAY->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |
                                   TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                   TC_CTRLA_PRESCSYNC_RESYNC;

  /* TIMER_TICK is used for accurate microsecond time keeping and hardware
   * timer callback queue. Setup at 1 MHz (1 us), and route to NVIC for
   * callback queue (MC1). The 1 ms tick is now handled by SysTick.
   */
  PM->APBCMASK.reg |= TIMER_TICK_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_TICK_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  TIMER_TICK->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
  while (TIMER_TICK->COUNT32.CTRLA.reg & TC_CTRLA_SWRST)
    ;

  TIMER_TICK->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |
                                  TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                  TC_CTRLA_PRESCSYNC_PRESC;

  /* MC1 will be enabled dynamically when callbacks are scheduled */
  NVIC_EnableIRQ(TIMER_TICK_IRQn);
  TIMER_TICK->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_TICK);

  /* Setup SysTick for 1 ms periodic tick
   * Core clock is 48 MHz, so 48000 ticks = 1 ms
   */
  SysTick->LOAD = 48000u - 1u;
  SysTick->VAL  = 0u;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | /* Use processor clock */
                  SysTick_CTRL_TICKINT_Msk |   /* Enable interrupt */
                  SysTick_CTRL_ENABLE_Msk;     /* Enable SysTick */
}

uint32_t timerUptime(void) { return timeSecondsCounter; }

void timerUptimeIncr(void) { timeSecondsCounter++; }

/* Timed callback queue implementation */

bool timerScheduleCallback(TimerCallback_t callback, uint32_t delay_us) {
  if (!callback) {
    return false;
  }

  /* Get current time OUTSIDE critical section */
  uint32_t current_us = timerMicros();
  uint32_t target_us  = current_us + delay_us;

  /* Short critical section - only memory operations */
  __disable_irq();

  /* Find an empty slot in the queue */
  int slot = -1;
  for (int i = 0; i < TIMER_CALLBACK_QUEUE_SIZE; i++) {
    if (!callbackQueue[i].active) {
      slot                          = i;
      callbackQueue[i].callback     = callback;
      callbackQueue[i].target_us    = target_us;
      callbackQueue[i].active       = true;
      callbackQueue[i].pending_exec = false;
      break;
    }
  }

  /* Check if this is the next event (memory read only) */
  uint32_t time_until_new  = target_us - current_us;
  uint32_t time_until_next = nextScheduledEvent_us - current_us;
  bool     need_update     = (slot >= 0) && (time_until_new < time_until_next);

  if (need_update) {
    nextScheduledEvent_us = target_us;
  }

  __enable_irq(); /* End critical section BEFORE hardware access */

  /* Hardware register access OUTSIDE critical section */
  if (need_update) {
    TIMER_TICK->COUNT32.INTENSET.reg = TC_INTENSET_MC1;
    TIMER_TICK->COUNT32.CC[1].reg    = target_us;
    timerSync(TIMER_TICK); /* Sync outside critical section */
  }

  return (slot >= 0);
}

bool timerCancelCallback(TimerCallback_t callback) {
  if (!callback) {
    return false;
  }

  __disable_irq();

  for (int i = 0; i < TIMER_CALLBACK_QUEUE_SIZE; i++) {
    if (callbackQueue[i].active && callbackQueue[i].callback == callback) {
      callbackQueue[i].active = false;
      __enable_irq();
      return true;
    }
  }

  __enable_irq();
  return false;
}

bool timerCallbackPending(TimerCallback_t callback) {
  if (!callback) {
    return false;
  }

  __disable_irq();

  for (int i = 0; i < TIMER_CALLBACK_QUEUE_SIZE; i++) {
    if (callbackQueue[i].active && callbackQueue[i].callback == callback) {
      __enable_irq();
      return true;
    }
  }

  __enable_irq();
  return false;
}

/*! @brief Check callback queue and mark callbacks ready for execution (ISR)
 *  @param [in] current_us : current microsecond count
 *  @note Called from ISR - only sets flags, does NOT execute callbacks
 */
static void processCallbackQueue(uint32_t current_us) {
  uint32_t next_event = UINT32_MAX;
  bool     found_next = false;

  for (int i = 0; i < TIMER_CALLBACK_QUEUE_SIZE; i++) {
    if (callbackQueue[i].active) {
      /* Check if this callback is due (accounting for wrap) */
      uint32_t time_until = callbackQueue[i].target_us - current_us;

      if (time_until > 0x80000000u) {
        /* Time has passed - mark for execution in main loop */
        callbackQueue[i].pending_exec = true;
        callbackQueue[i].active       = false;
      } else {
        /* Still pending - check if it's the next event */
        if (!found_next || time_until < (next_event - current_us)) {
          next_event = callbackQueue[i].target_us;
          found_next = true;
        }
      }
    }
  }

  /* Update next scheduled event */
  if (found_next) {
    nextScheduledEvent_us         = next_event;
    TIMER_TICK->COUNT32.CC[1].reg = next_event;
    timerSync(TIMER_TICK);
  } else {
    /* No more pending events */
    nextScheduledEvent_us            = UINT32_MAX;
    TIMER_TICK->COUNT32.INTENCLR.reg = TC_INTENCLR_MC1;
  }
}

/*! @brief Execute pending callbacks (main loop context)
 *  @details Call this from main loop to execute callbacks marked by ISR
 *
 *  SAFETY INVARIANT: The callback pointer is read from volatile memory (line
 * 354) and used outside the critical section (line 360). This is safe because
 * of the following invariant maintained by processCallbackQueue():
 *    - When pending_exec is set TRUE, active is set FALSE (line 321-322)
 *    - Once active is FALSE, the ISR will never modify that queue entry
 *    - Therefore, callback pointer remains stable after pending_exec is set
 *
 *  This design avoids executing callbacks in ISR context while maintaining
 * safety.
 */
void timerProcessPendingCallbacks(void) {
  for (int i = 0; i < TIMER_CALLBACK_QUEUE_SIZE; i++) {
    if (callbackQueue[i].pending_exec) {
      __disable_irq();
      bool            should_exec   = callbackQueue[i].pending_exec;
      TimerCallback_t cb            = callbackQueue[i].callback;
      callbackQueue[i].pending_exec = false;
      __enable_irq();

      /* Execute callback in main loop context (safe, can block)
       * The callback pointer 'cb' is safe to use here because the queue entry
       * is no longer active, so the ISR won't modify it.
       */
      if (should_exec && cb) {
        cb();
      }
    }
  }
}

/*! @brief SysTick interrupt handler for 1 ms periodic tick
 *  @details Handles millisecond counting and sets the 1kHz event flag
 */
void irq_handler_sys_tick(void) {
  timeMillisCounter++;
  emon32EventSet(EVT_TICK_1kHz);
}

/*! @brief TIMER_TICK interrupt handler for scheduled callbacks
 *  @details Handles the hardware timer callback queue (MC1)
 */
void IRQ_TIMER_TICK(void) {
  uint32_t intflags = TIMER_TICK->COUNT32.INTFLAG.reg;

  /* Handle scheduled callbacks (MC1) */
  if (intflags & TC_INTFLAG_MC1) {
    TIMER_TICK->COUNT32.INTFLAG.reg = TC_INTFLAG_MC1;
    uint32_t current_us             = timerMicros();
    processCallbackQueue(current_us);
  }
}
