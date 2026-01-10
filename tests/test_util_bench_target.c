/*
 * Benchmark for util functions on ARM Cortex-M0+ target
 *
 * This is a standalone test firmware that replaces emon32.c main().
 * Build with: make TEST=test_util_bench_target
 *
 * Results are output via serial at 115200 baud.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "emon32_samd.h"

#include "board_def.h"
#include "driver_CLK.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"

#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

/* itoa is non-standard but provided by newlib */
extern char *itoa(int value, char *str, int base);

/* Number of iterations for each test */
#define ITERATIONS 1000

/*************************************
 * Stubs for unused dependencies
 *************************************/

/* Stub for event system (used by timer interrupt) */
void emon32EventSet(const EVTSRC_t evt) { (void)evt; }

/* Stub for sbrk (heap allocation) - we don't use malloc */
void *_sbrk(int incr) {
  (void)incr;
  return (void *)-1;
}

/*************************************
 * putchar_ for printf
 *************************************/
void putchar_(char c) { uartPutcBlocking(SERCOM_UART, c); }

/*************************************
 * Benchmark functions
 *************************************/

static void benchItoa(void) {
  char          buf[32];
  uint32_t      tStart, tEnd;
  int32_t       testVals[] = {0, 1, -1, 123, -456, 999999, -999999};
  const int32_t numVals    = sizeof(testVals) / sizeof(testVals[0]);

  printf_("\r\n=== ITOA Benchmark ===\r\n");

  /* utilItoa */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numVals; v++) {
      utilItoa(buf, testVals[v], ITOA_BASE10);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("utilItoa:  %" PRIu32 " us\r\n", tEnd);

  /* itoa (newlib) */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numVals; v++) {
      itoa(testVals[v], buf, 10);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("itoa:      %" PRIu32 " us\r\n", tEnd);
}

static void benchFtoa(void) {
  char          buf[32];
  uint32_t      tStart, tEnd;
  float         testVals[] = {0.0f, 1.0f, -1.0f, 123.45f, -456.78f};
  const int32_t numVals    = sizeof(testVals) / sizeof(testVals[0]);

  printf_("\r\n=== FTOA Benchmark ===\r\n");

  /* utilFtoa */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numVals; v++) {
      utilFtoa(buf, testVals[v]);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("utilFtoa:  %" PRIu32 " us\r\n", tEnd);

  /* No standard ftoa() exists */
  printf_("ftoa:      N/A (no standard function)\r\n");
}

static void benchAtoi(void) {
  uint32_t      tStart, tEnd;
  const char   *testStrs[] = {"0", "1", "-1", "123", "-456", "999999"};
  const int32_t numStrs    = sizeof(testStrs) / sizeof(testStrs[0]);

  printf_("\r\n=== ATOI Benchmark ===\r\n");

  /* utilAtoi (no longer modifies buffer) */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numStrs; v++) {
      utilAtoi((char *)testStrs[v], ITOA_BASE10);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("utilAtoi:  %" PRIu32 " us\r\n", tEnd);

  /* atoi */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numStrs; v++) {
      atoi(testStrs[v]);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("atoi:      %" PRIu32 " us\r\n", tEnd);
}

static void benchAtof(void) {
  char          buf[32];
  uint32_t      tStart, tEnd;
  const char   *testStrs[] = {"0.0", "1.0", "-1.0", "123.45", "-456.78"};
  const int32_t numStrs    = sizeof(testStrs) / sizeof(testStrs[0]);

  printf_("\r\n=== ATOF Benchmark ===\r\n");

  /* utilAtof (no longer modifies buffer) */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numStrs; v++) {
      /* Cast away const - utilAtof no longer modifies input */
      utilAtof((char *)testStrs[v]);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("utilAtof:  %" PRIu32 " us\r\n", tEnd);

  /* atof */
  tStart = timerMicros();
  for (int32_t i = 0; i < ITERATIONS; i++) {
    for (int32_t v = 0; v < numStrs; v++) {
      atof(testStrs[v]);
    }
  }
  tEnd = timerMicrosDelta(tStart);
  printf_("atof:      %" PRIu32 " us\r\n", tEnd);

  (void)buf; /* Unused now */
}

static void benchCorrectness(void) {
  char buf1[32], buf2[32];

  printf_("\r\n=== Correctness Check ===\r\n");

  /* ITOA */
  int32_t testInts[] = {0, 1, -1, 123, -456, 999999};
  printf_("ITOA:\r\n");
  for (size_t i = 0; i < sizeof(testInts) / sizeof(testInts[0]); i++) {
    utilItoa(buf1, testInts[i], ITOA_BASE10);
    itoa(testInts[i], buf2, 10);
    printf_("  %ld: util='%s' itoa='%s' %s\r\n", (long)testInts[i], buf1, buf2,
            strcmp(buf1, buf2) == 0 ? "OK" : "MISMATCH");
  }

  /* FTOA - no standard comparison available */
  float testFloats[] = {0.0f, 1.0f, -1.0f, 123.45f, -456.78f};
  printf_("FTOA (no standard comparison):\r\n");
  for (size_t i = 0; i < sizeof(testFloats) / sizeof(testFloats[0]); i++) {
    utilFtoa(buf1, testFloats[i]);
    printf_("  input=%.2f: util='%s'\r\n", (double)testFloats[i], buf1);
  }
}

/*************************************
 * Main entry point
 *************************************/

int main(void) {
  /* Initialize clocks, ports, and UART */
  clkSetup();
  timerSetup();
  portSetup();
  sercomSetup();
  uartEnableTx(SERCOM_UART);

  /* Wait for UART to stabilize */
  timerDelay_ms(100);

  printf_("\r\n\r\n");
  printf_("================================\r\n");
  printf_("  Util Functions Benchmark\r\n");
  printf_("  ARM Cortex-M0+ Target Test\r\n");
  printf_("================================\r\n");
  printf_("Iterations: %d per value\r\n", ITERATIONS);

  benchItoa();
  benchFtoa();
  benchAtoi();
  benchAtof();
  benchCorrectness();

  printf_("\r\n=== Benchmark Complete ===\r\n");
  printf_("Note: utilAtoi/utilAtof modify input buffer.\r\n");

  /* Infinite loop */
  for (;;) {
    samdSleepIdle();
  }
}
