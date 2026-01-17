/*
 * Benchmark for asm_math functions on ARM Cortex-M0+ target
 *
 * This is a standalone test firmware that replaces emon32.c main().
 * Build with: make -f tests/Makefile.asm_math
 *
 * Results are output via serial at 115200 baud.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "emon32_samd.h"

#include "board_def.h"
#include "driver_CLK.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "asm_math.h"
#include "printf.h"
#include "qfplib-m0-full.h"

/* Number of iterations for performance tests */
#define PERF_ITERATIONS 100000

/*************************************
 * Stubs for unused dependencies
 *************************************/

/* Stub for event system (used by timer interrupt) */
void emon32EventSet(uint32_t evt) { (void)evt; }

/* Stubs for newlib syscalls - we don't use file I/O */
void *_sbrk(int incr) {
  (void)incr;
  return (void *)-1;
}

int _close(int fd) {
  (void)fd;
  return -1;
}

int _lseek(int fd, int offset, int whence) {
  (void)fd;
  (void)offset;
  (void)whence;
  return -1;
}

int _read(int fd, char *buf, int count) {
  (void)fd;
  (void)buf;
  (void)count;
  return -1;
}

int _write(int fd, char *buf, int count) {
  (void)fd;
  (void)buf;
  (void)count;
  return -1;
}

/*************************************
 * putchar_ for printf
 *************************************/
void putchar_(char c) { uartPutcBlocking(SERCOM_UART, c); }

/*************************************
 * Test vectors
 *************************************/

static const uint32_t test_unsigned[] = {
    0,          1,      2,      3,      255,     256,        65535,
    65536,      0x7FFF, 0x8000, 0xFFFF, 0x10000, 0x7FFFFFFF, 0x80000000,
    0xFFFFFFFF, 12345,  54321,  100000, 1000000, 0xDEADBEEF};

static const int32_t test_signed[] = {
    0,     1,      -1,    2,      -2,         127,         -128,
    32767, -32768, 65535, -65536, 0x7FFFFFFF, -2147483647, (int32_t)0x80000000,
    12345, -12345, 54321, -54321, 1000000,    -1000000};

/* Test pairs for multiplication (a, b) */
static const int32_t test_mul_pairs[][2] = {
    {0, 0},
    {1, 1},
    {-1, 1},
    {1, -1},
    {-1, -1},
    {100, 200},
    {-100, 200},
    {100, -200},
    {-100, -200},
    {32767, 32767},
    {-32768, 32768},
    {65535, 65536},
    {0x7FFF, 0x7FFF},
    {0x7FFFFFFF, 2},
    {(int32_t)0x80000000, 1},
    {12345, 67890},
    {-12345, 67890},
    {0x1234, 0x5678},
    {-0x1234, -0x5678},
};

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/*************************************
 * Validation tests
 *************************************/

static bool test_usqr64_validation(void) {
  printf_("Testing usqr64 validation...\r\n");
  bool pass = true;

  for (size_t i = 0; i < ARRAY_SIZE(test_unsigned); i++) {
    uint32_t x        = test_unsigned[i];
    uint64_t result   = usqr64(x);
    uint64_t expected = (uint64_t)x * x;

    if (result != expected) {
      printf_(
          "  FAIL: usqr64(0x%08lX) = 0x%08lX%08lX, expected 0x%08lX%08lX\r\n",
          (unsigned long)x, (unsigned long)(result >> 32),
          (unsigned long)(result & 0xFFFFFFFF), (unsigned long)(expected >> 32),
          (unsigned long)(expected & 0xFFFFFFFF));
      pass = false;
    }
  }

  if (pass) {
    printf_("  PASS: All usqr64 tests passed\r\n");
  }
  return pass;
}

static bool test_ssqr64_validation(void) {
  printf_("Testing ssqr64 validation...\r\n");
  bool pass = true;

  for (size_t i = 0; i < ARRAY_SIZE(test_signed); i++) {
    int32_t x        = test_signed[i];
    int64_t result   = ssqr64(x);
    int64_t expected = (int64_t)x * x;

    if (result != expected) {
      printf_("  FAIL: ssqr64(%ld) = 0x%08lX%08lX, expected 0x%08lX%08lX\r\n",
              (long)x, (unsigned long)(result >> 32),
              (unsigned long)(result & 0xFFFFFFFF),
              (unsigned long)(expected >> 32),
              (unsigned long)(expected & 0xFFFFFFFF));
      pass = false;
    }
  }

  if (pass) {
    printf_("  PASS: All ssqr64 tests passed\r\n");
  }
  return pass;
}

static bool test_smul64_validation(void) {
  printf_("Testing smul64 validation...\r\n");
  bool pass = true;

  /* Test with predefined pairs */
  for (size_t i = 0; i < ARRAY_SIZE(test_mul_pairs); i++) {
    int32_t a        = test_mul_pairs[i][0];
    int32_t b        = test_mul_pairs[i][1];
    int64_t result   = smul64(a, b);
    int64_t expected = (int64_t)a * b;

    if (result != expected) {
      printf_(
          "  FAIL: smul64(%ld, %ld) = 0x%08lX%08lX, expected 0x%08lX%08lX\r\n",
          (long)a, (long)b, (unsigned long)(result >> 32),
          (unsigned long)(result & 0xFFFFFFFF), (unsigned long)(expected >> 32),
          (unsigned long)(expected & 0xFFFFFFFF));
      pass = false;
    }
  }

  /* Test all combinations of test_signed values */
  for (size_t i = 0; i < ARRAY_SIZE(test_signed); i++) {
    for (size_t j = 0; j < ARRAY_SIZE(test_signed); j++) {
      int32_t a        = test_signed[i];
      int32_t b        = test_signed[j];
      int64_t result   = smul64(a, b);
      int64_t expected = (int64_t)a * b;

      if (result != expected) {
        printf_("  FAIL: smul64(%ld, %ld)\r\n", (long)a, (long)b);
        pass = false;
      }
    }
  }

  if (pass) {
    printf_("  PASS: All smul64 tests passed\r\n");
  }
  return pass;
}

/*************************************
 * Performance benchmarks
 *************************************/

static void test_performance(void) {
  printf_("\r\nPerformance tests (%d iterations):\r\n", PERF_ITERATIONS);

  volatile uint64_t sink = 0;
  uint32_t          start, elapsed;

  /* Test usqr64 (ASM) */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += usqr64((uint32_t)i);
  }
  elapsed = timerMicrosDelta(start);
  printf_("  usqr64 (ASM):   %" PRIu32 " us total\r\n", elapsed);

  /* Test standard 64-bit multiply for comparison */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    uint32_t x = (uint32_t)i;
    sink += (uint64_t)x * x;
  }
  elapsed = timerMicrosDelta(start);
  printf_("  (u64)x * x:     %" PRIu32 " us total\r\n", elapsed);

  /* Test ssqr64 (ASM) */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += (uint64_t)ssqr64((int32_t)i - PERF_ITERATIONS / 2);
  }
  elapsed = timerMicrosDelta(start);
  printf_("  ssqr64 (ASM):   %" PRIu32 " us total\r\n", elapsed);

  /* Test standard signed 64-bit multiply */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    int32_t x = (int32_t)i - PERF_ITERATIONS / 2;
    sink += (uint64_t)((int64_t)x * x);
  }
  elapsed = timerMicrosDelta(start);
  printf_("  (i64)x * x:     %" PRIu32 " us total\r\n", elapsed);

  /* Test smul64 (ASM) */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += (uint64_t)smul64((int32_t)i, (int32_t)(PERF_ITERATIONS - i));
  }
  elapsed = timerMicrosDelta(start);
  printf_("  smul64 (ASM):   %" PRIu32 " us total\r\n", elapsed);

  /* Test standard signed 64-bit multiply a*b */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    int32_t a = (int32_t)i;
    int32_t b = (int32_t)(PERF_ITERATIONS - i);
    sink += (uint64_t)((int64_t)a * b);
  }
  elapsed = timerMicrosDelta(start);
  printf_("  (i64)a * b:     %" PRIu32 " us total\r\n", elapsed);

  (void)sink; /* Prevent optimization */
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
  printf_("  ASM Math Functions Test\r\n");
  printf_("  ARM Cortex-M0+ Target\r\n");
  printf_("================================\r\n\r\n");

  bool all_pass = true;

  all_pass &= test_usqr64_validation();
  all_pass &= test_ssqr64_validation();
  all_pass &= test_smul64_validation();

  test_performance();

  printf_("\r\n=== %s ===\r\n",
          all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED");

  /* Infinite loop */
  for (;;) {
    samdSleepIdle();
  }
}
