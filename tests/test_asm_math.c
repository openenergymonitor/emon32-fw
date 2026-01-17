#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef HOSTED
/* Reference C implementations for host testing */

uint64_t usqr64(uint32_t x) {
  /* Mimic the assembly algorithm */
  uint32_t lo = x & 0xFFFF;
  uint32_t hi = x >> 16;

  uint32_t cross = lo * hi;
  uint32_t hi_sq = hi * hi;
  uint32_t lo_sq = lo * lo;

  uint64_t result = (uint64_t)hi_sq << 32;
  result += (uint64_t)cross << 17;
  result += lo_sq;

  return result;
}

int64_t ssqr64(int32_t x) {
  /* For signed, the result of x*x is always positive when x != INT32_MIN */
  /* Use the same decomposition but with sign extension */
  int32_t  hi = x >> 16; /* Arithmetic shift for sign extension */
  uint32_t lo = (uint32_t)x & 0xFFFF;

  int32_t  cross = (int32_t)(lo * (uint32_t)hi);
  uint32_t hi_sq = (uint32_t)(hi * hi);
  uint32_t lo_sq = lo * lo;

  int64_t result = (int64_t)(uint64_t)hi_sq << 32;
  result += (int64_t)cross << 17;
  result += lo_sq;

  return result;
}

int64_t smul64(int32_t a, int32_t b) {
  /* Reference implementation matching the assembly algorithm */
  uint32_t a_lo = (uint32_t)a & 0xFFFF;
  int32_t  a_hi = a >> 16;
  uint32_t b_lo = (uint32_t)b & 0xFFFF;
  int32_t  b_hi = b >> 16;

  uint32_t lo_lo = a_lo * b_lo;
  int32_t  lo_hi = (int32_t)(b_lo * (uint32_t)a_hi);
  int32_t  hi_hi = a_hi * b_hi;
  int32_t  hi_lo = (int32_t)((uint32_t)b_hi * a_lo);

  int64_t result = (int64_t)hi_hi << 32;
  result += lo_lo;
  result += (int64_t)lo_hi << 16;
  result += (int64_t)hi_lo << 16;

  return result;
}

#else
/* On target, use the actual assembly implementations */
#include "asm_math.h"
#include "driver_TIME.h"
#endif

/* Test vectors for validation */
static const uint32_t test_unsigned[] = {
    0,          1,      2,      3,      255,     256,        65535,
    65536,      0x7FFF, 0x8000, 0xFFFF, 0x10000, 0x7FFFFFFF, 0x80000000,
    0xFFFFFFFF, 12345,  54321,  100000, 1000000, 0xDEADBEEF};

static const int32_t test_signed[] = {
    0,     1,      -1,    2,      -2,         127,         -128,
    32767, -32768, 65535, -65536, 0x7FFFFFFF, -2147483647, -2147483648,
    12345, -12345, 54321, -54321, 1000000,    -1000000};

/* Test pairs for multiplication (a, b) */
static const int32_t test_mul_pairs[][2] = {
    {0, 0},           {1, 1},          {-1, 1},          {1, -1},
    {-1, -1},         {100, 200},      {-100, 200},      {100, -200},
    {-100, -200},     {32767, 32767},  {-32768, 32768},  {65535, 65536},
    {0x7FFF, 0x7FFF}, {0x7FFFFFFF, 2}, {-2147483648, 1}, {-2147483648, -1},
    {12345, 67890},   {-12345, 67890}, {0x1234, 0x5678}, {-0x1234, -0x5678},
};

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* Validation tests */
static bool test_usqr64_validation(void) {
  printf("Testing usqr64 validation...\n");
  bool pass = true;

  for (size_t i = 0; i < ARRAY_SIZE(test_unsigned); i++) {
    uint32_t x        = test_unsigned[i];
    uint64_t result   = usqr64(x);
    uint64_t expected = (uint64_t)x * x;

    if (result != expected) {
      printf("  FAIL: usqr64(%u) = %llu, expected %llu\n", x,
             (unsigned long long)result, (unsigned long long)expected);
      pass = false;
    }
  }

  /* Random tests */
  srand(42);
  for (int i = 0; i < 10000; i++) {
    uint32_t x        = ((uint32_t)rand() << 16) | (uint32_t)rand();
    uint64_t result   = usqr64(x);
    uint64_t expected = (uint64_t)x * x;

    if (result != expected) {
      printf("  FAIL: usqr64(%u) = %llu, expected %llu\n", x,
             (unsigned long long)result, (unsigned long long)expected);
      pass = false;
    }
  }

  if (pass) {
    printf("  PASS: All usqr64 tests passed\n");
  }
  return pass;
}

static bool test_ssqr64_validation(void) {
  printf("Testing ssqr64 validation...\n");
  bool pass = true;

  for (size_t i = 0; i < ARRAY_SIZE(test_signed); i++) {
    int32_t x        = test_signed[i];
    int64_t result   = ssqr64(x);
    int64_t expected = (int64_t)x * x;

    if (result != expected) {
      printf("  FAIL: ssqr64(%d) = %lld, expected %lld\n", x, (long long)result,
             (long long)expected);
      pass = false;
    }
  }

  /* Random tests */
  srand(42);
  for (int i = 0; i < 10000; i++) {
    int32_t x        = (int32_t)(((uint32_t)rand() << 16) | (uint32_t)rand());
    int64_t result   = ssqr64(x);
    int64_t expected = (int64_t)x * x;

    if (result != expected) {
      printf("  FAIL: ssqr64(%d) = %lld, expected %lld\n", x, (long long)result,
             (long long)expected);
      pass = false;
    }
  }

  if (pass) {
    printf("  PASS: All ssqr64 tests passed\n");
  }
  return pass;
}

static bool test_smul64_validation(void) {
  printf("Testing smul64 validation...\n");
  bool pass = true;

  /* Test with predefined pairs */
  for (size_t i = 0; i < ARRAY_SIZE(test_mul_pairs); i++) {
    int32_t a        = test_mul_pairs[i][0];
    int32_t b        = test_mul_pairs[i][1];
    int64_t result   = smul64(a, b);
    int64_t expected = (int64_t)a * b;

    if (result != expected) {
      printf("  FAIL: smul64(%d, %d) = %lld, expected %lld\n", a, b,
             (long long)result, (long long)expected);
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
        printf("  FAIL: smul64(%d, %d) = %lld, expected %lld\n", a, b,
               (long long)result, (long long)expected);
        pass = false;
      }
    }
  }

  /* Random tests */
  srand(42);
  for (int i = 0; i < 10000; i++) {
    int32_t a        = (int32_t)(((uint32_t)rand() << 16) | (uint32_t)rand());
    int32_t b        = (int32_t)(((uint32_t)rand() << 16) | (uint32_t)rand());
    int64_t result   = smul64(a, b);
    int64_t expected = (int64_t)a * b;

    if (result != expected) {
      printf("  FAIL: smul64(%d, %d) = %lld, expected %lld\n", a, b,
             (long long)result, (long long)expected);
      pass = false;
    }
  }

  if (pass) {
    printf("  PASS: All smul64 tests passed\n");
  }
  return pass;
}

/* Performance tests */
#define PERF_ITERATIONS 100000

#ifdef HOSTED
/* Host-based performance test using clock() */
static void test_performance_host(void) {
  printf("\nPerformance tests (host, %d iterations):\n", PERF_ITERATIONS);

  volatile uint64_t sink = 0;
  clock_t           start, end;
  double            elapsed;

  /* Test usqr64 */
  start = clock();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += usqr64((uint32_t)i);
  }
  end     = clock();
  elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1000000.0;
  printf("  usqr64: %.2f us total, %.3f us/call\n", elapsed,
         elapsed / PERF_ITERATIONS);

  /* Test standard multiply for comparison */
  start = clock();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    uint32_t x = (uint32_t)i;
    sink += (uint64_t)x * x;
  }
  end     = clock();
  elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1000000.0;
  printf("  x * x:  %.2f us total, %.3f us/call\n", elapsed,
         elapsed / PERF_ITERATIONS);

  /* Test ssqr64 */
  start = clock();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += (uint64_t)ssqr64((int32_t)i - PERF_ITERATIONS / 2);
  }
  end     = clock();
  elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1000000.0;
  printf("  ssqr64: %.2f us total, %.3f us/call\n", elapsed,
         elapsed / PERF_ITERATIONS);

  /* Test smul64 */
  start = clock();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += smul64((int32_t)i, (int32_t)(PERF_ITERATIONS - i));
  }
  end     = clock();
  elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1000000.0;
  printf("  smul64: %.2f us total, %.3f us/call\n", elapsed,
         elapsed / PERF_ITERATIONS);

  /* Test standard signed multiply for comparison */
  start = clock();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    int32_t a = (int32_t)i;
    int32_t b = (int32_t)(PERF_ITERATIONS - i);
    sink += (int64_t)a * b;
  }
  end     = clock();
  elapsed = (double)(end - start) / CLOCKS_PER_SEC * 1000000.0;
  printf("  a * b:  %.2f us total, %.3f us/call\n", elapsed,
         elapsed / PERF_ITERATIONS);

  (void)sink; /* Prevent optimization */
}

#else
/* Target-based performance test using hardware timer */
static void test_performance_target(void) {
  printf("\nPerformance tests (target, %d iterations):\n", PERF_ITERATIONS);

  volatile uint64_t sink = 0;
  uint32_t          start, end;
  uint32_t          elapsed;

  /* Test usqr64 */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += usqr64((uint32_t)i);
  }
  end     = timerMicros();
  elapsed = end - start;
  printf("  usqr64: %lu us total, %.3f us/call\n", elapsed,
         (float)elapsed / PERF_ITERATIONS);

  /* Test standard multiply for comparison */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    uint32_t x = (uint32_t)i;
    sink += (uint64_t)x * x;
  }
  end     = timerMicros();
  elapsed = end - start;
  printf("  x * x:  %lu us total, %.3f us/call\n", elapsed,
         (float)elapsed / PERF_ITERATIONS);

  /* Test ssqr64 */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += (uint64_t)ssqr64((int32_t)i - PERF_ITERATIONS / 2);
  }
  end     = timerMicros();
  elapsed = end - start;
  printf("  ssqr64: %lu us total, %.3f us/call\n", elapsed,
         (float)elapsed / PERF_ITERATIONS);

  /* Test smul64 */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    sink += smul64((int32_t)i, (int32_t)(PERF_ITERATIONS - i));
  }
  end     = timerMicros();
  elapsed = end - start;
  printf("  smul64: %lu us total, %.3f us/call\n", elapsed,
         (float)elapsed / PERF_ITERATIONS);

  /* Test standard signed multiply for comparison */
  start = timerMicros();
  for (int i = 0; i < PERF_ITERATIONS; i++) {
    int32_t a = (int32_t)i;
    int32_t b = (int32_t)(PERF_ITERATIONS - i);
    sink += (int64_t)a * b;
  }
  end     = timerMicros();
  elapsed = end - start;
  printf("  a * b:  %lu us total, %.3f us/call\n", elapsed,
         (float)elapsed / PERF_ITERATIONS);

  (void)sink; /* Prevent optimization */
}
#endif

int main(void) {
  printf("=== asm_math test suite ===\n\n");

  bool all_pass = true;

  all_pass &= test_usqr64_validation();
  all_pass &= test_ssqr64_validation();
  all_pass &= test_smul64_validation();

#ifdef HOSTED
  test_performance_host();
#else
  test_performance_target();
#endif

  printf("\n=== %s ===\n", all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED");

  return all_pass ? 0 : 1;
}
