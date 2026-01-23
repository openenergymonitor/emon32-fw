/*
 * Unit tests for util.c
 *
 * Tests the string/number conversion functions:
 *   - utilUtoa: unsigned int to string
 *   - utilItoa: signed int to string
 *   - utilFtoa: float to string
 *   - utilAtoui: string to unsigned int
 *   - utilAtoi: string to signed int
 *   - utilAtof: string to float
 *   - utilCharPrintable: character validation
 *
 * Compile: make util
 * Run: ./util.test
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "util.h"

/* Test counters */
static int tests_run    = 0;
static int tests_passed = 0;

#define TEST(name)                                                             \
  do {                                                                         \
    printf("  > %s ... ", name);                                               \
    tests_run++;                                                               \
  } while (0)

#define PASS()                                                                 \
  do {                                                                         \
    printf("PASS\n");                                                          \
    tests_passed++;                                                            \
  } while (0)

#define FAIL(msg)                                                              \
  do {                                                                         \
    printf("FAIL: %s\n", msg);                                                 \
  } while (0)

#define ASSERT_STR_EQ(expected, actual)                                        \
  do {                                                                         \
    if (strcmp(expected, actual) != 0) {                                       \
      printf("FAIL: expected \"%s\", got \"%s\"\n", expected, actual);         \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_EQ(expected, actual)                                            \
  do {                                                                         \
    if ((expected) != (actual)) {                                              \
      printf("FAIL: expected %d, got %d\n", (int)(expected), (int)(actual));   \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_FLOAT_EQ(expected, actual, epsilon)                             \
  do {                                                                         \
    if (fabs((expected) - (actual)) > (epsilon)) {                             \
      printf("FAIL: expected %f, got %f\n", (double)(expected),                \
             (double)(actual));                                                \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_TRUE(cond)                                                      \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL: condition false\n");                                       \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_FALSE(cond)                                                     \
  do {                                                                         \
    if (cond) {                                                                \
      printf("FAIL: condition true\n");                                        \
      return;                                                                  \
    }                                                                          \
  } while (0)

/* ========== utilUtoa tests ========== */

void test_utilUtoa_zero(void) {
  char   buf[16];
  size_t len;

  TEST("utilUtoa: zero");
  len = utilUtoa(buf, 0, ITOA_BASE10);
  ASSERT_STR_EQ("0", buf);
  ASSERT_EQ(2, len); /* includes null terminator */
  PASS();
}

void test_utilUtoa_positive(void) {
  char   buf[16];
  size_t len;

  TEST("utilUtoa: positive numbers");
  len = utilUtoa(buf, 123, ITOA_BASE10);
  ASSERT_STR_EQ("123", buf);
  ASSERT_EQ(4, len);

  len = utilUtoa(buf, 4294967295u, ITOA_BASE10);
  ASSERT_STR_EQ("4294967295", buf);
  ASSERT_EQ(11, len);

  len = utilUtoa(buf, 1, ITOA_BASE10);
  ASSERT_STR_EQ("1", buf);
  ASSERT_EQ(2, len);
  PASS();
}

void test_utilUtoa_hex(void) {
  char   buf[16];
  size_t len;

  TEST("utilUtoa: hexadecimal");
  len = utilUtoa(buf, 255, ITOA_BASE16);
  ASSERT_STR_EQ("ff", buf);
  ASSERT_EQ(3, len);

  len = utilUtoa(buf, 0xDEADBEEF, ITOA_BASE16);
  ASSERT_STR_EQ("deadbeef", buf);
  ASSERT_EQ(9, len);

  len = utilUtoa(buf, 16, ITOA_BASE16);
  ASSERT_STR_EQ("10", buf);
  ASSERT_EQ(3, len);
  PASS();
}

/* ========== utilItoa tests ========== */

void test_utilItoa_positive(void) {
  char   buf[16];
  size_t len;

  TEST("utilItoa: positive numbers");
  len = utilItoa(buf, 123, ITOA_BASE10);
  ASSERT_STR_EQ("123", buf);
  ASSERT_EQ(4, len);

  len = utilItoa(buf, 2147483647, ITOA_BASE10);
  ASSERT_STR_EQ("2147483647", buf);
  ASSERT_EQ(11, len);
  PASS();
}

void test_utilItoa_negative(void) {
  char   buf[16];
  size_t len;

  TEST("utilItoa: negative numbers");
  len = utilItoa(buf, -123, ITOA_BASE10);
  ASSERT_STR_EQ("-123", buf);
  ASSERT_EQ(5, len);

  len = utilItoa(buf, -1, ITOA_BASE10);
  ASSERT_STR_EQ("-1", buf);
  ASSERT_EQ(3, len);
  PASS();
}

void test_utilItoa_zero(void) {
  char   buf[16];
  size_t len;

  TEST("utilItoa: zero");
  len = utilItoa(buf, 0, ITOA_BASE10);
  ASSERT_STR_EQ("0", buf);
  ASSERT_EQ(2, len);
  PASS();
}

/* ========== utilFtoa tests ========== */

void test_utilFtoa_positive(void) {
  char   buf[16];
  size_t len;

  TEST("utilFtoa: positive floats");
  len = utilFtoa(buf, 123.45f);
  ASSERT_STR_EQ("123.45", buf);
  ASSERT_EQ(7, len);

  len = utilFtoa(buf, 0.0f);
  ASSERT_STR_EQ("0.00", buf);
  ASSERT_EQ(5, len);

  len = utilFtoa(buf, 1.0f);
  ASSERT_STR_EQ("1.00", buf);
  ASSERT_EQ(5, len);
  PASS();
}

void test_utilFtoa_negative(void) {
  char   buf[16];
  size_t len;

  TEST("utilFtoa: negative floats");
  len = utilFtoa(buf, -123.45f);
  ASSERT_STR_EQ("-123.45", buf);
  ASSERT_EQ(8, len);

  len = utilFtoa(buf, -0.5f);
  ASSERT_STR_EQ("-0.50", buf);
  ASSERT_EQ(6, len);
  PASS();
}

void test_utilFtoa_rounding(void) {
  char buf[16];

  TEST("utilFtoa: decimal precision (2 dp)");
  /* Should truncate, not round */
  utilFtoa(buf, 1.999f);
  ASSERT_STR_EQ("1.99", buf);

  utilFtoa(buf, 10.101f);
  ASSERT_STR_EQ("10.10", buf);
  PASS();
}

/* ========== utilAtoui tests ========== */

void test_utilAtoui_valid(void) {
  ConvUint_t result;

  TEST("utilAtoui: valid decimal strings");
  result = utilAtoui("123", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(123, result.val.u32);

  result = utilAtoui("0", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(0, result.val.u32);

  result = utilAtoui("4294967295", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(4294967295u, result.val.u32);
  PASS();
}

void test_utilAtoui_hex(void) {
  ConvUint_t result;

  TEST("utilAtoui: hexadecimal strings");
  result = utilAtoui("ff", ITOA_BASE16);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(255, result.val.u32);

  result = utilAtoui("FF", ITOA_BASE16);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(255, result.val.u32);

  result = utilAtoui("deadbeef", ITOA_BASE16);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(0xDEADBEEF, result.val.u32);

  result = utilAtoui("10", ITOA_BASE16);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(16, result.val.u32);
  PASS();
}

void test_utilAtoui_invalid(void) {
  ConvUint_t result;

  TEST("utilAtoui: invalid strings");
  result = utilAtoui("abc", ITOA_BASE10);
  ASSERT_FALSE(result.valid);

  result = utilAtoui("12.34", ITOA_BASE10);
  ASSERT_FALSE(result.valid);

  result = utilAtoui("-123", ITOA_BASE10);
  ASSERT_FALSE(result.valid);

  result = utilAtoui("gg", ITOA_BASE16);
  ASSERT_FALSE(result.valid);
  PASS();
}

/* ========== utilAtoi tests ========== */

void test_utilAtoi_positive(void) {
  ConvInt_t result;

  TEST("utilAtoi: positive numbers");
  result = utilAtoi("123", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(123, result.val.i32);

  result = utilAtoi("2147483647", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(2147483647, result.val.i32);
  PASS();
}

void test_utilAtoi_negative(void) {
  ConvInt_t result;

  TEST("utilAtoi: negative numbers");
  result = utilAtoi("-123", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(-123, result.val.i32);

  result = utilAtoi("-1", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(-1, result.val.i32);
  PASS();
}

void test_utilAtoi_zero(void) {
  ConvInt_t result;

  TEST("utilAtoi: zero");
  result = utilAtoi("0", ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(0, result.val.i32);
  PASS();
}

void test_utilAtoi_invalid(void) {
  ConvInt_t result;

  TEST("utilAtoi: invalid strings");
  result = utilAtoi("abc", ITOA_BASE10);
  ASSERT_FALSE(result.valid);

  result = utilAtoi("12.34", ITOA_BASE10);
  ASSERT_FALSE(result.valid);
  PASS();
}

/* ========== utilAtof tests ========== */

void test_utilAtof_positive(void) {
  ConvFloat_t result;

  TEST("utilAtof: positive floats");
  result = utilAtof("123.45");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(123.45f, result.val, 0.001f);

  result = utilAtof("0.5");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(0.5f, result.val, 0.001f);

  result = utilAtof("100");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(100.0f, result.val, 0.001f);
  PASS();
}

void test_utilAtof_negative(void) {
  ConvFloat_t result;

  TEST("utilAtof: negative floats");
  result = utilAtof("-123.45");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(-123.45f, result.val, 0.001f);

  result = utilAtof("-0.5");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(-0.5f, result.val, 0.001f);
  PASS();
}

void test_utilAtof_comma_decimal(void) {
  ConvFloat_t result;

  TEST("utilAtof: comma as decimal separator");
  result = utilAtof("123,45");
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(123.45f, result.val, 0.001f);
  PASS();
}

void test_utilAtof_invalid(void) {
  ConvFloat_t result;

  TEST("utilAtof: invalid strings");
  result = utilAtof("abc");
  ASSERT_FALSE(result.valid);

  result = utilAtof("12a34");
  ASSERT_FALSE(result.valid);

  result = utilAtof("12.3x");
  ASSERT_FALSE(result.valid);
  PASS();
}

/* ========== utilCharPrintable tests ========== */

void test_utilCharPrintable_printable(void) {
  TEST("utilCharPrintable: printable characters");
  ASSERT_TRUE(utilCharPrintable('a'));
  ASSERT_TRUE(utilCharPrintable('Z'));
  ASSERT_TRUE(utilCharPrintable('0'));
  ASSERT_TRUE(utilCharPrintable(' '));
  ASSERT_TRUE(utilCharPrintable('~'));
  ASSERT_TRUE(utilCharPrintable('\r'));
  ASSERT_TRUE(utilCharPrintable('\n'));
  PASS();
}

void test_utilCharPrintable_nonprintable(void) {
  TEST("utilCharPrintable: non-printable characters");
  ASSERT_FALSE(utilCharPrintable('\0'));
  ASSERT_FALSE(utilCharPrintable('\t'));
  ASSERT_FALSE(utilCharPrintable(0x1F)); /* Unit separator */
  ASSERT_FALSE(utilCharPrintable(0x7F)); /* DEL */
  PASS();
}

/* ========== Round-trip tests ========== */

void test_roundtrip_uint(void) {
  char       buf[16];
  ConvUint_t result;

  TEST("Round-trip: uint -> string -> uint");
  utilUtoa(buf, 12345, ITOA_BASE10);
  result = utilAtoui(buf, ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(12345, result.val.u32);

  utilUtoa(buf, 0xABCD, ITOA_BASE16);
  result = utilAtoui(buf, ITOA_BASE16);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(0xABCD, result.val.u32);
  PASS();
}

void test_roundtrip_int(void) {
  char      buf[16];
  ConvInt_t result;

  TEST("Round-trip: int -> string -> int");
  utilItoa(buf, -12345, ITOA_BASE10);
  result = utilAtoi(buf, ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(-12345, result.val.i32);

  utilItoa(buf, 12345, ITOA_BASE10);
  result = utilAtoi(buf, ITOA_BASE10);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(12345, result.val.i32);
  PASS();
}

void test_roundtrip_float(void) {
  char        buf[16];
  ConvFloat_t result;

  TEST("Round-trip: float -> string -> float");
  utilFtoa(buf, 123.45f);
  result = utilAtof(buf);
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(123.45f, result.val, 0.01f);

  utilFtoa(buf, -99.99f);
  result = utilAtof(buf);
  ASSERT_TRUE(result.valid);
  ASSERT_FLOAT_EQ(-99.99f, result.val, 0.01f);
  PASS();
}

/* ========== Main ========== */

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  printf("---- emon32 util.c unit tests ----\n\n");

  /* utilUtoa tests */
  printf("utilUtoa:\n");
  test_utilUtoa_zero();
  test_utilUtoa_positive();
  test_utilUtoa_hex();

  /* utilItoa tests */
  printf("\nutilItoa:\n");
  test_utilItoa_positive();
  test_utilItoa_negative();
  test_utilItoa_zero();

  /* utilFtoa tests */
  printf("\nutilFtoa:\n");
  test_utilFtoa_positive();
  test_utilFtoa_negative();
  test_utilFtoa_rounding();

  /* utilAtoui tests */
  printf("\nutilAtoui:\n");
  test_utilAtoui_valid();
  test_utilAtoui_hex();
  test_utilAtoui_invalid();

  /* utilAtoi tests */
  printf("\nutilAtoi:\n");
  test_utilAtoi_positive();
  test_utilAtoi_negative();
  test_utilAtoi_zero();
  test_utilAtoi_invalid();

  /* utilAtof tests */
  printf("\nutilAtof:\n");
  test_utilAtof_positive();
  test_utilAtof_negative();
  test_utilAtof_comma_decimal();
  test_utilAtof_invalid();

  /* utilCharPrintable tests */
  printf("\nutilCharPrintable:\n");
  test_utilCharPrintable_printable();
  test_utilCharPrintable_nonprintable();

  /* Round-trip tests */
  printf("\nRound-trip tests:\n");
  test_roundtrip_uint();
  test_roundtrip_int();
  test_roundtrip_float();

  /* Summary */
  printf("\n==================================\n");
  printf("Tests: %d/%d passed\n", tests_passed, tests_run);

  return (tests_passed == tests_run) ? 0 : 1;
}
