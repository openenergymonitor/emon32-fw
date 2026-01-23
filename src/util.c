#include <stdbool.h>
#include <stdio.h>

#include "util.h"

#ifndef HOSTED
#include "qfplib-m0-full.h"
#else
#include "qfplib_host.h"
#endif

static bool isnumeric(const char c);

static bool isnumeric(const char c) {
  if (('0' <= c) && ('9' >= c)) {
    return true;
  }
  return false;
}

/* Fast divide by 10 using shifts/adds only (no hardware divide) */
static inline uint32_t fastDiv10(uint32_t n) {
  uint32_t q = (n >> 1) + (n >> 2);
  q          = q + (q >> 4);
  q          = q + (q >> 8);
  q          = q + (q >> 16);
  q          = q >> 3;
  uint32_t r = n - q * 10;
  return q + ((r + 6) >> 4);
}

size_t utilUtoa(char *pBuf, uint32_t val, const ITOA_BASE_t base) {
  char  buf[11]; /* 4294967295 = 10 chars + null */
  char *p = &buf[10];

  *p = '\0';

  /* Handle 0 explicitly */
  if (0 == val) {
    pBuf[0] = '0';
    pBuf[1] = '\0';
    return 2u;
  }

  if (ITOA_BASE10 == base) {
    while (val != 0) {
      uint32_t q = fastDiv10(val);
      *--p       = (char)('0' + (val - q * 10));
      val        = q;
    }
  } else {
    static const char itohex[] = "0123456789abcdef";

    while (0 != val) {
      *--p = itohex[val & 0xFu];
      val >>= 4;
    }
  }

  /* Copy to output buffer */
  char  *dst = pBuf;
  size_t len = 0;
  while (*p) {
    *dst++ = *p++;
    len++;
  }
  *dst = '\0';

  return len + 1u;
}

size_t utilItoa(char *pBuf, int32_t val, const ITOA_BASE_t base) {
  if ((ITOA_BASE10 == base) && (val < 0)) {
    *pBuf = '-';
    return 1u + utilUtoa(pBuf + 1, (uint32_t)(-val), base);
  }
  return utilUtoa(pBuf, (uint32_t)val, base);
}

ConvUint_t utilAtoui(const char *pBuf, ITOA_BASE_t base) {
  uint32_t   result = 0;
  ConvUint_t conv   = {false, {0}};

  /* Process left-to-right, no string reversal needed */
  if (ITOA_BASE10 == base) {
    while (*pBuf) {
      if (!isnumeric(*pBuf)) {
        return conv;
      }
      result = result * 10 + (uint32_t)(*pBuf - '0');
      pBuf++;
    }
  } else {
    while (*pBuf) {
      char     c = *pBuf;
      uint32_t digit;
      if (('a' <= c) && ('f' >= c)) {
        digit = (uint32_t)(c - 'a' + 10);
      } else if (('A' <= c) && ('F' >= c)) {
        digit = (uint32_t)(c - 'A' + 10);
      } else if (isnumeric(c)) {
        digit = (uint32_t)(c - '0');
      } else {
        return conv;
      }
      /* result = result * 16 + digit */
      result = (result << 4) + digit;
      pBuf++;
    }
  }

  conv.val.u32 = result;
  conv.valid   = true;
  return conv;
}

ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base) {
  bool isNegative = ('-' == *pBuf);
  if (isNegative) {
    pBuf++;
  }

  ConvUint_t u    = utilAtoui(pBuf, base);
  ConvInt_t  conv = {u.valid,
                     {isNegative ? -(int32_t)u.val.u32 : (int32_t)u.val.u32}};
  return conv;
}

bool utilCharPrintable(const char c) {
  /* Allow any printable character plus \r and \n */
  return (((c >= 32) && (c <= 126)) || ('\r' == c) || ('\n' == c));
}

size_t utilFtoa(char *pBuf, float val) {
  char     buf[16]; /* Enough for -2147483648.99 + null */
  char    *p = &buf[15];
  uint32_t units;
  uint32_t decimals;
  bool     neg = false;

  *p = '\0';

  if (val < 0.0f) {
    neg = true;
    val = qfp_fmul(val, -1.0f);
  }

  /* Extract integer and fractional parts */
  units    = (uint32_t)qfp_float2int_z(val);
  decimals = (uint32_t)qfp_float2int_z(qfp_fmul(val, 100.0f)) - (units * 100);

  /* Write decimals (always 2 digits) using fast division */
  uint32_t q = fastDiv10(decimals);
  *--p       = (char)('0' + (decimals - q * 10));
  *--p       = (char)('0' + q);
  *--p       = '.';

  /* Write integer part */
  if (units == 0) {
    *--p = '0';
  } else {
    while (units != 0) {
      q     = fastDiv10(units);
      *--p  = (char)('0' + (units - q * 10));
      units = q;
    }
  }

  if (neg) {
    *--p = '-';
  }

  /* Copy to output buffer */
  char    *dst = pBuf;
  uint32_t len = 0;
  while (*p) {
    *dst++ = *p++;
    len++;
  }
  *dst = '\0';

  return len + 1u;
}

ConvFloat_t utilAtof(const char *pBuf) {
  bool        isNegative = false;
  uint32_t    intPart    = 0;
  uint32_t    fracPart   = 0;
  uint32_t    fracDiv    = 1;
  bool        inFraction = false;
  ConvFloat_t conv       = {false, 0.0f};

  if ('-' == *pBuf) {
    isNegative = true;
    pBuf++;
  }

  /* Process left-to-right, no string reversal needed */
  while (*pBuf) {
    const char c = *pBuf++;
    if (('.' == c) || (',' == c)) {
      inFraction = true;
    } else if (isnumeric(c)) {
      uint32_t digit = (uint32_t)(c - '0');
      if (inFraction) {
        fracPart = fracPart * 10 + digit;
        fracDiv  = fracDiv * 10;
      } else {
        intPart = intPart * 10 + digit;
      }
    } else {
      /* Invalid character found */
      return conv;
    }
  }

  /* Convert to float only at the end */
  conv.val = qfp_uint2float(intPart);
  if (fracDiv > 1u) {
    conv.val = qfp_fadd(
        conv.val, qfp_fdiv(qfp_uint2float(fracPart), qfp_uint2float(fracDiv)));
  }

  if (isNegative) {
    conv.val = qfp_fmul(conv.val, -1.0f);
  }

  conv.valid = true;
  return conv;
}
