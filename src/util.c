#include <stdbool.h>
#include <stdio.h>

#include "util.h"

#include "qfplib-m0-full.h"

static bool isnumeric(const char c);

static bool isnumeric(const char c) {
  if (('0' <= c) && ('9' >= c)) {
    return true;
  }
  return false;
}

uint32_t utilAbs(const int32_t x) { return (x < 0) ? -x : x; }

void utilStrReverse(char *pBuf, uint32_t len) {
  char     tmp;
  uint32_t idxEnd = len - 1u;
  for (uint32_t idx = 0; idx < (len / 2); idx++) {
    tmp          = pBuf[idx];
    pBuf[idx]    = pBuf[idxEnd];
    pBuf[idxEnd] = tmp;
    idxEnd--;
  }
}

uint32_t utilStrlen(const char *pBuf) {
  uint32_t charCnt = 0;
  while (*pBuf++) {
    charCnt++;
  }
  return charCnt;
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

uint32_t utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base) {
  char     buf[12]; /* -2147483648 = 11 chars + null */
  char    *p = &buf[11];
  uint32_t uval;
  bool     neg = false;

  *p = '\0';

  /* Handle 0 explicitly */
  if (0 == val) {
    pBuf[0] = '0';
    pBuf[1] = '\0';
    return 2u;
  }

  if (ITOA_BASE10 == base) {
    if (val < 0) {
      neg  = true;
      uval = (uint32_t)(-val);
    } else {
      uval = (uint32_t)val;
    }

    while (uval != 0) {
      uint32_t q = fastDiv10(uval);
      *--p       = (char)('0' + (uval - q * 10));
      uval       = q;
    }

    if (neg) {
      *--p = '-';
    }
  } else {
    static const char itohex[] = "0123456789abcdef";
    uint32_t          val_u    = (uint32_t)val;

    while (0 != val_u) {
      *--p = itohex[val_u & 0xFu];
      val_u >>= 4;
    }
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

ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base) {
  bool      isNegative = false;
  uint32_t  result     = 0;
  ConvInt_t conv       = {false, 0};

  if ('-' == *pBuf) {
    isNegative = true;
    pBuf++;
  }

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

  conv.val   = isNegative ? -(int32_t)result : (int32_t)result;
  conv.valid = true;
  return conv;
}

bool utilCharPrintable(const char c) {
  /* Allow any printable character plus \r and \n */
  return (((c >= 32) && (c <= 126)) || ('\r' == c) || ('\n' == c));
}

uint32_t utilFtoa(char *pBuf, float val) {
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
  units           = (uint32_t)qfp_float2int_z(val);
  /* decimals = (val * 100) - (units * 100), using shifts for *100 */
  uint32_t val100 = (uint32_t)qfp_float2int_z(qfp_fmul(val, 100.0f));
  uint32_t units100 =
      (units << 6) + (units << 5) + (units << 2); /* units * 100 */
  decimals = val100 - units100;

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
