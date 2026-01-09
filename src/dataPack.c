#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emon32_assert.h"
#include "temperature.h"
#include "util.h"

#include "qfplib-m0-full.h"

#define CONV_STR_W 16
#define STR_MSG    0
#define STR_V      1
#define STR_P      2
#define STR_E      3
#define STR_PULSE  4
#define STR_TEMP   5
#define STR_COLON  6
#define STR_CRLF   7
#define STR_DQUOTE 8
#define STR_LCURL  9
#define STR_RCURL  10
#define STR_COMMA  11

/* "Fat" string with current length and buffer size. */
typedef struct StrN {
  char  *str; /* Pointer to the string */
  size_t n;   /* Length of the string  */
  size_t m;   /* Buffer length */
} StrN_t;

static void    catId(StrN_t *strD, const int32_t id, const int32_t field,
                     const bool json);
static void    catMsg(StrN_t *strD, const int32_t msg, const bool json);
static void    initFields(StrN_t *pD, char *pS, const size_t m);
static int32_t strnFtoa(StrN_t *strD, const float v);
static int32_t strnItoa(StrN_t *strD, const int32_t v);
static size_t  strnCat(StrN_t *strD, const StrN_t *strS);
static int32_t strnLen(StrN_t *str);

static char   tmpStr[CONV_STR_W] = {0};
static StrN_t strConv; /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[12] = {
    {.str = "MSG", .n = 3, .m = 4},   {.str = "V", .n = 1, .m = 2},
    {.str = "P", .n = 1, .m = 2},     {.str = "E", .n = 1, .m = 2},
    {.str = "pulse", .n = 5, .m = 6}, {.str = "t", .n = 1, .m = 2},
    {.str = ":", .n = 1, .m = 2},     {.str = "\r\n", .n = 2, .m = 3},
    {.str = "\"", .n = 1, .m = 2},    {.str = "{", .n = 1, .m = 2},
    {.str = "}", .n = 1, .m = 2},     {.str = ",", .n = 1, .m = 2}};

/*! @brief Append "<field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 *  @param [in] json : select format
 */
static void catId(StrN_t *strD, const int32_t id, const int32_t field,
                  const bool json) {
  strD->n += strnCat(strD, &baseStr[STR_COMMA]);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[field]);
  (void)strnItoa(&strConv, id);
  strD->n += strnCat(strD, &strConv);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
}

/*! @brief Append the MSG field to the fat string
 *  @param [out] strD : pointer to the destination fat string
 *  @param [in] msg : message number
 *  @param [in] json : select format
 */
static void catMsg(StrN_t *strD, const int32_t msg, const bool json) {
  /* <{">MSG<">:<"><#><"> */

  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_LCURL]);
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_MSG]);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
  (void)strnItoa(&strConv, msg);
  strD->n += strnCat(strD, &strConv);
}

/*! @brief Initialise a fat string
 *  @param [out] pD : pointer to fat string
 *  @param [in] pS : pointer to string buffer
 *  @param [in] m : maximum width of the string
 */
static void initFields(StrN_t *pD, char *pS, const size_t m) {
  /* Setup destination string */
  pD->str = pS;
  pD->n   = 0;
  pD->m   = m;
  memset(pD->str, 0, m);

  /* Setup conversion string */
  strConv.str = tmpStr;
  strConv.n   = 0;
  strConv.m   = CONV_STR_W;
}

/*! @brief Add a float to a fat string
 *  @param [out] strD : pointer to destination fat string
 *  @param [in] v : value to convert
 *  @return length of the string
 */
static int32_t strnFtoa(StrN_t *strD, const float v) {

  /* Zero the destination buffer then convert */
  memset(strD->str, 0, strD->m);
  utilFtoa(strD->str, v);
  strD->n = strnLen(strD) - 1;

  /* Truncate if it exceeds the length of the string */
  if (-1 == strD->n) {
    strD->n = strD->m;
  }
  return strD->n;
}

/*! @brief Add an integer to a fat string
 *  @param [out] strD : pointer to destination fat string
 *  @param [in] v : value to convert
 *  @return length of the string
 */
static int32_t strnItoa(StrN_t *strD, const int32_t v) {
  /* Zero the destination buffer then convert */
  memset(strD->str, 0, strD->m);

  strD->n = utilItoa(strD->str, v, ITOA_BASE10) - 1;
  return strD->n;
}

/*! @brief Concatenate two fat strings
 *  @param [out] strD : pointer to destination string
 *  @param [in] strS : pointer to string to concatenate onto strD
 *  @return number of characters concatenated
 */
static size_t strnCat(StrN_t *strD, const StrN_t *strS) {
  /* Check bounds to make sure it won't go over the end. If so, return the
   * actual number of bytes that are copied.
   */
  size_t newLen;
  size_t bytesToCopy;

  bytesToCopy = strS->n;
  newLen      = strS->n + strD->n;
  if (newLen >= strD->m) {
    bytesToCopy = strD->m - strD->n;
  }

  memcpy((strD->str + strD->n), strS->str, bytesToCopy);
  return bytesToCopy;
}

/*! @brief Get the length of a fat string, not including NULL
 *  @return length of string, not including NULL. -1 if exceeds buffer length
 */
static int32_t strnLen(StrN_t *str) {
  int32_t i = 0;
  while (str->str[i++]) {
    /* Terminate if exceeded the maximum length */
    if (i >= str->m) {
      return -1;
    }
  }
  return i;
}

size_t dataPackSerial(const Emon32Dataset_t *pData, char *pDst, const size_t m,
                      const bool json) {
  EMON32_ASSERT(pData);
  EMON32_ASSERT(pDst);

  StrN_t strn;
  initFields(&strn, pDst, m);

  catMsg(&strn, pData->msgNum, json);

  /* V channels; only print V2/V3 if either active */
  int32_t numV = (pData->pECM->activeCh & 0x6) ? NUM_V : 1;

  for (int32_t i = 0; i < numV; i++) {
    catId(&strn, (i + 1), STR_V, json);
    (void)strnFtoa(&strConv, pData->pECM->rmsV[i]);
    strn.n += strnCat(&strn, &strConv);
  }

  /* CT channels (power and energy)
   * Only print onboard CTs 7-12 if any are present
   */
  int32_t numCT = (pData->pECM->activeCh & (0x3f << (NUM_V + (NUM_CT / 2))))
                      ? NUM_CT
                      : (NUM_CT / 2);

  for (int32_t i = 0; i < numCT; i++) {
    catId(&strn, (i + 1), STR_P, json);
    (void)strnItoa(&strConv, pData->pECM->CT[i].realPower);
    strn.n += strnCat(&strn, &strConv);
  }
  for (int32_t i = 0; i < numCT; i++) {
    catId(&strn, (i + 1), STR_E, json);
    (void)strnItoa(&strConv, pData->pECM->CT[i].wattHour);
    strn.n += strnCat(&strn, &strConv);
  }

  for (int32_t i = 0; i < NUM_OPA; i++) {
    catId(&strn, (i + 1), STR_PULSE, json);
    (void)strnItoa(&strConv, pData->pulseCnt[i]);
    strn.n += strnCat(&strn, &strConv);
  }

  for (int32_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    catId(&strn, (i + 1), STR_TEMP, json);
    (void)strnFtoa(&strConv, tempAsFloat(TEMP_INTF_ONEWIRE, pData->temp[i]));
    strn.n += strnCat(&strn, &strConv);
  }

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}

int8_t dataPackPacked(const Emon32Dataset_t *pData, void *pPacked,
                      const PackedRange_t range) {

  /* Both upper and lower packets share the same initial data structure.
   * Differentiate for pulse or temperature readings. */

  bool isUpper = (PACKED_UPPER == range);

  PackedDataCommon_t *pCommon = pPacked;
  pCommon->msg                = pData->msgNum;

  for (int32_t v = 0; v < NUM_V; v++) {
    pCommon->V[v] = qfp_float2int_z(qfp_fmul(pData->pECM->rmsV[v], 100.0f));
  }

  for (int32_t i = 0; i < (NUM_CT / 2); i++) {
    pCommon->P[i] = pData->pECM->CT[i + ((NUM_CT / 2) * isUpper)].realPower;
    pCommon->E[i] = pData->pECM->CT[i + ((NUM_CT / 2) * isUpper)].wattHour;
  }

  if (PACKED_LOWER == range) {
    PackedDataLower6_t *pLower = pPacked;
    for (int32_t p = 0; p < NUM_OPA; p++) {
      pLower->pulse[p] = pData->pulseCnt[p];
    }
    return sizeof(*pLower);
  } else if (PACKED_UPPER == range) {
    PackedDataUpper6_t *pUpper = pPacked;
    for (int32_t t = 0; t < (TEMP_MAX_ONEWIRE / 2); t++) {
      /* Sent as 100x the temperature value. */
      pUpper->temp[t] = (pData->temp[t] * 6) + (pData->temp[t] >> 2);
    }
    return sizeof(*pUpper);
  }

  return 0;
}
