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
  char   *str; /* Pointer to the string */
  int32_t n;   /* Length of the string  */
  int32_t m;   /* Buffer length */
} StrN_t;

static void    catId(StrN_t *strD, int32_t id, int32_t field, bool json);
static void    catMsg(StrN_t *strD, int32_t msg, bool json);
static void    initFields(StrN_t *pD, char *pS, const int32_t m);
static int32_t strnCat(StrN_t *strD, const StrN_t *strS);
static int32_t strnCatFloat(StrN_t *strD, float v);
static int32_t strnCatInt(StrN_t *strD, int32_t v);

static char tmpStr[CONV_STR_W] = {0};

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
static void catId(StrN_t *strD, int32_t id, int32_t field, bool json) {
  strD->n += strnCat(strD, &baseStr[STR_COMMA]);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[field]);
  strD->n += strnCatInt(strD, id);
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
static void catMsg(StrN_t *strD, int32_t msg, bool json) {
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
  strD->n += strnCatInt(strD, msg);
}

/*! @brief Initialise a fat string
 *  @param [out] pD : pointer to fat string
 *  @param [in] pS : pointer to string buffer
 *  @param [in] m : maximum width of the string
 */
static void initFields(StrN_t *pD, char *pS, const int32_t m) {
  pD->str = pS;
  pD->n   = 0;
  pD->m   = m;
  memset(pD->str, 0, m);
}

/*! @brief Convert float and append to fat string
 *  @param [out] strD : pointer to destination fat string
 *  @param [in] v : value to convert and append
 *  @return number of characters appended
 */
static int32_t strnCatFloat(StrN_t *strD, float v) {
  int32_t len    = utilFtoa(tmpStr, v) - 1; /* exclude null terminator */
  int32_t space  = strD->m - strD->n;
  int32_t toCopy = (len < space) ? len : space;

  memcpy(strD->str + strD->n, tmpStr, toCopy);
  return toCopy;
}

/*! @brief Convert integer and append to fat string
 *  @param [out] strD : pointer to destination fat string
 *  @param [in] v : value to convert and append
 *  @return number of characters appended
 */
static int32_t strnCatInt(StrN_t *strD, int32_t v) {
  int32_t len    = utilItoa(tmpStr, v, ITOA_BASE10) - 1; /* exclude null */
  int32_t space  = strD->m - strD->n;
  int32_t toCopy = (len < space) ? len : space;

  memcpy(strD->str + strD->n, tmpStr, toCopy);
  return toCopy;
}

/*! @brief Concatenate two fat strings
 *  @param [out] strD : pointer to destination string
 *  @param [in] strS : pointer to string to concatenate onto strD
 *  @return number of characters concatenated
 */
static int32_t strnCat(StrN_t *strD, const StrN_t *strS) {
  /* Check bounds to make sure it won't go over the end. If so, return the
   * actual number of bytes that are copied.
   */
  int32_t newLen;
  int32_t bytesToCopy;

  bytesToCopy = strS->n;
  newLen      = strS->n + strD->n;
  if (newLen >= strD->m) {
    bytesToCopy = strD->m - strD->n;
  }

  memcpy((strD->str + strD->n), strS->str, bytesToCopy);
  return bytesToCopy;
}

int32_t dataPackSerial(const Emon32Dataset_t *pData, char *pDst, int32_t m,
                       bool json) {
  EMON32_ASSERT(pData);
  EMON32_ASSERT(pDst);

  StrN_t strn;
  initFields(&strn, pDst, m);

  catMsg(&strn, pData->msgNum, json);

  /* V channels; only print V2/V3 if either active */
  uint32_t numV = (pData->pECM->activeCh & 0x6) ? NUM_V : 1;

  for (uint32_t i = 0; i < numV; i++) {
    catId(&strn, (i + 1), STR_V, json);
    strn.n += strnCatFloat(&strn, pData->pECM->rmsV[i]);
  }

  /* CT channels (power and energy)
   * Only print onboard CTs 7-12 if any are present
   */
  uint32_t numCT = (pData->pECM->activeCh & (0x3f << (NUM_V + (NUM_CT / 2))))
                       ? NUM_CT
                       : (NUM_CT / 2);

  for (uint32_t i = 0; i < numCT; i++) {
    catId(&strn, (i + 1), STR_P, json);
    strn.n += strnCatInt(&strn, pData->pECM->CT[i].realPower);
  }
  for (uint32_t i = 0; i < numCT; i++) {
    catId(&strn, (i + 1), STR_E, json);
    strn.n += strnCatInt(&strn, pData->pECM->CT[i].wattHour);
  }

  for (uint32_t i = 0; i < NUM_OPA; i++) {
    catId(&strn, (i + 1), STR_PULSE, json);
    strn.n += strnCatInt(&strn, pData->pulseCnt[i]);
  }

  for (uint32_t i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    catId(&strn, (i + 1), STR_TEMP, json);
    strn.n +=
        strnCatFloat(&strn, tempAsFloat(TEMP_INTF_ONEWIRE, pData->temp[i]));
  }

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}

int_fast8_t dataPackPacked(const Emon32Dataset_t *pData, void *pPacked,
                           PackedRange_t range) {

  /* Both upper and lower packets share the same initial data structure.
   * Differentiate for pulse or temperature readings. */

  bool isUpper = (PACKED_UPPER == range);

  PackedDataCommon_t *pCommon = pPacked;
  pCommon->msg                = pData->msgNum;

  for (uint32_t v = 0; v < NUM_V; v++) {
    pCommon->V[v] = qfp_float2int_z(qfp_fmul(pData->pECM->rmsV[v], 100.0f));
  }

  for (uint32_t i = 0; i < (NUM_CT / 2); i++) {
    pCommon->P[i] = pData->pECM->CT[i + ((NUM_CT / 2) * isUpper)].realPower;
    pCommon->E[i] = pData->pECM->CT[i + ((NUM_CT / 2) * isUpper)].wattHour;
  }

  if (PACKED_LOWER == range) {
    PackedDataLower6_t *pLower = pPacked;
    for (uint32_t p = 0; p < 2u; p++) {
      pLower->pulse[p] = pData->pulseCnt[p];
    }
    return sizeof(*pLower);
  } else if (PACKED_UPPER == range) {
    PackedDataUpper6_t *pUpper = pPacked;
    for (uint32_t t = 0; t < (TEMP_MAX_ONEWIRE / 2); t++) {
      /* Sent as 100x the temperature value. */
      pUpper->temp[t] = (pData->temp[t] * 6) + (pData->temp[t] >> 2);
    }
    return sizeof(*pUpper);
  }

  return 0;
}
