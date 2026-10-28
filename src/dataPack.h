#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "emon32.h"

typedef struct CHActive_ {
  bool V[NUM_V];
  bool CT[NUM_CT];
  bool pulse[NUM_OPA];
  bool analog;
} CHActive_t;

typedef struct DataPackOpts_ {
  CHActive_t *pChsActive;
  size_t      bufSize;
  bool        json;
  uint8_t     serialLog;
} DataPackOpts_t;

typedef enum PackedRange_ {
  PACKED_CT1_6,
  PACKED_TEMP_PULSE,
  PACKED_CT7_12,
} PackedRange_t;

/*! @brief Packs the emon_CM packet into serial format.
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of m. If the return value != m, then the
 *         buffer would have overflowed (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] pOpts : pointer to the options struct
 *  @return the number of the characters that would be packed
 */
size_t dataPackSerial(const Emon32Dataset_t *pData, char *pDst,
                      const DataPackOpts_t *pOpts);

/*! @brief Pack the voltage, power, energy, temperature, and pulse data into a
 *         packed structure for transmission over RFM link.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pPacked : pointer to the destination packet
 *  @param [in] range : select the packing range when packet size > buffer
 *  @return number of bytes in the packet
 */
uint8_t dataPackPacked(const Emon32Dataset_t *restrict pData,
                       void *restrict pPacked, const PackedRange_t range);
