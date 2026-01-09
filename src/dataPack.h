#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "emon32.h"

typedef enum PackedRange_ {
  PACKED_ALL,
  PACKED_UPPER,
  PACKED_LOWER
} PackedRange_t;

/*! @brief Packs the emon_CM packet into serial format.
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of m. If the return value != m, then the
 *         buffer would have overflowed (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] m : width of the destination buffer
 *  @param [in] json : false -> K:V; true -> JSON
 *  @return the number of the characters that would be packed
 */
size_t dataPackSerial(const Emon32Dataset_t *pData, char *pDst, const size_t m,
                      const bool json);

/*! @brief Pack the voltage, power, energy, temperature, and pulse data into a
 *         packed structure for transmission over RFM link.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pPacked : pointer to the destination packet
 *  @param [in] range : select the packing range when packet size > buffer
 *  @return number of bytes in the packet
 */
int8_t dataPackPacked(const Emon32Dataset_t *restrict pData,
                      void *restrict pPacked, const PackedRange_t range);
