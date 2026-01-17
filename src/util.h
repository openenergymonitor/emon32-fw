#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum ITOA_BASE_ { ITOA_BASE10, ITOA_BASE16 } ITOA_BASE_t;

typedef struct ConvFloat_ {
  bool  valid; /* true if the value in val is valid */
  float val;   /* converted float value */
} ConvFloat_t;

typedef struct ConvInt_ {
  bool valid; /* true if the value in val is valid */
  union {
    int32_t i32;
    int16_t i16;
    int8_t  i8;
  } val;
} ConvInt_t;

typedef struct ConvUint_ {
  bool valid; /* true if the value in val is valid */
  union {
    uint32_t u32;
    uint16_t u16;
    uint8_t  u8;
  } val;
} ConvUint_t;

/*! @brief Convert null terminated string to float, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @return converted float and status
 */
ConvFloat_t utilAtof(const char *pBuf);

/*! @brief Convert null terminated string to signed integer.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return converted integer and status
 */
ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base);

/*! @brief Convert null terminated string to unsigned integer.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return converted unsigned integer and status
 */
ConvUint_t utilAtoui(const char *pBuf, ITOA_BASE_t base);

/*! @brief Indicate if a character is printable
 *  @param [in] c : character to check
 *  @return true if printable, false otherwise
 */
bool utilCharPrintable(const char c);

/*! @brief Convert float to null terminated base 10 string, with 2 dp.
 *         precision.
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @return the number of characters (including NULL).
 */
size_t utilFtoa(char *pBuf, float val);

/*! @brief Convert signed integer to null terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 12 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return the number of characters (including NULL).
 */
size_t utilItoa(char *pBuf, int32_t val, const ITOA_BASE_t base);

/*! @brief Convert unsigned integer to null terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return the number of characters (including NULL).
 */
size_t utilUtoa(char *pBuf, uint32_t val, const ITOA_BASE_t base);
