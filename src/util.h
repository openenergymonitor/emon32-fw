#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum ITOA_BASE_ { ITOA_BASE10, ITOA_BASE16 } ITOA_BASE_t;

typedef struct ConvFloat_ {
  bool  valid; /* true if the value in val is valid */
  float val;   /* converted float value */
} ConvFloat_t;

typedef struct ConvInt_ {
  bool    valid; /* true if the value in val is valid */
  int32_t val;   /* converted integer value */
} ConvInt_t;

/*! @brief Return the absolute value of an integer
 *  @param [in] x : integer to convert
 *  @return absolute value of x
 */
uint32_t utilAbs(const int32_t x);

/*! @brief Convert null terminated string to float, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @return converted float and status
 */
ConvFloat_t utilAtof(const char *pBuf);

/*! @brief Convert null terminated string to integer, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return converted integer and status
 */
ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base);

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
uint32_t utilFtoa(char *pBuf, float val);

/*! @brief Convert integer to null terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return the number of characters (including NULL).
 */
uint32_t utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base);

/*! @brief Returns the number of characters up to, but not including, NULL
 *  @param [in] pBuf : pointer to the NULL terminated string buffer
 *  @param number of characters, not including NULL
 */
uint32_t utilStrlen(const char *pBuf);

/*! @brief Reverse an array (typically string)
 *  @param [in] pBuf : pointer to the buffer
 *  @param [in] len : length of buffer to reverse
 */
void utilStrReverse(char *pBuf, uint32_t len);
