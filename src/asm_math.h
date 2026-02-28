#pragma once

#include <stdint.h>

/*! @brief Compute the square of a 32-bit unsigned integer with 64-bit result.
 *
 *  This is an optimized assembly routine for Cortex-M0+ that avoids the
 *  overhead of the compiler-generated 64-bit multiply sequence.
 *
 *  @param [in] x : 32-bit unsigned value to square
 *  @return 64-bit result (x * x)
 */
uint64_t usqr64(uint32_t x);

/*! @brief Compute the square of a 32-bit signed integer with 64-bit result.
 *
 *  This is an optimized assembly routine for Cortex-M0+ that avoids the
 *  overhead of the compiler-generated 64-bit multiply sequence.
 *
 *  @param [in] x : 32-bit signed value to square
 *  @return 64-bit unsigned result (x * x), always non-negative
 */
uint64_t ssqr64(int32_t x);
