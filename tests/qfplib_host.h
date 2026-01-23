/*
 * Host-side mock implementations of qfplib functions
 * Used for native compilation of unit tests
 */

#pragma once

#include <math.h>
#include <stdint.h>

/* Basic arithmetic */
static inline float qfp_fadd(float a, float b) { return a + b; }
static inline float qfp_fsub(float a, float b) { return a - b; }
static inline float qfp_fmul(float a, float b) { return a * b; }
static inline float qfp_fdiv(float a, float b) { return a / b; }

/* Trigonometric */
static inline float qfp_fcos(float a) { return (float)cos((double)a); }
static inline float qfp_fsin(float a) { return (float)sin((double)a); }
static inline float qfp_fsqrt(float a) { return (float)sqrt((double)a); }

/* Conversions: int/uint to float */
static inline float qfp_int2float(int32_t a) { return (float)a; }
static inline float qfp_uint2float(uint32_t a) { return (float)a; }
static inline float qfp_int642float(int64_t a) { return (float)a; }
static inline float qfp_uint642float(uint64_t a) { return (float)a; }

/* Conversions: float to int/uint (truncate toward zero) */
static inline int32_t  qfp_float2int_z(float a) { return (int32_t)a; }
static inline int32_t  qfp_float2int(float a) { return (int32_t)a; }
static inline uint32_t qfp_float2uint(float a) { return (uint32_t)a; }

/* Fixed point conversions */
static inline float qfp_fix2float(int32_t x, int f) {
  return (float)x / (float)(1 << f);
}
