#pragma once

#include <math.h>
#include <stdint.h>

/* Redefine the QFPLIB functions to use native floats */
float qfp_fadd(float a, float b) { return a + b; }

float qfp_fcos(float a) { return (float)cos((double)a); }

float qfp_fdiv(float a, float b) { return a / b; }

float qfp_fmul(float a, float b) { return a * b; }

float qfp_fsin(float a) { return (float)sin((double)a); }

float qfp_fsub(float a, float b) { return a - b; }

float qfp_fsqrt(float a) { return sqrt(a); }

float qfp_int2float(int a) { return (float)a; }

float qfp_int642float(int64_t a) { return (float)a; }

float qfp_uint2float(unsigned int a) { return (float)a; }

int qfp_float2int_z(float a) { return (int)a; }

int qfp_float2int(float a) { return (int)a; }
