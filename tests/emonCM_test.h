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

float qfp_uint642float(uint64_t a) { return (float)a; }

int qfp_float2int_z(float a) { return (int)a; }

int qfp_float2int(float a) { return (int)a; }

float qfp_double2float(double a) { return (float)a; }

double qfp_float2double(float a) { return (double)a; }

double qfp_int2double(int32_t a) { return (double)a; }

double qfp_uint2double(uint32_t a) { return (double)a; }

double qfp_uint642double(uint64_t a) { return (double)a; }

double qfp_ddiv(double a, double b) { return a / b; }

double qfp_dmul(double a, double b) { return a * b; }

double qfp_dsqrt(double a) { return sqrt(a); }

double qfp_dsub(double a, double b) { return a - b; }

/* Reference C implementations for asm_math functions */
uint64_t usqr64(uint32_t x) { return (uint64_t)x * x; }

int64_t smul64(int32_t a, int32_t b) { return (int64_t)a * (int64_t)b; }

uint64_t ssqr64(int32_t x) { return (uint64_t)((int64_t)x * x); }
