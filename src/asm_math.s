@ Optimized math routines for Cortex-M0+

.syntax unified
.cpu cortex-m0plus
.thumb

.section .ramfunc,"ax",%progbits

@ exported symbols
.global usqr64
.global ssqr64
.global smul64

@*******************************
@ Unsigned square 32^2 -> 64 bit
@*******************************
@ Input:  r0 = 32-bit unsigned value
@ Output: r0 = low 32 bits of result
@         r1 = high 32 bits of result

.thumb_func
usqr64:
    lsrs    r1, r0, #16         @ input [16:31] >> 16
    uxth    r0, r0              @ input [0:15]

    mov     r2, r1              @ copy [16:31] >> 16

    muls    r2, r0              @ input [0:15] * input [16:31]
    muls    r1, r1              @ input [16:31] >> 16 squared
    muls    r0, r0              @ input [0:15] squared

    lsrs    r3, r2, #15         @ (input [0:15] * input [16:31]) >> 15
    lsls    r2, r2, #17         @ (input [0:15] * input [16:31]) << 17
    adds    r0, r2              @ input [0:15]^2 + (input [0:15] * input [16:31]) << 17
    adcs    r1, r3              @ add carry + high part

    bx      lr

@*****************************
@ Signed square 32^2 -> 64 bit
@*****************************
@ Input:  r0 = 32-bit signed value
@ Output: r0 = low 32 bits of result
@         r1 = high 32 bits of result
@ Note: x^2 = (-x)^2, so take absolute value and use unsigned algorithm

.thumb_func
ssqr64:
    cmp     r0, #0
    bge     usqr64              @ if positive, use unsigned
    negs    r0, r0              @ r0 = -r0
    b       usqr64              @ branch to unsigned square

@*************************************
@ Signed 32 x 32 -> 64 bit multiply
@*************************************
@ Input:  r0 = 32-bit signed factor A
@         r1 = 32-bit signed factor B
@ Output: r0 = low 32 bits of result
@         r1 = high 32 bits of result

.thumb_func
smul64:
    push    {r4}

    uxth    r2, r0              @ Factor0 lo [0:15]
    asrs    r0, r0, #16         @ Factor0 hi [16:31]
    asrs    r3, r1, #16         @ Factor1 hi [16:31]
    uxth    r1, r1              @ Factor1 lo [0:15]

    mov     r4, r1              @ Copy Factor1 lo [0:15]

    muls    r1, r2              @ Factor1 lo * Factor0 lo
    muls    r4, r0              @ Factor1 lo * Factor0 hi
    muls    r0, r3              @ Factor0 hi * Factor1 hi
    muls    r3, r2              @ Factor1 hi * Factor0 lo

    lsls    r2, r4, #16         @ (Factor1 lo * Factor0 hi) << 16
    asrs    r4, r4, #16         @ (Factor1 lo * Factor0 hi) >> 16
    adds    r1, r2              @ low += (Factor1 lo * Factor0 hi) << 16
    adcs    r0, r4              @ high += (Factor1 lo * Factor0 hi) >> 16 + carry

    lsls    r2, r3, #16         @ (Factor1 hi * Factor0 lo) << 16
    asrs    r3, r3, #16         @ (Factor1 hi * Factor0 lo) >> 16
    adds    r1, r2              @ low += (Factor1 hi * Factor0 lo) << 16
    adcs    r0, r3              @ high += (Factor1 hi * Factor0 lo) >> 16 + carry

    @ Swap r0/r1 to match ARM EABI (r0=low, r1=high)
    mov     r2, r0
    mov     r0, r1
    mov     r1, r2

    pop     {r4}
    bx      lr
