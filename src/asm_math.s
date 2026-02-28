@ Optimized math routines for Cortex-M0+

.syntax unified
.cpu cortex-m0plus
.thumb

.section .ramfunc,"ax",%progbits

@ exported symbols
.global usqr64
.global ssqr64

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
