/*
 * Tiny Code Generator for QEMU
 *
 * Copyright (c) 2008 Fabrice Bellard
 * Copyright (c) 2008 Andrzej Zaborowski
 * Copyright (C) 2012 Imagination Technologies Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define TCG_TARGET_METAG 1

#include "metag/units.h"

#if (TARGET_LONG_BITS != 32) || \
    (defined(TARGET_WORDS_BIGENDIAN))
#error unsupported target
#endif

#define UNTESTED() \
    do { \
        fprintf(stderr, "%s:%d: %s WARNING: Entering untested frame\n", \
                        __FILE__, __LINE__, __func__); \
    } while (0)

#define UNIMPLEMENTED(fmt, ...) \
    do { \
        fprintf(stderr, "%s:%d: Unimplemented %s(" fmt ")\n", \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
        tcg_abort(); \
    } while (0)

/* not all registers listed here are used by tcg, see tcg_target_init */
typedef enum {
    TCG_REG_D0Re0 = 0,
    TCG_REG_D0Ar6,
    TCG_REG_D0Ar4,
    TCG_REG_D0Ar2,
    TCG_REG_D0FrT,
    TCG_REG_D0_5,
    TCG_REG_D0_6,
    TCG_REG_D0_7,
    TCG_REG_D0_8,
    TCG_REG_D0_9,
    TCG_REG_D0_10,
    TCG_REG_D0_11,
    TCG_REG_D0_12,
    TCG_REG_D0_13,
    TCG_REG_D0_14,
    TCG_REG_D0_15,
    TCG_REG_D1Re0,
    TCG_REG_D1Ar5,
    TCG_REG_D1Ar3,
    TCG_REG_D1Ar1,
    TCG_REG_D1RtP,
    TCG_REG_D1_5,
    TCG_REG_D1_6,
    TCG_REG_D1_7,
    TCG_REG_D1_8,
    TCG_REG_D1_9,
    TCG_REG_D1_10,
    TCG_REG_D1_11,
    TCG_REG_D1_12,
    TCG_REG_D1_13,
    TCG_REG_D1_14,
    TCG_REG_D1_15,
    TCG_REG_A0StP = 32,
    TCG_REG_A0FrP,
    TCG_REG_A0_2,
    TCG_REG_A0_3,
    TCG_REG_A0_4,
    TCG_REG_A0_5,
    TCG_REG_A0_6,
    TCG_REG_A0_7,
    TCG_REG_A1GbP,
    TCG_REG_A1LbP,
    TCG_REG_A1_2,
    TCG_REG_A1_3,
    TCG_REG_A1_4,
    TCG_REG_A1_5,
    TCG_REG_A1_6,
    TCG_REG_A1_7,
    TCG_REG_PC,
    /* global regs - never used by tcg only explicitly for debugging */
    TCG_REG_D0_16,
    TCG_REG_D0_17,
    TCG_REG_D0_18,
    TCG_REG_D0_19,
    TCG_REG_D0_20,
    TCG_REG_D0_21,
    TCG_REG_D0_22,
    TCG_REG_D0_23,
    TCG_REG_D1_16,
    TCG_REG_D1_17,
    TCG_REG_D1_18,
    TCG_REG_D1_19,
    TCG_REG_D1_20,
    TCG_REG_D1_21,
    TCG_REG_D1_22,
    TCG_REG_D1_23,
    TCG_REG_D0_24,
    TCG_REG_D0_25,
    TCG_REG_D0_26,
    TCG_REG_D0_27,
    TCG_REG_D0_28,
    TCG_REG_D0_29,
    TCG_REG_D0_30,
    TCG_REG_D0_31,
    TCG_REG_D1_24,
    TCG_REG_D1_25,
    TCG_REG_D1_26,
    TCG_REG_D1_27,
    TCG_REG_D1_28,
    TCG_REG_D1_29,
    TCG_REG_D1_30,
    TCG_REG_D1_31,
} TCGReg;

typedef struct {
    unsigned short r;   /* register number */
    MetaUnit unit;      /* meta unit */
    unsigned char bu;   /* BU representation or 0xff if n/a */
} reg_info;

static const reg_info metag_reg_info[] = {
    [TCG_REG_D0Re0] =   {  0, META_UNIT_D0, 0x01 },
    [TCG_REG_D0Ar6] =   {  1, META_UNIT_D0, 0x01 },
    [TCG_REG_D0Ar4] =   {  2, META_UNIT_D0, 0x01 },
    [TCG_REG_D0Ar2] =   {  3, META_UNIT_D0, 0x01 },
    [TCG_REG_D0FrT] =   {  4, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_5] =    {  5, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_6] =    {  6, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_7] =    {  7, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_8] =    {  8, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_9] =    {  9, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_10] =   { 10, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_11] =   { 11, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_12] =   { 12, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_13] =   { 13, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_14] =   { 14, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_15] =   { 15, META_UNIT_D0, 0x01 },
    [TCG_REG_D1Re0] =   {  0, META_UNIT_D1, 0x02 },
    [TCG_REG_D1Ar5] =   {  1, META_UNIT_D1, 0x02 },
    [TCG_REG_D1Ar3] =   {  2, META_UNIT_D1, 0x02 },
    [TCG_REG_D1Ar1] =   {  3, META_UNIT_D1, 0x02 },
    [TCG_REG_D1RtP] =   {  4, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_5] =    {  5, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_6] =    {  6, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_7] =    {  7, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_8] =    {  8, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_9] =    {  9, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_10] =   { 10, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_11] =   { 11, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_12] =   { 12, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_13] =   { 13, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_14] =   { 14, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_15] =   { 15, META_UNIT_D1, 0x02 },
    [TCG_REG_A0StP] =   {  0, META_UNIT_A0, 0x03 },
    [TCG_REG_A0FrP] =   {  1, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_2] =    {  2, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_3] =    {  3, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_4] =    {  4, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_5] =    {  5, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_6] =    {  6, META_UNIT_A0, 0x03 },
    [TCG_REG_A0_7] =    {  7, META_UNIT_A0, 0x03 },
    [TCG_REG_A1GbP] =   {  0, META_UNIT_A1, 0x00 },
    [TCG_REG_A1LbP] =   {  1, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_2] =    {  2, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_3] =    {  3, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_4] =    {  4, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_5] =    {  5, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_6] =    {  6, META_UNIT_A1, 0x00 },
    [TCG_REG_A1_7] =    {  7, META_UNIT_A1, 0x00 },
    [TCG_REG_PC] =      {  0, META_UNIT_PC, 0xff },
    /* global regs */
    [TCG_REG_D0_16] =   { 16, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_17] =   { 17, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_18] =   { 18, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_19] =   { 19, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_20] =   { 20, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_21] =   { 21, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_22] =   { 22, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_23] =   { 23, META_UNIT_D0, 0x01 },
    [TCG_REG_D1_16] =   { 16, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_17] =   { 17, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_18] =   { 18, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_19] =   { 19, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_20] =   { 20, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_21] =   { 21, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_22] =   { 22, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_23] =   { 23, META_UNIT_D1, 0x02 },
    [TCG_REG_D0_24] =   { 24, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_25] =   { 25, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_26] =   { 26, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_27] =   { 27, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_28] =   { 28, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_29] =   { 29, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_30] =   { 30, META_UNIT_D0, 0x01 },
    [TCG_REG_D0_31] =   { 31, META_UNIT_D0, 0x01 },
    [TCG_REG_D1_24] =   { 24, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_25] =   { 25, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_26] =   { 26, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_27] =   { 27, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_28] =   { 28, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_29] =   { 29, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_30] =   { 30, META_UNIT_D1, 0x02 },
    [TCG_REG_D1_31] =   { 31, META_UNIT_D1, 0x02 },
};

typedef enum {
    COND_A  = 0x0,      /* Always (true) */
    COND_EQ = 0x1,      /* Equal, Zero (==) */
    COND_Z  = 0x1,
    COND_NE = 0x2,      /* Not equal, Not zero (!=) */
    COND_NZ = 0x2,
    COND_CS = 0x3,      /* Carry set, Lower (Unsigned <) */
    COND_LO = 0x3,
    COND_CC = 0x4,      /* Carry clear, Higher or same (unsigned >=) */
    COND_HS = 0x4,
    COND_MI = 0x5,      /* Minus, Negative (signed < 0)*/
    COND_N  = 0x5,
    COND_PL = 0x6,      /* Positive, N clear (signed >= 0) */
    COND_NC = 0x6,
    COND_VS = 0x7,      /* Overflow set */
    COND_VC = 0x8,      /* Overflow clear */
    COND_HI = 0x9,      /* Higher (unsigned >) */
    COND_LS = 0xa,      /* Lower or same (unsigned <=) */
    COND_GE = 0xb,      /* Greater than or equal (signed >=) */
    COND_LT = 0xc,      /* Less than (signed <) */
    COND_GT = 0xd,      /* Greater than (signed >) */
    COND_LE = 0xe,      /* Less than or equal (signed <=) */
    COND_NV = 0xf,      /* Never (false) */
} MetaCond;

/* don't include global regs as tcg doesn't need to know about them */
#define TCG_TARGET_NB_REGS              49 /* must not be > 64 */
#define TCG_TARGET_NB_UNITS             10
#define TCG_TARGET_REG_BITS             32

#undef TCG_TARGET_WORDS_BIGENDIAN
#undef TCG_TARGET_STACK_GROWSDOWN

/* Meta imm constraints */
#define TCG_CT_META_16_CONST            0x100   /* 16 bit constant */
#define TCG_CT_META_5_CONST             0x200   /* 5 bit constant (shifts) */
#define TCG_CT_META_6_CONST             0x400   /* 6 bit constant (get/sets) */
#define TCG_CT_META_8_CONST             0x800   /* 8 bit constant */
#define TCG_CT_META_CONST_HAS_M         0x1000  /* constant has mask flag */

/* used for function call generation */
#define TCG_REG_CALL_STACK              TCG_REG_A0StP
#define TCG_TARGET_STACK_ALIGN          8
#define TCG_TARGET_CALL_ALIGN_ARGS      1
/* The offset is the size of our negative size of the frame because
 * A0StP is at the end of the frame */
#define TCG_TARGET_CALL_STACK_OFFSET -(((TCG_STATIC_CALL_ARGS_SIZE \
                                        + CPU_TEMP_BUF_NLONGS * sizeof(long)) \
                                      + TCG_TARGET_STACK_ALIGN - 1)\
                                      & ~(TCG_TARGET_STACK_ALIGN - 1))
#define TCG_TARGET_STACK_GROWSUP

/* optional instructions */
#define TCG_TARGET_HAS_andc_i32         0
#define TCG_TARGET_HAS_orc_i32          0
#define TCG_TARGET_HAS_not_i32          0
#define TCG_TARGET_HAS_neg_i32          1
#define TCG_TARGET_HAS_rot_i32          0
#define TCG_TARGET_HAS_eqv_i32          0
#define TCG_TARGET_HAS_nand_i32         0
#define TCG_TARGET_HAS_nor_i32          0
#define TCG_TARGET_HAS_div_i32          0
#define TCG_TARGET_HAS_deposit_i32      0
#define TCG_TARGET_HAS_movcond_i32      1
/* we dont really have these but they are implemented for one reason
   or another - mainly for use in qemu_st/ld ops */
#define TCG_TARGET_HAS_ext8s_i32        1
#define TCG_TARGET_HAS_ext16s_i32       1
#define TCG_TARGET_HAS_ext8u_i32        1
#define TCG_TARGET_HAS_ext16u_i32       1
#define TCG_TARGET_HAS_bswap16_i32      1
#define TCG_TARGET_HAS_bswap32_i32      1

#define TCG_TARGET_HAS_GUEST_BASE

enum {
    /* Note: must be synced with dyngen-exec.h and op constraints! */
    TCG_AREG0 = TCG_REG_D0_5,
};

#define CACHEW_DATA     0
#define CACHEW_CODE     1
#define CACHEW_CACHE    0
#define CACHEW_TLB      2
#define CACHEW_ICACHE   (CACHEW_CODE | CACHEW_CACHE)

static inline void flush_icache_range(unsigned long start, unsigned long stop)
{
    unsigned long long tmp;
    /* Must 0 out lower 6 bits */
    start &= (-1 << 6);

    for (; start < stop; start += 64) {
        asm volatile("CACHEWD [%0],%1\n"
                    :
                    : "da" (start), "da" (CACHEW_ICACHE));
    }
    start = ((stop - 4) & (-1 << 6));
    asm volatile("CACHEWD [%1],%2\n"
                 "CACHERL %0,%t0,[%1]\n"
                 : "=da" (tmp)
                 : "da" (start), "da" (CACHEW_ICACHE));
}
