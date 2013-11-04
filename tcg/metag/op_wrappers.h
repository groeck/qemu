/*
 * Tiny Code Generator for QEMU
 *
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

#ifndef OP_WRAPPERS_H_
#define OP_WRAPPERS_H_

typedef enum {
    TCG_META_LOGOP_AND = 0x2,
    TCG_META_LOGOP_OR = 0x3,
    TCG_META_LOGOP_XOR = 0x4,
} TCGMetaLogOp;

typedef enum {
    TCG_META_CMPOP_CMP = 0, TCG_META_CMPOP_TST = 1,
} TCGMetaCmpOp;

static void tcg_out_addsub(TCGContext *s, int setc, int sub, int o1z,
                TCGReg dst, TCGReg src1, TCGReg src2, MetaCond cc);
static void tcg_out_addsubi(TCGContext *s, int setc, int sub, int o1z,
                TCGReg dst, TCGReg src1, tcg_target_long imm, MetaCond cc);
static void tcg_out_shift(TCGContext *s, int setc, int arith, int right,
                TCGReg dst, TCGReg src1, TCGReg src2, MetaCond cc);
static void tcg_out_shifti(TCGContext *s, int setc, int arith, int right,
                TCGReg dst, TCGReg src1, tcg_target_ulong imm, MetaCond cc);
static void tcg_out_multiply(TCGContext *s, int l, TCGReg dst, TCGReg src1,
                TCGReg src2);
static void tcg_out_multiplyi(TCGContext *s, int l, TCGReg dst, TCGReg src1,
                tcg_target_long imm);
static void tcg_out_logop(TCGContext *s, int setc, TCGMetaLogOp op, TCGReg dst,
                TCGReg src1, TCGReg src2, MetaCond cc);
static void tcg_out_logopi(TCGContext *s, int setc, TCGMetaLogOp op,
                TCGReg dst, TCGReg src1, tcg_target_long imm, MetaCond cc);
static void tcg_out_cmptst(TCGContext *s, TCGMetaCmpOp op, MetaCond cc,
                TCGReg src1, TCGReg src2);
static void tcg_out_cmptsti(TCGContext *s, TCGMetaCmpOp op, MetaCond cc,
                TCGReg src1, tcg_target_long imm);
static void tcg_out_movcc(TCGContext *s, MetaCond cc, TCGReg dst, TCGReg src,
                int swap);
static void tcg_out_neg(TCGContext *s, TCGReg dst, TCGReg src);
static void tcg_out_negi(TCGContext *s, TCGReg dst, tcg_target_long imm);
static void tcg_out_b(TCGContext *s, tcg_target_long relocd_dest, MetaCond cc,
                int loop);
static void tcg_out_jmpi(TCGContext *s, tcg_target_ulong relocd_dest);
static void tcg_out_exts(TCGContext *s, int w, TCGReg dst, TCGReg src);
static void tcg_out_getset(TCGContext *s, int set, int l, int ua, int pp,
                int se, TCGReg src, TCGReg base, TCGReg offset);
static void tcg_out_getseti(TCGContext *s, int set, int l, int ua, int pp,
                int se, TCGReg dst, TCGReg base, tcg_target_long offset);
static void tcg_out_mgetl(TCGContext *s, TCGReg base, TCGReg first,
                unsigned int rmask);
static void tcg_out_msetl(TCGContext *s, TCGReg base, TCGReg first,
                unsigned int rmask);
static void tcg_out_call(TCGContext *s, TCGReg addr);
static void tcg_out_calli(TCGContext *s, tcg_target_ulong abs_addr);
static void tcg_out_bswap(TCGContext *s, int l, TCGReg dst, TCGReg src);

static inline void tcg_out_getb(TCGContext *s, int se, TCGReg dst,
                TCGReg base, TCGReg offset)
{
    tcg_out_getset(s, 0, 0, 0, 0, se, dst, base, offset);
}

static inline void tcg_out_getbi(TCGContext *s, int se, TCGReg dst,
                TCGReg base, tcg_target_long offset)
{
    tcg_out_getseti(s, 0, 0, 0, 0, se, dst, base, offset);
}

static inline void tcg_out_setb(TCGContext *s, TCGReg src, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 1, 0, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_setbi(TCGContext *s, TCGReg src, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 1, 0, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_getw(TCGContext *s, int se, TCGReg dst,
                TCGReg base, TCGReg offset)
{
    tcg_out_getset(s, 0, 1, 0, 0, se, dst, base, offset);
}

static inline void tcg_out_getwi(TCGContext *s, int se, TCGReg dst,
                TCGReg base, tcg_target_long offset)
{
    tcg_out_getseti(s, 0, 1, 0, 0, se, dst, base, offset);
}

static inline void tcg_out_setw(TCGContext *s, TCGReg src, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 1, 1, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_setwi(TCGContext *s, TCGReg src, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 1, 1, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_getd(TCGContext *s, TCGReg dst, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 0, 2, 0, 0, 0, dst, base, offset);
}

static inline void tcg_out_getdi(TCGContext *s, TCGReg dst, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 0, 2, 0, 0, 0, dst, base, offset);
}

static inline void tcg_out_setd(TCGContext *s, TCGReg src, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 1, 2, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_setdi(TCGContext *s, TCGReg src, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 1, 2, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_getl(TCGContext *s, TCGReg dst, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 0, 3, 0, 0, 0, dst, base, offset);
}

static inline void tcg_out_getli(TCGContext *s, TCGReg dst, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 0, 3, 0, 0, 0, dst, base, offset);
}

static inline void tcg_out_setl(TCGContext *s, TCGReg src, TCGReg base,
                TCGReg offset)
{
    tcg_out_getset(s, 1, 3, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_setli(TCGContext *s, TCGReg src, TCGReg base,
                tcg_target_long offset)
{
    tcg_out_getseti(s, 1, 3, 0, 0, 0, src, base, offset);
}

static inline void tcg_out_b_noaddr(TCGContext *s, MetaCond cc)
{
    tcg_out_b(s, 0, cc, 0);
}

static inline void tcg_out_jmp_noaddr(TCGContext *s)
{
    tcg_out_jmpi(s, 0);
}

static inline void tcg_out_swap(TCGContext *s, TCGReg r1, TCGReg r2)
{
    tcg_out_movcc(s, COND_A, r1, r2, 1);
}

static inline void tcg_out_add(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_addsub(s, setc, 0, 0, dst, src1, src2, cc);
}

static inline void tcg_out_addi(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_addsubi(s, setc, 0, 0, dst, src1, imm, cc);
}

static inline void tcg_out_addz(TCGContext *s, TCGReg dst, TCGReg src)
{
    tcg_out_addsub(s, 0, 0, 1, dst, 0, src, COND_A);
}

static inline void tcg_out_addzi(TCGContext *s, TCGReg dst,
                tcg_target_long imm)
{
    tcg_out_addsubi(s, 0, 0, 1, dst, 0, imm, COND_A);
}

static inline void tcg_out_sub(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_addsub(s, setc, 1, 0, dst, src1, src2, cc);
}

static inline void tcg_out_subi(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_addsubi(s, setc, 1, 0, dst, src1, imm, cc);
}

static inline void tcg_out_subz(TCGContext *s, TCGReg dst, TCGReg src)
{
    tcg_out_addsub(s, 0, 1, 1, dst, 0, src, COND_A);
}

static inline void tcg_out_subzi(TCGContext *s, TCGReg dst,
                tcg_target_long imm)
{
    tcg_out_addsubi(s, 0, 1, 1, dst, 0, imm, COND_A);
}

static inline void tcg_out_lsl(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_shift(s, setc, 0, 0, dst, src1, src2, cc);
}

static inline void tcg_out_lsli(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_shifti(s, setc, 0, 0, dst, src1, imm, cc);
}

static inline void tcg_out_lsr(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_shift(s, setc, 0, 1, dst, src1, src2, cc);
}

static inline void tcg_out_lsri(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_shifti(s, setc, 0, 1, dst, src1, imm, cc);
}

static inline void tcg_out_asr(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_shift(s, setc, 1, 1, dst, src1, src2, cc);
}

static inline void tcg_out_asri(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_shifti(s, setc, 1, 1, dst, src1, imm, cc);
}

static inline void tcg_out_mulw(TCGContext *s, TCGReg dst, TCGReg src1,
                TCGReg src2)
{
    tcg_out_multiply(s, 0, dst, src1, src2);
}

static inline void tcg_out_mulwi(TCGContext *s, TCGReg dst, TCGReg src1,
                tcg_target_long imm)
{
    tcg_out_multiplyi(s, 0, dst, src1, imm);
}

static inline void tcg_out_muld(TCGContext *s, TCGReg dst, TCGReg src1,
                TCGReg src2)
{
    tcg_out_multiply(s, 1, dst, src1, src2);
}

static inline void tcg_out_muldi(TCGContext *s, TCGReg dst, TCGReg src1,
                tcg_target_long imm)
{
    tcg_out_multiplyi(s, 1, dst, src1, imm);
}

static inline void tcg_out_and(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_logop(s, setc, TCG_META_LOGOP_AND, dst, src1, src2, cc);
}

static inline void tcg_out_andi(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_logopi(s, setc, TCG_META_LOGOP_AND, dst, src1, imm, cc);
}

static inline void tcg_out_or(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_logop(s, setc, TCG_META_LOGOP_OR, dst, src1, src2, cc);
}

static inline void tcg_out_ori(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_logopi(s, setc, TCG_META_LOGOP_OR, dst, src1, imm, cc);
}

static inline void tcg_out_xor(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, TCGReg src2)
{
    tcg_out_logop(s, setc, TCG_META_LOGOP_XOR, dst, src1, src2, cc);
}

static inline void tcg_out_xori(TCGContext *s, MetaCond cc, int setc,
                TCGReg dst, TCGReg src1, tcg_target_long imm)
{
    tcg_out_logopi(s, setc, TCG_META_LOGOP_XOR, dst, src1, imm, cc);
}

static inline void tcg_out_cmp(TCGContext *s, MetaCond cc, TCGReg src1,
                TCGReg src2)
{
    tcg_out_cmptst(s, TCG_META_CMPOP_CMP, cc, src1, src2);
}

static inline void tcg_out_tst(TCGContext *s, MetaCond cc, TCGReg src1,
                TCGReg src2)
{
    tcg_out_cmptst(s, TCG_META_CMPOP_TST, cc, src1, src2);
}

static inline void tcg_out_cmpi(TCGContext *s, MetaCond cc, TCGReg src1,
                tcg_target_long imm)
{
    tcg_out_cmptsti(s, TCG_META_CMPOP_CMP, cc, src1, imm);
}

static inline void tcg_out_tsti(TCGContext *s, MetaCond cc, TCGReg src1,
                tcg_target_long imm)
{
    tcg_out_cmptsti(s, TCG_META_CMPOP_TST, cc, src1, imm);
}

static inline void tcg_out_ext8s(TCGContext *s, MetaCond cc, TCGReg dst,
                TCGReg src)
{
    if (cc == COND_A) {
        tcg_out_exts(s, 0, dst, src);
    } else {
        tcg_out_lsli(s, cc, 0, dst, src, 24);
        tcg_out_asri(s, cc, 0, dst, dst, 24);
    }
}

static inline void tcg_out_ext16s(TCGContext *s, MetaCond cc, TCGReg dst,
                TCGReg src)
{
    if (cc == COND_A) {
        tcg_out_exts(s, 1, dst, src);
    } else {
        tcg_out_lsli(s, cc, 0, dst, src, 16);
        tcg_out_asri(s, cc, 0, dst, dst, 16);
    }
}

static inline void tcg_out_ext8u(TCGContext *s, MetaCond cc, TCGReg dst,
                TCGReg src)
{
    tcg_out_logopi(s, 0, TCG_META_LOGOP_AND, dst, src, 0xffu, cc);
}

static inline void tcg_out_ext16u(TCGContext *s, MetaCond cc, TCGReg dst,
                TCGReg src)
{
    tcg_out_logopi(s, 0, TCG_META_LOGOP_AND, dst, src, 0xffffu, cc);
}


static inline void tcg_out_bswap32(TCGContext *s, TCGReg dst, TCGReg src)
{
    tcg_out_bswap(s, 0, dst, src);
}

/* Automatically swaps dst and src */
static inline void tcg_out_bswap64(TCGContext *s, TCGReg dst, TCGReg src)
{
    tcg_out_bswap(s, 1, dst, src);
}

static uint64_t tcg_out_umull_helper(uint32_t x, uint32_t y)
{
    return (uint64_t)x * (uint64_t)y;
}

/*
 * unsigned 32bit multiply with 64 bit result
 */
static inline void tcg_out_umull(TCGContext *s, TCGReg dst_lo, TCGReg dst_hi,
                TCGReg src1, TCGReg src2)
{
    /* Our constraints force src1/2 into D1Ar1 and D0Ar2
     * and dst_lo/hi into D[0|1]Re0 */
    assert(dst_lo == TCG_REG_D0Re0);
    assert(dst_hi == TCG_REG_D1Re0);
    assert(src1 == TCG_REG_D1Ar1);
    assert(src2 == TCG_REG_D0Ar2);

    /* Call our helper */
    tcg_out_calli(s, (tcg_target_long) tcg_out_umull_helper);
}

static inline void tcg_out_add2(TCGContext *s, TCGReg dst_lo, TCGReg dst_hi,
                TCGReg src1_lo, TCGReg src1_hi, TCGReg src2_lo, TCGReg src2_hi)
{
    tcg_out_add(s, COND_A, 0, dst_hi, src1_hi, src2_hi);
    tcg_out_add(s, COND_A, 1, dst_lo, src1_lo, src2_lo);
    tcg_out_addi(s, COND_CS, 0, dst_hi, dst_hi, 1);
}

static inline void tcg_out_sub2(TCGContext *s, TCGReg dst_lo, TCGReg dst_hi,
                TCGReg src1_lo, TCGReg src1_hi, TCGReg src2_lo, TCGReg src2_hi)
{
    tcg_out_sub(s, COND_A, 1, dst_hi, src1_hi, src2_hi);
    tcg_out_sub(s, COND_A, 0, dst_lo, src1_lo, src2_lo);
    tcg_out_subi(s, COND_CS, 0, dst_lo, dst_lo, 1);
}

#endif /* OP_WRAPPERS_H_ */
