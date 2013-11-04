/*
 * Tiny Code Generator for QEMU
 *
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

#include "op_wrappers.h"

#ifndef NDEBUG
static const char * const tcg_target_reg_names[TCG_TARGET_NB_REGS] = {
                "D0Re0", "D0Ar6", "D0Ar4", "D0Ar2",
                "D0FrT", "D0_5", "D0_6", "D0_7",
                "D1Re0", "D1Ar5", "D1Ar3", "D1Ar1",
                "D1RtP", "D1_5", "D1_6", "D1_7",
                "A0StP", "A0FrP", "A0_2", "A0_3",
                "A1GbP", "A1LbP", "A1_2", "A1_3",
                "PC",
                "D0_8", "D0_9", "D0_10", "D0_11",
                "D0_12", "D0_13", "D0_14", "D0_15",
                "D1_8", "D1_9", "D1_10", "D1_11",
                "D1_12", "D1_13", "D1_14", "D1_15", };
#endif

static const int tcg_target_reg_alloc_order[] = {
                TCG_REG_D0_6, TCG_REG_D1_6,
                TCG_REG_D0_7, TCG_REG_D1_7,
                TCG_REG_D0_8, TCG_REG_D1_8,
                TCG_REG_D0_9, TCG_REG_D1_9,
                TCG_REG_D0_10, TCG_REG_D1_10,
                TCG_REG_D0_11, TCG_REG_D1_11,
                TCG_REG_D0_12, TCG_REG_D1_12,
                TCG_REG_D0_13, TCG_REG_D1_13,
                TCG_REG_D0_14, TCG_REG_D1_14,
                TCG_REG_D0_15, TCG_REG_D1_15,
                TCG_REG_D0Ar6, TCG_REG_D1Ar5,
                TCG_REG_D0Ar4, TCG_REG_D1Ar3,
                TCG_REG_D0Ar2, TCG_REG_D1Ar1,
                TCG_REG_D1Re0, TCG_REG_D0Re0,
                TCG_REG_A0_2, TCG_REG_A1_2,
                TCG_REG_A0_3, TCG_REG_A1_3,
                TCG_REG_A0_4, TCG_REG_A1_4,
                TCG_REG_A0_5, TCG_REG_A1_5,
                TCG_REG_A0_6, TCG_REG_A1_6,
                TCG_REG_A0_7, TCG_REG_A1_7, };

/* Argument registers */
static const int tcg_target_call_iarg_regs[6] = {
                TCG_REG_D1Ar1, TCG_REG_D0Ar2,
                TCG_REG_D1Ar3, TCG_REG_D0Ar4,
                TCG_REG_D1Ar5, TCG_REG_D0Ar6, };

/* Return value registers */
static const int tcg_target_call_oarg_regs[2] = {
                TCG_REG_D0Re0, TCG_REG_D1Re0, };

static int meta_unit_base[TCG_TARGET_NB_UNITS] = {
                [META_UNIT_D0] = TCG_REG_D0Re0,
                [META_UNIT_D1] = TCG_REG_D1Re0,
                [META_UNIT_A0] = TCG_REG_A0StP,
                [META_UNIT_A1] = TCG_REG_A1GbP,
                [META_UNIT_PC] = TCG_REG_PC, };

static inline int tcg_reg_to_r(TCGReg reg)
{
    return metag_reg_info[reg].r;
}

/* find the meta unit a register belongs to */
static inline MetaUnit tcg_reg_to_meta_unit(TCGReg reg)
{
    return metag_reg_info[reg].unit;
}

static inline unsigned int tcg_reg_to_bu(TCGReg reg)
{
    assert(metag_reg_info[reg].bu < 0x4);
    return metag_reg_info[reg].bu;
}

static inline unsigned int is_in_data_unit(TCGReg reg)
{
    MetaUnit unit = metag_reg_info[reg].unit;
    return (unit == META_UNIT_D0 || unit == META_UNIT_D1);
}

static inline unsigned int is_in_addr_unit(TCGReg reg)
{
    MetaUnit unit = metag_reg_info[reg].unit;
    return (unit == META_UNIT_A0 || unit == META_UNIT_A1);
}

static inline unsigned int is_in_data_or_addr_unit(TCGReg reg)
{
    return is_in_data_unit(reg) || is_in_addr_unit(reg);
}

/* returns 1 if registers are paired up adequately for doing long
 * operations on them. (Same register number, opposite unit) */
static inline unsigned int are_registers_partnered(TCGReg r1, TCGReg r2)
{
    return ((tcg_reg_to_r(r1) == tcg_reg_to_r(r2)) &&
            ((tcg_reg_to_meta_unit(r1) ==
                            meta_unit_partner(tcg_reg_to_meta_unit(r2)))));
}

/* encodes top bits of rs2 for o2r */
static int tcg_o2r_map_du0[] = {
                [META_UNIT_A1] = 0x0 << 3,
                [META_UNIT_D1] = 0x1 << 3,
                [META_UNIT_A0] = 0x3 << 3, };
static int tcg_o2r_map_du1[] = {
                [META_UNIT_A1] = 0x0 << 3,
                [META_UNIT_D0] = 0x1 << 3,
                [META_UNIT_A0] = 0x3 << 3, };
static int tcg_o2r_map_au0[] = {
                [META_UNIT_A1] = 0x0 << 3,
                [META_UNIT_D0] = 0x1 << 3,
                [META_UNIT_D1] = 0x3 << 3, };
static int tcg_o2r_map_au1[] = {
                [META_UNIT_D1] = 0x0 << 3,
                [META_UNIT_D0] = 0x1 << 3,
                [META_UNIT_A0] = 0x3 << 3, };

static const uint8_t tcg_cond_to_metag_cond[] = {
                /* non-signed */
                [TCG_COND_NEVER]    = COND_NV,
                [TCG_COND_ALWAYS]   = COND_A,
                [TCG_COND_EQ]       = COND_EQ,
                [TCG_COND_NE]       = COND_NE,
                /* signed */
                [TCG_COND_LT]       = COND_LT,
                [TCG_COND_GE]       = COND_GE,
                [TCG_COND_LE]       = COND_LE,
                [TCG_COND_GT]       = COND_GT,
                /* unsigned */
                [TCG_COND_LTU]      = COND_LO,
                [TCG_COND_GEU]      = COND_HS,
                [TCG_COND_LEU]      = COND_LS,
                [TCG_COND_GTU]      = COND_HI, };

static const uint8_t inv_metag_cond[] = {
                [COND_A] = COND_NV,
                [COND_EQ] = COND_NE,
                [COND_NE] = COND_EQ,
                [COND_CS] = COND_CC,
                [COND_CC] = COND_CS,
                [COND_MI] = COND_PL,
                [COND_PL] = COND_MI,
                [COND_VS] = COND_VC,
                [COND_VC] = COND_VS,
                [COND_HI] = COND_LS,
                [COND_LS] = COND_HI,
                [COND_GE] = COND_LT,
                [COND_LT] = COND_GE,
                [COND_GT] = COND_LE,
                [COND_LE] = COND_GT,
                [COND_NV] = COND_A, };

static inline void reloc_rel(void *code_ptr, tcg_target_long target)
{
    tcg_target_long reloc_dst = (target - (tcg_target_long) code_ptr) >> 2;

    /* Fail if we are not on a br instruction */
    assert((*(tcg_target_ulong *) code_ptr >> 24) == 0xa0);

    /* 19 bit signed offset */
    assert(reloc_dst >= -0x40000 && reloc_dst < 0x40000);
    reloc_dst &= ((1 << 19) - 1);

    *(tcg_target_ulong *) code_ptr |= (reloc_dst << 5);
}

static void patch_reloc(uint8_t *code_ptr, int type, tcg_target_long value,
                tcg_target_long addend)
{
    value += addend;

    switch (type) {
    case (R_METAG_RELBRANCH):
        reloc_rel(code_ptr, value);
        break;
    default:
        UNIMPLEMENTED("type=%d", type);
    }
}

static int target_parse_constraint(TCGArgConstraint *ct, const char **pct_str)
{
    const char *ct_str;

    ct_str = *pct_str;
    /* Cases are defined towards the bottom of this file */
    switch (ct_str[0]) {
    case 'r': /* [D|A]*.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_D0],
                        (1ull << TCG_REG_PC) - 1);
        break;

    case 't': /* A*.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_A0],
                        (1 << 16) - 1);
        break;

    case 's': /* D*.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_D0],
                        (1ull << 32) - 1);
        break;

    case 'b': /* A0.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_A0],
                        (1 << 8) - 1);
        break;

    case 'c': /* A1.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_A1],
                        (1 << 8) - 1);
        break;

    case 'd': /* D0.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_D0],
                        (1 << 16) - 1);
        break;

    case 'e': /* D1.* */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, meta_unit_base[META_UNIT_D1],
                        (1 << 16) - 1);
        break;

    case 'z': /* D0Re0 */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, TCG_REG_D0Re0, 1);
        break;

    case 'y': /* D1Re0 */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, TCG_REG_D1Re0, 1);
        break;

    case 'u': /* D0Ar2 */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, TCG_REG_D0Ar2, 1);
        break;

    case 'v': /* D1Ar1 */
        ct->ct |= TCG_CT_REG;
        tcg_regset_set_bits(ct->u.regs, TCG_REG_D1Ar1, 1);
        break;

    case 'h': /* 5 bit imm */
        ct->ct |= TCG_CT_META_5_CONST;
        break;

    case 'g': /* 6 bit imm */
        ct->ct |= TCG_CT_META_6_CONST;
        break;

    case 'f': /* 8 bit imm */
        ct->ct |= TCG_CT_META_8_CONST;
        break;

    case 'j': /* 16 bit imm */
        ct->ct |= TCG_CT_META_16_CONST;
        break;

    case 'm': /* has mask bit */
        ct->ct |= TCG_CT_META_CONST_HAS_M;
        break;

        /* No DxArY registers */
    case 'q':
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar1);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Ar2);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar3);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Ar4);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar5);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Ar6);
        break;

        /* No DxRe0 registers */
    case 'p':
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Re0);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Re0);
        break;

        /* qemu ld/st - dont use certain DxAry regs */
    case 'w':
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar1);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Ar2);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar3);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D1Ar5);
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_D0Ar6);
        break;

    default:
        UNIMPLEMENTED("ct_str[0]=%c", ct_str[0]);
        return -1;
    }

    ct_str++;
    *pct_str = ct_str;

    return 0;
}

/* generate o2r encoding (rs2 and o2r fields) */
static inline uint32_t tcg_encode_o2r(MetaUnit us1, MetaUnit us2, int rs2,
                int map[])
{
    /* can avoid o2r altogether if units match */
    if (us1 == us2) {
        return rs2 << 9; /* normal rs2 with no o2r bit */
    }
    /* register number must fit in 3 bits for o2r to be used */
    assert(rs2 <= 0x7);

    return (map[us2] | rs2) << 9 | /* o2r unit and register */
                           1 << 0; /* o2r bit */
}

/* encode 16 bit immediate with M, SE, and H flags
 * returns 1 if it imm can be encoded, 0 otherwise */
static int tcg_encode_imm16(tcg_target_ulong imm, int have_m,
                uint32_t *encoded_imm)
{
    int m = 0, se = 0, h = 0, i16;

    if (imm <= 0xffff) {
        /* it fits in bottom 16 bits */
        i16 = imm;
        goto out;
    }

    if (!(imm & 0xffff)) {
        /* it fits in top 16 bits */
        h = 1;
        i16 = imm >> 16;
        goto out;
    }

    if ((imm & 0xffff8000) == 0xffff8000) {
        /* it fits in bottom 16 bits with sign extension */
        se = 1;
        i16 = imm & 0xffff;
        goto out;
    }

    if (have_m) {
        if ((imm & 0xffff0000) == 0xffff0000) {
            /* it fits in bottom 16 bits as mask */
            m = 1;
            i16 = imm & 0xffff;
            goto out;
        }
        if ((imm & 0xffff) == 0xffff) {
            /* it fits in top 16 bits with mask */
            m = 1;
            h = 1;
            i16 = imm >> 16;
            goto out;
        }
    }

    return 0;

out:
    if (encoded_imm) {
        *encoded_imm = i16 << 3 | m << 2 | se << 1 | h;
    }

    return 1;
}

/*
 * Test if a constant matches the constraint.
 */
static inline int tcg_target_const_match(tcg_target_long val,
                const TCGArgConstraint *arg_ct)
{
    int ct = arg_ct->ct;

    if (ct & TCG_CT_CONST) {
        return 1;
    }
    if ((ct & TCG_CT_META_16_CONST) && tcg_encode_imm16(val, ct
                    & TCG_CT_META_CONST_HAS_M, NULL)) {
        return 1;
    }
    if ((ct & TCG_CT_META_5_CONST) && (unsigned int) val <= 0x1f) {
        return 1;
    }
    if ((ct & TCG_CT_META_6_CONST) && (unsigned int) val <= 0x3f) {
        return 1;
    }
    if ((ct & TCG_CT_META_8_CONST) && (unsigned int) val <= 0xff) {
        return 1;
    }

    return 0;
}

/*
 * Base will always be AREG0 or frame_reg (which currently == AREG0)
 * Hopefully this always uses the faster 6 or 12 bit getset imm
 */
static inline void tcg_out_ld(TCGContext *s, TCGType type, TCGReg dst,
                TCGReg base, tcg_target_long offset)
{
    assert(type == TCG_TYPE_I32);
    tcg_out_getdi(s, dst, base, offset);
}

/*
 * (see tcg_out_ld comment)
 */
static inline void tcg_out_st(TCGContext *s, TCGType type, TCGReg src,
                TCGReg base, tcg_target_long offset)
{
    assert(type == TCG_TYPE_I32);
    tcg_out_setdi(s, src, base, offset);
}

static void tcg_out_multiply(TCGContext *s, int l, TCGReg dst, TCGReg src1,
                TCGReg src2)
{
    MetaUnit ud, us1, us2;
    int rd, rs1, rs2, du, o2r;
    uint32_t ca = 0;

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);
    us2 = tcg_reg_to_meta_unit(src2);
    du = (us1 == META_UNIT_D1);

    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);
    rs2 = tcg_reg_to_r(src2);

    /* Only data unit registers can be multiplied */
    assert(is_in_data_unit(src1));
    /* 32bit multiply can only target D or A regs */
    assert(!l || is_in_data_or_addr_unit(dst));

    o2r = tcg_encode_o2r(us1, us2, rs2, (du == 0) ? tcg_o2r_map_du0
                    : tcg_o2r_map_du1);

    if (ud != us1) {
        ca = 1 << 26 |   /* conditional */
             1 << 5  |   /* condition always */
             ud << 1;    /* destination unit */
    }

    /* if l is set, the op must be unconditional
     * since we never set C this is ok */
    tcg_out32(s, (0x60000000 |
                    du << 24 | /* first/second data unit */
                    rd << 19 | /* rd */
                    rs1 << 14 | /* rs1 */
                    l << 6 | /* 1 = 32bit multiply */
                    ca |    /* condition or ud */
                    o2r)); /* o2r */
}

static void tcg_out_multiplyi(TCGContext *s, int l, TCGReg dst, TCGReg src1,
                tcg_target_long imm)
{
    MetaUnit ud, us1;
    int rd, rs1, du;
    uint32_t i;

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);
    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);

    if (tcg_encode_imm16(imm, 0, &i)) {
        tcg_out_mov(s, TCG_TYPE_REG, dst, src1);

        /* Only data unit registers can be multiplied */
        if (!is_in_data_unit(dst)) {
            assert(dst != TCG_REG_D0FrT && src1 != TCG_REG_D0FrT);
            tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D0FrT, imm);
            tcg_out_multiply(s, l, dst, src1, TCG_REG_D0FrT);
            return;
        }

        du = (ud == META_UNIT_D1);

        tcg_out32(s, (0x60000000 |
                        1 << 25 | /* immediate */
                        du << 24 | /* dst unit */
                        rd << 19 | /* rd */
                        l << 2 | /* 16/32 bit multiply */
                        i)); /* encoded imm */
    } else {
        /* 8 bit unsigned imm */
        assert((tcg_target_ulong)imm <= 0xff);

        du = (us1 == META_UNIT_D1);

        /* Only data unit registers can be multiplied */
        assert(is_in_data_unit(src1));
        /* 32bit multiply can only target D or A regs */
        assert(!l || is_in_data_or_addr_unit(dst));

        /* if l is set, the op must be unconditional
         * since we never set C this is ok */
        tcg_out32(s, (0x60000000 |
                        1 << 26 |  /* conditional */
                        1 << 25 | /* immediate */
                        du << 24 | /* first/second unit */
                        rd << 19 | /* rd */
                        rs1 << 14 | /* rs1 */
                        imm << 6 | /* imm8 */
                        1 << 5 |   /* condition always (must be 1 if l==1) */
                        ud << 1 | /* destination unit */
                        l)); /* 16/32 bit multiply */
    }
}

static void tcg_out_shift(TCGContext *s, int setc, int arith, int right,
                TCGReg dst, TCGReg src1, TCGReg src2, MetaCond cc)
{
    MetaUnit ud, us1, us2;
    int rd, du, rs1, rs2;
    uint32_t ca = 0;

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);
    us2 = tcg_reg_to_meta_unit(src2);

    /* only data registers can be shifted */
    assert(is_in_data_unit(src1));

    /* us1 and src2 must come from the same unit */
    if (us1 != us2) {
        if (us1 == META_UNIT_D0) {
            /* src2 must be D1 */
            assert(src1 != TCG_REG_D0FrT && dst != TCG_REG_D0FrT);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, src2);
            src2 = TCG_REG_D0FrT;
            us2 = tcg_reg_to_meta_unit(src2);
        } else {
            /* src2 must be D0 */
            assert(src2 != TCG_REG_D0FrT && dst != TCG_REG_D0FrT);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, src1);
            src1 = TCG_REG_D0FrT;
            us1 = tcg_reg_to_meta_unit(src1);
        }
    }

    /* true if src1 is in unit D1 */
    du = (us1 == META_UNIT_D1);

    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);
    rs2 = tcg_reg_to_r(src2);

    if (cc != COND_A) {
        assert(ud == us1);
        ca = 1 << 26 | /* conditional */
             cc << 1;  /* condition */
    } else if (ud != us1) {
        ca = 1 << 26 | /* conditional */
             1 << 5 |  /* condition always */
             ud << 1;  /* destination unit */
    }

    tcg_out32(s, (0x50000000 | /* shift opcode */
                    setc << 27 | /* set thread cond flags */
                    du << 24 | /* unit */
                    rd << 19 | /* rd */
                    rs1 << 14 | /* rs1 */
                    rs2 << 9 | /* rs2 */
                    arith << 7 | /* arithmetic shift */
                    right << 6 | /* shift right */
                    ca)); /* condition and ud */
}

static void tcg_out_shifti(TCGContext *s, int setc, int arith, int right,
                TCGReg dst, TCGReg src1, tcg_target_ulong imm, MetaCond cc)
{
    MetaUnit ud, us1;
    int rd, rs, du;
    uint32_t ca = 0;

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);
    du = (us1 == META_UNIT_D1);

    /* only data registers can be shifted */
    assert(is_in_data_unit(src1));
    assert(imm <= 0x1f);

    rd = tcg_reg_to_r(dst);
    rs = tcg_reg_to_r(src1);

    if (cc != COND_A) {
        assert(ud == us1);
        ca = 1 << 26 | /* conditional */
             cc << 1;  /* condition */
    } else if (ud != us1) {
        ca = 1 << 26 | /* conditional */
             1 << 5 |  /* condition always */
             ud << 1;  /* destination unit */
    }

    tcg_out32(s, (0x50000000 | /* shift opcode */
                    setc << 27 | /* set thread cond flags */
                    1 << 25 | /* immediate */
                    du << 24 | /* unit */
                    rd << 19 | /* rd */
                    rs << 14 | /* rs */
                    imm << 9 | /* shift specifier */
                    arith << 7 | /* arithmetic shift */
                    right << 6 | /* shift right */
                    ca));   /* ca */
}

static void tcg_out_logop(TCGContext *s, int setc, TCGMetaLogOp op,
                TCGReg dst, TCGReg src1, TCGReg src2, MetaCond cc)
{
    MetaUnit ud, us1, us2;
    int rd, rs1, rs2, du, o2r;
    uint32_t ca = 0;

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);
    us2 = tcg_reg_to_meta_unit(src2);
    du = (us1 == META_UNIT_D1);

    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);
    rs2 = tcg_reg_to_r(src2);

    /* logical operations only available in data units */
    if (!is_in_data_unit(src1)) {
        /* Use o2r if we can */
        TCGReg tmp_r;
        MetaUnit tmp_u;

        /* swap src1 and src2 in hope we can use the faster op later */
        tmp_r = src1;
        src1 = src2;
        src2 = tmp_r;

        tmp_u = us1;
        us1 = us2;
        us2 = tmp_u;
    }
    assert(is_in_data_unit(src1));

    o2r = tcg_encode_o2r(us1, us2, rs2, (du == 0) ? tcg_o2r_map_du0
                    : tcg_o2r_map_du1);

    if (cc != COND_A || ud != us1) {
        ca = 1 << 26 | /* conditional */
             ud << 5 | /* destination unit */
             cc << 1;  /* condition */
    }

    /* use within-unit version */
    tcg_out32(s, (op << 28 | /* logical operation */
                    setc << 27 | /* set thread cond flags */
                    du << 24 | /* first/second data unit */
                    rd << 19 | /* rd */
                    rs1 << 14 | /* rs1 */
                    ca |    /* conditional and du */
                    o2r)); /* o2r */
}

static void tcg_out_logopi(TCGContext *s, int setc, TCGMetaLogOp op,
                TCGReg dst, TCGReg src1, tcg_target_long imm, MetaCond cc)
{
    MetaUnit ud, us1;
    int rd, rs1, du;
    uint32_t i;

    /* logical operations only available in data units */
    assert(is_in_data_unit(src1));

    ud = tcg_reg_to_meta_unit(dst);
    us1 = tcg_reg_to_meta_unit(src1);

    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);
    du = (us1 == META_UNIT_D1);

    if ((unsigned int) imm <= 0xff) {
        uint32_t ca = 0;

        if (cc != COND_A) {
            assert(ud == us1);
            ca = cc << 1;  /* condition */
        } else {
            ca = 1 << 5 |  /* condition always */
                 ud << 1;  /* destination unit */
        }

        /* 8 bit version */
        tcg_out32(s, (op << 28 | /* logical operation */
                        setc << 27 | /* set thread cond flags */
                        1 << 26 | /* conditional */
                        1 << 25 | /* immediate */
                        du << 24 | /* first/second unit */
                        rd << 19 | /* rd */
                        rs1 << 14 | /* rs1 */
                        imm << 6 | /* imm8 */
                        ca)); /* cc */
    } else if (cc != COND_A || !tcg_encode_imm16(imm, 1, &i) ||
               (ud != META_UNIT_D0 && ud != META_UNIT_D1)) {
        assert(dst != TCG_REG_D0FrT && src1 != TCG_REG_D0FrT);
        tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D0FrT, imm);
        tcg_out_logop(s, setc, op, dst, src1, TCG_REG_D0FrT, cc);
        return;
    } else {
        tcg_out_mov(s, TCG_TYPE_REG, dst, src1);

        du = (ud == META_UNIT_D1);

        tcg_out32(s, (op << 28 | /* logical operation */
                        setc << 27 | /* set thread cond flags */
                        1 << 25 | /* immediate */
                        du << 24 | /* dst unit */
                        rd << 19 | /* rd */
                        i)); /* encoded imm */
    }
}

static void tcg_out_addsub(TCGContext *s, int setc, int sub, int o1z,
                TCGReg dst, TCGReg src1, TCGReg src2, MetaCond cc)
{
    MetaUnit ud, us1, us2;
    int rd, rs1, rs2, a, u, o2r;

    ud = tcg_reg_to_meta_unit(dst);
    if (o1z) {
        assert(is_in_data_or_addr_unit(dst));
        /* TODO this is a toss up between O2R and ud */
        /* keep the MOV in unit */
        src1 = dst;
    }
    us1 = tcg_reg_to_meta_unit(src1);
    us2 = tcg_reg_to_meta_unit(src2);

    if (!o1z && !sub && ud != us1 && ud == us2) {
        TCGReg tmp_r;
        MetaUnit tmp_u;

        /* swap src1 and src2 in hope we can use the faster op later */
        tmp_r = src1;
        src1 = src2;
        src2 = tmp_r;

        tmp_u = us1;
        us1 = us2;
        us2 = tmp_u;
    }

    a = (is_in_addr_unit(src1));

    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);
    rs2 = tcg_reg_to_r(src2);

    /* Cannot be A unit ALU op that sets flags */
    assert(!a || !setc);

    if (a) {
        u = (us1 == META_UNIT_A1);
        o2r = tcg_encode_o2r(us1, us2, rs2, (u == 0) ? tcg_o2r_map_au0
                        : tcg_o2r_map_au1);
    } else {
        u = (us1 == META_UNIT_D1);
        o2r = tcg_encode_o2r(us1, us2, rs2, (u == 0) ? tcg_o2r_map_du0
                        : tcg_o2r_map_du1);
    }

    if (o1z) {
        assert(cc == COND_A);
        tcg_out32(s, (a << 31 | /* data/address unit */
                     (sub << 28) >> a | /* sub (bit 28 or 27) */
                     u << 24 | /* first/second unit */
                     rd << 19 | /* rd */
                     1 << 2 | /* o1z */
                     o2r)); /* o2r */
    } else {
        uint32_t ca = 0;
        if (cc != COND_A || ud != us1) {
            ca = 1 << 26 | /* conditional */
                 ud << 5 | /* destination unit */
                 cc << 1;  /* condition */
        }

        tcg_out32(s, (a << 31 | /* data/address unit */
                     (sub << 28) >> a | /* sub (bit 28 or 27) */
                     setc << 27 |    /* set thread condition flags */
                     u << 24 | /* first/second unit */
                     rd << 19 | /* rd */
                     rs1 << 14 | /* rs1 */
                     ca |   /* condional and du */
                     o2r)); /* ca and o2r bit */
    }
}

static void tcg_out_addsubi_16(TCGContext *s, int setc, int sub, int o1z,
                TCGReg dst, TCGReg src1, tcg_target_long encoded_imm)
{
    MetaUnit ud, us1;
    int rd, rs1, a, u;

    ud = tcg_reg_to_meta_unit(dst);
    if (o1z) {
        assert(is_in_data_or_addr_unit(dst));
        src1 = dst;
    }
    us1 = tcg_reg_to_meta_unit(src1);
    rd = tcg_reg_to_r(dst);
    rs1 = tcg_reg_to_r(src1);

    a = (is_in_addr_unit(dst));
    u = (us1 == (a ? META_UNIT_A1 : META_UNIT_D1));

    if (o1z) {
        /* 16bit immediate version */
        tcg_out32(s, (a << 31 | /* data/address unit */
                    (sub << 28) >> a | /* sub (bit 28 or 27) */
                    1 << 25 | /* immediate */
                    u << 24 | /* first/second unit */
                    rd << 19 | /* rd */
                    1 << 2 | /* o1z */
                    encoded_imm)); /* imm16, se, h fields */
    } else {
        /* move src into dst */
        tcg_out_mov(s, TCG_TYPE_REG, dst, src1);

        u = (ud == (a ? META_UNIT_A1 : META_UNIT_D1));

        /* then add to it */
        tcg_out32(s, (a << 31 | /* data/address unit */
                    (sub << 28) >> a | /* sub (bit 28 or 27) */
                    setc << 27 |    /* set thread condition flags */
                    1 << 25 | /* immediate */
                    u << 24 | /* first/second unit */
                    rd << 19 | /* rd */
                    o1z << 2 | /* o1z */
                    encoded_imm)); /* imm16, se, h fields */
    }
}

/* ADD/SUB dst,src,#imm */
static void tcg_out_addsubi(TCGContext *s, int setc, int sub, int o1z,
                TCGReg dst, TCGReg src1, tcg_target_long imm, MetaCond cc)
{
    MetaUnit ud, us1;
    int rd, rs1, a, u;
    uint32_t i, ca = 0;

    if ((unsigned int)imm > 0xff || o1z) {
        if (!tcg_encode_imm16(imm, 0, &i)) {
            UNIMPLEMENTED("imm=%d", imm);
        }

        assert(cc == COND_A);

        tcg_out_addsubi_16(s, setc, sub, o1z, dst, src1, i);
    } else {
        ud = tcg_reg_to_meta_unit(dst);
        us1 = tcg_reg_to_meta_unit(src1);
        rd = tcg_reg_to_r(dst);
        rs1 = tcg_reg_to_r(src1);

        a = (is_in_addr_unit(src1));
        u = (us1 == (a ? META_UNIT_A1 : META_UNIT_D1));

        /* Cannot be A unit ALU op that sets flags */
        assert(!a || !setc);

        assert(is_in_data_or_addr_unit(src1));

        /* if destination is in a different unit, use the CA field */
        if (cc != COND_A) {
            assert(ud == us1);
            ca = cc << 1;  /* condition */
        } else {
            ca = 1 << 5 |  /* condition always */
                 ud << 1;  /* destination unit */
        }

        /* 8bit immediate version */
        tcg_out32(s, (a << 31 | /* data/address unit */
                    (sub << 28) >> a | /* sub (bit 28 or 27) */
                    setc << 27 |    /* set thread condition flags */
                    1 << 26 | /* conditional */
                    1 << 25 | /* immediate */
                    u << 24 | /* first/second unit */
                    rd << 19 | /* rd */
                    rs1 << 14 | /* rs1 */
                    imm << 6 | /* imm8 */
                    ca)); /* ca */
    }
}

static void tcg_out_cmptst(TCGContext *s, TCGMetaCmpOp op, MetaCond cc,
                TCGReg src1, TCGReg src2)
{
    MetaUnit us1, us2;
    int rs1, rs2, du, o2r;

    if (is_in_addr_unit(src1)) {
        TCGReg tmp;

        assert(is_in_data_unit(src2));

        /* swap registers around so we can do the compare/test with o2r */
        tmp = src1;
        src1 = src2;
        src2 = tmp;
    }

    us1 = tcg_reg_to_meta_unit(src1);
    us2 = tcg_reg_to_meta_unit(src2);
    rs1 = tcg_reg_to_r(src1);
    rs2 = tcg_reg_to_r(src2);
    du = (us1 == META_UNIT_D1);

    /* operation only available in data units */
    assert(is_in_data_unit(src1));

    o2r = tcg_encode_o2r(us1, us2, rs2, (du == 0) ? tcg_o2r_map_du0
                    : tcg_o2r_map_du1);

    tcg_out32(s, (0x70000000 |
                    op << 27 | /* compare operation */
                    (cc != COND_A) << 26 | /* conditional */
                    du << 24 | /* first/second data unit */
                    rs1 << 14 | /* rs1 */
                    cc << 1 | /* cc */
                    o2r)); /* rs2 and o2r bit */
}

static void tcg_out_cmptsti(TCGContext *s, TCGMetaCmpOp op, MetaCond cc,
                TCGReg src1, tcg_target_long imm)
{
    MetaUnit us1;
    int rs1, du;
    uint32_t i;

    us1 = tcg_reg_to_meta_unit(src1);
    rs1 = tcg_reg_to_r(src1);
    du = (us1 == META_UNIT_D1);

    /* operation only available in data units */
    assert(is_in_data_unit(src1));

    if (cc == COND_A) {
        /* 16 bit immediate unconditional */
        if (!tcg_encode_imm16(imm, 1, &i)) {
            UNIMPLEMENTED("imm=%d", imm);
        }

        tcg_out32(s, (0x70000000 |
                        op << 27 | /* compare operation */
                        1 << 25 | /* immediate */
                        du << 24 | /* first/second unit */
                        rs1 << 19 | /* rs */
                        i)); /* imm16, m, se, h fields */
    } else {
        /* 8 bit unsigned(?) conditional imm */
        assert((tcg_target_ulong)imm <= 0xff);

        tcg_out32(s, (0x70000000 |
                        op << 27 | /* compare operation */
                        1 << 26 | /* conditional */
                        1 << 25 | /* immediate */
                        du << 24 | /* first/second unit */
                        rs1 << 14 | /* rs */
                        imm << 6 | /* imm8 */
                        cc << 1)); /* cc */
    }
}

/* conditional cross-unit MOV */
static void tcg_out_movcc(TCGContext *s, MetaCond cc, TCGReg dst, TCGReg src,
                int swap)
{
    MetaUnit ud, us;
    int rd, rs;

    if (dst == src) {
        /* NOP */
        return;
    }

    assert(!swap || tcg_reg_to_meta_unit(dst) != tcg_reg_to_meta_unit(src));

    ud = tcg_reg_to_meta_unit(dst);
    us = tcg_reg_to_meta_unit(src);
    rd = tcg_reg_to_r(dst);
    rs = tcg_reg_to_r(src);

    tcg_out32(s, (0xa3000000 |
                    rs << 19 |  /* source register */
                    rd << 14 |  /* destination register */
                    us << 10 |  /* source unit */
                    swap << 9 | /* SWAP or movcc */
                    ud << 5 |   /* destination unit */
                    cc << 1));  /* condition */
}

static void tcg_out_mov(TCGContext *s, TCGType type, TCGReg dst, TCGReg src)
{
    if (dst == src) {
        /* NOP */
        return;
    }

    /* TCG_TYPE_REG is aliased as I32 for metag */
    assert(type == TCG_TYPE_I32);

    /* TODO pick the best one to avoid pipeline stalls (depending on previous
     * instructions)
     * O2R: reads previous instruction
     * MOVcc/CA: writes next instruction
     */
    if (is_in_data_or_addr_unit(dst) && is_in_data_or_addr_unit(src)) {
        /* O2R */
        tcg_out_addz(s, dst, src);
    } else {
        /* MOVcc */
        tcg_out_movcc(s, COND_A, dst, src, 0);
    }
}

static void tcg_out_movi(TCGContext *s, TCGType type, TCGReg dst,
                tcg_target_long imm)
{
    assert(type == TCG_TYPE_I32);

    if (is_in_data_or_addr_unit(dst)) {
        if (tcg_encode_imm16(imm, 0, NULL)) {
            /* MOV dst,#arg */
            tcg_out_addzi(s, dst, imm);
        } else {
            /* MOVT dst,#HI(arg) */
            tcg_out_addzi(s, dst, imm & 0xffff0000);
            if (imm & 0xffff) {
                /* ADD dst,dst,#LO(arg) */
                tcg_out_addi(s, COND_A, 0, dst, dst, imm & 0xffff);
            }
        }
    } else {
        UNIMPLEMENTED("type=%d, dst=%d, imm=%d", type, dst, imm);
    }
}

static void tcg_out_neg(TCGContext *s, TCGReg dst, TCGReg src)
{
    MetaUnit ud, us;

    /* pick a type of MOV */
    ud = tcg_reg_to_meta_unit(dst);
    us = tcg_reg_to_meta_unit(src);

    if (is_in_data_or_addr_unit(dst) && is_in_data_or_addr_unit(src)) {
        /* O2R */
        tcg_out_subz(s, dst, src);
    } else {
        UNIMPLEMENTED("dst=%d, src=%d", dst, src);
    }
}

static void tcg_out_negi(TCGContext *s, TCGReg dst, tcg_target_long imm)
{
    if (is_in_data_or_addr_unit(dst)) {
        if (tcg_encode_imm16(imm, 0, NULL)) {
            /* MOV dst,#arg */
            tcg_out_subzi(s, dst, imm);
        } else {
            /* MOVT dst,#HI(arg) */
            tcg_out_subzi(s, dst, imm & 0xffff0000);
            if (imm & 0xffff) {
                /* ADD dst,dst,#LO(arg) */
                tcg_out_subi(s, COND_A, 0, dst, dst, imm & 0xffff);
            }
        }
    } else {
        UNIMPLEMENTED("dst=%d, imm=%d", dst, imm);
    }
}

static void tcg_out_exts(TCGContext *s, int w, TCGReg dst, TCGReg src)
{
    int rd, rs, du;

    assert(is_in_data_unit(dst) && is_in_data_unit(src));
    if (tcg_reg_to_meta_unit(dst) != tcg_reg_to_meta_unit(src))
    {
        /* If registers are not of the same unit do a mov first.
         * Our current constraints allow this to happen */
        tcg_out_mov(s, TCG_TYPE_REG, dst, src);
        src = dst;
    }

    rd = tcg_reg_to_r(dst);
    rs = tcg_reg_to_r(src);

    du = (tcg_reg_to_meta_unit(dst) == META_UNIT_D1);

    tcg_out32(s, (0xaa000000 | /* XSDB/W */
                  rd << 19 | /* rd */
                  rs << 14 | /* rs */
                  w << 1 | /* 16bit (1) or 8bit (0) se */
                  du)); /* DU */
}

static void tcg_out_getset(TCGContext *s, int set, int l, int ua, int pp,
                int se, TCGReg src, TCGReg base, TCGReg offset)
{
    uint32_t op;
    MetaUnit us, ub, uo;
    int bu, rs, rb, ro;

    assert(is_in_data_or_addr_unit(base));

    if (set && l == 3) {
        /* Special case for SETL, handle it exclusively */
        if ((is_in_data_unit(src) && is_in_data_unit(base)) ||
            (is_in_addr_unit(src) && is_in_addr_unit(base))) {
            /* src unit(s) can match base unit but offset must be 0 */

            assert(base != TCG_REG_D1_5 && src != TCG_REG_D1_5);
            tcg_out_add(s, COND_A, 0, TCG_REG_D1_5, base, offset);

            tcg_out_getseti(s, 1, 3, ua, pp, se, src, TCG_REG_D1_5, 0);
            return;
        }
    }

    us = tcg_reg_to_meta_unit(src);
    ub = tcg_reg_to_meta_unit(base);
    uo = tcg_reg_to_meta_unit(offset);

    if (ub != uo) {
        /*
         * Make the base and offset register units match
         */
        switch (ub) {
        case (META_UNIT_D1):
            assert(base != TCG_REG_D1_5 && src != TCG_REG_D1_5);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D1_5, offset);
            offset = TCG_REG_D1_5;
            break;
        case (META_UNIT_D0):
            assert(base != TCG_REG_D0FrT && src != TCG_REG_D0FrT);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, offset);
            offset = TCG_REG_D0FrT;
            break;
        default:
            UNIMPLEMENTED("ub=%d\n", ub);
        }
        uo = tcg_reg_to_meta_unit(offset);
    }

    /*
     * When setting, src unit (us) cannot be the same as base unit (ub)
     */
    if (set && us == ub) {
        /* so try and use our other spare reg */
        switch (ub) {
        case (META_UNIT_D1):
            assert(base != TCG_REG_D0FrT && offset != TCG_REG_D0FrT);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, src);
            src = TCG_REG_D0FrT;
            break;
        case (META_UNIT_D0):
            assert(base != TCG_REG_D1_5 && offset != TCG_REG_D1_5);
            tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D1_5, src);
            src = TCG_REG_D1_5;
            break;
        default:
            UNIMPLEMENTED("ub=%d\n", ub);
        }
        us = tcg_reg_to_meta_unit(src);
    }

    assert(ub == uo);
    assert(!set || us != ub);

    bu = tcg_reg_to_bu(base);
    rs = tcg_reg_to_r(src);
    rb = tcg_reg_to_r(base);
    ro = tcg_reg_to_r(offset);

    op = set ? 0xb0000000 : 0xc0000000;
    tcg_out32(s, (op | /* set/get */
                (l & 0x2) << 25 | /* l2 */
                (l & 0x1) << 24 | /* l1 */
                rs << 19 | /* rs/rd */
                rb << 14 | /* rb */
                ro << 9 | /* ro */
                ua << 7 | /* update address */
                bu << 5 | /* base unit */
                us << 1 | /* src/dst unit */
                pp)); /* increment/decrement */

    /* Note: Conditional set exists with 0xA4 major */

    /* If we are getting and we want to sign-extend */
    if (!set && se) {
        switch (l) {
        case (0):
            if (!is_in_data_unit(src)) {
                tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, src);
                tcg_out_ext8s(s, COND_A, TCG_REG_D0FrT, TCG_REG_D0FrT);
                tcg_out_mov(s, TCG_TYPE_REG, src, TCG_REG_D0FrT);
            } else {
                tcg_out_ext8s(s, COND_A, src, src);
            }
            break;
        case (1):
            if (!is_in_data_unit(src)) {
                tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, src);
                tcg_out_ext16s(s, COND_A, TCG_REG_D0FrT, TCG_REG_D0FrT);
                tcg_out_mov(s, TCG_TYPE_REG, src, TCG_REG_D0FrT);
            } else {
                tcg_out_ext16s(s, COND_A, src, src);
            }
            break;
        case (2):
        case (3):
            break;
        default:
            UNIMPLEMENTED("l=%d", l);
        }
    }
}

static void tcg_out_getseti(TCGContext *s, int set, int l, int ua, int pp,
                int se, TCGReg src, TCGReg base, tcg_target_long offset)
{
    uint8_t op;
    int imm;
    int bua, bus, rs, rb;
    MetaUnit us, ub;

    bua = tcg_reg_to_bu(base);
    rs = tcg_reg_to_r(src);
    rb = tcg_reg_to_r(base);

    ub = tcg_reg_to_meta_unit(base);
    us = tcg_reg_to_meta_unit(src);

    assert(is_in_data_or_addr_unit(base));
    /* when setting, ud is allowed to equal ub for immediate form */

    imm = offset >> l;

    /* prefer the 6 bit immediate version as it's more flexible */
    if (imm >= -0x20 && imm < 0x20) {
        imm &= ((1 << 6) - 1);
        op = set ? 0xb : 0xc;

        tcg_out32(s, (op << 28 | /* set/get */
                    (l & 0x2) << 25 | /* l2 */
                    1 << 25 | /* immediate */
                    (l & 0x1) << 24 | /* l1 */
                    rs << 19 | /* rs/rd */
                    rb << 14 | /* rb */
                    imm << 8 | /* imm */
                    ua << 7 | /* update address */
                    bua << 5 | /* base unit */
                    us << 1 | /* src/dst unit */
                    pp)); /* increment/decrement */
    } else {
        /* try 12 bit imm version */
        if (imm >= -0x800 && imm < 0x800 && pp == 0 && rb < 2) {
            imm &= ((1 << 12) - 1);
            bus = tcg_reg_to_bu(src);
            op = set ? 0xa5 : 0xa7;

            tcg_out32(s, (op << 24 | /* set/get */
                        rs << 19 | /* rs/rd */
                        imm << 7 | /* imm */
                        bua << 5 | /* base unit */
                        bus << 3 | /* src/dst unit */
                        (l & 0x3) << 1 | /* l */
                        (rb & 1))); /* base reg */
        } else {
            /*
             * We can't do a imm version, so generate a mov into
             * a reserved register thats chosen based on the unit
             * of the base register
             */
            switch (ub) {
            case (META_UNIT_D0):
                assert(base != TCG_REG_D0FrT && src != TCG_REG_D0FrT);
                tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D0FrT, offset);
                tcg_out_getset(s, set, l, ua, pp, se, src, base, TCG_REG_D0FrT);
                break;
            case (META_UNIT_D1):
                assert(base != TCG_REG_D1_5 && src != TCG_REG_D1_5);
                tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D1_5, offset);
                tcg_out_getset(s, set, l, ua, pp, se, src, base, TCG_REG_D1_5);
                break;
            default:
                UNIMPLEMENTED("ub=%d\n", ub);
            }
            return;
        }
    }

    /* If we are getting and we want to sign-extend */
    if (!set && se) {
        switch (l) {
        case (0):
            tcg_out_ext8s(s, COND_A, src, src);
            break;
        case (1):
            tcg_out_ext16s(s, COND_A, src, src);
            break;
        case (2):
        case (3):
            break;
        default:
            UNIMPLEMENTED("l=%d", l);
        }
    }
}

static void tcg_out_mgetl(TCGContext *s, TCGReg base, TCGReg first,
                unsigned int rmask)
{
    int rb = tcg_reg_to_r(base);
    int rd = tcg_reg_to_r(first);
    int bua = tcg_reg_to_bu(base);
    int bur = tcg_reg_to_bu(first);

    assert(((1 << base) & ((1 << first)|(rmask << (first+1)))) == 0);
    assert(tcg_reg_to_meta_unit(base) != tcg_reg_to_meta_unit(first));

    tcg_out32(s, (0xc9000000 |   /* mgetl */
                    rd << 19 |   /* first reg in group */
                    rb << 14 |   /* base reg */
                    rmask << 7 | /* mask defining which regs are in group */
                    bua << 5 |   /* BU of base reg */
                    bur << 3));  /* BU or first reg */
}

static void tcg_out_msetl(TCGContext *s, TCGReg base, TCGReg first,
                unsigned int rmask)
{
    int rb = tcg_reg_to_r(base);
    int rd = tcg_reg_to_r(first);
    int bua = tcg_reg_to_bu(base);
    int bur = tcg_reg_to_bu(first);

    assert(((1 << base) & ((1 << first)|(rmask << (first+1)))) == 0);

    tcg_out32(s, (0xb9000000 |   /* msetl */
                    rd << 19 |   /* first reg in group */
                    rb << 14 |   /* base reg */
                    rmask << 7 | /* mask defining which regs are in group */
                    bua << 5 |   /* BU of base reg */
                    bur << 3));  /* BU or first reg */
}

/* XXX If this ever generates more than 4 bytes tcg_out_goto_label
   needs fixing up (it assumes only 4 bytes are written) */
static void tcg_out_b(TCGContext *s, tcg_target_long reloc_dst, MetaCond cc,
                int loop)
{
    reloc_dst = reloc_dst >> 2;

    /* 19 bit signed offset */
    assert(reloc_dst >= -0x40000 && reloc_dst < 0x40000);

    reloc_dst &= ((1 << 19) - 1);

    tcg_out32(s, 0xa0000000 |        /* branch */
                    reloc_dst << 5 | /* pc relative addr */
                    cc << 1 |        /* condition */
                    loop);           /* 1 = loop */
}

static void tcg_out_jmpi(TCGContext *s, tcg_target_ulong relocd_dest)
{
    tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D1RtP, relocd_dest & 0xffff0000);
    tcg_out32(s, (0xac000000 |                          /* JMP */
                    tcg_reg_to_r(TCG_REG_D1RtP) << 19 | /* src */
                    (relocd_dest & 0x0000ffff) << 3 | /* lo immediate */
                    tcg_reg_to_bu(TCG_REG_D1RtP))); /* bu */
}

static inline int tcg_out_callr(TCGContext *s, TCGReg lr,
                                tcg_target_long abs_addr)
{
    int bu, rr;
    int32_t rel, reloc_dst;

    assert(is_in_data_or_addr_unit(lr));

    /* 19 bit signed offset */
    rel = (abs_addr - (tcg_target_long) s->code_ptr);
    if (rel < -0x100000 || rel >= 0x100000) {
        return 0;
    }
    reloc_dst = rel >> 2;
    reloc_dst &= ((1 << 19) - 1);

    bu = tcg_reg_to_bu(lr);
    rr = tcg_reg_to_r(lr);

    assert(rr < 8);

    tcg_out32(s, (0xab000000 |          /* pc relative call */
                    reloc_dst << 5 |    /* pc relative addr */
                    bu << 3 |           /* BU of reg */
                    rr));               /* reg num */

    return 1;
}


static void tcg_out_call(TCGContext *s, TCGReg addr)
{
    /* We dont have an instruction for Call to address in reg.
     * instead we move the destination into D1RtP (for return
     * prediction) then swap it with the PC */
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D1RtP, addr);
    tcg_out_swap(s, TCG_REG_PC, TCG_REG_D1RtP);
}

static void tcg_out_calli(TCGContext *s, tcg_target_ulong abs_addr)
{
    int rs, bu;

    /* can we do a CALLR with 19bit signed word offset? */
    if (!tcg_out_callr(s, TCG_REG_D1RtP, abs_addr)) {
        /* Else do an absolute call*/

        /* MOVT D1RtP, #HI(addr)
           CALL D1RtP, #LO(addr) */
        tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D1RtP, abs_addr & 0xffff0000);

        rs = tcg_reg_to_r(TCG_REG_D1RtP);
        bu = tcg_reg_to_bu(TCG_REG_D1RtP);

        tcg_out32(s, (0xac000004 |                     /* abs call */
                        rs << 19 |                     /* reg num */
                        (abs_addr & 0x0000ffff) << 3 | /* lower bits of addr */
                        bu));                          /* reg BU */
    }
}

static inline void tcg_out_goto_label(TCGContext *s, MetaCond cc,
                int label_index)
{
    TCGLabel *l = &s->labels[label_index];

    if (l->has_value) {
        tcg_out_b(s, (l->u.value - (tcg_target_long) s->code_ptr), cc, 0);
    } else {
        tcg_out_b_noaddr(s, cc);
        /* this may call patch_reloc instantly hence it goes
           after then does ptr-4 */
        tcg_out_reloc(s, s->code_ptr-4, R_METAG_RELBRANCH, label_index, 0);
    }
}

#if defined(USE_DIRECT_JUMP)
void metag_tb_set_jmp_target(uintptr_t jmp_addr, uintptr_t addr)
{
    uint32_t addr_hi = (addr >> 16) & 0x0000ffff;
    uint32_t addr_lo = addr & 0x0000ffff;

    /* reset imms */
    *(uint32_t *)jmp_addr |= 0x7FFFD;
    *(uint32_t *)(jmp_addr+4) |= 0x7FFF8;

    /* set new jmp address */
    *(uint32_t *)jmp_addr &= (0xfff80007 | (addr_hi << 3));
    *(uint32_t *)(jmp_addr+4) &= (0xfff80007 | (addr_lo << 3));

    /* flush icache */
    flush_icache_range(jmp_addr, jmp_addr+12);
}
#endif

static void tcg_out_bswap(TCGContext *s, int l, TCGReg dst, TCGReg src)
{
    int rd, rs, du;

    assert(is_in_data_unit(dst) && is_in_data_unit(src));
    if (tcg_reg_to_meta_unit(dst) != tcg_reg_to_meta_unit(src))
    {
        /* If registers are not of the same unit do a mov first.
         * Our current constraints allow this to happen */
        tcg_out_mov(s, TCG_TYPE_REG, dst, src);
        src = dst;
    }

    rd = tcg_reg_to_r(dst);
    rs = tcg_reg_to_r(src);

    du = (tcg_reg_to_meta_unit(dst) == META_UNIT_D1);

    tcg_out32(s, (0xaa000000 | /* BEXD/L */
                  rd << 19 | /* rd */
                  rs << 14 | /* rs */
                  l << 4 | /* 0 = 32bit, 1 = 64bit */
                  1 << 2 | /* differentiates this from XSDx */
                  du)); /* DU */
}

static inline void tcg_out_bswap16(TCGContext *s, TCGReg dst, TCGReg src)
{
    UNTESTED();

    assert(src != TCG_REG_D0FrT);

    tcg_out_ext8u(s, COND_A, TCG_REG_D0FrT, src);
    tcg_out_lsli(s, COND_A, 0, TCG_REG_D0FrT, TCG_REG_D0FrT, 8);
    tcg_out_lsri(s, COND_A, 0, dst, src, 8);
    tcg_out_or(s, COND_A, 0, dst, dst, TCG_REG_D0FrT);
}

#ifdef CONFIG_SOFTMMU
#include "../../softmmu_defs.h"

/* helper signature: helper_ld_mmu(CPUState *env, target_ulong addr,
   int mmu_idx) */
static const void * const qemu_ld_helpers[4] = {
    helper_ldb_mmu,
    helper_ldw_mmu,
    helper_ldl_mmu,
    helper_ldq_mmu,
};

/* helper signature: helper_st_mmu(CPUState *env, target_ulong addr,
   uintxx_t val, int mmu_idx) */
static const void * const qemu_st_helpers[4] = {
    helper_stb_mmu,
    helper_stw_mmu,
    helper_stl_mmu,
    helper_stq_mmu,
};

#define TLB_SHIFT   (CPU_TLB_ENTRY_BITS + CPU_TLB_BITS)

static TCGReg tcg_out_tlb_load(TCGContext *s, TCGReg addr_reg,
                int s_bits, tcg_target_long **label_ptr,
                int op_offset, int addend)
{
    TCGReg r1, r2, mmu_offs;

    switch (tcg_reg_to_meta_unit(addr_reg)) {
    case (META_UNIT_D0):
        r1 = TCG_REG_D1Ar1;
        mmu_offs = TCG_REG_D0Ar6;
        break;
    case (META_UNIT_D1):
        mmu_offs = TCG_REG_D1Ar5;
        r1 = TCG_REG_D0Ar2;
        break;
    default:
        tcg_abort();
    }
    r2 = TCG_REG_D1Ar3;

    tcg_out_lsri(s, COND_A, 0, r2, addr_reg, TARGET_PAGE_BITS);
    tcg_out_andi(s, COND_A, 0, r1, r2, (CPU_TLB_SIZE - 1));
    tcg_out_lsli(s, COND_A, 0, r1, r1, CPU_TLB_ENTRY_BITS);
    tcg_out_add(s, COND_A, 0, r1, r1, TCG_AREG0);

    tcg_out_getdi(s, mmu_offs, r1, op_offset);
    tcg_out_lsli(s, COND_A, 0, r2, r2, TARGET_PAGE_BITS);
    tcg_out_cmp(s, COND_A, mmu_offs, r2);

    /* Check alignment. */
    if (s_bits) {
        tcg_out_tsti(s, COND_EQ, addr_reg, (1 << s_bits) - 1);
    }
    label_ptr[0] = (tcg_target_long *) s->code_ptr;
    tcg_out_b_noaddr(s, COND_NE);

    /* this should use the 12bit imm version */
    tcg_out_getdi(s, mmu_offs, r1, addend);

    /* only mmu_offs is live by this point in the tb */
    return mmu_offs;
}
#endif

static void tcg_out_qemu_ld(TCGContext *s, const TCGArg *args, int opc)
{
    TCGReg addr_reg, data_reg, data_reg2;
#ifdef CONFIG_SOFTMMU
    TCGReg mmu_offs;
    int mem_index, s_bits;
    tcg_target_long *label_ptr[2];
#endif

    data_reg = *args++;
    if (opc == 3)
        data_reg2 = *args++;
    else
        data_reg2 = 0; /* suppress warning */
    addr_reg = *args++;
#ifdef CONFIG_SOFTMMU
    mem_index = *args;
    s_bits = opc & 3;

    mmu_offs = tcg_out_tlb_load(s, addr_reg, s_bits, label_ptr,
                    offsetof(CPUArchState, tlb_table[mem_index][0].addr_read),
                    offsetof(CPUArchState, tlb_table[mem_index][0].addend));

    /* TLB Hit */
    switch (opc) {
    case 0:
        tcg_out_getb(s, 0, data_reg, addr_reg, mmu_offs);
        break;
    case 0 | 4:
        tcg_out_getb(s, 1, data_reg, addr_reg, mmu_offs);
        break;
    case 1:
        tcg_out_getw(s, 0, data_reg, addr_reg, mmu_offs);
        break;
    case 1 | 4:
        tcg_out_getw(s, 1, data_reg, addr_reg, mmu_offs);
        break;
    case 2:
    default:
        tcg_out_getd(s, data_reg, addr_reg, mmu_offs);
        break;
    case 3:
        assert(addr_reg != data_reg);
        tcg_out_getd(s, data_reg, addr_reg, mmu_offs);
        tcg_out_addi(s, COND_A, 0, mmu_offs, mmu_offs, 4);
        tcg_out_getd(s, data_reg2, addr_reg, mmu_offs);
        break;
    }

    label_ptr[1] = (tcg_target_long *) s->code_ptr;
    tcg_out_b_noaddr(s, COND_EQ);
    reloc_rel(label_ptr[0], (tcg_target_long)s->code_ptr);

    /* TLB Miss */
    /*
     * TODO Look into doing DSP DL MOV
     */
    tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[0], TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[1], addr_reg);
    tcg_out_movi(s, TCG_TYPE_I32, tcg_target_call_iarg_regs[2], mem_index);
    tcg_out_calli(s, (tcg_target_long) qemu_ld_helpers[s_bits]);

    tcg_out_mov(s, TCG_TYPE_REG, data_reg, TCG_REG_D0Re0);
    switch (opc) {
    case 0 | 4:
        tcg_out_ext8s(s, COND_A, data_reg, data_reg);
        break;
    case 1 | 4:
        tcg_out_ext16s(s, COND_A, data_reg, data_reg);
        break;
    case 3:
        tcg_out_mov(s, TCG_TYPE_REG, data_reg2, TCG_REG_D1Re0);
        break;
    }

    reloc_rel(label_ptr[1], (tcg_target_long)s->code_ptr);
#else /* !CONFIG_SOFTMMU */
    switch (opc) {
    case 0:
        tcg_out_getb(s, 0, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 0 | 4:
        tcg_out_getb(s, 1, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 1:
        tcg_out_getw(s, 0, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 1 | 4:
        tcg_out_getw(s, 1, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 2:
    default:
        tcg_out_getd(s, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 3:
        UNTESTED();
        if (data_reg == addr_reg) {
            tcg_abort();
        }
        tcg_out_getseti(s, 0, 2, 1, 0, 0, data_reg, GUEST_BASE, addr_reg);
        tcg_out_getseti(s, 0, 2, 0, 0, 0, data_reg2, GUEST_BASE, addr_reg);
        tcg_out_subi(s, COND_A, 0, addr_reg, addr_reg,
                        sizeof(tcg_target_long));
        break;
    }
#endif
}

static void tcg_out_qemu_st(TCGContext *s, const TCGArg *args, int opc)
{
    TCGReg addr_reg, data_reg, data_reg2;
    int r_paired;
#ifdef CONFIG_SOFTMMU
    TCGReg mmu_offs;
    int mem_index, s_bits;
    tcg_target_long *label_ptr[2];
#endif

    r_paired = 0;
    data_reg = *args++;
    if (opc == 3)
        data_reg2 = *args++;
    else
        data_reg2 = 0; /* suppress warning */
    addr_reg = *args++;
#ifdef CONFIG_SOFTMMU
    mem_index = *args;
    s_bits = opc & 3;

    mmu_offs = tcg_out_tlb_load(s, addr_reg, s_bits, label_ptr,
                    offsetof(CPUArchState, tlb_table[mem_index][0].addr_write),
                    offsetof(CPUArchState, tlb_table[mem_index][0].addend));

    /* TLB Hit */
    switch (opc) {
    case 0:
        tcg_out_setb(s, data_reg, addr_reg, mmu_offs);
        break;
    case 1:
        tcg_out_setw(s, data_reg, addr_reg, mmu_offs);
        break;
    case 2:
    default:
        tcg_out_setd(s, data_reg, addr_reg, mmu_offs);
        break;
    case 3:
        tcg_out_setd(s, data_reg, addr_reg, mmu_offs);
        tcg_out_addi(s, COND_A, 0, mmu_offs, mmu_offs, 4);
        tcg_out_setd(s, data_reg2, addr_reg, mmu_offs);
        break;
    }

    label_ptr[1] = (tcg_target_long *) s->code_ptr;
    tcg_out_b_noaddr(s, COND_EQ);
    reloc_rel(label_ptr[0], (tcg_target_long)s->code_ptr);

    /* TLB Miss */
    /*
     * TODO Look into doing DSP DL MOVs
     */
    tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[0], TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[1], addr_reg);
    switch (opc) {
    case 0:
        tcg_out_ext8u(s, COND_A, tcg_target_call_iarg_regs[2], data_reg);
        tcg_out_movi(s, TCG_TYPE_I32, tcg_target_call_iarg_regs[3], mem_index);
        break;
    case 1:
        tcg_out_ext16u(s, COND_A, tcg_target_call_iarg_regs[2], data_reg);
        tcg_out_movi(s, TCG_TYPE_I32, tcg_target_call_iarg_regs[3], mem_index);
        break;
    case 2:
        tcg_out_mov(s, TCG_TYPE_REG, tcg_target_call_iarg_regs[2], data_reg);
        tcg_out_movi(s, TCG_TYPE_I32, tcg_target_call_iarg_regs[3], mem_index);
        break;
    case 3:
        if (data_reg2 == tcg_target_call_iarg_regs[2]) {
            tcg_out_mov(s, TCG_TYPE_REG,
                            tcg_target_call_iarg_regs[3], data_reg2);
            tcg_out_mov(s, TCG_TYPE_REG,
                            tcg_target_call_iarg_regs[2], data_reg);
        } else {
            tcg_out_mov(s, TCG_TYPE_REG,
                            tcg_target_call_iarg_regs[2], data_reg);
            tcg_out_mov(s, TCG_TYPE_REG,
                            tcg_target_call_iarg_regs[3], data_reg2);
        }
        tcg_out_movi(s, TCG_TYPE_I32, tcg_target_call_iarg_regs[4], mem_index);
        break;
    }
    tcg_out_calli(s, (tcg_target_long) qemu_st_helpers[s_bits]);

    reloc_rel(label_ptr[1], (tcg_target_long) s->code_ptr);
#else /* !CONFIG_SOFTMMU */
    switch (opc) {
    case 0:
        tcg_out_setb(s, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 1:
        tcg_out_setw(s, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 2:
    default:
        tcg_out_setd(s, data_reg, TCG_REG_D1_7, addr_reg);
        break;
    case 3:
        UNTESTED();
        tcg_out_getset(s, 1, 2, 1, 0, 0, data_reg, GUEST_BASE, addr_reg);
        tcg_out_getset(s, 1, 2, 0, 0, 0, data_reg2, GUEST_BASE, addr_reg);
        tcg_out_subi(s, COND_A, 0, addr_reg, addr_reg,
                        sizeof(tcg_target_long));
        break;
    }
#endif
}

static uint8_t *tb_ret_addr;

static inline void tcg_out_op(TCGContext *s, TCGOpcode opc, const TCGArg *args,
                const int *const_args)
{
    MetaCond cc;

    switch (opc) {
    case INDEX_op_exit_tb:
        tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D0Re0, args[0]);
        tcg_out_jmpi(s, (tcg_target_ulong) tb_ret_addr);
        break;
    case INDEX_op_goto_tb:
        if (s->tb_jmp_offset) {
            /* direct jump method */
            s->tb_jmp_offset[args[0]] = s->code_ptr - s->code_buf;
            tcg_out_jmp_noaddr(s);
        } else {
            /* indirect jump method */
            tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D0FrT,
                            (tcg_target_long) (s->tb_next + args[0]));
            tcg_out_getdi(s, TCG_REG_PC, TCG_REG_D0FrT, 0);
        }
        s->tb_next_offset[args[0]] = s->code_ptr - s->code_buf;
        break;
    case INDEX_op_call:
        (const_args[0]) ? tcg_out_calli(s, args[0])
                        : tcg_out_call(s, args[0]);
        break;
    case INDEX_op_br:
        tcg_out_goto_label(s, COND_A, args[0]);
        break;

    case INDEX_op_ld8u_i32:
        tcg_out_getbi(s, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld8s_i32:
        tcg_out_getbi(s, 1, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld16u_i32:
        tcg_out_getwi(s, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld16s_i32:
        tcg_out_getwi(s, 1, args[0], args[1], args[2]);
        break;
    case INDEX_op_ld_i32:
        tcg_out_getdi(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_st8_i32:
        tcg_out_setbi(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_st16_i32:
        tcg_out_setwi(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_st_i32:
        tcg_out_setdi(s, args[0], args[1], args[2]);
        break;

    case INDEX_op_mov_i32:
        tcg_out_mov(s, TCG_TYPE_REG, args[0], args[1]);
        break;
    case INDEX_op_movi_i32:
        tcg_out_movi(s, TCG_TYPE_I32, args[0], args[1]);
        break;

    case INDEX_op_add_i32:
        (const_args[2]) ? tcg_out_addi(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_add(s, COND_A, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_sub_i32:
        (const_args[2]) ? tcg_out_subi(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_sub(s, COND_A, 0, args[0], args[1], args[2]);
        break;

    case INDEX_op_and_i32:
        (const_args[2]) ? tcg_out_andi(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_and(s, COND_A, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_or_i32:
        (const_args[2]) ? tcg_out_ori(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_or(s, COND_A, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_xor_i32:
        (const_args[2]) ? tcg_out_xori(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_xor(s, COND_A, 0, args[0], args[1], args[2]);
        break;

    case INDEX_op_add2_i32:
        tcg_out_add2(s, args[0], args[1], args[2], args[3], args[4], args[5]);
        break;
    case INDEX_op_sub2_i32:
        tcg_out_sub2(s, args[0], args[1], args[2], args[3], args[4], args[5]);
        break;
    case INDEX_op_neg_i32:
        (const_args[1]) ? tcg_out_negi(s, args[0], args[1])
                        : tcg_out_neg(s, args[0], args[1]);
        break;
    case INDEX_op_mul_i32:
        (const_args[2]) ? tcg_out_muldi(s, args[0], args[1], args[2])
                        : tcg_out_muld(s, args[0], args[1], args[2]);
        break;
    case INDEX_op_mulu2_i32:
        tcg_out_umull(s, args[0], args[1], args[2], args[3]);
        break;
    case INDEX_op_shl_i32:
        (const_args[2]) ? tcg_out_lsli(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_lsl(s, COND_A, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_shr_i32:
        (const_args[2]) ? tcg_out_lsri(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_lsr(s, COND_A, 0, args[0], args[1], args[2]);
        break;
    case INDEX_op_sar_i32:
        (const_args[2]) ? tcg_out_asri(s, COND_A, 0, args[0], args[1], args[2])
                        : tcg_out_asr(s, COND_A, 0, args[0], args[1], args[2]);
        break;

    case INDEX_op_ext8s_i32:
        tcg_out_ext8s(s, COND_A, args[0], args[1]);
        break;
    case INDEX_op_ext8u_i32:
        tcg_out_ext8u(s, COND_A, args[0], args[1]);
        break;
    case INDEX_op_ext16s_i32:
        tcg_out_ext16s(s, COND_A, args[0], args[1]);
        break;
    case INDEX_op_ext16u_i32:
        tcg_out_ext16u(s, COND_A, args[0], args[1]);
        break;

    case INDEX_op_bswap16_i32:
        tcg_out_bswap16(s, args[0], args[1]);
        break;
    case INDEX_op_bswap32_i32:
        tcg_out_bswap32(s, args[0], args[1]);
        break;

    case INDEX_op_brcond_i32:
        (const_args[1]) ? tcg_out_cmpi(s, COND_A, args[0], args[1])
                        : tcg_out_cmp(s, COND_A, args[0], args[1]);
        tcg_out_goto_label(s, tcg_cond_to_metag_cond[args[2]], args[3]);
        break;
    case INDEX_op_brcond2_i32:
        UNTESTED();
        /* The resulting conditions are:
         * TCG_COND_EQ    -->  a0 == a2 && a1 == a3,
         * TCG_COND_NE    --> (a0 != a2 && a1 == a3) ||  a1 != a3,
         * TCG_COND_LT(U) --> (a0 <  a2 && a1 == a3) ||  a1 <  a3,
         * TCG_COND_GE(U) --> (a0 >= a2 && a1 == a3) || (a1 >= a3 && a1 != a3),
         * TCG_COND_LE(U) --> (a0 <= a2 && a1 == a3) || (a1 <= a3 && a1 != a3),
         * TCG_COND_GT(U) --> (a0 >  a2 && a1 == a3) ||  a1 >  a3,
         */
        tcg_out_cmp(s, COND_A, args[1], args[3]);
        tcg_out_cmp(s, COND_EQ, args[0], args[2]);
        tcg_out_goto_label(s, tcg_cond_to_metag_cond[args[4]], args[5]);
        break;
    case INDEX_op_setcond_i32:
        (const_args[2]) ? tcg_out_cmpi(s, COND_A, args[1], args[2])
                        : tcg_out_cmp(s, COND_A, args[1], args[2]);
        tcg_out_movi(s, TCG_TYPE_I32, args[0], 0);
        tcg_out_xori(s, tcg_cond_to_metag_cond[args[3]], 0,
                        args[0], args[0], 1);
        break;
    case INDEX_op_setcond2_i32:
        UNTESTED();
        /* See brcond2_i32 comments */
        tcg_out_cmp(s, COND_A, args[2], args[4]);
        tcg_out_cmp(s, COND_EQ, args[1], args[3]);
        tcg_out_movi(s, TCG_TYPE_I32, args[0], 0);
        tcg_out_xori(s, tcg_cond_to_metag_cond[args[5]], 0,
                        args[0], args[0], 1);
        break;
    case INDEX_op_movcond_i32:
        cc = tcg_cond_to_metag_cond[args[5]];

        (const_args[2]) ? tcg_out_cmpi(s, COND_A, args[1], args[2])
                        : tcg_out_cmp(s, COND_A, args[1], args[2]);
        tcg_out_movcc(s, cc, args[0], args[3], 0);
        tcg_out_movcc(s, inv_metag_cond[cc], args[0], args[4], 0);
        break;

    case INDEX_op_qemu_ld8u:
        tcg_out_qemu_ld(s, args, 0);
        break;
    case INDEX_op_qemu_ld8s:
        tcg_out_qemu_ld(s, args, 0 | 4);
        break;
    case INDEX_op_qemu_ld16u:
        tcg_out_qemu_ld(s, args, 1);
        break;
    case INDEX_op_qemu_ld16s:
        tcg_out_qemu_ld(s, args, 1 | 4);
        break;
    case INDEX_op_qemu_ld32:
        tcg_out_qemu_ld(s, args, 2);
        break;
    case INDEX_op_qemu_ld64:
        tcg_out_qemu_ld(s, args, 3);
        break;

    case INDEX_op_qemu_st8:
        tcg_out_qemu_st(s, args, 0);
        break;
    case INDEX_op_qemu_st16:
        tcg_out_qemu_st(s, args, 1);
        break;
    case INDEX_op_qemu_st32:
        tcg_out_qemu_st(s, args, 2);
        break;
    case INDEX_op_qemu_st64:
        tcg_out_qemu_st(s, args, 3);
        break;

    default:
        UNIMPLEMENTED("op=%d", opc);
    };
}

/*
 * META Register Constraints
 * NOTE - constraints that remove registers should come last
 *
 * 0-9 - alias (reserved by tcg.c)
 * a -
 * b - A0 unit register
 * c - A1 unit register
 * d - D0 unit register
 * e - D1 unit register
 * f - 8bit constant
 * g - 6bit constant
 * h - 5bit constant
 * i - any constant (reserved by tcg.c)
 * j - 16bit constant
 * k -
 * l -
 * m - Has M for constant encoding
 * n -
 * o -
 * p - No DxRe0 registers
 * q - No DxArY registers
 * r - any data or address register
 * s - any data unit register
 * t - any address unit register
 * u - D0Ar2
 * v - D1Ar1
 * w - qemu_ld/st - dont use certain DxAry regs
 * x -
 * y - D1Re0
 * z - D0Re0
 *
 */

static const TCGTargetOpDef metag_op_defs[] = {
                { INDEX_op_exit_tb, {} },
                { INDEX_op_goto_tb, {} },
                { INDEX_op_call, { "ri" } },
                { INDEX_op_br, {} },

                { INDEX_op_add_i32, { "r", "r", "rj" } },
                { INDEX_op_sub_i32, { "r", "r", "rj" } },

                { INDEX_op_add2_i32, { "r", "r", "r", "r", "r", "r" } },
                { INDEX_op_sub2_i32, { "r", "r", "r", "r", "r", "r" } },

                { INDEX_op_neg_i32, { "r", "r" } },
                { INDEX_op_mov_i32, { "r", "r" } },
                { INDEX_op_movi_i32, { "r" } },

                /* D0 only for base so if offset > 6bit(signed)
                 * we can use our D0FrT reserved register */
                { INDEX_op_ld8u_i32, { "r", "s" } },
                { INDEX_op_ld8s_i32, { "r", "s" } },
                { INDEX_op_ld16u_i32, { "r", "s" } },
                { INDEX_op_ld16s_i32, { "r", "s" } },
                { INDEX_op_ld_i32, { "r", "s" } },
                { INDEX_op_st8_i32, { "r", "s" } },
                { INDEX_op_st16_i32, { "r", "s" } },
                { INDEX_op_st_i32, { "r", "s" } },

                { INDEX_op_mul_i32, { "r", "s", "sj" } },
                { INDEX_op_mulu2_i32, { "z", "y", "v", "u" } },

                { INDEX_op_and_i32, { "r", "s", "rjm" } },
                { INDEX_op_or_i32, { "r", "s", "rjm" } },
                { INDEX_op_xor_i32, { "r", "s", "rjm" } },

                { INDEX_op_shl_i32, { "r", "s", "si" } },
                { INDEX_op_shr_i32, { "r", "s", "si" } },
                { INDEX_op_sar_i32, { "r", "s", "si" } },

                { INDEX_op_ext8s_i32, { "s", "s" } },
                { INDEX_op_ext16s_i32, { "s", "s" } },
                { INDEX_op_ext8u_i32, { "r", "s" } },
                { INDEX_op_ext16u_i32, { "r", "s" } },

                { INDEX_op_bswap16_i32, { "s", "s" } },
                { INDEX_op_bswap32_i32, { "e", "e" } },

                { INDEX_op_brcond_i32, { "s", "rjm" } },
                { INDEX_op_setcond_i32, { "s", "s", "rjm" } },
                { INDEX_op_brcond2_i32, { "s", "s", "r", "r" } },
                { INDEX_op_setcond2_i32, { "s", "s", "s", "r", "r" } },
                { INDEX_op_movcond_i32, { "r", "s", "rjm", "r", "r" } },


# define LD_DATA_CT      "r"
# define ST_DATA_CT      "rw"
#if defined(CONFIG_SOFTMMU)
# define ADDR_CT         "sw"
#else /* !defined(CONFIG_SOFTMMU) */
/* Use D1 for address reg along with D1.7 which is GUEST_BASE */
# define ADDR_CT         "ew"
#endif

                { INDEX_op_qemu_ld8u, { LD_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_ld8s, { LD_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_ld16u, { LD_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_ld16s, { LD_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_ld32, { LD_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_ld64, { LD_DATA_CT, LD_DATA_CT, ADDR_CT } },

                { INDEX_op_qemu_st8, { ST_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_st16, { ST_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_st32, { ST_DATA_CT, ADDR_CT } },
                { INDEX_op_qemu_st64, { ST_DATA_CT, ST_DATA_CT, ADDR_CT } },

                { -1 }, };

static inline int is_hw_dsp_enabled(void)
{
    unsigned long txenable;
    asm volatile("mov %0,txenable\n" : "=da" (txenable));

    return ((txenable & 0xF000) == 0x0);
}

static inline int is_dsp_enabled(void)
{
#if 1
    /* Usage of DSP regs cause problems with our register constraints + o2r
     * disable for now. Options:
     *  - Only use DSP regs internally (instead of reserving D0FrT etc)
     *  - Use only D1 regs ignoring D0 - no longer need to worry as about units
     *  - Dont use DSP regs ever
     *  - Enable DSP regs on few insns but this will increase the amount of
     *    movs tcg spits out */
    return 0;
#else
    FILE *f;
    char line[512];
    int ret = 0;

    if (is_hw_dsp_enabled()) {
        f = fopen("/proc/cpuinfo", "rb");

        do {
            if(!fgets(line, sizeof(line), f)) {
                break;
            }

            if (!strncmp(line, "capabilities", 12)) {
                if (strstr(line, "dsp")) {
                    ret = 1;
                    break;
                }
            }
        } while (*line);

        fclose(f);
    }

    return ret;
#endif
}

static inline int is_smp_enabled(void)
{
    FILE *f = fopen("/proc/cpuinfo", "rb");
    char line[512];
    int cpu_count = 0;

    do {
        if(!fgets(line, sizeof(line), f)) {
            break;
        }

        if (!strncmp(line, "CPU", 3)) {
            cpu_count ++;
        }
    } while (*line);

    fclose(f);

    return (cpu_count > 1);
}

static void tcg_target_init(TCGContext *s)
{
    int i;

#if !defined(CONFIG_USER_ONLY)
    /* fail safe */
    if ((1 << CPU_TLB_ENTRY_BITS) != sizeof(CPUTLBEntry))
        tcg_abort();
#endif

    tcg_regset_clear(tcg_target_available_regs[TCG_TYPE_I32]);
    for (i = 0; i < ARRAY_SIZE(tcg_target_reg_alloc_order); ++i) {
        tcg_regset_set_reg(tcg_target_available_regs[TCG_TYPE_I32],
                            (1 << tcg_target_reg_alloc_order[i]));
    }

    tcg_regset_set_bits(tcg_target_call_clobber_regs, 0,
                    (1ull << TCG_REG_D0Re0) |
                    (1ull << TCG_REG_D1Re0) |
                    (1ull << TCG_REG_D0Ar6) |
                    (1ull << TCG_REG_D1Ar5) |
                    (1ull << TCG_REG_D0Ar4) |
                    (1ull << TCG_REG_D1Ar3) |
                    (1ull << TCG_REG_D0Ar2) |
                    (1ull << TCG_REG_D1Ar1) |
                    (1ull << TCG_REG_A0_2) |
                    (1ull << TCG_REG_A1_2) |
                    (1ull << TCG_REG_A0_3) |
                    (1ull << TCG_REG_A1_3));

    tcg_regset_clear(s->reserved_regs);
    tcg_regset_set_bits(s->reserved_regs, 0,
                    (1ull << TCG_REG_PC) |      /* pc */
                    (1ull << TCG_REG_A0StP) |   /* stack ptr */
                    (1ull << TCG_REG_A1GbP) |   /* global ptr */
                    (1ull << TCG_REG_A1LbP) |   /* not saved */
                    (1ull << TCG_REG_D1RtP) |   /* return ptr */
                    (1ull << TCG_REG_A0FrP) |   /* frame ptr */
#if !defined(CONFIG_SOFTMMU)
                    (1ull << TCG_REG_D1_7) |    /* guest_base */
#endif
                    (1ull << TCG_REG_D0_8) |    /* used in DSP ABI */
    /* the following are for mismatched units, we generate movs with them */
                    (1ull << TCG_REG_D0FrT) |
                    (1ull << TCG_REG_D1_5));

    if (!is_dsp_enabled()) {
        /* disable use of any dsp regs */
        tcg_regset_set_bits(s->reserved_regs, 0,
                        (1ull << TCG_REG_D0_9) |
                        (1ull << TCG_REG_D0_10) |
                        (1ull << TCG_REG_D0_11) |
                        (1ull << TCG_REG_D0_12) |
                        (1ull << TCG_REG_D0_13) |
                        (1ull << TCG_REG_D0_14) |
                        (1ull << TCG_REG_D0_15) |
                        (1ull << TCG_REG_D1_8) |
                        (1ull << TCG_REG_D1_9) |
                        (1ull << TCG_REG_D1_10) |
                        (1ull << TCG_REG_D1_11) |
                        (1ull << TCG_REG_D1_12) |
                        (1ull << TCG_REG_D1_13) |
                        (1ull << TCG_REG_D1_14) |
                        (1ull << TCG_REG_D1_15) |
                        (1ull << TCG_REG_A0_4) |
                        (1ull << TCG_REG_A0_5) |
                        (1ull << TCG_REG_A0_6) |
                        (1ull << TCG_REG_A0_7) |
                        (1ull << TCG_REG_A1_4) |
                        (1ull << TCG_REG_A1_5) |
                        (1ull << TCG_REG_A1_6) |
                        (1ull << TCG_REG_A1_7));
    } else if (is_smp_enabled()) {
        printf("DSP support on an SMP kernel detected; "
               "assuming all threads have same privileges.\n");
    }

    tcg_add_target_add_op_defs(metag_op_defs);
}

static void tcg_target_qemu_prologue(TCGContext *s)
{
    /* Size of stack saved by prologue (incl. D0FrT and D1RtP) */
    const int saved_regs_size = 0x20;
    /* TCG_TARGET_CALL_STACK_OFFSET is a negative value,
     * so negate it again for a positive size */
    const int frame_size = -TCG_TARGET_CALL_STACK_OFFSET;

    tcg_set_frame(s, TCG_REG_A0FrP, saved_regs_size, frame_size);

    /* save registers onto stack */
    /* MOV      D0FrT,A0FrP */
    /* ADD      A0FrP,A0StP,0x0 */
    /* MSETL    [A0StP++] D0FrT...D0.7 */
    /* ADD      A0StP,A0StP,#(frame_size) */
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_D0FrT, TCG_REG_A0FrP);
    tcg_out_addi(s, COND_A, 0, TCG_REG_A0FrP, TCG_REG_A0StP, 0);
    tcg_out_msetl(s, TCG_REG_A0StP, TCG_REG_D0FrT, 0x7);
    tcg_out_addi(s, COND_A, 0, TCG_REG_A0StP, TCG_REG_A0StP, frame_size);

    /* setup env and jump into tb */
#if !defined(CONFIG_SOFTMMU)
    /* MOV      D1.7,#GUEST_BASE */
    tcg_out_movi(s, TCG_TYPE_I32, TCG_REG_D1_7, GUEST_BASE);
#endif
    /* MOV      AREG0,D1Ar1 */
    /* MOV      PC, D0Ar2 */
    tcg_out_mov(s, TCG_TYPE_REG, TCG_AREG0, tcg_target_call_iarg_regs[0]);
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_PC, tcg_target_call_iarg_regs[1]);
    tb_ret_addr = s->code_ptr;

    /* pop registers off the stack and return */
    /* SUB      A0FrP,A0StP,#(saved_regs_size+frame_size) */
    /* MGETL    D0FrT...D0.7 [A0FrP++] */
    /* SUB      A0StP,A0FrP,#saved_regs_size */
    /* MOV      A0FrP,D0FrT */
    /* MOV      PC,D1RtP */
    tcg_out_subi(s, COND_A, 0, TCG_REG_A0FrP, TCG_REG_A0StP,
                    saved_regs_size+frame_size);
    tcg_out_mgetl(s, TCG_REG_A0FrP, TCG_REG_D0FrT, 0x7);
    tcg_out_subi(s, COND_A, 0, TCG_REG_A0StP, TCG_REG_A0FrP, saved_regs_size);
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_A0FrP, TCG_REG_D0FrT);
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_PC, TCG_REG_D1RtP);
}
