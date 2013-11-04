/*
 * ImgTec Debug Adapter (DA) UnExpXXX driver
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <stdio.h>
#include "meta_switch.h"

typedef struct MetaDAUnExpState {
    MetaSwitchDevice dev;
} MetaDAUnExpState;

static int meta_daunexp_handler(MetaSwitchDevice *dev, CPUArchState *env,
                               const uint32_t *args, int nargs,
                               uint32_t *ret, int nret)
{
    const target_ulong ctx_off_pc = 4;
    const target_ulong ctx_off_stp =
        8 +       /* Flags, SaveMask, CurrPC */
        (8 * 8) + /* DX[8] */
        16;       /* CurrRPT, CurrBPOBITS, CurrMODE, CurrDIVTIME */

    target_ulong ctx_stp, pctx = args[0];
    uint32_t signum = args[2];
    uint32_t nested_switch = args[4];
    int mmu_idx = cpu_mmu_index(env);
    int retcode;

    if (signum > 3 /* TBID_SIGNUM_SW3 */) {
        /* this UnExpXXX wasn't caused by a SWITCH */
        return -1;
    }

    ctx_stp = helper_ldl_mmu(env, pctx + ctx_off_stp, mmu_idx);
    ret[0] = args[0];
    ret[1] = args[1];

    retcode = meta_switch_handle(env, nested_switch, ctx_stp);
    if (!retcode) {
        /* advance context CurrPC */
        uint32_t pc = helper_ldl_mmu(env, pctx + ctx_off_pc, mmu_idx);
        MetaISA isa = meta_current_isa_pc(env, pc);
        if (isa == MINIM) {
            uint16_t insn_core = helper_ldw_mmu(env, pc, mmu_idx);
            size_t minim_words = meta_minim_decode_size(insn_core);
            pc += 4 * minim_words;
        } else {
            pc += 4;
        }
        helper_stl_mmu(env, pctx + ctx_off_pc, pc, mmu_idx);
    }
    return retcode;
}

static int meta_daunexp_initfn(MetaSwitchDevice *dev)
{
    dev->handler = meta_daunexp_handler;
    return 0;
}

static void meta_daunexp_class_initfn(ObjectClass *klass, void *data)
{
    MetaSwitchDeviceClass *sc = META_SWITCH_DEVICE_CLASS(klass);
    sc->init = meta_daunexp_initfn;
    sc->subgroup = META_SWITCH_UNEXP;
}

static TypeInfo meta_daunexp_info = {
    .name          = "daunexp",
    .parent        = TYPE_META_SWITCH_DEVICE,
    .instance_size = sizeof(MetaDAUnExpState),
    .class_init    = meta_daunexp_class_initfn,
};

static void meta_daunexp_register(void)
{
    type_register_static(&meta_daunexp_info);
}

type_init(meta_daunexp_register)
