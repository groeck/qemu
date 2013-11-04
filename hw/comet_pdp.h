/*
 * Imagination Technologies PDP in Toumaz XENIF SoC (Comet).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 *
 * Comet specific PDP register interface.
 */

#ifndef hw_comet_pdp_h
# define hw_comet_pdp_h "comet_pdp.h"

#include "memory.h"
#include "pdp.h"

typedef struct comet_pdp_gfx_plane_s {
    pdp_plane plane;
    target_ulong offset;
    int use_lut;
} comet_pdp_gfx_plane;

typedef struct comet_pdp_state_s {
    pdp_state pdp;
    comet_pdp_gfx_plane gfx;
    hwaddr base_addr;
    MemoryRegion iomem;
    uint8_t palette_addr;
} comet_pdp_state;

/* Initialise PDP state */
comet_pdp_state *comet_pdp_init(MemoryRegion *address_space,
                                hwaddr iomem, target_ulong len,
                                qemu_irq irq, CPUArchState *env);

/* Set base address of PDP */
void comet_pdp_set_base(comet_pdp_state *cpdp, hwaddr base);

/* Resolution change */
void comet_pdp_set_res(comet_pdp_state *cpdp, int width, int height);

#endif
