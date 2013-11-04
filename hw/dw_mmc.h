/*
 * Synopsys DesignWare Mobile Storage Host Devices.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef hw_dw_mmc_h
# define hw_dw_mmc_h "comet_mmc.h"

#include "qemu-common.h"
#include "memory.h"
#include "irq.h"

struct dw_mmc_state;

/* reset the block */
void dw_mmc_reset(struct dw_mmc_state *s);

/* create the block */
struct dw_mmc_state *dw_mmc_init(MemoryRegion *address_space,
                                 hwaddr iomem, target_ulong len,
                                 unsigned int num_cards,
                                 unsigned int fifo_depth,
                                 BlockDriverState **bd,
                                 qemu_irq irq, qemu_irq rxdrq, qemu_irq txdrq);

/* get the card detect bit for a specific card (0 = detected) */
int dw_mmc_get_cd(struct dw_mmc_state *s, int card);

/* access DMA data */
void dw_mmc_set_dma_rdata(struct dw_mmc_state *s, uint32_t val);
uint32_t dw_mmc_get_dma_wdata(struct dw_mmc_state *s);

#endif
