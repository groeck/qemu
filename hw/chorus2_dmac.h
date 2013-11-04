/*
 * Frontier Silicon Chorus2 DMA controller.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef hw_chorus2_dmac_h
# define hw_chorsu2_dmac_h "chorus2_dmac.h"

#include <inttypes.h>
#include "hwaddr.h"
#include "qemu-common.h"
#include "memory.h"

#define C2_DMAC_SIZE        0x1000

struct chorus2_dmac_state_s;

MemoryRegion *chorus2_dmac_iomem(struct chorus2_dmac_state_s *dmac);
struct soc_dma_s *chorus2_dmac_dma(struct chorus2_dmac_state_s *dmac);

struct chorus2_dmac_state_s *chorus2_dmac_init(unsigned int chans,
                                               unsigned int perips,
                                               hwaddr base);
void chorus2_dmac_reset(struct chorus2_dmac_state_s *dmac);
void chorus2_dmac_set_perip(struct chorus2_dmac_state_s *dmac,
                            unsigned int chan, unsigned int perip);

#endif
