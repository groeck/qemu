/*
 * Imagination Technologies PDP in Toumaz XENIF SoC (Comet).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 *
 * IMG Powerdown Controller in XENIF (Comet).
 */

#ifndef hw_comet_pdc_h
# define hw_comet_pdc_h "comet_pdc.h"

#include "qemu-common.h"
#include "memory.h"
#include "irq.h"

/* Initialise PDC state */
void comet_pdc_init(MemoryRegion *address_space, hwaddr iomem,
                    target_ulong len, qemu_irq irq, qemu_irq rtc_irq,
                    qemu_irq ir_irq, qemu_irq wd_irq, uint8_t num_syswakes);

#endif
