/*
 * BCM2835 Poor-man's version of CM (clock management)
 *
 * Copyright (C) 2018 Guenter Roeck <linux@roeck-us.net>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_CM_H
#define BCM2835_CM_H

#include "hw/sysbus.h"

#define TYPE_BCM2835_CM "bcm2835-cm"
#define BCM2835_CM(obj) \
        OBJECT_CHECK(BCM2835CmState, (obj), TYPE_BCM2835_CM)

#define CM_NUM_REGS         (0x1200 / 4)

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[CM_NUM_REGS];
} BCM2835CmState;

#endif
