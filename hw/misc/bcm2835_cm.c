/*
 * BCM2835 Clock subsystem (poor man's version)
 *
 * Copyright (C) 2018 Guenter Roeck <linux@roeck-us.net>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "crypto/random.h"
#include "hw/misc/bcm2835_cm.h"
#include "migration/vmstate.h"

static uint64_t bcm2835_cm_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    BCM2835CmState *s = (BCM2835CmState *)opaque;
    uint32_t res = 0;

    assert(size == 4);

    if (offset / 4 < CM_NUM_REGS) {
        res = s->regs[offset / 4];
    }

    return res;
}

#define CM_PASSWORD             0x5a000000
#define CM_PASSWORD_MASK        0xff000000

static void bcm2835_cm_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    BCM2835CmState *s = (BCM2835CmState *)opaque;

    assert(size == 4);

    if ((value & 0xff000000) == CM_PASSWORD &&
        offset / 4 < CM_NUM_REGS)
            s->regs[offset / 4] = value & ~CM_PASSWORD_MASK;
}

static const MemoryRegionOps bcm2835_cm_ops = {
    .read = bcm2835_cm_read,
    .write = bcm2835_cm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_cm = {
    .name = TYPE_BCM2835_CM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, BCM2835CmState, CM_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_cm_init(Object *obj)
{
    BCM2835CmState *s = BCM2835_CM(obj);

    memory_region_init_io(&s->iomem, obj, &bcm2835_cm_ops, s,
                          TYPE_BCM2835_CM, 0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

#define CM_GNRICCTL     (0x000 / 4)
#define CM_VECCTL       (0x0f8 / 4)
#define CM_LOCK         (0x114 / 4)
#define CM_DSI1ECTL     (0x158 / 4)
#define CM_DFTCTL       (0x168 / 4)
#define CM_PULSECTL     (0x190 / 4)
#define CM_EMMCCTL      (0x1c0 / 4)

#define A2W_PLLA_CTRL   (0x1100 / 4)
#define A2W_PLLB_CTRL   (0x11e0 / 4)

static void bcm2835_cm_reset(DeviceState *dev)
{
    BCM2835CmState *s = BCM2835_CM(dev);
    int i;

    /*
     * Available information suggests that CM registers have default
     * values which are not overwritten by ROMMON (u-boot). The hardware
     * default values are unknown at this time.
     * The default values selected here are necessary and sufficient
     * to boot Linux directly (on raspi2 and raspi3). The selected
     * values enable all clocks and set clock rates to match their
     * parent rates.
     */
    for (i = CM_GNRICCTL; i <= CM_VECCTL; i += 2) {
        s->regs[i] = 0x11;      /* bit 4:      enable,
                                   bit 3..0:   source (oscillator, 19.2 MHz) */
        s->regs[i + 1] = 0x1000;/* bit 23..12: Integer part of divider       */
    }
    for (i = CM_DSI1ECTL; i <= CM_DFTCTL; i += 2) {
        s->regs[i] = 0x11;
        s->regs[i + 1] = 0x1000;
    }
    for (i = CM_PULSECTL; i <= CM_EMMCCTL; i += 2) {
        s->regs[i] = 0x11;
        s->regs[i + 1] = 0x1000;
    }
    for (i = A2W_PLLA_CTRL; i <= A2W_PLLB_CTRL; i += 8) {
        s->regs[i] = 0x10001;   /* bit 16: power down, bit 0..9: ndiv */
    }

    s->regs[CM_LOCK] = 0x1f00;  /* set all lock bits */
}

static void bcm2835_cm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = bcm2835_cm_reset;
    dc->vmsd = &vmstate_bcm2835_cm;
}

static TypeInfo bcm2835_cm_info = {
    .name          = TYPE_BCM2835_CM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835CmState),
    .class_init    = bcm2835_cm_class_init,
    .instance_init = bcm2835_cm_init,
};

static void bcm2835_cm_register_types(void)
{
    type_register_static(&bcm2835_cm_info);
}

type_init(bcm2835_cm_register_types)
