/*
 * ImgTec Debug Adapter (DA) driver
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include "cpu.h"
#include "helper.h"
#include "meta_switch.h"
#include "sysbus.h"

typedef struct MetaDAState {
    SysBusDevice busdev;
    int32_t exit_threads;
} MetaDAState;

static int meta_da_init(SysBusDevice *busdev)
{
    MetaDAState *s = FROM_SYSBUS(MetaDAState, busdev);
    DeviceState *dev;

    /* UnExpXXX unwind */
    dev = qdev_create(NULL, "daunexp");
    qdev_init_nofail(dev);

    /* LogF */
    dev = qdev_create(NULL, "dalogf");
    qdev_init_nofail(dev);

    /* misc */
    dev = qdev_create(NULL, "damisc");
    if (s->exit_threads >= 0) {
        qdev_prop_set_int32(dev, "exit_threads", s->exit_threads);
    }
    qdev_init_nofail(dev);

    return 0;
}

static Property meta_da_properties[] = {
    DEFINE_PROP_INT32("exit_threads", MetaDAState, exit_threads, -1),
    DEFINE_PROP_END_OF_LIST(),
};

static void meta_da_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = meta_da_init;
    dc->props = meta_da_properties;
}

static TypeInfo meta_da_info = {
    .name          = "da",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MetaDAState),
    .class_init    = meta_da_class_initfn,
};

static void meta_da_register(void)
{
    type_register_static(&meta_da_info);
}

type_init(meta_da_register)
