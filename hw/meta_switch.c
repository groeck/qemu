/*
 * ImgTec Meta SWITCH bus
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
#include "dasim.h"
#include "meta_switch.h"
#include "sysbus.h"

static MetaSwitchBus *switch_bus = NULL;

static int device_qdev_init(DeviceState *qdev)
{
    MetaSwitchDevice *dev = META_SWITCH_DEVICE(qdev);
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(dev);
    uint8_t thread_mask;
    int i;

    if (klass->init) {
        int ret = klass->init(dev);
        if (ret) {
            return ret;
        }
    }

    if (klass->per_thread) {
        thread_mask = dev->thread_mask;
    } else {
        thread_mask = (1 << META_MAX_THREADS) - 1;
    }

    for (i = 0; i < META_MAX_THREADS; i++) {
        if (!(thread_mask & (1 << i))) {
            continue;
        }
        if (switch_bus->devices[klass->subgroup][i]) {
            fprintf(stderr, "Multiple SWITCH devices for thread %d"
                    " subgroup %d\n", i, klass->subgroup);
        }
        switch_bus->devices[klass->subgroup][i] = dev;
    }

    return 0;
}

static int device_qdev_exit(DeviceState *qdev)
{
    MetaSwitchDevice *dev = META_SWITCH_DEVICE(qdev);
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(dev);
    int i;

    if (klass->destroy) {
        int ret = klass->destroy(dev);
        if (ret) {
            return ret;
        }
    }

    for (i = 0; i < META_MAX_THREADS; i++) {
        if (switch_bus->devices[klass->subgroup][i] == dev) {
            switch_bus->devices[klass->subgroup][i] = NULL;
        }
    }

    return 0;
}

static void device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->init = device_qdev_init;
    k->exit = device_qdev_exit;
    k->bus_type = TYPE_META_SWITCH_BUS;
}

static TypeInfo device_type_info = {
    .name = TYPE_META_SWITCH_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(MetaSwitchDevice),
    .abstract = true,
    .class_size = sizeof(MetaSwitchDeviceClass),
    .class_init = device_class_init,
};

static int bridge_init(SysBusDevice *dev)
{
    return 0;
}

static void bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = bridge_init;
    dc->fw_name = "meta_switch";
    dc->no_user = 1;
}

static TypeInfo bus_bridge_info = {
    .name          = "meta_switch_bus-bridge",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusDevice),
    .class_init    = bridge_class_init,
};

static void bus_dev_print(Monitor *mon, DeviceState *dev, int indent)
{
    monitor_printf(mon, "%*smeta_switch\n", indent, "");
}

static char *bus_get_fw_dev_path(DeviceState *dev)
{
    MetaSwitchDevice *d = (MetaSwitchDevice*)dev;
    MetaSwitchDeviceClass *klass = META_SWITCH_DEVICE_GET_CLASS(d);
    char path[40];
    int off;

    off = snprintf(path, sizeof(path), "%s", qdev_fw_name(dev));
    snprintf(path + off, sizeof(path) - off, "@%02x", klass->subgroup);

    return g_strdup(path);
}

static void bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->print_dev = bus_dev_print;
    k->get_fw_dev_path = bus_get_fw_dev_path;
}

static const TypeInfo bus_info = {
    .name = TYPE_META_SWITCH_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(MetaSwitchBus),
    .class_init = bus_class_init,
};

MetaSwitchBus *meta_switch_bus_new(DeviceState *dev)
{
    if (switch_bus) {
        fprintf(stderr, "Can't create a second meta switch bus\n");
        return NULL;
    }
    if (!dev) {
        dev = qdev_create(NULL, "meta_switch_bus-bridge");
        qdev_init_nofail(dev);
    }

    switch_bus = FROM_QBUS(MetaSwitchBus,
                           qbus_create(TYPE_META_SWITCH_BUS, dev, NULL));
    return switch_bus;
}

/**
 * Determine whether any device is attached to the bus.
 * @return true if any device is attached, else false
 */
bool meta_switch_bus_has_device(MetaSwitchBus *bus)
{
    int i, t;

    for (i = 0; i < META_SWITCH_SUBGROUP_MAX; i++) {
        for (t = 0; t < META_MAX_THREADS; t++) {
            if (bus->devices[i][t]) {
                return true;
            }
        }
    }

    return false;
}

/**
 * Find a device handling the given subgroup for the given thread.
 * @return device handling the given subgroup, else NULL
 */
MetaSwitchDevice *meta_switch_bus_lookup_device(MetaSwitchBus *bus,
                                                CPUArchState *env,
	                                            uint8_t subgroup)
{
    return switch_bus->devices[subgroup][env->thread_num];
}

/**
 * Attempt to handle a SWITCH given its code & stack pointer.
 * @return zero on success, otherwise non-zero
 */
int meta_switch_handle(CPUArchState *env, uint32_t swtch, target_ulong stp)
{
    MetaSwitchDevice *dev;
    uint32_t scope, subgroup, res_window, stack_window;
    uint32_t args_onstack[6], *args = args_onstack;
    uint32_t ret_onstack[2], *ret = ret_onstack;
    int nargs, nret, i, retcode, mmu_idx = cpu_mmu_index(env);

    if (meta_current_isa(env) == MINIM) {
        scope = (swtch >> 6) & 0x3;
        subgroup = (swtch >> 4) & 0x3;
        res_window = ((swtch >> 3) & 0x1) << 1;
        stack_window = (swtch & 0x7) << 1;
    } else {
        scope = (swtch >> 22) & 0x3;
        subgroup = (swtch >> 16) & 0x3f;
        res_window = (swtch >> 8) & 0xff;
        stack_window = swtch & 0xff;
    }

    nargs = stack_window - res_window;
    nret = res_window;

    if (scope != META_SWITCH_EXTERNAL) {
        return -1;
    }
    if (!switch_bus) {
        return -2;
    }

    /* retrieve the appropriate switch device */
    dev = meta_switch_bus_lookup_device(switch_bus, env, subgroup);
    if (!dev) {
        if (!sap_comms) {
            fprintf(stderr, "Unhandled external SWITCH subgroup %d\n",
                    subgroup);
        }
        return -3;
    }

    /* allocate enough memory for args & return */
    if (nargs > ARRAY_SIZE(args_onstack)) {
        args = g_malloc0(sizeof(args[0]) * nargs);
    }
    if (nret > ARRAY_SIZE(ret_onstack)) {
        ret = g_malloc0(sizeof(ret[0]) * nret);
    }

    /* retrieve arg values */
    for (i = 0; i < nargs; i++) {
        uint32_t addr = stp - ((res_window + i + 1) << 2);
        args[i] = helper_ldl_mmu(env, addr, mmu_idx);
    }

    /* handle the switch */
    retcode = dev->handler(dev, env, args, nargs, ret, nret);

    /* store result */
    for (i = 0; i < nret; i++) {
        uint32_t addr = stp - ((i + 1) << 2);
        helper_stl_mmu(env, addr, ret[i], mmu_idx);
    }

    /* cleanup */
    if (args != args_onstack) {
        g_free(args);
    }
    if (ret != ret_onstack) {
        g_free(ret);
    }

    return retcode;
}

char *meta_switch_arg_string(CPUArchState *env,
                             target_ulong addr, uint32_t max_len)
{
    char buf[65], *ret;
    int nlen, len = 0;

    buf[sizeof(buf) - 1] = 0;

    /* find the null terminator to detemine string length */
    while (len < max_len) {
        if (cpu_memory_rw_debug(env, addr + len, (unsigned char *)buf,
                                sizeof(buf) - 1, 0) < 0) {
            break;
        }
        nlen = strlen(buf);
        len += nlen;
        if (nlen < (sizeof(buf) - 1)) {
            break;
        }
    }

    /* limit length */
    if (len > max_len) {
        len = max_len;
    }

    /* allocate null-terminated string */
    ret = g_malloc(len + 1);
    ret[len] = 0;

    /* if we only required one buffer read to find length, reuse it */
    if (len < sizeof(buf)) {
        memcpy(ret, buf, len);
        return ret;
    }

    /* re-copy the memory */
    if (cpu_memory_rw_debug(env, addr, (unsigned char *)ret,
                            len + 1, 0) < 0) {
        len = 0;
    }
    ret[len] = 0;
    return ret;
}

static void meta_switch_register(void)
{
    type_register_static(&bus_info);
    type_register_static(&bus_bridge_info);
    type_register_static(&device_type_info);
}

type_init(meta_switch_register)
