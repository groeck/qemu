/*
 * "Stmp" device
 *
 * Written by Guenter Roeck
 */

#ifndef HW_MISC_STMP_H
#define HW_MISC_STMP_H

#include "hw/qdev-properties.h"
#include "hw/sysbus.h"

#define TYPE_STMP_DEVICE "stmp-device"

#define STMP_DEVICE(obj) \
    OBJECT_CHECK(StmpDeviceState, (obj), TYPE_STMP_DEVICE)

typedef struct {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    char *name;
    uint64_t size;
    uint32_t regval;
} StmpDeviceState;

/**
 * create_stmp_device: create and map a dummy device with STMP register layout
 * @name: name of the device for debug logging
 * @base: base address of the device's MMIO region
 * @size: size of the device's MMIO region
 *
 * This utility function creates and maps an instance of stmp-device,
 * which is a dummy device which follows STMP register layout.
 */
static inline void create_stmp_device(const char *name, hwaddr base,
				      hwaddr size)
{
    DeviceState *dev = qdev_create(NULL, TYPE_STMP_DEVICE);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    qdev_init_nofail(dev);

    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base, 0);
}

#endif
